# echo_drone 竞赛系统架构（纯比赛版）

> 本仓库只保留国机赛无人机任务的最小可飞链路，去掉了旧 pymavlink fallback、
> 旧行为树、离线建图工具和开发期文档。任务流程以 `behavior_control` 的状态机为准。

## 1. 一句话架构

ROS2 Foxy 机载系统：Livox MID-360s + Point-LIO 提供定位，点云障碍分割喂 costmap，
Nav2 + TEB 负责导航，behavior_control 负责任务决策，flight_control 通过 MAVROS
驱动 PX4，servo_node 控制 STM32 投放机构。

## 2. 总体数据流

```text
Livox MID-360s
  └─ livox_ros_driver2 ──► /livox/lidar, /livox/imu
                                  │
                                  ▼
                             Point-LIO ──► /Odometry
                                  │           /robot/current_pose
                                  │           TF: odom -> livox_raw -> livox
                                  └─► /cloud_registered_body
                                              │
                                              ▼
                                  obstacle_segmentation ──► /cloud_obstacle
                                              │
                                              ▼
                     Nav2 (NavFn + TEB + costmaps) ──► /cmd_vel ──┐
                                                                   │
behavior_control ──► NavigateToPose action ────────────────────────┤
        │           /robot/target_pose ────────────────────────────┤
        │           /robot/nav_state /passing_door_state /turning ─┤
        │                                                          ▼
        │                                  flight_control/mavros_adapter
        │                                                    │
        │                                                    ▼
        │                                          MAVROS ──► PX4
        │
        └──► /servo_node/set_parameters ──► servo_node ──► STM32 舵机投放

D435 / USB 相机 ──► /robot/image_location, /robot/usb_camera ──► behavior_control
```

## 3. 分层模块

| 层 | 包/目录 | 职责 |
|---|---|---|
| 传感器 | `third_party/livox_ros_driver2` | MID-360s 点云 + IMU 驱动 |
| 定位 | `Point-LIO/` | LiDAR-IMU 里程计：`/Odometry`、`/robot/current_pose`、TF、机体系点云 |
| 感知 | `obstacle_segmentation_tc/` | 机体系点云 → `/cloud_obstacle`，供 costmap 避障 |
| 导航 | `teb_local_planner/`、Nav2 | NavFn 全局规划 + TEB 局部规划，输出 `/cmd_vel` |
| 决策 | `behavior_control/` | 任务状态机：起飞、搜索、投掷、穿门、降落 |
| 飞控适配 | `flight_control/` | `/robot/*` ↔ MAVROS：vision pose、位置/速度 setpoint、解锁/模式 |
| 执行机构 | `servo_node/` | STM32 舵机串口，收到参数/命令后执行投放 |
| 硬件健康 | `robot_serial_manager/` | 串口存在性和 owner 检查，不抢占设备 |
| 接口 | `robot_interfaces/` | `ImageLocation`、`ServoPos` 等自定义消息 |
| 启动/配置 | `robot_bring_up/` | 各层 launch 与 `drone.yaml` 总参数 |

## 4. 任务状态机（behavior_control）

`current_step` 编号与动作：

| step | 动作 |
|---|---|
| 0 | 等待 `/robot/arm_state` 解锁 |
| 1 | 位置设定点爬升到 `cruise_height` |
| 111~114 | 起飞点左右两点搜索随机靶 |
| 21~24 / 31~34 / 41~44 / 51~54 | 静态靶：导航 → 拉高 → 识别 → 下降投弹 |
| 91~93 | 三个航点搜索随机靶 / 随机 tank |
| 61~64 | 随机 tank：导航 → 识别 → 投弹 |
| 101~104 | 随机靶：导航 → 识别 → 投弹 |
| 71~74 | 穿门：起点 → 原地转向 → 穿门终点 |
| 75 / 81 / 82 | 返航 / 降落 |

说明：

- 原“起飞后匀速圆周”调试截断已注释，step 1 现在直接进入原任务流程。
- 目标 ID 见 `robot_interfaces/msg/Receive/ImageLocation.msg`：
  `0-H, 1-car, 2-bridge, 3-tent, 4-pillbox, 5-tank, 6-随机靶`。
- `target_positions_` 未注册 `tank`，`target_sequence` 不要写 `tank`。

## 5. 关键接口

### 定位与感知

| 话题 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 驱动 → Point-LIO | 点云 |
| `/livox/imu` | `sensor_msgs/Imu` | 驱动 → Point-LIO | IMU |
| `/Odometry` | `nav_msgs/Odometry` | Point-LIO → Nav2/TEB | 里程计 |
| `/robot/current_pose` | `TransformStamped` | Point-LIO → 决策/飞控/感知 | 当前位姿 |
| `/cloud_registered_body` | `PointCloud2` | Point-LIO → 分割 | 机体系点云 |
| `/cloud_obstacle` | `PointCloud2` | 分割 → costmap | 障碍物 |

### 导航与决策

| 话题/动作 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action` | 决策 → Nav2 | 定点导航 |
| `/cmd_vel` | `Twist` | TEB → adapter | 速度控制 |
| `/robot/target_pose` | `TransformStamped` | 决策 → adapter | 位置设定点 |
| `/robot/nav_state` | `Bool` | 决策 → adapter | true 速度 / false 位置 |
| `/robot/passing_door_state` | `Bool` | 决策 → adapter | 穿门标志 |
| `/robot/turning_state` | `Bool` | 决策 → adapter | 转向标志 |
| `/robot/obstacle_height` | `Float64` | 决策 → 分割 | 障碍高度上限 |
| `/robot/clear_state` | `Bool` | 决策 → 感知/costmap | 清障碍 |

### 飞控与执行

| 话题/服务 | 方向 | 用途 |
|---|---|---|
| `/robot/arm_state` | adapter → 决策 | 解锁状态 |
| `/mavros/state` | MAVROS → adapter | 心跳/模式 |
| `/mavros/rc/in` | MAVROS → adapter | RC 解锁 |
| `/mavros/vision_pose/pose` | adapter → MAVROS | 视觉位姿 |
| `/mavros/setpoint_position/local` | adapter → MAVROS | 位置设定点 |
| `/mavros/setpoint_velocity/cmd_vel` | adapter → MAVROS | 速度设定点 |
| `/mavros/cmd/arming` / `set_mode` | adapter → MAVROS | 解锁 / OFFBOARD |
| `/servo_node/set_parameters` | 决策 → servo | 投放触发 |
| `/servo/status` | servo → 调试 | 舵机状态 |

### 视觉输入

| 话题 | 类型 | 来源 | 用途 |
|---|---|---|---|
| `/robot/image_location` | `ImageLocation` | D435 | 静态/随机靶识别 |
| `/robot/usb_camera` | `ImageLocation` | USB 相机 | 起飞点随机靶识别 |
| `/camera/choose` | `Bool` | 决策 → 相机节点 | 相机切换 |

## 6. 硬件接口

| 硬件 | 逻辑设备/接口 | 说明 |
|---|---|---|
| 机载计算机 | Jetson / ARM64 / ROS2 Foxy | 跑全部节点 |
| Livox MID-360s | 以太网 | `/livox/lidar`、`/livox/imu` |
| PX4 飞控 | `/dev/px4_fcu:230400` | MAVROS 独占 |
| STM32 舵机板 | `/dev/stm32_servo:115200` | servo_node 独占 |
| D435 | `camera_link` | `/robot/image_location` |
| USB 相机 | livox 系 | `/robot/usb_camera` |
| RC 遥控器 | `/mavros/rc/in` | 解锁 / OFFBOARD / 接管 |

## 7. 启动顺序

```bash
./run_echo_drone.sh check
./run_echo_drone.sh hardware-dry          # 台架
./run_echo_drone.sh livox                 # MID-360s 驱动
./run_echo_drone.sh pointlio rviz:=false  # Point-LIO
./run_echo_drone.sh obstacle              # 障碍分割
./run_echo_drone.sh nav                   # Nav2 bringup
./run_echo_drone.sh hardware-real         # MAVROS + adapter + servo
./run_echo_drone.sh behavior              # 任务决策（需 Nav2/servo 已起）
```

注意：

- `behavior_control` 构造函数会等待 Nav2 action server 和三个 set_parameters 服务，
  必须最后启动。
- `map->odom`、`livox_raw->livox` 两个 static TF 只在 `drone.launch.py` 中发布；
  用 `full` 模式或自行补 TF，否则 Nav2 无法工作。

## 8. 配置入口

| 文件 | 内容 |
|---|---|
| `robot_bring_up/config/drone.yaml` | 任务靶点/高度/开关、Point-LIO、障碍分割、Nav2、TEB、costmap、map_server |
| `flight_control/config/mavros_adapter.yaml` | dry_run、坐标语义、高度偏移、MAVROS 话题 |
| `robot_bring_up/config/hardware/ports.yaml` | PX4/STM32 串口和 owner |
| `robot_bring_up/config/hardware/servo.yaml` | 舵机命令范围、投放命令、写周期 |
| `robot_bring_up/config/hardware/mavros.yaml` | MAVROS 连接参数 |

## 9. 已知限制（试飞前必读）

- 雷达驱动默认入口是标准 MID-360，Mid-360s 需使用 `msg_MID360s_launch.py` 和
  `MID360s_config.json`（本仓库已改为 360s 入口，仍需按实机核对 IP）。
- Point-LIO 是 LIO 里程计，无回环和全局重定位；预建地图与原点的对齐需要人工保证。
- MID-360 点云约 10 Hz；若实测 `/Odometry` 更低，TEB/costmap 频率必须相应降低。
- behavior_control 在导航状态每 100 ms 重发一次 `NavigateToPose`，对 TEB 不友好；
  定点测试建议先用 RViz 发 `/goal_pose` 验证 Nav2。
- 高度偏置分散：behavior `+0.39`、adapter `+0.31/+0.08`、obstacle `+0.27`；
  修改任一处都要重新标定。
- Point-LIO 先验地图路径、map_server 地图路径、串口逻辑名都需要按比赛机环境替换。
