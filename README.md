# echo_drone 竞赛版

国机赛无人机任务系统的精简仓库，只保留比赛飞行所需的定位、感知、导航、决策、
飞控适配和投放链路。

- 系统架构：[`ARCHITECTURE.md`](ARCHITECTURE.md)
- 试飞检查：[`COMPETITION_CHECKLIST.md`](COMPETITION_CHECKLIST.md)

## 明天试飞计划（新机首飞）

> 目标：只验证 **起飞 + 定点导航**，不跑完整投掷/穿门任务。

### 一、地面 / 无桨

- [ ] 确认雷达是 MID-360s，入口用 `third_party/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py` 和 `MID360s_config.json`。
- [ ] `colcon build` + `source install/setup.bash`。
- [ ] `./run_echo_drone.sh check`、`./run_echo_drone.sh hardware-dry`。
- [ ] `FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-state`，确认 `/mavros/state` connected。
- [ ] 起 `livox → pointlio → obstacle → nav`，检查：
  - `/livox/lidar`、`/livox/imu`、`/Odometry`、`/robot/current_pose` 有数据；
  - TF 完整：`map → odom → livox_raw → livox → camera_link`；
  - costmap 有数据，RViz 中障碍物位置正常。
- [ ] 先只测 Nav2：RViz 发 `/goal_pose`，短距离（0.5~1 m）定点，绕过 behavior_control。

### 二、定点参数（上桨前改好）

- [ ] `behavior_control/src/behavior_control.cpp`：step 1 当前是 `current_step = 111`；
      只测定点时临时改成 `current_step = 21;`，跳过随机靶搜索。
- [ ] `robot_bring_up/config/drone.yaml`：
  - `if_hit_tank/car/pillbox/tent/bridge: false`（不投弹）
  - `if_passing_door: false`
  - `if_need_passing_all: false`
  - `target_sequence` 不要包含 `tank`
- [ ] TEB 降载：`controller_frequency` 5~7，costmap `update_frequency` 10，
      `max_vel_x/y` 先 0.3 左右。
- [ ] `flight_control/config/mavros_adapter.yaml`：确认 `cruise_height: 1.0`，与 behavior 一致。
- [ ] 关闭 Point-LIO 自动重启或准备好 RC 接管（`respawn=True` 有空中跳变风险）。

### 三、真机起飞

- [ ] 先起 `./run_echo_drone.sh hardware-real`，再起 sensors/nav，最后起 `./run_echo_drone.sh behavior`。
- [ ] `behavior_control` 构造函数会等 Nav2 action server 和三个 set_parameters 服务，必须最后启动。
- [ ] RC 确认：解锁、切 OFFBOARD、切回 Position、kill switch。
- [ ] 起飞后先悬停 5~10 s，观察高度稳定性和 LIO/气压一致性。
- [ ] 全程录 bag，准备随时 RC 接管。

### 四、明天重点盯的已知问题

- [ ] Nav2 goal 目前每 100 ms 重发一次，对 TEB 不友好：先用 RViz 验证 Nav2，再测 behavior。
- [ ] LIO 实测约 6.7 Hz，TEB/costmap 频率和速度都要按实际位姿率降下来。
- [ ] `map->odom`、`livox_raw->livox` 只在 `drone.launch.py` 里发；分模式启动要自己补 TF。
- [ ] Point-LIO 先验地图路径、`map_server.yaml_filename`、串口逻辑名要按比赛机实际路径改。
- [ ] 高度偏置：behavior `+0.39`、adapter `+0.31/+0.08`、obstacle `+0.27`，改动任一处都要重新标定。
- [ ] 穿门速度模式的绝对 yaw 走 `/mavros/setpoint_raw/local`，确认 MAVROS `setpoint_raw` 插件已加载（明天大概率不测穿门，先记着）。

## 核心链路

```text
Livox MID-360s → Point-LIO → /robot/current_pose
                                   │
                                   ▼
behavior_control → /robot/target_pose or /cmd_vel
                                   │
                                   ▼
flight_control/mavros_adapter → MAVROS → PX4
                                   │
                                   └→ servo_node → STM32 投放

obstacle_segmentation → /cloud_obstacle → Nav2/TEB costmap
```

## 包清单

| 包/目录 | 职责 |
|---|---|
| `third_party/livox_ros_driver2` | MID-360s 雷达驱动 |
| `Point-LIO/` | LiDAR-IMU 里程计 |
| `obstacle_segmentation_tc/` | 点云障碍物分割 |
| `teb_local_planner/` | TEB 局部规划器（Foxy fork） |
| `behavior_control/` | 任务决策状态机 |
| `flight_control/` | `/robot/*` ↔ MAVROS 适配层 |
| `mavlink_control/` | 旧 pymavlink 直连 fallback（默认不启动） |
| `robot_mapping/` | 离线建图：merge_pcd、pcd2pgm |
| `servo_node/` | STM32 舵机/投放控制 |
| `robot_serial_manager/` | 串口存在性与 owner 检查 |
| `robot_interfaces/` | 自定义消息 |
| `robot_bring_up/` | launch 与全局配置 |

说明：`mavlink_control` 和 `robot_mapping` 属于备用/离线工具，不参与主飞行链路；
旧行为树和开发期重构文档已移除。

## 快速开始

```bash
colcon build
source install/setup.bash

./run_echo_drone.sh check
./run_echo_drone.sh hardware-dry
./run_echo_drone.sh livox
./run_echo_drone.sh pointlio rviz:=false
./run_echo_drone.sh obstacle
./run_echo_drone.sh nav
./run_echo_drone.sh hardware-real
./run_echo_drone.sh behavior
```

启动顺序和注意事项见 [`ARCHITECTURE.md`](ARCHITECTURE.md) 第 7、9 节。
试飞前逐项对照 [`COMPETITION_CHECKLIST.md`](COMPETITION_CHECKLIST.md)。

## 环境变量

| 变量 | 默认 | 用途 |
|---|---|---|
| `FCU_URL` | `/dev/px4_fcu:230400` | MAVROS 飞控串口 |
| `DRONE_PARAMS` | `install/.../drone.yaml` | 主参数文件 |
| `ECHO_DRONE_WS` | 仓库根目录 | 工作区路径 |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | RMW |
