# echo_drone 竞赛版

国机赛无人机任务系统的精简仓库，只保留比赛飞行所需的定位、感知、导航、决策、
飞控适配和投放链路。

- 系统架构：[`ARCHITECTURE.md`](ARCHITECTURE.md)
- 试飞检查：[`COMPETITION_CHECKLIST.md`](COMPETITION_CHECKLIST.md)

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
| `servo_node/` | STM32 舵机/投放控制 |
| `robot_serial_manager/` | 串口存在性与 owner 检查 |
| `robot_interfaces/` | 自定义消息 |
| `robot_bring_up/` | launch 与全局配置 |

已移除：旧 `pymavlink` fallback、旧行为树、离线建图工具和开发期重构文档。

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
