# TEB 局部轨迹位置跟踪实验

当前配置仍使用原来的 `/cmd_vel` → MAVROS 速度 setpoint；纯位置跟踪开关
`teb_position_follow_enabled` 默认保持 `false`。
打开后，适配器从 `/local_plan` 读取 TEB 的
`nav_msgs/Path`，只取 map 系 XY，沿路径生成短前瞻位置 setpoint；
`Path.z` 是 TEB 可视化时间，不作为飞行高度。高度仍取适配器的
`cruise_height` / `passing_door_height`。

实验模式在位置目标与当前位置之间限制最大超前距离和推进速度。
TEB 路径或 `/cmd_vel` 过期、Nav2 停止发运动指令、横向偏差过大时，
会持续发布当前位置的保持 setpoint，不再沿旧轨迹前进。位姿或
`map <- odom` TF 缺失时不发布 setpoint，不能用于实际飞行。
启用期间不发布 MAVROS 速度 setpoint；这不是 PX4 位置加速度/速度前馈控制。

先运行不触及飞控的隔离测试：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 tools/test_teb_position_dry_run.py
python3 tools/test_cmd_vel_dry_run.py
```

真实系统仅在验证 `/local_plan` 话题、坐标轴、地图与里程计对齐，
并完成无桨/系留测试后使用。不要同时运行另一个向相同 MAVROS
setpoint 话题发布命令的节点。

组合控制开关 `teb_position_velocity_follow_enabled` 默认启用，且不能与纯位置
跟踪同时打开。可通过 launch 参数 `teb_position_velocity_follow_enabled:=false`
关闭。它使用 MAVROS raw-local
`PositionTarget` 单消息同时发送短前瞻位置目标和限幅后的水平速度前馈；
高度仍用位置目标，速度上限由
`teb_position_velocity_feedforward_max` 控制（默认 0.15 m/s）。输入路径、
速度或位姿失效时，位置目标锁到当前位姿、速度前馈归零。该模式不是当前
已验证的飞行配置；先在隔离话题 dry-run 检查字段与坐标，再做无桨/系留
测试。启用前还需确认本机 MAVROS raw-local 订阅确实是配置的
`/mavros/mavros/local` 上的 `mavros_msgs/msg/PositionTarget` 订阅；同名的
`geometry_msgs/msg/PoseStamped` 端点属于位置插件，需按消息类型区分。
如果 `ros2 topic info` 显示该地址不匹配，
先通过 `mavros_raw_setpoint_topic` 指向真实 raw-local subscriber；不要在未确认
subscriber 类型/数量时进行实飞。

默认启动即进入该模式（例如 `./run_echo_drone.sh mavros-real`）；需要关闭时传入
`teb_position_velocity_follow_enabled:=false`。先保持螺旋桨拆除，
确认 `/flight_control/status` 中模式为 `teb_position_velocity`，并在 MAVROS
的 raw target 回环话题确认位置、XY 速度和 type mask 正确后，才考虑系留测试。

隔离 dry-run 命令：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 tools/test_teb_position_velocity_dry_run.py
```
