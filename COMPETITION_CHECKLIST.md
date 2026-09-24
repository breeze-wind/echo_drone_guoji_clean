# 国机赛试飞检查清单

## 0. 赛前必改

- [ ] 雷达型号确认 MID-360s，驱动使用 `msg_MID360s_launch.py` + `MID360s_config.json`。
- [ ] `robot_bring_up/config/hardware/ports.yaml` 的串口逻辑名与 udev/实际设备一致。
- [ ] `robot_bring_up/config/drone.yaml` 中 `map_server.yaml_filename`、
      `laser_mapping.pcd_save.prior_PCD_map_path` 换成比赛机真实路径。
- [ ] `drone.yaml` 的靶点坐标、`target_sequence`、`if_hit_*`、`if_passing_door`、
      `if_need_passing_all` 与当天赛制一致。
- [ ] `target_sequence` 不包含 `tank`（代码未注册该 key）。
- [ ] 原“起飞后圆周”调试已注释；如需定点测试，临时把 step 1 的
      `current_step = 111` 改成 `21`。
- [ ] 实机测试前把 `if_hit_*` 全设 `false`、`if_passing_door: false`，避免误投弹/穿门。

## 1. 地面检查（不装桨）

- [ ] `colcon build` 通过，`source install/setup.bash`。
- [ ] `./run_echo_drone.sh check` 无缺失。
- [ ] `./run_echo_drone.sh hardware-dry`：串口、舵机、adapter 状态正常。
- [ ] `FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-state`：`/mavros/state` connected。
- [ ] 传感器频率：`ros2 topic hz /livox/lidar`、`/livox/imu`、`/Odometry`、`/robot/current_pose`。
- [ ] `ros2 topic delay /Odometry` 延迟不随时间增长。
- [ ] TF 完整：`ros2 run tf2_tools view_frames`，确认
      `map -> odom -> livox_raw -> livox -> camera_link`。
- [ ] Nav2 lifecycle active，local/global costmap 有数据，TEB 正常加载。
- [ ] RViz 中 `/cloud_obstacle` 与真实障碍位置一致，无 90° 旋转/高度错位。
- [ ] `/servo/status` 正常，投放机构空载或断开。

## 2. 高度与坐标

- [ ] 核对 `behavior +0.39`、`adapter vision +0.31 / fcu_ref 0.08`、
      `obstacle +0.27` 的符号和含义。
- [ ] 无桨状态用尺子/测距对比 LIO z、气压、目标高度。
- [ ] 确认 PX4 `EKF2_HGT_MODE`、`EKF2_EV_*`、baro 权重；室内优先考虑视觉/测距高度。
- [ ] 检查 `/mavros/vision_pose/pose` 的 z 符号和单位正确。

## 3. 起飞前

- [ ] 电池满电，重心/桨叶/电机方向正确，护网就位。
- [ ] PX4 failsafe：RC、数据链、电池、geofence、OFFBOARD 失控保护。
- [ ] RC 功能：解锁、切 OFFBOARD、切回 Position、kill switch 全部试过。
- [ ] `mavros_adapter.yaml` 确认 `dry_run:false`、`auto_set_mode` 设置符合预期。
- [ ] 先起 Nav2/servo，再起 behavior（否则 behavior 构造函数会卡住）。
- [ ] 首飞目标先选 0.5~1 m 近距离，速度先降到 0.3 m/s 左右。
- [ ] 开 rosbag 记录关键话题。

## 4. 飞行中

- [ ] 起飞后先悬停 5~10 s，观察高度稳定性、LIO/气压一致性。
- [ ] 观察 `/robot/current_pose` 连续且时间戳新鲜。
- [ ] 观察 `/cmd_vel` 无剧烈抖动；`/robot/nav_state` 切换正确。
- [ ] 定点导航优先用 RViz `/goal_pose`，确认 Nav2/TEB 稳定后再测 behavior。
- [ ] 随时准备 RC 切出 OFFBOARD 接管。

## 5. 定点导航专项

- [ ] 目标点在地图/costmap 内，`target_positions_` 有对应 key。
- [ ] 到位判定 0.25 m 使用 `/robot/current_pose`，确认有状态推进。
- [ ] 不用 behavior 时，直接给 Nav2 发 goal，绕过 10 Hz goal 重发问题。
- [ ] 用 behavior 时，先接受导航状态会每 100 ms 重发 goal，密切观察 TEB 是否抖动。

## 6. 异常处理

- [ ] LIO 丢失/跳变：立即 RC 接管，必要时切 Position 降落。
- [ ] TF 超时/ Nav2 失败：RC 接管，检查 TF 树和 `/Odometry`。
- [ ] 位姿频率过低：降 TEB/costmap 频率和飞行速度。
- [ ] 投放误触发：确认 `if_hit_*`、`/servo/servo` 当前值、`last_servo_index_`。

## 7. 记录与分析

```bash
ros2 bag record /livox/lidar /livox/imu /Odometry /robot/current_pose \
  /robot/target_pose /cmd_vel /robot/nav_state /robot/arm_state \
  /mavros/state /mavros/setpoint_position/local /mavros/vision_pose/pose \
  /tf /tf_static
```

- [ ] 保存 PX4 日志，回看 EKF 高度、视觉延迟、速度跟踪。
- [ ] 记录每次炸机/异常前的 step、话题状态和时间戳。
