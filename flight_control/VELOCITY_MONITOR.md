# 速度对比诊断

导航、Point-LIO 与 MAVROS 适配器已经启动时，另开终端运行：

```bash
./run_echo_drone.sh speed-check
```

节点只订阅数据，不下发飞控命令。终端每秒打印一次：

- `TEB机体系`：Nav2 `/cmd_vel` 期望的水平速度；
- `实测`：从 `/robot/current_pose` 最近约 0.6 秒的 odom 位姿回归估计的速度；
- `沿轨 ... (%)`：实际沿期望方向的速度与期望速度之比；
- `横向`：垂直于期望方向的实际速度；
- `发送速度`：适配器发布到 MAVROS 速度话题的值。位置模式下为“无”；
- `位置目标误差`：位置模式下的目标与当前 XY 距离。

完整结构化数据在 `/flight_control/velocity_comparison`（`std_msgs/String`
中的 JSON）。出现 `速度比较等待数据` 时，先检查日志中的
`pose_age`、`cmd_age` 与 `pose_frame`；没有活动 Nav2 目标时不计算速度比。

“发送速度”只能证明 ROS 适配器发布了消息，不能证明 PX4 已执行。
实测速度也不是独立测速仪，而是 Point-LIO 位姿微分；定位跳变或严重延迟
会使比较不可靠。本机的 `map -> odom` 只有竖直平移；若以后加入 XY 旋转，
需更新 ENU 比较坐标变换。机体系比较不依赖这项对齐。

隔离测试：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 tools/test_velocity_monitor_dry_run.py
```
