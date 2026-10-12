# 独立蛇形找靶任务

本包独立于 `behavior_control` 的固定 5 点任务；不修改、不启动那段代码。
依赖已有 `coverage_search` 航线生成器、Nav2、`target_perception`、MAVROS 和独立的 `electromagnet_node`。

## 使用前配置

编辑 `robot_bring_up/config/coverage_mission.yaml` 的 `corner_a`、`corner_b`，填写 `map` 坐标系下
搜索矩形的两个对角点，例如 `[x, y]`，这两项为空时任务不会启动。同时核对
`door_start_xy`、`door_end_xy` 分别填写门口起点和门后终点的 `map` 系坐标；任务会先导航到
起点，再用位置 setpoint 降到 `door_height`（默认 0.9 m），切换门洞 TEB 参数后只发送
一次门后终点的 Nav2 目标，不要求固定的门中心坐标。门内位置变化的障碍物由实时
代价地图与 TEB 处理；如果通道实际堵死，导航会失败并悬停重试，不应靠缩小机体
footprint 强行穿越。穿门速度模式高度由
`flight_control/config/mavros_adapter.yaml` 的 `passing_door_height` 控制，必须与
`door_height` 保持一致。固定门坐标还要求重启后的 Point-LIO 地图与建图时的坐标水平对齐；
当前 `map→odom` 只有固定高度偏置，并不提供自动全局重定位，不能仅凭固定航点保证
物理门的位置正确。飞前需用静态地图、点云与位姿验证对齐，偏移未解决时不要自动穿门。
`lane_spacing` 要依据实测地面视野留重叠；
`search_height` 必须与 `flight_control/config/mavros_adapter.yaml` 的
`cruise_height` 一致；航线高度由 MAVROS adapter 的速度模式高度控制器维持。
需要手动指定航点时，在 `custom_waypoints_xy` 填入 map 系平面坐标的扁平序列，例如
`[x1, y1, x2, y2]`。非空时按顺序导航到这些点，航向自动朝向下一个点，末点沿用上一段航向；
留空 `[]` 则照常根据矩形生成蛇形航线。`corner_a`/`corner_b` 仍作为靶标检测的有效区域边界，
手动航点和搜索区域都应在 Nav2 地图可行驶区域内。
`drop_offsets_xy` 是三个货物相对靶心的 map 平面偏置（每两个数一组），
`drop_commands` 使用与控制 UI 相同的一基状态号。状态 1/2/3/4 分别表示 3/2/1/0 个实体磁铁吸合；任务默认投三件，因此使用 `[2, 3, 4]`，依次释放三件货物。请先确认货物与电磁铁通道的物理对应顺序。
相机/雷达/机体的静态 TF 集中在 `robot_bring_up/config/static_frames.yaml`，平移单位为米，
旋转为 ROS 四元数 `[x, y, z, w]`；相机内参、模型路径和检测参数在
`target_perception/config/target_perception.yaml`。修改这些源配置后重新构建相关包并 source。

分别启动整机导航/定位、感知、电磁铁和任务节点：

```bash
./run_echo_drone.sh full-real
./run_echo_drone.sh target-perception
./run_echo_drone.sh electromagnet-real
./run_echo_drone.sh coverage-mission
```

`electromagnet-real` 默认通过 `/dev/ttyUSB0` 以 115200 8-N-1 打开串口，启动只发送 `GET`，不改变电磁铁状态；若设备号变化，可用 `ELECTROMAGNET_PORT=/dev/ttyUSB1 ./run_echo_drone.sh electromagnet-real` 覆盖。先用 `./run_echo_drone.sh electromagnet-dry` 验证 ROS 接口；dry-run 不会打开串口，也不能驱动电磁铁。节点报告 MCU 状态后才会发布 `ready=true`。

ROS 接口：向 `/electromagnet/set_state` 发布 `std_msgs/Int32`，值 1/2/3/4 与 UI 四个按钮一一对应。决策节点按 `actuator_repeat_period_sec`（默认 1 Hz）重复发布当前状态：启动后状态1；投递一件后保持状态2，投递两件后保持状态3，投递三件后保持状态4。状态 1/2/3/4 分别表示 3/2/1/0 个实体磁铁吸合。`/electromagnet/status.state`、`last_sent_command` 和 `pending_command` 用于 MCU 状态反馈；ACK 只证明 MCU 收到命令并更新了固件状态，不代表有独立传感器确认货物确实脱离。

任务配置当前启用了真实投递（`actuator_output_enabled: true`），并且 `require_actuator_status: true`。因此电磁铁节点未就绪、状态超时或处于 dry-run 时，任务不会发布起飞 setpoint；每次投递还必须收到与目标状态匹配的新 MCU ACK，否则任务中止并请求 `AUTO.LAND`。第一次上电请拆桨/固定机体、不要装货，先单独测试串口状态和单路释放。确认端口确为电磁铁控制板、通道/货物顺序正确后，再进行带载试验。不要让 `servo-real` 和电磁铁节点共用同一个串口设备。

默认 `auto_start: true`：
节点启动时打印并发布航点。收到新鲜的 `/robot/current_pose` 后，任务会锁定当前位置
`map` 系的 XY，并以 10 Hz 持续向 `/robot/target_pose` 发布该点上方的巡航高度目标，
同时保持 `/robot/nav_state=false`，由 MAVROS adapter 转成 PX4 位置 setpoint。
Point-LIO 的 `/robot/current_pose` 当前 `header.frame_id=odom`；任务接受此 frame，
并用静态 `map→odom` 的 0.39 m Z 偏移计算高度（要求该静态变换没有 XY 平移和 yaw 旋转）。
这条 setpoint 流用于让飞手可以手动切入 OFFBOARD；任务不会自动解锁或切模式。
飞手手动解锁、切入 OFFBOARD，且飞机到达巡航高度并稳定后，任务才切给 Nav2 并开始搜索；
不需要调用 Trigger。若 `auto_start: false`，仍需手动调用 `/coverage_mission/start`，
之后按同样流程先发布起飞 setpoint。
任务运行期间每秒打印一次当前阶段；搜索时包含当前航点序号和坐标，发现靶标/投递时包含
靶标类别、坐标和投递序号，返回阶段持续打印穿门起点坐标。
如将 `auto_start` 设为 `false`，则需要手动调用 `/coverage_mission/start` Trigger。
搜索中连续确认靶标后，按当前投递序号使用对应偏置，
导航到投递点、切换位置 setpoint 精确对准、下降到 `drop_height`，再向 `/electromagnet/set_state` 发送相应状态，并等待 `/electromagnet/status` 中的 MCU ACK。
已成功投递过的视觉类别 ID 会视为重复靶，后续同类目标不再投递；不同类别各投一件，
最多三类。完成一件后爬回巡航高度并继续剩余蛇形航点；完成三件或路线扫完后停止搜索，飞到
`door_start_xy`、下降并连续导航到 `door_end_xy`。导航失败且位姿新鲜时先悬停，随后重试
同一目标；位姿或飞行状态丢失等情况仍可能请求 PX4 `AUTO.LAND`。飞控拒绝时飞手应立即
接管。先无桨/系留验证电磁铁命令、坐标偏置、高度切换与穿门任务状态。
