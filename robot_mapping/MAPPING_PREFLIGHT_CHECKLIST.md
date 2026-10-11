# 建图、生成地图与起飞前检查清单

适用于 Point-LIO PCD → `pcd2pgm` PNG/YAML → Nav2 `map_server` → coverage 搜索区域配置。
生成地图文件不代表 Nav2 已切换到该地图；按下面顺序逐项确认。

## 1. 采集前：安全与坐标系

- [ ] 场地已清空人员，飞行边界、围挡、急停/遥控接管方式已确认；首次采集低速、低高度进行。
- [ ] 雷达、相机和飞控安装牢固；确认雷达朝向、相机朝向和静态外参与实物一致。
- [ ] 检查 `robot_bring_up/config/static_frames.yaml`：`map→odom`、雷达/机体/相机变换正确，尤其没有遗留的雷达 Z 轴 90° 安装补偿。
- [ ] Point-LIO 的 `/Odometry`、`/robot/current_pose` 和 TF 连续稳定；机体静止时没有明显跳变。
- [ ] 明确本次点云采用的地图坐标系。之后的地图、Nav2 目标点、coverage 矩形角点都必须使用同一 `map` 坐标系。

## 2. Point-LIO 采集 PCD

Point-LIO 将分段点云写到源码目录 `Point-LIO/PCD/`。默认配置不会保存：在
`robot_bring_up/config/drone.yaml` 的 `laser_mapping.ros__parameters.pcd_save` 下，采集时临时设置：

```yaml
pcd_save_en: true
interval: 900       # 每累计一定扫描帧写一个分段 PCD；按文件大小/采集时长调整
```

- [ ] 确认 `Point-LIO/PCD/` 有足够磁盘空间，并记录采集开始时间。
- [ ] 启动 Point-LIO，按安全路线覆盖需要导航的场地；避免人员/车辆移动，减少动态物体被建入地图。
- [ ] 用独立采集入口启动（自动启动 Livox、必要静态 TF 和 Point-LIO；强制开启 PCD 保存，不启动 Nav2/MAVROS）：

  ```bash
  ./tools/record_mapping_pcd.sh 900
  ```

  `900` 是累计扫描帧数的分段间隔，可按磁盘空间/采集时长调整。也可用 `./run_echo_drone.sh map-record 900`。
- [ ] 采集结束后正常停止 Point-LIO，让析构逻辑把缓冲区最后一段点云写盘；检查本次新生成的 PCD 文件及大小。
- [ ] 只处理本次采集的数据。若要运行 `merge_pcd`，输入目录必须是本次数据的干净目录，只放需要合并的分段文件。
- [ ] `merge_pcd` 会把目录中的所有 `.pcd` 合并，并把新文件写回同一目录；不要在含有旧 merged 文件的目录重复合并，否则会把旧地图再次并入。
- [ ] 如果已有一个完整 PCD，可跳过合并，直接使用它。

## 3. 统一路径并转换地图

编辑 [map_paths.yaml](../robot_bring_up/config/map_paths.yaml)：

```yaml
map_paths:
  pcd_file: ""  # 单文件模式填写；合并模式可留空
  merged_pcd_dir: "/绝对路径/本次建图分段目录"  # 非空时优先合并此目录内原始 PCD
  map_output_dir: "/home/echo/echo_drone_guoji_clean/robot_bring_up/maps"
  map_name: "field_map"
  nav_map_yaml: "blank_20m.yaml"  # 验收前先保留空白地图
```

- [ ] 单文件模式确认 `merged_pcd_dir: ""` 且 `pcd_file` 指向完整 PCD；分段模式确认 `merged_pcd_dir` 只包含本轮原始分段。`map_output_dir` 存在且可写；`map_name` 不带扩展名。
- [ ] 生成 PNG/YAML：

  ```bash
  ./run_echo_drone.sh pcd-to-map
  ```

  单 PCD 模式也可以显式传参：

  ```bash
  ./tools/pcd_to_nav_map.sh /绝对路径/本次地图.pcd /输出目录 field_map
  ```

- [ ] 默认转换分辨率为 0.05 m/cell，`point` 方法按 Z 高度范围提取障碍。若地图缺障碍/噪点多，先在 RViz 查看 PCD，再调整脚本的 `MAP_X_MIN/MAX`、`MAP_Y_MIN/MAX`、`MAP_Z_MIN/MAX`、滤波半径和最小邻居数；不要直接把滤波阈值当作已验证参数。
- [ ] 确认输出 `field_map.png` 和 `field_map.yaml` 都存在且非空；YAML 的 `image` 文件名与 PNG 一致，`resolution`、`origin` 合理。

## 4. 地图验收与 Nav2 配置

- [ ] 在 RViz 中加载生成的 YAML/PNG，检查地图上下方向、X/Y 方向、边界、墙体和障碍物是否与现场对应。
- [ ] 用至少两个已知现场位置核对地图坐标与 `/robot/current_pose`；检查 PCD→地图 `origin`，不要仅凭图像“看起来对”判断坐标对齐。
- [ ] 确认地图没有将地面/天花板误标成障碍，也没有因采集时人员或移动物体留下大片假障碍。
- [ ] 验收通过后，把 `map_paths.yaml` 的 `nav_map_yaml` 改为地图文件名，例如 `field_map.yaml`。相对路径按 `robot_bring_up/maps/` 下文件解析。
- [ ] 重新构建并 source，使配置和新地图复制到 install：

  ```bash
  ./run_echo_drone.sh build
  source install/setup.bash
  ```

- [ ] 启动 `full`/`nav` 后确认 map server 成功加载正确文件；RViz Fixed Frame 使用 `map`，确认显示的地图范围与现场一致。
- [ ] 若只想恢复空白地图，把 `nav_map_yaml` 改回 `blank_20m.yaml`，再构建并重启。

## 5. 起飞前最后检查

- [ ] `map_paths.yaml` 的 `nav_map_yaml` 是本次验收通过的地图；Nav2 日志显示 map server 已加载，RViz 地图没有偏移/翻转。
- [ ] 检查 TF 树连通：Nav2 所用 `map` 到机器人底盘/雷达 frame 可变换，且没有重复发布冲突的静态变换。
- [ ] 检查 costmap 机器人位置落在地图内；没有持续的 out-of-bounds、传感器原点越界或点云观测过期警告。
- [ ] 在 RViz 点一个近距离、空旷、可人工接管的目标，确认全局路径避开障碍；低速验证控制方向和停止响应。
- [ ] 如果运行 coverage 搜索，再配置 `coverage_mission/config/mission.yaml` 的 `corner_a`、`corner_b`：必须是新地图 `map` 坐标系下、位于地图边界内的两个对角点；核对 `lane_spacing`、巡航高度和门口坐标。
- [ ] 确认电池、桨叶/电机状态、飞手接管、急停及降落区域。先空载/无投递验证；地图或坐标有疑问时不执行自主航线。

## 本次记录

- 采集日期/场地：
- 输入 PCD：
- 输出 YAML：
- `resolution` / `origin`：
- RViz 与现场核对人/结果：
- 使用的 `corner_a` / `corner_b`（如运行 coverage）：
- 起飞前异常/未解决项：
