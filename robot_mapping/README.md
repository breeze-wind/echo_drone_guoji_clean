# robot_mapping

pcd2pgm节点实现了三种方法，可以从yaml文件中修改。

第一种是通过直通滤波将点云限制在一个比较合理的范围，将天花板和地面排除，将剩下的所有点都视为障碍物。这种方法的优点是简单、直观，但是比较依赖于直通滤波的参数，而且对于高地的处理不好。

第二种是将所有点转变为高程图，然后遍历每个像素，如果某点与周围的点高度差比较大，则视为障碍物。

第三种是将计算所有点的法向量，判断法向量的角度。

提供了三种PCL滤波器，直通滤波、半径滤波、统计滤波、体素滤波，各有不同的作用，可以按需调整。其中，作者个人感觉统计滤波效果比半径滤波效果好。

在保存地图时进行了开运算。如果效果不好可以关闭。

不管如果处理，受到point-lio建图效果的影响（会把场地中的人也建进去），都需要进行大量后处理，无法拿来即用。

## Point-LIO PCD 转 Nav2 地图

工作区提供 `./run_echo_drone.sh pcd-to-map` 一键离线建图。若 `map_paths.yaml` 中 `merged_pcd_dir` 非空，脚本会先按文件名排序合并该目录内的原始 PCD（自动跳过已有的合并产物），再用合并结果生成 PNG 和 YAML。否则使用 `pcd_file` 或命令行输入的单个 PCD：

```bash
./run_echo_drone.sh pcd-to-map
```

单 PCD 模式仍可指定输入、输出目录和地图名：

```bash
./tools/pcd_to_nav_map.sh /path/to/field.pcd /path/to/maps field_map
```

PCD 文件、分段 PCD 目录、PNG/YAML 输出目录、地图名和 Nav2 使用的地图 YAML 集中配置在 `robot_bring_up/config/map_paths.yaml`。合并模式优先使用 `merged_pcd_dir`，`pcd_file` 留空即可；单文件模式则清空 `merged_pcd_dir` 并填写 `pcd_file`。合并输出以唯一时间戳命名并留在输入目录中；重复运行不会把此前生成的合并文件再次叠加。默认栅格分辨率为 0.05 m，默认采用 `point` 方法；可通过 `MAP_*` 环境变量调整裁剪范围和滤波参数，运行 `./tools/pcd_to_nav_map.sh --help` 查看参数。

转换结果仍需人工检查：确认 PNG 中障碍物、地图方向和 YAML 的 `origin` 正确。Nav2 读取同一配置中的 `nav_map_yaml`；将它改成生成的地图 YAML 后，重新构建 `robot_bring_up` 并重启 Nav2 即可切换，不会影响正在运行的实例。
