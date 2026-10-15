# RViz 矩形 costmap 编辑

使用 `./run_echo_drone.sh nav` 启动带 RViz 的导航后，工具栏中有两个 Publish Point 工具：

- `add_click`：连续点矩形的两个对角点，将矩形标为障碍。
- `erase_click`：连续点两个对角点，将矩形强制标为 free。

RViz Fixed Frame 必须是 `map`。选中对应工具后，工具属性中的 Topic 可用于确认当前是添加还是擦除。矩形以半透明红色（障碍）或绿色（强制 free）显示。编辑只作用于运行中的 Nav2 costmap，不会修改静态地图 PNG/YAML。

要撤销最近一次添加或擦除，在 RViz 的 `Manual Costmap Undo` 面板点击 `Undo last rectangle`。撤销会同时从 costmap 操作历史和矩形可视化中移除最后一个矩形。也可通过 ROS 服务调用：

```bash
ros2 service call /manual_costmap/undo std_srvs/srv/Trigger "{}"
```

每个矩形持续有效到导航节点退出；清空 costmap 不会清除这些手工矩形。擦除矩形会覆盖静态地图、障碍点云以及膨胀层在矩形内部写入的代价，因此那里会被规划器当作可通行区域，即使现场实际上存在障碍。谨慎使用，飞行前检查范围。
