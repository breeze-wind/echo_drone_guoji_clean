# Point-LIO config

Point-LIO 的实际参数统一放在 `robot_bring_up/config/drone.yaml` 的
`laser_mapping` 段，并由 `pointlio.launch.py` 通过 `config_path` 传入。
这里保留空目录是为了兼容 `Point-LIO/CMakeLists.txt` 的 install(DIRECTORY config ...)。