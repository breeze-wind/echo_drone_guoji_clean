#!/usr/bin/env bash
set -Eeuo pipefail

# 重构后的一线操作入口。台架 dry-run、硬件只读检查和旧整机启动都放在
# 同一个脚本下，方便机载单元按子系统逐层验证。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ECHO_DRONE_WS:-$SCRIPT_DIR}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
DRONE_PARAMS="${DRONE_PARAMS:-${WS_DIR}/install/robot_bring_up/share/robot_bring_up/config/drone.yaml}"
MODE="${1:-help}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# help 模式打印这一段；命令说明直接放在脚本内，避免联调时文档和实际命令脱节。
usage() {
  cat <<'EOF'
用法:
  ./run_echo_drone.sh <模式> [额外 ros2 launch 参数...]

安全软件检查:
  check             打印 ROS、工作区和串口设备状态。
  build             构建当前工作区。
  topics            打印当前 ROS 话题列表。

硬件层 dry-run:
  hardware-dry      dry-run 启动串口管理、舵机节点和 MAVROS adapter。
  serial-dry        只 dry-run 启动串口管理。
  servo-dry         只 dry-run 启动舵机节点。
  adapter-dry       只用 dry-run 配置启动 MAVROS adapter。

真实硬件层:
  mavros-state      只启动 MAVROS 连接飞控，不启动 adapter，不下发命令。
  hardware-real     启动串口管理、舵机节点、MAVROS 和 MAVROS adapter。
  mavros-real       启动串口管理、MAVROS 和 MAVROS adapter，不启动舵机。
  servo-real        只启动真实舵机节点，默认设备 /dev/stm32_servo。

传感器、里程计、感知和决策:
  livox             启动 Livox MID-360s 驱动。
  pointlio          只启动 Point-LIO。
  obstacle          只启动点云障碍物分割。
  behavior          只启动 behavior_control。
  nav               只启动 Nav2 bringup。
  full              启动当前旧整机 drone.launch.py，默认关闭 RViz。

常用环境变量覆盖:
  ROS_DOMAIN_ID=0
  ROS_LOCALHOST_ONLY=0
  FCU_URL=/dev/px4_fcu:230400
  DRONE_PARAMS=/path/to/drone.yaml
  ECHO_DRONE_WS=/home/sfx/echo_drone

示例:
  ./run_echo_drone.sh check
  ./run_echo_drone.sh hardware-dry
  ./run_echo_drone.sh livox
  ./run_echo_drone.sh pointlio rviz:=false
  FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-real
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

# source ROS 和工作区时临时关闭 nounset，因为 Foxy 的 setup 脚本会读取一些
# 可选但未设置的 shell 变量。
source_env() {
  [[ -f "$ROS_SETUP" ]] || die "ROS setup 不存在: $ROS_SETUP"
  # shellcheck source=/dev/null
  set +u
  source "$ROS_SETUP"
  set -u

  [[ -f "$WS_SETUP" ]] || die "工作区 setup 不存在: $WS_SETUP; 请先运行 './run_echo_drone.sh build'"
  # shellcheck source=/dev/null
  set +u
  source "$WS_SETUP"
  set -u
}

# 优先显示稳定 udev 别名，再显示原始 USB/ACM 设备；WSL/USBIP 台架环境里
# udev 规则可能尚未生效。
print_devices() {
  local devices=(/dev/px4_fcu /dev/stm32_servo /dev/ttyUSB* /dev/ttyACM*)
  local found=0

  shopt -s nullglob
  for dev in "${devices[@]}"; do
    if [[ -e "$dev" ]]; then
      ls -l "$dev"
      found=1
    fi
  done
  shopt -u nullglob

  if [[ "$found" -eq 0 ]]; then
    echo "未发现匹配的串口设备。"
  fi
}

# 用 exec 让 Ctrl-C 和退出码直接归属到被启动的 ROS 命令。
run() {
  echo "+ $*"
  exec "$@"
}

run_launch() {
  echo "+ ros2 launch $*"
  exec ros2 launch "$@"
}

case "$MODE" in
  help|-h|--help)
    usage
    ;;

  # 不会下发硬件命令的检查入口。
  check)
    source_env
    echo "Workspace: $WS_DIR"
    echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
    echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
    echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
    echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
    uname -a
    echo
    echo "Packages:"
    colcon list --base-paths "$WS_DIR" --names-only
    echo
    echo "Serial devices:"
    print_devices
    ;;

  build)
    [[ -f "$ROS_SETUP" ]] || die "ROS setup 不存在: $ROS_SETUP"
    # shellcheck source=/dev/null
    set +u
    source "$ROS_SETUP"
    set -u
    cd "$WS_DIR"
    run colcon build --symlink-install
    ;;

  topics)
    source_env
    run ros2 topic list
    ;;

  # 硬件层 dry-run 保持 MAVROS adapter 运行，但避免飞控服务调用，适合无桨、
  # 无电机台架验证。
  hardware-dry)
    source_env
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=true \
      use_serial_manager:=true \
      use_servo:=true \
      use_mavros:=true \
      "${@:2}"
    ;;

  serial-dry)
    source_env
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=true \
      use_serial_manager:=true \
      use_servo:=false \
      use_mavros:=false \
      "${@:2}"
    ;;

  servo-dry)
    source_env
    run_launch robot_bring_up servo.launch.py dry_run:=true "${@:2}"
    ;;

  adapter-dry|mavros-adapter-dry)
    source_env
    run_launch flight_control mavros_adapter.launch.py "${@:2}"
    ;;

  # 真实硬件入口拆开，先确认飞控心跳，再启用 adapter 或舵机命令链路。
  mavros-state)
    source_env
    print_devices
    mavros_args=(
      "${WS_DIR}/flight_control/launch/mavros_state.launch.py"
      "fcu_url:=${FCU_URL:-/dev/px4_fcu:230400}"
      "tgt_system:=${TARGET_SYSTEM:-1}"
      "tgt_component:=${TARGET_COMPONENT:-1}"
      "fcu_protocol:=${FCU_PROTOCOL:-v2.0}"
    )
    run_launch "${mavros_args[@]}" "${@:2}"
    ;;

  hardware-real)
    source_env
    print_devices
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=false \
      use_serial_manager:=true \
      use_servo:=true \
      use_mavros:=true \
      fcu_url:="${FCU_URL:-/dev/px4_fcu:230400}" \
      "${@:2}"
    ;;

  mavros-real)
    source_env
    print_devices
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=false \
      use_serial_manager:=true \
      use_servo:=false \
      use_mavros:=true \
      fcu_url:="${FCU_URL:-/dev/px4_fcu:230400}" \
      "${@:2}"
    ;;

  servo-real)
    source_env
    print_devices
    run_launch robot_bring_up servo.launch.py dry_run:=false "${@:2}"
    ;;

  # 传感器、里程计、感知、决策和整机切片入口。
  livox)
    source_env
    run_launch livox_ros_driver2 msg_MID360s_launch.py "${@:2}"
    ;;

  pointlio)
    source_env
    run_launch point_lio pointlio.launch.py config_path:="$DRONE_PARAMS" rviz:=false "${@:2}"
    ;;

  obstacle)
    source_env
    run_launch obstacle_segmentation obstacle_segmentation.launch.py params_file:="$DRONE_PARAMS" "${@:2}"
    ;;

  behavior)
    source_env
    run_launch behavior_control behavior_control.launch.py params_file:="$DRONE_PARAMS" "${@:2}"
    ;;

  nav)
    source_env
    run_launch robot_bring_up bringup_launch.py \
      params_file:="$DRONE_PARAMS" \
      use_sim_time:=false \
      use_respawn:=false \
      "${@:2}"
    ;;

  full)
    source_env
    run_launch robot_bring_up drone.launch.py \
      params_file:="$DRONE_PARAMS" \
      launch_rviz:=false \
      if_map:=false \
      "${@:2}"
    ;;

  *)
    usage
    die "unknown mode: $MODE"
    ;;
esac
