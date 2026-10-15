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
  electromagnet-dry dry-run 启动电磁铁节点；不会打开串口或驱动电磁铁。
  adapter-dry       只用 dry-run 配置启动 MAVROS adapter。

真实硬件层:
  mavros            mavros-real 的短别名。
  mavros-state      只启动 MAVROS 连接飞控，不启动 adapter，不下发命令。
  hardware-real     启动串口管理、舵机节点、MAVROS 和 MAVROS adapter。
  mavros-real       启动串口管理、MAVROS 和 MAVROS adapter，不启动舵机。
  mavros-vision     兼容旧工作区命名；等同 mavros-real，走 MAVROS vision_pose。
  speed-check       只读比较 TEB 期望速度、适配器 setpoint 与 Point-LIO 实测速度。
  servo-real        只启动真实舵机节点，默认设备 /dev/stm32_servo。
  magnet            electromagnet-real 的短别名；默认设备 /dev/ttyUSB0。

传感器、里程计、感知和决策:
  livox             启动 Livox MID-360s 驱动。
  pointlio          只启动 Point-LIO。
  map-record        独立启动 Livox + Point-LIO 并保存 PCD（不启动导航/飞控）。
  obstacle          只启动点云障碍物分割。
  vision            启动 YOLO 靶标检测；按配置显示预览。full2 会显式关闭预览 UI。
  behavior          只启动 behavior_control。
  decision          coverage-mission 的短别名：启动独立蛇形搜索任务。
  coverage-mission  启动独立蛇形搜索任务（不修改/启动 behavior_control）。
  pcd-to-map        将一个 Point-LIO PCD 转成 PNG + Nav2 地图 YAML。
  nav               启动整套导航 drone.launch.py（Livox + Point-LIO + Nav2 + RViz）。
  nav2              只启动 Nav2 bringup。
  nav-mavros         同时启动 nav 和 mavros，不启动舵机。
  full              快速并发启动 nav、mavros、vision、decision、magnet 五个窗口。
  full2             分阶段启动：magnet/decision → nav/Point-LIO → MAVROS → vision（无预览 UI）。
  launch-four       full 的兼容别名。

常用环境变量覆盖:
  ROS_DOMAIN_ID=0
  ROS_LOCALHOST_ONLY=0
  FCU_URL=/dev/ttyACM0:230400
  GCS_URL=udp-b://@:14550
  ELECTROMAGNET_PORT=/dev/ttyUSB0
  MAVROS_START_DELAY=12
  POINTLIO_WAIT_TIMEOUT=100
  MAVROS_WAIT_TIMEOUT=100
  FULL2_STAGE_DELAY=3
  DRONE_PARAMS=/path/to/drone.yaml
  ECHO_DRONE_WS=/home/sfx/echo_drone

示例:
  ./run_echo_drone.sh check
  ./run_echo_drone.sh mavros
  ./run_echo_drone.sh vision
  ./run_echo_drone.sh decision
  ./run_echo_drone.sh magnet
  ./run_echo_drone.sh full
  ./run_echo_drone.sh full2
  ./run_echo_drone.sh nav
  ./run_echo_drone.sh speed-check
  ./run_echo_drone.sh hardware-dry
  ./run_echo_drone.sh livox
  ./run_echo_drone.sh pointlio rviz:=false
  ./tools/record_mapping_pcd.sh 900
  ./tools/pcd_to_nav_map.sh ~/maps/field.pcd ~/maps/generated field_map
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

wait_for_pointlio_pose() {
  local timeout_sec="${POINTLIO_WAIT_TIMEOUT:-100}"
  local attempt wait_status
  local cmdline_path pid args
  local -a pointlio_pids=()

  for attempt in 1 2; do
    echo "Point-LIO 监控：等待 /robot/current_pose（第 ${attempt}/2 轮，超时 ${timeout_sec}s）。"
    if python3 "$WS_DIR/tools/wait_for_ros_ready.py" pointlio "$timeout_sec"; then
      echo "已收到 Point-LIO 位姿 /robot/current_pose。"
      return 0
    else
      wait_status=$?
      if (( wait_status == 130 )); then
        return 130
      fi
    fi

    if (( attempt == 2 )); then
      die "Point-LIO 重启一次后仍未在 ${timeout_sec}s 内发布 /robot/current_pose；不再重启，也不启动 MAVROS。"
    fi

    pointlio_pids=()
    for cmdline_path in /proc/[0-9]*/cmdline; do
      pid="${cmdline_path#/proc/}"
      pid="${pid%/cmdline}"
      args="$(tr '\0' ' ' < "$cmdline_path" 2>/dev/null || true)"
      if [[ "$args" == *"${WS_DIR}/install/point_lio/lib/point_lio/pointlio_mapping"* ]]; then
        pointlio_pids+=("$pid")
      fi
    done
    if (( ${#pointlio_pids[@]} != 1 )); then
      die "位姿超时，但在当前工作区定位到 ${#pointlio_pids[@]} 个 Point-LIO 进程；为避免误杀/重复启动，停止自动重启。"
    fi

    pid="${pointlio_pids[0]}"
    echo "${timeout_sec}s 无位姿，向 Point-LIO PID ${pid} 发送 SIGINT；只尝试这一次重启。"
    kill -INT "$pid" || die "无法向 Point-LIO PID ${pid} 发送 SIGINT。"
    for _ in {1..100}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
      die "Point-LIO PID ${pid} 未能正常退出；不强制杀进程，也不启动 MAVROS。"
    fi
    echo "已请求 ROS launch 重新拉起 Point-LIO；继续等待其位姿，最多 ${timeout_sec}s。"
  done
}

wait_for_mavros_connection() {
  local timeout_sec="${MAVROS_WAIT_TIMEOUT:-100}"
  echo "full2：等待 MAVROS 与 FCU 建立连接（最多 ${timeout_sec}s），连接后才启动视觉。"
  if python3 "$WS_DIR/tools/wait_for_ros_ready.py" mavros "$timeout_sec"; then
    echo "full2：FCU 已连接。"
    return 0
  else
    local wait_status=$?
    if (( wait_status == 130 )); then
      return 130
    fi
  fi
  die "full2：等待 FCU 连接超时；为避免增加负载，不启动视觉。"
}

open_terminal() {
  local title="$1" mode="$2"
  shift 2
  gnome-terminal --title="$title" -- bash -lc \
    'cd "$1" && shift && exec "$@"' _ "$WS_DIR" \
    "$WS_DIR/run_echo_drone.sh" "$mode" "$@"
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
      use_legacy_mavlink:=false \
      "${@:2}"
    ;;

  serial-dry)
    source_env
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=true \
      use_serial_manager:=true \
      use_servo:=false \
      use_mavros:=false \
      use_legacy_mavlink:=false \
      "${@:2}"
    ;;

  servo-dry)
    source_env
    run_launch robot_bring_up servo.launch.py dry_run:=true "${@:2}"
    ;;

  electromagnet-dry)
    source_env
    run_launch electromagnet_node electromagnet.launch.py dry_run:=true "${@:2}"
    ;;

  electromagnet-real|magnet)
    source_env
    print_devices
    run_launch electromagnet_node electromagnet.launch.py \
      dry_run:=false port:="${ELECTROMAGNET_PORT:-/dev/ttyUSB0}" "${@:2}"
    ;;

  full|launch-four|mission-terminals)
    command -v gnome-terminal >/dev/null 2>&1 || die "找不到 gnome-terminal；请在 GNOME 桌面会话中运行。"
    echo "快速并发启动五个终端：nav、mavros、vision、decision、magnet。"
    open_terminal "Echo Drone - Navigation" nav
    open_terminal "Echo Drone - MAVROS" mavros
    open_terminal "Echo Drone - Vision" vision
    open_terminal "Echo Drone - Decision" decision
    open_terminal "Echo Drone - Magnet" magnet
    ;;

  full2)
    command -v gnome-terminal >/dev/null 2>&1 || die "找不到 gnome-terminal；请在 GNOME 桌面会话中运行。"
    source_env
    echo "full2 分阶段启动；每个模块单独占用一个可见 GNOME 终端。"
    echo "阶段 1/4：启动低负载的电磁铁与决策节点。"
    open_terminal "Echo Drone - Magnet" magnet
    sleep "${FULL2_STAGE_DELAY:-3}"
    open_terminal "Echo Drone - Decision" decision
    sleep "${FULL2_STAGE_DELAY:-3}"

    echo "阶段 2/4：启动 nav（Livox + Point-LIO + Nav2），等待 /robot/current_pose。"
    open_terminal "Echo Drone - Navigation" nav
    wait_for_pointlio_pose

    echo "阶段 3/4：Point-LIO 位姿已就绪，启动 MAVROS。"
    open_terminal "Echo Drone - MAVROS" mavros
    wait_for_mavros_connection

    echo "阶段 4/4：FCU 已连接，最后启动视觉；show_window:=false，不创建 YOLO 预览窗口。"
    open_terminal "Echo Drone - Vision" vision show_window:=false
    ;;

  adapter-dry|mavros-adapter-dry)
    source_env
    run_launch flight_control mavros_adapter.launch.py "${@:2}"
    ;;

  speed-check)
    source_env
    run_launch flight_control velocity_monitor.launch.py "${@:2}"
    ;;

  # 真实硬件入口拆开，先确认飞控心跳，再启用 adapter 或舵机命令链路。
  mavros-state)
    source_env
    print_devices
    mavros_args=(
      "${WS_DIR}/flight_control/launch/mavros_state.launch.py"
      "fcu_url:=${FCU_URL:-/dev/ttyACM0:230400}"
      "gcs_url:=${GCS_URL-udp-b://@:14550}"
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
      use_legacy_mavlink:=false \
      fcu_url:="${FCU_URL:-/dev/ttyACM0:230400}" \
      gcs_url:="${GCS_URL-udp-b://@:14550}" \
      "${@:2}"
    ;;

  mavros-real|mavros-vision|mavros)
    source_env
    print_devices
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=false \
      use_serial_manager:=true \
      use_servo:=false \
      use_mavros:=true \
      use_legacy_mavlink:=false \
      fcu_url:="${FCU_URL:-/dev/ttyACM0:230400}" \
      gcs_url:="${GCS_URL-udp-b://@:14550}" \
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

  map-record)
    exec "${WS_DIR}/tools/record_mapping_pcd.sh" "${@:2}"
    ;;

  obstacle)
    source_env
    run_launch obstacle_segmentation obstacle_segmentation.launch.py params_file:="$DRONE_PARAMS" "${@:2}"
    ;;

  target-perception|vision)
    source_env
    run_launch target_perception target_localization.launch.py "${@:2}"
    ;;

  coverage-mission|decision)
    source_env
    run_launch coverage_mission coverage_mission.launch.py "${@:2}"
    ;;

  pcd-to-map)
    exec "${WS_DIR}/tools/pcd_to_nav_map.sh" "${@:2}"
    ;;

  behavior)
    source_env
    run_launch behavior_control behavior_control.launch.py params_file:="$DRONE_PARAMS" "${@:2}"
    ;;

  nav2)
    source_env
    run_launch robot_bring_up bringup_launch.py \
      params_file:="$DRONE_PARAMS" \
      use_sim_time:=false \
      use_respawn:=false \
      "${@:2}"
    ;;

  nav)
    source_env
    run_launch robot_bring_up drone.launch.py \
      params_file:="$DRONE_PARAMS" \
      launch_rviz:=true \
      if_map:=false \
      "${@:2}"
    ;;

  nav-mavros|nav-real|full-mavros|full-real)
    source_env
    print_devices

    full_args=(
      robot_bring_up drone.launch.py
      "params_file:=$DRONE_PARAMS"
      "launch_rviz:=true"
      "if_map:=false"
    )
    mavros_args=(
      robot_bring_up hardware.launch.py
      "dry_run:=false"
      "use_serial_manager:=true"
      "use_servo:=false"
      "use_mavros:=true"
      "use_legacy_mavlink:=false"
      "fcu_url:=${FCU_URL:-/dev/ttyACM0:230400}"
      "gcs_url:=${GCS_URL-udp-b://@:14550}"
    )

    cleanup_combined() {
      local status=$?
      trap - INT TERM EXIT
      if [[ -n "${mavros_pid:-}" ]] && kill -0 "$mavros_pid" 2>/dev/null; then
        kill -INT "$mavros_pid" 2>/dev/null || true
      fi
      if [[ -n "${full_pid:-}" ]] && kill -0 "$full_pid" 2>/dev/null; then
        kill -INT "$full_pid" 2>/dev/null || true
      fi
      wait "${mavros_pid:-}" 2>/dev/null || true
      wait "${full_pid:-}" 2>/dev/null || true
      exit "$status"
    }
    trap cleanup_combined INT TERM EXIT

    echo "+ ros2 launch ${full_args[*]}"
    ros2 launch "${full_args[@]}" "${@:2}" &
    full_pid=$!

    echo "等待 ${MAVROS_START_DELAY:-12}s 后启动 MAVROS..."
    sleep "${MAVROS_START_DELAY:-12}"

    echo "+ ros2 launch ${mavros_args[*]}"
    ros2 launch "${mavros_args[@]}" &
    mavros_pid=$!

    wait -n "$full_pid" "$mavros_pid"
    cleanup_combined
    ;;

  *)
    usage
    die "unknown mode: $MODE"
    ;;
esac
