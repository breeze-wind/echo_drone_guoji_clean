#!/usr/bin/env bash
# Request a PX4 autopilot reboot through MAVROS. This is intentionally
# interactive: only use it on the ground with the vehicle disarmed.
set -Ee

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WS_SETUP="${WS_SETUP:-${WS_ROOT}/install/setup.bash}"
MAVROS_STATE_TOPIC="${MAVROS_STATE_TOPIC:-/mavros/state}"
MAVROS_COMMAND_SERVICE="${MAVROS_COMMAND_SERVICE:-}"

usage() {
  cat <<'EOF'
通过 MAVROS 请求 PX4 飞控重启。

用法:
  ./tools/reboot_px4.sh

仅在飞行器位于地面、已解除锁定且螺旋桨安全时运行。此操作会立即
中断飞控与 MAVROS 的连接；PX4 重启后需等待 MAVROS 自动重连。

可覆盖:
  ROS_SETUP=/path/to/ros/setup.bash
  WS_SETUP=/path/to/workspace/install/setup.bash
  MAVROS_STATE_TOPIC=/mavros/state
  MAVROS_COMMAND_SERVICE=/mavros/mavros/command
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  usage >&2
  exit 2
fi
[[ -f "${ROS_SETUP}" ]] || { echo "ROS setup 不存在: ${ROS_SETUP}" >&2; exit 2; }
[[ -f "${WS_SETUP}" ]] || { echo "工作区 setup 不存在: ${WS_SETUP}" >&2; exit 2; }

# ROS Foxy setup scripts may read unset shell variables.
set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "找不到 ros2；请检查 ROS 环境。" >&2
  exit 1
fi

echo "正在检查 MAVROS 飞控状态：${MAVROS_STATE_TOPIC}"
state_output=""
state_output="$(timeout --signal=INT --kill-after=2s 8s ros2 topic echo "${MAVROS_STATE_TOPIC}" 2>&1)" || true
if ! printf '%s\n' "${state_output}" | grep -Eiq '(^|[[:space:]])connected:[[:space:]]*(true|false)([[:space:]]|$)'; then
  echo "在超时时间内没有收到有效的 MAVROS 状态消息。" >&2
  echo "${state_output}" >&2
  exit 1
fi
printf '%s\n' "${state_output}"
if ! printf '%s\n' "${state_output}" | grep -Eiq '(^|[[:space:]])connected:[[:space:]]*true([[:space:]]|$)'; then
  echo "MAVROS 尚未报告 connected: True，取消重启。" >&2
  exit 1
fi
if ! printf '%s\n' "${state_output}" | grep -Eiq '(^|[[:space:]])armed:[[:space:]]*false([[:space:]]|$)'; then
  echo "无法确认飞控处于未解锁状态（armed: False）；取消重启。" >&2
  exit 1
fi

if [[ -n "${MAVROS_COMMAND_SERVICE}" ]]; then
  command_services=("${MAVROS_COMMAND_SERVICE}")
else
  # This workspace runs MAVROS in namespace /mavros. With the installed
  # Foxy MAVROS 2.4 plugins, private command endpoints resolve under
  # /mavros/mavros, like the configured vision_pose and setpoint endpoints.
  command_services=(/mavros/mavros/command /mavros/cmd/command /mavros/mavros/cmd/command)
fi
command_service=""
for candidate in "${command_services[@]}"; do
  service_type="$(ros2 service type "${candidate}" 2>/dev/null || true)"
  if [[ "${service_type}" == "mavros_msgs/srv/CommandLong" ]]; then
    command_service="${candidate}"
    break
  fi
done
if [[ -z "${command_service}" ]]; then
  echo "MAVROS CommandLong 服务未发现。尝试过: ${command_services[*]}" >&2
  echo "确认 MAVROS 的 command 插件已加载，并用 'ros2 service list | grep command' 查看实际服务名。" >&2
  exit 1
fi
echo "已发现 MAVROS 命令服务: ${command_service}"

echo
echo "警告：PX4 将立即重启，MAVROS 会短暂断开。"
read -r -p '确认飞行器已在地面、解除锁定且螺旋桨安全？输入 REBOOT 继续: ' confirmation
if [[ "${confirmation}" != "REBOOT" ]]; then
  echo "已取消。"
  exit 0
fi

echo "发送 PX4 reboot 命令（MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246）..."
timeout 10s ros2 service call "${command_service}" mavros_msgs/srv/CommandLong \
  "{broadcast: false, command: 246, confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}" || {
  echo "服务调用失败或等待超时。若飞控已开始重启，MAVROS 断开可能导致此处超时；稍后检查连接是否恢复。" >&2
  exit 1
}
echo "重启请求已发送。等待 PX4 启动并由 MAVROS 自动重连；不要立即重复发送。"
