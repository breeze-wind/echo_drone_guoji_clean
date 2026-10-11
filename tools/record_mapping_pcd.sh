#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
DRONE_PARAMS="${DRONE_PARAMS:-${WS_DIR}/install/robot_bring_up/share/robot_bring_up/config/drone.yaml}"
PCD_SAVE_INTERVAL="${1:-${PCD_SAVE_INTERVAL:-900}}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "用法: ./tools/record_mapping_pcd.sh [正整数interval]"
  echo "启动 Livox + Point-LIO 并强制开启 PCD 分段保存；默认 interval=900。"
  echo "点云保存在 Point-LIO/PCD/；Ctrl-C 正常结束并 flush 最后一段。"
  exit 0
fi
if [[ $# -gt 1 || ! "${PCD_SAVE_INTERVAL}" =~ ^[1-9][0-9]*$ ]]; then
  echo "用法: ./tools/record_mapping_pcd.sh [正整数interval]" >&2
  echo "interval 表示累计多少帧扫描写一个 PCD 分段，默认 900。" >&2
  exit 2
fi
[[ -f "${ROS_SETUP}" ]] || { echo "ROS setup 不存在: ${ROS_SETUP}" >&2; exit 2; }
[[ -f "${WS_SETUP}" ]] || {
  echo "工作区尚未编译: ${WS_SETUP}; 先运行 ./run_echo_drone.sh build" >&2
  exit 2
}
[[ -f "${DRONE_PARAMS}" ]] || { echo "参数文件不存在: ${DRONE_PARAMS}" >&2; exit 2; }

# Point-LIO writes PCD chunks under its compile-time ROOT_DIR/PCD path.
mkdir -p "${WS_DIR}/Point-LIO/PCD"

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

echo "请求 Point-LIO 开启 PCD 保存；分段间隔=${PCD_SAVE_INTERVAL}。启动后应看到 ‘PCD saving: enabled=true’。"
echo "输出目录: ${WS_DIR}/Point-LIO/PCD"
echo "仅启动 Livox + 静态 TF + Point-LIO；采集结束按 Ctrl-C 正常停止以保存最后缓冲段。"

mapfile -t BEFORE_FILES < <(find "${WS_DIR}/Point-LIO/PCD" -maxdepth 1 -type f -name '*.pcd' -printf '%f\n' | sort)
set +e
ros2 launch robot_bring_up mapping_capture.launch.py \
  params_file:="${DRONE_PARAMS}" \
  pcd_save_interval:="${PCD_SAVE_INTERVAL}"
LAUNCH_STATUS=$?
set -e

mapfile -t AFTER_FILES < <(find "${WS_DIR}/Point-LIO/PCD" -maxdepth 1 -type f -name '*.pcd' -printf '%f\n' | sort)
NEW_FILES=()
for file in "${AFTER_FILES[@]}"; do
  if [[ ! " ${BEFORE_FILES[*]} " =~ " ${file} " ]]; then
    NEW_FILES+=("${file}")
  fi
done

if [[ ${#NEW_FILES[@]} -gt 0 ]]; then
  echo "本次新写入的 PCD："
  for file in "${NEW_FILES[@]}"; do
    ls -lh "${WS_DIR}/Point-LIO/PCD/${file}"
  done
else
  echo "警告：本次运行没有发现新 PCD 文件。" >&2
  echo "核对 Point-LIO 启动日志中的 ‘PCD saving: enabled=true’；若是 false，检查参数覆盖或重新编译。" >&2
  echo "若为 true 但无 scan_wait_num，检查 Point-LIO 是否持续处理 /livox/lidar；并确认正常 Ctrl-C 退出。" >&2
fi
exit "${LAUNCH_STATUS}"
