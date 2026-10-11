#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
MAP_PATHS_FILE="${MAP_PATHS_FILE:-${WS_DIR}/robot_bring_up/config/map_paths.yaml}"

usage() {
  cat <<'EOF'
用法:
  ./tools/pcd_to_nav_map.sh [input.pcd] [output_dir] [map_name]

示例:
  ./tools/pcd_to_nav_map.sh
  ./tools/pcd_to_nav_map.sh ~/maps/field.pcd ~/maps/generated field_map

输出:
  <output_dir>/<map_name>.png
  <output_dir>/<map_name>.yaml

可选环境变量:
  MAP_RESOLUTION=0.05
  MAP_METHOD=point             # point / normal / height
  MAP_X_MIN=-10 MAP_X_MAX=10
  MAP_Y_MIN=-10 MAP_Y_MAX=10
  MAP_Z_MIN=-0.56 MAP_Z_MAX=1.2
  MAP_FILTER_RADIUS=0.05
  MAP_FILTER_MIN_POINTS=30

生成后先在 RViz 检查地图方向、origin 和障碍物，再将 YAML 路径配置给
map_server；不要未经检查就用于实机导航。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -gt 3 ]]; then
  usage
  exit 2
fi

[[ -f "${MAP_PATHS_FILE}" ]] || { echo "地图路径配置不存在: ${MAP_PATHS_FILE}" >&2; exit 2; }
map_path_value() {
  python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["map_paths"].get(sys.argv[2]) or "")' \
    "${MAP_PATHS_FILE}" "$1"
}
CONFIG_PCD="$(map_path_value pcd_file)"
CONFIG_MERGED_DIR="$(map_path_value merged_pcd_dir)"
CONFIG_OUT_DIR="$(map_path_value map_output_dir)"
CONFIG_MAP_NAME="$(map_path_value map_name)"
OUT_DIR="${2:-${CONFIG_OUT_DIR}}"
if [[ -n "${CONFIG_MERGED_DIR}" ]]; then
  MERGED_DIR="$(realpath "${CONFIG_MERGED_DIR}")"
  [[ -d "${MERGED_DIR}" ]] || { echo "合并 PCD 目录不存在: ${MERGED_DIR}" >&2; exit 2; }
  INPUT_PCD="${MERGED_DIR}/merged_for_map_$(date +%Y%m%d_%H%M%S).pcd"
elif [[ -n "${1:-}" ]]; then
  INPUT_PCD="$(realpath "${1}")"
else
  INPUT_PCD="${CONFIG_PCD}"
fi
[[ -n "${INPUT_PCD}" ]] || {
  echo "请在 ${MAP_PATHS_FILE} 填写 map_paths.pcd_file，或配置 merged_pcd_dir，或运行时传入 PCD 文件。" >&2
  exit 2
}
if [[ -z "${CONFIG_MERGED_DIR}" ]]; then
  INPUT_PCD="$(realpath "${INPUT_PCD}")"
  [[ -f "${INPUT_PCD}" ]] || { echo "PCD 文件不存在: ${INPUT_PCD}" >&2; exit 2; }
fi
OUT_DIR="${OUT_DIR:-$(dirname "${INPUT_PCD}")/nav_map}"
mkdir -p "${OUT_DIR}"
OUT_DIR="$(realpath "${OUT_DIR}")"
MAP_NAME="${3:-${CONFIG_MAP_NAME:-$(basename "${INPUT_PCD}" .pcd)}}"
[[ "${MAP_NAME}" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "地图名只能包含字母、数字、点、下划线和连字符: ${MAP_NAME}" >&2
  exit 2
}

[[ -f "${ROS_SETUP}" ]] || { echo "ROS setup 不存在: ${ROS_SETUP}" >&2; exit 2; }
[[ -f "${WS_SETUP}" ]] || {
  echo "工作区尚未编译: ${WS_SETUP}; 先运行 ./run_echo_drone.sh build" >&2
  exit 2
}

# Foxy setup scripts read optional unset variables, so temporarily disable nounset.
set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

if [[ -n "${CONFIG_MERGED_DIR}" ]]; then
  echo "合并目录: ${MERGED_DIR}"
  echo "将按文件名排序合并其中的原始 PCD（忽略已有合并产物）。"
  ros2 run merge_pcd merge_pcd_node --ros-args \
    -p "pcd_folder:=${MERGED_DIR}" \
    -p "output_file:=${INPUT_PCD}"
  [[ -s "${INPUT_PCD}" ]] || {
    echo "PCD 合并失败，未生成: ${INPUT_PCD}" >&2
    exit 1
  }
else
  echo "直接使用 PCD: ${INPUT_PCD}"
fi

echo "输入 PCD: ${INPUT_PCD}"
echo "输出地图: ${OUT_DIR}/${MAP_NAME}.{png,yaml}"

ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p "pcd_path:=$(dirname "${INPUT_PCD}")" \
  -p "pcd_name:=$(basename "${INPUT_PCD}")" \
  -p "map_path:=${OUT_DIR}" \
  -p "map_name:=${MAP_NAME}.png" \
  -p "yaml_name:=${MAP_NAME}.yaml" \
  -p "method:=${MAP_METHOD:-point}" \
  -p "map_resolution:=${MAP_RESOLUTION:-0.05}" \
  -p "thre_x_min:=${MAP_X_MIN:--10.0}" \
  -p "thre_x_max:=${MAP_X_MAX:-10.0}" \
  -p "thre_y_min:=${MAP_Y_MIN:--10.0}" \
  -p "thre_y_max:=${MAP_Y_MAX:-10.0}" \
  -p "thre_z_min:=${MAP_Z_MIN:--0.56}" \
  -p "thre_z_max:=${MAP_Z_MAX:-1.2}" \
  -p "thre_radius:=${MAP_FILTER_RADIUS:-0.05}" \
  -p "thres_point_count:=${MAP_FILTER_MIN_POINTS:-30}"

[[ -s "${OUT_DIR}/${MAP_NAME}.png" && -s "${OUT_DIR}/${MAP_NAME}.yaml" ]] || {
  echo "地图转换没有生成完整的 PNG/YAML 文件。检查节点输出和 PCD 内容。" >&2
  exit 1
}
echo "地图生成完成：${OUT_DIR}/${MAP_NAME}.yaml"
