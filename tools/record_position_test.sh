#!/usr/bin/env bash
# Foxy setup scripts reference optional unset variables (e.g.
# AMENT_TRACE_SETUP_FILES), so nounset cannot be enabled until after sourcing.
set -Ee

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Record a lightweight ROS 2 bag for a position/vision flight test.

Usage:
  ./tools/record_position_test.sh [--output DIR] [--with-imu] [--with-lidar]

By default the bag excludes high-rate IMU and the large Livox point cloud.
--with-lidar is optional and can create a large bag / add CPU and disk load.
Press Ctrl-C to stop recording cleanly.
EOF
}

OUTPUT_DIR=""
WITH_IMU=false
WITH_LIDAR=false
while (($#)); do
  case "$1" in
    --output)
      (($# >= 2)) || { echo "--output needs a directory" >&2; exit 2; }
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --with-imu) WITH_IMU=true; shift ;;
    --with-lidar) WITH_LIDAR=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${WS_ROOT}/rosbags/position_test_$(date +%Y%m%d_%H%M%S)"
elif [[ "$OUTPUT_DIR" != /* ]]; then
  OUTPUT_DIR="${WS_ROOT}/${OUTPUT_DIR}"
fi

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ROS 2 Foxy setup not found at /opt/ros/foxy/setup.bash" >&2
  exit 1
fi
if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
  echo "Workspace is not built: ${WS_ROOT}/install/setup.bash is missing" >&2
  exit 1
fi

source /opt/ros/foxy/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u
mkdir -p "$(dirname -- "$OUTPUT_DIR")"

TOPICS=(
  /robot/current_pose
  /mavros/mavros/pose
  /mavros/odometry/out
  /mavros/state
  /mavros/mavros/local
  /mavros/mavros/cmd_vel
  /mavros/setpoint_raw/local
  /cmd_vel
  /robot/target_pose
  /robot/nav_state
  /flight_control/status
  /tf
  /tf_static
)

if [[ "$WITH_IMU" == true ]]; then
  TOPICS+=(/livox/imu /mavros/imu/data)
fi
if [[ "$WITH_LIDAR" == true ]]; then
  TOPICS+=(/livox/lidar)
fi

echo "Recording position-test ROS bag to: ${OUTPUT_DIR}"
echo "Included topics: ${TOPICS[*]}"
if [[ "$WITH_LIDAR" == true ]]; then
  echo "WARNING: Livox point cloud recording adds substantial disk/network load."
fi
echo "Press Ctrl-C to stop and finalize the bag."
exec ros2 bag record --output "$OUTPUT_DIR" "${TOPICS[@]}"
