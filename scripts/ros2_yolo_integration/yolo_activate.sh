#!/bin/bash

# --- Configuration ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Optional port mapping: --port host_port container_port
PORT_MAPPING=""
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
  PORT_MAPPING="-p $2:$3"
  shift 3
fi

# Detect architecture
ARCH="$(uname -m)"
if [ "$ARCH" != "aarch64" ]; then
  echo "This script only runs on Jetson (aarch64). Detected: $ARCH"
  exit 1
fi
echo "Detected architecture: arm64 (Jetson)"

# Docker network & env
NETWORK_OPTS="--network compose_my_bridge_network"
ENV_FILE_OPTS="--env-file ${SCRIPT_DIR}/.env"

# Volume mounts
SRC_MOUNT="${SCRIPT_DIR}/src:/workspace/src"
SCREENSHOT_MOUNTS=(
  "-v" "${SCRIPT_DIR}/screenshots:/workspace/screenshots"
  "-v" "${SCRIPT_DIR}/fps_screenshots:/workspace/fps_screenshots"
)

# Final CMD to execute inside container
CMD='
source /opt/ros/humble/setup.bash &&
cd /workspace &&
colcon build --symlink-install &&
source /workspace/install/local_setup.bash &&
export PYTHONPATH=/workspace/install/cv_bridge/local/lib/python3.10/dist-packages:$PYTHONPATH &&
ros2 run yolo_pkg yolo_detection_node
'



# Docker run (非互動模式、可 systemd 啟動)
docker run -d --rm \
  --name yolo_node \
  $NETWORK_OPTS \
  $PORT_MAPPING \
  --runtime=nvidia \
  $ENV_FILE_OPTS \
  -v "$SRC_MOUNT" \
  "${SCREENSHOT_MOUNTS[@]}" \
  registry.screamtrumpet.csie.ncku.edu.tw/screamlab/ros2_yolo_opencv_image:latest \
  bash -ic "$CMD"
