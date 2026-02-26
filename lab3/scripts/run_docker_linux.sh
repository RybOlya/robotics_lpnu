#!/usr/bin/env bash
set -euo pipefail

IMAGE="ros2_mobile_robot_lab"

# Allow docker containers to use X server (Linux X11)
xhost +local:docker >/dev/null 2>&1 || true

docker run -it --rm \
  --name ros2_mobile_robot_lab_container \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device=/dev/dri:/dev/dri \
  -v "$(pwd)/ros2_ws:/ros2_ws" \
  "${IMAGE}"
