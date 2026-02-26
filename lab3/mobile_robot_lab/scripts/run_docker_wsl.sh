#!/usr/bin/env bash
set -euo pipefail

IMAGE="ros2_mobile_robot_lab"

# WSLg typically provides DISPLAY already. If not:
export DISPLAY=${DISPLAY:-:0}

docker run -it --rm \
  --name ros2_mobile_robot_lab_container \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)/ros2_ws:/ros2_ws" \
  "${IMAGE}"
