#!/bin/bash
# Watchdog wrapper: keep one auto_aim_node alive and restart it if it
# ever exits. The node itself handles image-stream stalls by rebuilding
# its subscription; the wrapper is the safety net for real crashes.
# Stop with Ctrl-C.
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
while true; do
  ros2 run auto_aim auto_aim_node --ros-args --params-file config/params.yaml
  echo "[run_node] node exited; restarting in 1s"
  sleep 1
done
