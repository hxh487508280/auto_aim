#!/bin/bash
# 一键跑分：按正确的启动顺序管理节点与游戏。
#
# DDS 特性限制：本模拟器的图像发布端只与「先于它启动」的订阅者匹配，
# 节点若在游戏之后启动将永远收不到 /image_raw。因此本脚本严格按
# 「先节点、后游戏」的顺序拉起两者；游戏窗口被关闭时，自动以全新
# 状态重启两者（每个对局都是干净的节点进程）。
#
# 用法：./scripts/run_all.sh        （游戏窗口关闭后自动进入下一局）
# 停止：Ctrl-C（同时退出节点与游戏）

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
source /opt/ros/humble/setup.bash
if [ -f "$REPO_DIR/install/setup.bash" ]; then
  source "$REPO_DIR/install/setup.bash"
fi
export DISPLAY="${DISPLAY:-:0}"
# 游戏目录可被环境变量覆盖（默认 ~/homework2026，即附件解压位置）
GAME_DIR="${GAME_DIR:-$HOME/homework2026}"
PARAMS="$REPO_DIR/config/params.yaml"

# pgrep -x 匹配被截断的进程名（comm 上限 15 字符）
game_running() { pgrep -f homework2026.x86_64 > /dev/null; }

echo "[run_all] repo=$REPO_DIR game=$GAME_DIR"
while true; do
  # 1) 节点先行（DDS 匹配顺序要求）
  ros2 run auto_aim auto_aim_node --ros-args --params-file "$PARAMS" &
  NODE_PID=$!
  sleep 2

  # 2) 游戏后行；已在运行则等待其退出（领养）。
  # 等待期间同时监控节点存活：节点若因 DDS 失配自杀退出（零图像/
  # 对局中失联看门狗），立即拉起新节点——新参与者才能重新匹配上
  # 游戏的发布端，绝不让对局在无视觉状态下空转。
  if game_running; then
    echo "[run_all] game already running; waiting for it to exit"
    while game_running; do
      if ! kill -0 "$NODE_PID" 2>/dev/null; then
        echo "[run_all] node died; restarting it"
        pkill -9 -x auto_aim_node 2>/dev/null
        ros2 run auto_aim auto_aim_node --ros-args --params-file "$PARAMS" &
        NODE_PID=$!
      fi
      sleep 2
    done
  else
    echo "[run_all] launching game"
    GAME_LOG=$(mktemp /tmp/homework_game_XXXX.log)
    ( cd "$GAME_DIR" && exec ./homework2026.sh \
        --rendering-method gl_compatibility --rendering-driver opengl3 \
        --position 60,80 ) > "$GAME_LOG" 2>&1 &
    GAME_PID=$!
    # The game's ROS bridge (rclgd) races at extension load: sometimes the
    # ImagePublisher class fails to register and the round runs without an
    # image stream. Detect that within the first seconds and relaunch.
    for _ in 1 2 3 4 5 6; do
      sleep 2
      if ! kill -0 "$GAME_PID" 2>/dev/null; then
        break
      fi
      if grep -q "Cannot get class 'ImagePublisher'" "$GAME_LOG" 2>/dev/null; then
        echo "[run_all] game ROS bridge failed to load; relaunching game"
        kill -9 "$GAME_PID" 2>/dev/null
        sleep 1
        ( cd "$GAME_DIR" && exec ./homework2026.sh \
            --rendering-method gl_compatibility --rendering-driver opengl3 \
            --position 60,80 ) > "$GAME_LOG" 2>&1 &
        GAME_PID=$!
      fi
    done
    while kill -0 "$GAME_PID" 2>/dev/null; do
      if ! kill -0 "$NODE_PID" 2>/dev/null; then
        echo "[run_all] node died; restarting it"
        pkill -9 -x auto_aim_node 2>/dev/null
        ros2 run auto_aim auto_aim_node --ros-args --params-file "$PARAMS" &
        NODE_PID=$!
      fi
      sleep 2
    done
  fi
  echo "[run_all] game exited; restarting node+game in 3s"

  kill "$NODE_PID" 2>/dev/null
  wait "$NODE_PID" 2>/dev/null
  sleep 3
done
