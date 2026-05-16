#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UDP_SCRIPT="$WORKSPACE_DIR/script/udp_client.py"
VIS_SCRIPT="$WORKSPACE_DIR/script/visualize_routing_map_realtime.py"

# 按需修改这几个变量
MAP_PATH="$WORKSPACE_DIR/map/malasong"
UDP_SERVER_HOST="10.11.177.221"
UDP_SERVER_PORT="8080"
# 远程调速步长（m/s）：按 '.'/'>' 每次 +SPEED_DELTA，按 ','/'<' 每次 -SPEED_DELTA
SPEED_DELTA="0.5"
# 导航目标点坐标（map 坐标系）：按 'ctrl+g' 下发 /hric/nav/start_nav
NAV_TARGET_X="453000.47"
NAV_TARGET_Y="4404732.38"

source_workspace_env() {
  local setup_path=""
  local restore_nounset=0

  for setup_path in \
    "$WORKSPACE_DIR/install/setup.bash" \
    "$WORKSPACE_DIR/install/local_setup.bash" \
    "$WORKSPACE_DIR/setup.bash" \
    "$WORKSPACE_DIR/local_setup.bash"; do
    if [ -f "$setup_path" ]; then
      if [[ $- == *u* ]]; then
        restore_nounset=1
        set +u
      fi

      # shellcheck disable=SC1090
      source "$setup_path"

      if [ "$restore_nounset" -eq 1 ]; then
        set -u
      fi

      echo "已加载工作区环境: $setup_path"
      return 0
    fi
  done

  echo "错误: 未找到可 source 的工作区环境脚本，请先运行 ./build_colcon.sh 完成编译。"
  exit 1
}

cleanup() {
  if [ -n "${UDP_PID:-}" ] && kill -0 "$UDP_PID" 2>/dev/null; then
    echo "停止 udp_client.py (PID: $UDP_PID)"
    kill "$UDP_PID" 2>/dev/null || true
    wait "$UDP_PID" 2>/dev/null || true
  fi
}

if [ ! -f "$UDP_SCRIPT" ]; then
  echo "错误: 未找到脚本 $UDP_SCRIPT"
  exit 1
fi

if [ ! -f "$VIS_SCRIPT" ]; then
  echo "错误: 未找到脚本 $VIS_SCRIPT"
  exit 1
fi

if [ ! -e "$MAP_PATH" ]; then
  echo "错误: 未找到地图目录 $MAP_PATH"
  exit 1
fi

trap cleanup EXIT INT TERM

source_workspace_env
echo "启动 UDP 客户端: $UDP_SCRIPT"
echo "UDP 服务地址: $UDP_SERVER_HOST:$UDP_SERVER_PORT"
python3 "$UDP_SCRIPT" \
  --server-host "$UDP_SERVER_HOST" \
  --server-port "$UDP_SERVER_PORT" &
UDP_PID=$!

sleep 1

source_workspace_env
echo "启动实时路由可视化: $VIS_SCRIPT"
python3 "$VIS_SCRIPT" \
  -m "$MAP_PATH" \
  --interactive \
  --enable-search \
  --use-origin-obs \
  --use-converted-obs \
  --planning-path dp_path \
  --qp_path \
  --speed-delta "$SPEED_DELTA" \
  --nav-target-x "$NAV_TARGET_X" \
  --nav-target-y "$NAV_TARGET_Y"
