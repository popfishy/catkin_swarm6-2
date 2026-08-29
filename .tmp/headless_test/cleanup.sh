#!/bin/bash
# 无头测试清理：逆序停止所有测试进程（精确 PID + 进程组，不用宽泛 killall）
set -u
cd "$(dirname "$0")"
source ./env.sh

echo "===== cleanup ====="
kill_group() {
  local f=$1
  [ -f "$f" ] || return 0
  local pid
  pid=$(cat "$f")
  kill -- -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  rm -f "$f"
  echo "stopped $f (pid=$pid)"
}

# 1. TCP server
kill_group "$HLOG/05_tcp.pid"
# 2. GCS backend + bridge
kill_group "$HLOG/04_gcs_backend.pid"
kill_group "$HLOG/03_gcs_bridge.pid"
# 3. 正在跑的 task / takeoff
kill_group "$HLOG"/07_task_*.pid 2>/dev/null || true
kill_group "$HLOG/06_takeoff.pid"
sleep 3

# 4. 机载层：优先用官方 stop 脚本（按 pid 文件 + 受限 grep），再杀 runner
cd "$WS"
if [ -d "$WS/src/safe_valley_exp" ]; then
  bash src/safe_valley_exp/startup_offboard_ego.sh stop \
    1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 >/dev/null 2>&1 || true
fi
kill_group "$HLOG/02_onboard.pid"
sleep 3

# 5. 仿真层
kill_group "$HLOG/01_sim.pid"
sleep 8

# 6. 兜底：确保本测试残留的 rosmaster/gzserver/px4 退出（仅按本测试典型进程名）
ps -eo pid,comm | awk '$2 ~ /^rosmaster$|^gazebo$|^px4$|^roslaunch$|^gzserver$|^gzclient$/ {print $1}' \
  | xargs -r kill -TERM 2>/dev/null || true
# 6b. 残留 swarm_bridge（bridge_node.py）也必须清：旧 bridge 残留会占用 UAV 的
# ZMQ topic/service 端口（4300+/4400+），导致新 bridge bind 失败退出（2026-08-26
# SITL 标定卡死根因）。comm 为 python3，需按 cmdline 匹配。
ps -eo pid,args | grep '[b]ridge_node.py' | awk '{print $1}' \
  | xargs -r kill -TERM 2>/dev/null || true
sleep 3

# 7. 拷贝机载层日志回工作空间并清理 /tmp（脚本硬编码 /tmp，测试后归位）
cp -f /tmp/UAV*_offboard_ego.log "$HLOG/" 2>/dev/null || true
rm -f /tmp/UAV*_offboard_ego.pid /tmp/UAV*_offboard_ego.log

echo "===== 残留检查 ====="
ss -ltnp 2>/dev/null | grep -E '11300|11310|1131[1-9]|1132[0-5]|39001|11345' || echo "ports: clean"
ps -ef | grep -E '[g]azebo|[p]x4 |[m]avros|[r]osmaster|[e]go_planner|[u]av_executor|[t]cp_server_node|[b]ridge_node' \
  || echo "processes: clean"
echo "===== cleanup done ====="
