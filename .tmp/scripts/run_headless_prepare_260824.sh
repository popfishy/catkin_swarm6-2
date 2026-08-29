#!/bin/bash
# 无头 15 机 prepare 端到端测试编排（2026-08-24，PVA PositionTarget 链路验证）
# 用法：bash .tmp/scripts/run_headless_prepare_260824.sh   （建议 nohup 后台运行）
set -u
WS=/home/ub20tg/catkin_swarm6-2
TMP="$WS/.tmp"
PX4_ROOT=/home/ub20tg/PX4_Firmware
PX4_BUILD="$PX4_ROOT/build/px4_sitl_default"
LOG="$TMP/logs/headless_prepare_260824.log"
: > "$LOG"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

export ROS_HOME="$WS/.ros_home"
export ROS_LOG_DIR="$WS/.ros_home/log"
export ROS_HOSTNAME=localhost
mkdir -p "$ROS_LOG_DIR" "$TMP/logs"

# 0) 清空 .ros_home（保留文件夹）
log "== 清空 .ros_home =="
rm -rf "$ROS_HOME"/log "$ROS_HOME"/sitl_iris_* "$ROS_HOME"/rospack_cache_* 2>/dev/null
mkdir -p "$ROS_LOG_DIR"

# 1) 仿真层（master 11300）
log "== 启动仿真层（11300, gui:=false）=="
(
  source /opt/ros/noetic/setup.bash
  source "$WS/devel/setup.bash"
  source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_BUILD"
  export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"
  export ROS_MASTER_URI=http://localhost:11300
  export GAZEBO_MASTER_URI=http://localhost:11345
  export ROS_HOME="$WS/.ros_home"
  export ROS_LOG_DIR="$WS/.ros_home/log"
  roslaunch safe_valley_exp multi_uav_ego_15sim.launch gui:=false
) > "$TMP/logs/sim_layer.log" 2>&1 &

log "等待仿真层 master 11300 ..."
timeout 180 bash -c "until ROS_MASTER_URI=http://localhost:11300 rosnode list >/dev/null 2>&1; do sleep 2; done" || { log "ERR: sim master 未就绪"; exit 1; }
log "仿真层 master 就绪"

# 2) 机载层 15 机
log "== 启动机载层 15 机（startup_offboard_ego.sh）=="
bash "$WS/src/safe_valley_exp/startup_offboard_ego.sh" 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 \
  >> "$TMP/logs/startup_onboard.log" 2>&1 &
ONBOARD_PID=$!

log "等待 15 个机载 master + MAVROS state ..."
ready=0
for i in $(seq 1 300); do
  n=0
  for idx in $(seq 1 15); do
    if timeout 3 bash -c "ROS_MASTER_URI=http://localhost:$((11310+idx)) rostopic echo -n 1 /mavros/state >/dev/null 2>&1"; then
      n=$((n+1))
    fi
  done
  if [ "$n" -ge 15 ]; then ready=1; break; fi
  sleep 5
done
[ "$ready" = 1 ] || { log "ERR: 机载层 15 机未全部就绪 (n=$n)"; tail -30 "$TMP/logs/startup_onboard.log" >> "$LOG"; exit 1; }
log "机载层 15/15 MAVROS 就绪"

# 3) GCS_A bridge + backend + tcp
log "== 启动 GCS_A bridge/backend/tcp =="
(
  source /opt/ros/noetic/setup.bash
  source "$WS/devel/setup.bash"
  export ROS_MASTER_URI=http://localhost:11310
  export ROS_HOME="$WS/.ros_home"
  export ROS_LOG_DIR="$WS/.ros_home/log"
  roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false
) > "$TMP/logs/gcs_bridge.log" 2>&1 &
sleep 8
(
  source /opt/ros/noetic/setup.bash
  source "$WS/devel/setup.bash"
  export ROS_MASTER_URI=http://localhost:11310
  export ROS_HOME="$WS/.ros_home"
  export ROS_LOG_DIR="$WS/.ros_home/log"
  roslaunch "$WS/src/tcp_to_ros/launch/gcs_a_backend.launch"
) > "$TMP/logs/gcs_backend.log" 2>&1 &
sleep 8
(
  source /opt/ros/noetic/setup.bash
  source "$WS/devel/setup.bash"
  export ROS_MASTER_URI=http://localhost:11310
  export ROS_HOME="$WS/.ros_home"
  export ROS_LOG_DIR="$WS/.ros_home/log"
  rosrun tcp_to_ros tcp_server_node.py _host:=0.0.0.0 _port:=39001 \
    _action_name:=/group_a/execute_task _state_topic:=/group_a/group_task_state \
    _emergency_hold_service:=/group_a/emergency_hold
) > "$TMP/logs/tcp_server.log" 2>&1 &

log "等待标定 ready（calibration_ready=true，15/15 origin）..."
cal_ok=0
for i in $(seq 1 120); do
  v=$(timeout 3 bash -c "ROS_MASTER_URI=http://localhost:11310 rostopic echo -n 1 /group_a/calibration_ready 2>/dev/null" | grep -io 'data: *true' | head -1)
  if [ -n "$v" ]; then cal_ok=1; break; fi
  sleep 5
done
[ "$cal_ok" = 1 ] || { log "ERR: 标定未 ready"; grep -E 'ready|origin|frozen' "$TMP/logs/gcs_backend.log" | tail -20 >> "$LOG"; exit 1; }
log "标定 ready"

# 4) 起飞 15 机
log "== 起飞 15 机（arm + OFFBOARD，软起飞 5m）=="
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
python3 "$WS/src/safe_valley_exp/scripts/offboard_takeoff_15.py" 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 2>&1 | tee -a "$LOG"
log "== 起飞完成，等待 30s 稳定 =="
sleep 30

# 5) prepare 批次
log "== 发送 prepare（A01-A12 MOVE_TO，timeout 180s）=="
python3 "$WS/references/runtime/task_test.py" --backend tcp --group A --task prepare \
  --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s 180 2>&1 | tee -a "$LOG"
PREPARE_RC=${PIPESTATUS[0]}
log "prepare 退出码=$PREPARE_RC"

# 6) 停机（逆序）
log "== 清理测试进程 =="
kill "$(cat "$WS/.tmp/logs/tcp_server.pid" 2>/dev/null)" 2>/dev/null
pkill -f 'tcp_server_node.py' 2>/dev/null
pkill -f 'gcs_a_backend.launch' 2>/dev/null
pkill -f 'topology_group_a_sim.launch' 2>/dev/null
bash "$WS/src/safe_valley_exp/startup_offboard_ego.sh" stop 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 2>/dev/null
sleep 5
pkill -f 'multi_uav_ego_15sim' 2>/dev/null
pkill -f 'gzserver' 2>/dev/null
pkill -f 'px4' 2>/dev/null
pkill -f 'rosmaster' 2>/dev/null
sleep 5
log "== DONE prepare_rc=$PREPARE_RC =="
exit "$PREPARE_RC"
