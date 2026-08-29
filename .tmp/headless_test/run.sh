#!/bin/bash
# 无头测试主入口：bash run.sh <phase> [task]
# phase: sim | onboard | gcs | tcp | takeoff | task | status | cleanup | uav_z <idx>
set -u
cd "$(dirname "$0")"
source ./env.sh

case "${1:-}" in
  sim)
    export ROS_MASTER_URI=http://localhost:11300
    export ROS_HOSTNAME=localhost
    export GAZEBO_MASTER_URI=http://localhost:11345
    setsid nohup roslaunch safe_valley_exp multi_uav_ego_15sim.launch gui:=false \
      > "$HLOG/01_sim.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/01_sim.pid"
    echo "SIM launched pid=$! (gui:=false, master 11300, gazebo 11345)"
    ;;
  onboard)
    cd "$WS"
    setsid nohup bash src/safe_valley_exp/startup_offboard_ego.sh \
      1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 \
      > "$HLOG/02_onboard.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/02_onboard.pid"
    echo "ONBOARD launched pid=$! (15 UAV masters 11311..11325)"
    ;;
  gcs_bridge_only)
    export ROS_MASTER_URI=http://localhost:11310
    export ROS_HOSTNAME=localhost
    setsid nohup roslaunch tcp_to_ros topology_group_a_sim.launch \
      uav_name:=GCS_A run_uav_mock:=false \
      > "$HLOG/03_gcs_bridge.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/03_gcs_bridge.pid"
    echo "GCS bridge launched pid=$! (master 11310)"
    ;;
  gcs_backend_only)
    export ROS_MASTER_URI=http://localhost:11310
    export ROS_HOSTNAME=localhost
    setsid nohup roslaunch tcp_to_ros gcs_a_backend.launch \
      > "$HLOG/04_gcs_backend.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/04_gcs_backend.pid"
    echo "GCS backend launched pid=$! (master 11310)"
    ;;
  gcs)
    export ROS_MASTER_URI=http://localhost:11310
    export ROS_HOSTNAME=localhost
    setsid nohup roslaunch tcp_to_ros topology_group_a_sim.launch \
      uav_name:=GCS_A run_uav_mock:=false \
      > "$HLOG/03_gcs_bridge.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/03_gcs_bridge.pid"
    setsid nohup roslaunch tcp_to_ros gcs_a_backend.launch \
      > "$HLOG/04_gcs_backend.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/04_gcs_backend.pid"
    echo "GCS launched (bridge pid=$(cat "$HLOG/03_gcs_bridge.pid") backend pid=$(cat "$HLOG/04_gcs_backend.pid") master 11310)"
    ;;
  tcp)
    export ROS_MASTER_URI=http://localhost:11310
    setsid nohup rosrun tcp_to_ros tcp_server_node.py _host:=0.0.0.0 _port:=39001 \
      _action_name:=/group_a/execute_task _state_topic:=/group_a/group_task_state \
      _emergency_hold_service:=/group_a/emergency_hold \
      > "$HLOG/05_tcp.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/05_tcp.pid"
    echo "TCP launched pid=$! (39001)"
    ;;
  takeoff)
    cd "$WS"
    setsid nohup python3 src/safe_valley_exp/scripts/offboard_takeoff_15.py \
      1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 \
      > "$HLOG/06_takeoff.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/06_takeoff.pid"
    echo "TAKEOFF launched pid=$! (15 UAVs)"
    ;;
  task)
    TASK="${2:-prepare}"
    TIMEOUT="${3:-150}"
    cd "$WS"
    setsid nohup python3 references/runtime/task_test.py \
      --backend tcp --group A --task "$TASK" \
      --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s "$TIMEOUT" \
      > "$HLOG/07_task_${TASK}.log" 2>&1 < /dev/null &
    echo $! > "$HLOG/07_task_${TASK}.pid"
    echo "TASK $TASK launched pid=$! (timeout ${TIMEOUT}s)"
    ;;
  uav_z)
    IDX="${2:-1}"
    export ROS_MASTER_URI="http://localhost:$((11310 + IDX))"
    rostopic echo -n 1 /mavros/local_position/pose 2>/dev/null | \
      awk '/^z:/{print; exit}'
    ;;
  status)
    bash ./status.sh
    ;;
  cleanup)
    bash ./cleanup.sh
    ;;
  *)
    echo "usage: run.sh sim|onboard|gcs|tcp|takeoff|task [name]|status|cleanup|uav_z <idx>"
    ;;
esac
