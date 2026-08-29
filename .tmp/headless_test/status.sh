#!/bin/bash
# 无头测试状态汇总
set -u
cd "$(dirname "$0")"
source ./env.sh

echo "===== headless test status ====="
echo "HLOG=$HLOG"

# 1. sim layer (master 11300)
export ROS_MASTER_URI=http://localhost:11300
SIM_SITL=$(rosnode list 2>/dev/null | grep -c 'sitl_' || true)
SIM_GZ=$(rosnode list 2>/dev/null | grep -c '^/gazebo$' || true)
echo "[sim]    sitl_nodes=${SIM_SITL:-0}/15 gazebo=${SIM_GZ:-0}"

# 2. onboard layer (masters 11311..11325)
NPID=$(ls /tmp/UAV*_offboard_ego.pid 2>/dev/null | wc -l)
echo "[onboard] pid_files=${NPID:-0}/15"
ON_READY=$(grep -c '^ready UAV' "$HLOG/02_onboard.log" 2>/dev/null || echo 0)
ON_ERR=$(grep -c 'ERROR' "$HLOG/02_onboard.log" 2>/dev/null || echo 0)
echo "[onboard] ready_lines=${ON_READY:-0} errors=${ON_ERR:-0}"

# 3. GCS layer (master 11310)
export ROS_MASTER_URI=http://localhost:11310
GCS_NODES=$(rosnode list 2>/dev/null | grep -cE 'gcs_a_backend|gcs_a_calibration|swarm_bridge' || true)
CALIB_PARAM=$(rosparam get /group_a/calibration_ready 2>/dev/null || echo "unset")
CALIB_LOG=$(grep -c 'calibration ready: 15/15' "$HLOG/04_gcs_backend.log" 2>/dev/null || echo 0)
BE_READY=$(grep -c 'GCS_A backend ready' "$HLOG/04_gcs_backend.log" 2>/dev/null || echo 0)
echo "[gcs]    nodes=${GCS_NODES:-0} calibration_ready_param=${CALIB_PARAM} calib_log=${CALIB_LOG:-0} backend_ready_log=${BE_READY:-0}"

# 4. TCP (39001)
TCP_LISTEN=$(ss -ltn 2>/dev/null | grep -c ':39001 ' || true)
echo "[tcp]    listening=${TCP_LISTEN:-0}"

# 5. takeoff state
TKOFF_OK=$(grep -c '成功 15/15' "$HLOG/06_takeoff.log" 2>/dev/null || echo 0)
echo "[takeoff] summary_15ok=${TKOFF_OK:-0}"
if [ -f "$HLOG/06_takeoff.log" ]; then
  tail -3 "$HLOG/06_takeoff.log" 2>/dev/null
fi

# 6. task
for f in "$HLOG"/07_task_*.log; do
  [ -f "$f" ] || continue
  echo "[task]   $f: $(tail -1 "$f")"
done
echo "===== end ====="
