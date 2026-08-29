#!/bin/bash
# 一键干净启动 headless 15 机 SITL 至 takeoff（机载层统一按 SAFETY_MODE 直接启动，不再 shadow→active 重启）。
#
# 用法:
#   bash .tmp/scripts/startup_ego_sitl.sh [UAV_LIST] [SAFETY_MODE]
#     UAV_LIST    默认 "1 2"  —— takeoff 的测试机（批次任务传 "1 2 ... 15"）
#     SAFETY_MODE 默认 active —— 机载层监督模式（active | shadow | off），15 机统一一致
#
# 步骤: 预检(无残留进程) -> sim(15机PX4) -> onboard(15机 MAVROS+executor, 统一 SAFETY_MODE)
#       -> gcs(bridge->backend) -> 标定(gp_origin统一坐标系, 必须 takeoff 前)
#       -> takeoff(arm+OFFBOARD) -> 高度确认
# 任一步失败即退出（非零）。调用方决定是否整体干净重启；
# 本脚本绝不尝试“就地恢复 offboard / 单机重启机载层”，测试失败后的重跑一律先 cleanup.sh。
set -u
cd "$(dirname "$0")/../.."
WS=$(pwd)
source devel/setup.bash 2>/dev/null
export ROS_HOME="$WS/.ros_home"
export ROS_LOG_DIR="$WS/.ros_home/log"
export ROS_HOSTNAME=localhost
HLOG="$ROS_LOG_DIR/headless_test"
export HLOG   # wait_until 用 bash -c 子进程检查日志，必须 export（否则 $HLOG 为空卡死）
mkdir -p "$HLOG"
UAVS="${1:-1 2}"
SAFETY_MODE="${2:-active}"
PICKLE_TIMEOUT_S=${PICKLE_TIMEOUT_S:-240}   # sim/onboard 单步上限
SERVICE_TIMEOUT_S=${SERVICE_TIMEOUT_S:-60}

fail() { echo "[FAIL] $*" >&2; exit 1; }

# wait_until <desc> <timeout_s> <cmd...> —— 轮询直到命令成功
wait_until() {
  local desc=$1 timeout=$2; shift 2
  local deadline=$(( $(date +%s) + timeout ))
  while [ "$(date +%s)" -lt "$deadline" ]; do
    if "$@" >/dev/null 2>&1; then echo "[OK] $desc"; return 0; fi
    sleep 3
  done
  fail "timeout ($timeout s): $desc"
}

px4_count() { ps aux | grep '[p]x4' | wc -l; }
spawn_count() { grep -c 'Spawn status' "$HLOG/01_sim.log" 2>/dev/null || echo 0; }
onboard_ready() { [ "$(grep -c '^ready UAV' "$HLOG/02_onboard.log" 2>/dev/null)" -ge 15 ]; }
uav_service() {
  local port=${1}; shift
  ROS_MASTER_URI="http://localhost:${port}" timeout 4 rosservice list 2>/dev/null \
    | grep -q "$1"
}
calibrated() {
  ROS_MASTER_URI="http://localhost:11310" timeout 4 rostopic echo -n 1 \
    /group_a/calibration_ready 2>/dev/null | grep -q 'data: True'
}
uav_alt() { # uav_alt <idx> <min_z>
  local idx=$1 min_z=$2
  ROS_MASTER_URI="http://localhost:$((11310 + idx))" timeout 4 rostopic echo -n 1 \
    /mavros/local_position/pose 2>/dev/null | awk -v m="$min_z" \
    '/z:/{z=$2; gsub(/[^0-9.eE+-]/,"",z); if (z+0 > m) exit 0; exit 1}'
}

echo "===== 预检：无残留仿真进程 ====="
[ "$(px4_count)" -eq 0 ] || fail "残留 PX4 进程 $(px4_count) 个，请先 cleanup.sh"
[ "$(ps aux | grep -cE '[g]azebo|[r]osmaster|[m]avros')" -eq 0 ] \
  || fail "残留 gazebo/rosmaster/mavros 进程，请先 cleanup.sh"
[ "$(ps aux | grep -c '[b]ridge_node.py')" -eq 0 ] \
  || fail "残留 bridge_node.py 进程，请先 cleanup.sh（旧 bridge 占用 UAV ZMQ 端口，新 bridge bind 失败退出，导致标定卡死）"

echo "===== 1/5 sim：15 机 headless PX4 ====="
bash "$WS/.tmp/headless_test/run.sh" sim || fail "sim 启动失败"
wait_until "15 架 px4 进程" "$PICKLE_TIMEOUT_S" bash -c '[ "$(ps aux | grep "[p]x4" | wc -l)" -eq 15 ]'
wait_until "gazebo 15 模型 spawn" "$PICKLE_TIMEOUT_S" bash -c '[ "$(grep -c "Spawn status" "$HLOG/01_sim.log" 2>/dev/null || echo 0)" -ge 15 ]'
echo "[OK] sim 就绪"

echo "===== 2/5 onboard：15 机 MAVROS + executor（统一 mode=$SAFETY_MODE，一次性启动，不重启） ====="
EGO_SAFETY_SUPERVISOR_MODE="$SAFETY_MODE" bash "$WS/.tmp/headless_test/run.sh" onboard \
  || fail "onboard 启动失败"
wait_until "15 机 onboard ready" "$PICKLE_TIMEOUT_S" bash -c \
  '[ "$(grep -c "^ready UAV" "$HLOG/02_onboard.log" 2>/dev/null || echo 0)" -ge 15 ]'
echo "[OK] onboard 就绪（15 机均 $SAFETY_MODE）"

echo "===== 3/5 gcs：bridge -> backend（顺序启动避免 11310 竞态） ====="
bash "$WS/.tmp/headless_test/run.sh" gcs_bridge_only || fail "gcs bridge 启动失败"
wait_until "gcs bridge 服务代理 /UAV1/uav_task" "$SERVICE_TIMEOUT_S" \
  bash -c "ROS_MASTER_URI=http://localhost:11310 timeout 4 rosservice list 2>/dev/null | grep -q '/UAV1/uav_task'"
bash "$WS/.tmp/headless_test/run.sh" gcs_backend_only || fail "gcs backend 启动失败"
echo "[OK] gcs 就绪"

echo "===== 4/5 标定：disarm 时冻结 gp_origin（统一坐标系） ====="
wait_until "标定完成 /group_a/calibration_ready=True" "$SERVICE_TIMEOUT_S" \
  bash -c "ROS_MASTER_URI=http://localhost:11310 timeout 4 rostopic echo -n 1 /group_a/calibration_ready 2>/dev/null | grep -q 'data: True'"
echo "[OK] 标定完成，坐标系已统一"

echo "===== 5/5 takeoff：arm + OFFBOARD（测试机: $UAVS） ====="
python3 "$WS/src/safe_valley_exp/scripts/offboard_takeoff_15.py" $UAVS >/dev/null 2>&1 \
  || fail "takeoff $UAVS 失败"
for idx in $UAVS; do
  wait_until "UAV$idx 起飞高度" "$SERVICE_TIMEOUT_S" \
    bash -c "ROS_MASTER_URI=http://localhost:$((11310 + idx)) timeout 4 rostopic echo -n 1 /mavros/local_position/pose 2>/dev/null | awk '/z:/{z=\$2; gsub(/[^0-9.eE+-]/,\"\",z); if (z+0 > 3.0) exit 0; exit 1}'"
done
echo "[READY] SITL 干净启动完成：sim/onboard($SAFETY_MODE)/gcs/标定/takeoff($UAVS) 全部就绪"
echo "        下一步运行 reciprocal："
echo "        ROS_MASTER_URI=http://localhost:11310 python3 verification/ego_smoke/reciprocal_15sitl.py --uav-a UAV1 --index-a 1 --uav-b UAV2 --index-b 2 --no-require-rebound --execute --confirm-sitl I_UNDERSTAND_SITL"
