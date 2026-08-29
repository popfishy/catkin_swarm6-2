#!/bin/bash
# 验证邻机 odom / 轨迹 intent 是否发布并被各无人机接收（软起飞后）。
set -u
WS=/home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

hz_of() {  # master topic label
  local out
  out=$(timeout 6 bash -c "ROS_MASTER_URI=$1 rostopic hz $2" 2>/dev/null | grep 'average rate' | awk '{print $3}')
  echo "${out:-NONE}"
}
info_of() {  # master topic -> "P:x S:y"
  local p s
  p=$(timeout 6 bash -c "ROS_MASTER_URI=$1 rostopic info $2" 2>/dev/null | grep -c 'Publishers')
  s=$(timeout 6 bash -c "ROS_MASTER_URI=$1 rostopic info $2" 2>/dev/null | grep -c 'Subscribers')
  echo "pub=$p sub=$s"
}

echo "=== 1) 本机 odom（MAVROS 发布，软起飞后应 ~10-30Hz）==="
for n in 1 4 6; do
  echo "UAV$n 本机 /mavros/local_position/odom : hz=$(hz_of http://localhost:$((11310+n)) /mavros/local_position/odom) $(info_of http://localhost:$((11310+n)) /mavros/local_position/odom)"
done

echo
echo "=== 2) 邻机 odom 是否到达各 UAV（bridge 跨 master 转发）==="
for n in 1 4 6; do
  for m in 2 5 7; do
    [ "$m" = "$n" ] && continue
    echo "UAV$n <- /UAV$m/mavros/local_position/odom : hz=$(hz_of http://localhost:$((11310+n)) /UAV$m/mavros/local_position/odom) $(info_of http://localhost:$((11310+n)) /UAV$m/mavros/local_position/odom)"
  done
done

echo
echo "=== 3) UAV6 全部邻机 odom（距离门禁数据源，重点）==="
for m in 1 2 3 4 5 7 8 9 10 11 12 13 14 15; do
  echo "UAV6 <- /UAV$m/mavros/local_position/odom : hz=$(hz_of http://localhost:11316 /UAV$m/mavros/local_position/odom)"
done

echo
echo "=== 4) 轨迹 intent 链路（本机发布 / 邻机接收）==="
for n in 1 6; do
  echo "UAV$n 本机 /trajectory_intent : $(info_of http://localhost:$((11310+n)) /trajectory_intent)"
  for m in 5 7; do
    echo "UAV$n <- /UAV$m/trajectory_intent : $(info_of http://localhost:$((11310+n)) /UAV$m/trajectory_intent)"
  done
done

echo
echo "=== 5) executor 订阅列表（确认 EgoSwarmDriver 订阅邻机 odom）==="
timeout 8 bash -c "ROS_MASTER_URI=http://localhost:11316 rosnode info /uav_executor_UAV6" 2>/dev/null | grep -iE 'Subscribed|odom|intent|exec_state|local_pose' | head -20
