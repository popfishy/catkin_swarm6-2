#!/bin/bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
export ROS_MASTER_URI=http://localhost:11310
exec python3 verification/ego_smoke/reciprocal_15sitl.py \
  --uav-a UAV1 --index-a 1 --uav-b UAV2 --index-b 2 \
  --no-require-rebound --execute --confirm-sitl I_UNDERSTAND_SITL
