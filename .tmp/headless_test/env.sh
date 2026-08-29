#!/bin/bash
# 无头测试公共环境（日志统一到工作空间 .ros_home/log/headless_test）
export WS=/home/ub20tg/catkin_swarm6-2
export PX4_ROOT=/home/ub20tg/PX4_Firmware
export PX4_BUILD=$PX4_ROOT/build/px4_sitl_default
export ROS_HOME=$WS/.ros_home
export ROS_LOG_DIR=$WS/.ros_home/log
export HLOG=$ROS_LOG_DIR/headless_test
export LIBGL_ALWAYS_SOFTWARE=1
mkdir -p "$HLOG"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_BUILD" >/dev/null 2>&1
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"
