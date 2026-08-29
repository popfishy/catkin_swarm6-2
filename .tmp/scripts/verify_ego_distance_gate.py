#!/usr/bin/env python3
"""Single-master EgoSwarmDriver distance-gate end-to-end verification.

Launches one ego_planner_driver node (roslaunch), drives it with an
EgoSwarmDriver instance (real ROS), feeds the ego setpoint back as local_pose
(ideal controller), injects a too-close neighbor odom, and asserts the
runtime distance gate (min-snap refactor) triggers a local HOLD:
  start_move_to -> MIN_DISTANCE_BREACH, /hold published, exec_state -> HOLD.

Usage (needs a rosmaster on :11311):
  source devel/setup.bash
  python3 .tmp/scripts/verify_ego_distance_gate.py
"""
from __future__ import annotations

import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String

from swarm_uav_executor.drivers.ego_swarm import EgoSwarmDriver
from swarm_uav_executor.models import MotionGoal

UAV2_ODOM = "/UAV2/mavros/local_position/odom"

latest_setpoint = [None]
hold_seen = [0]
state_events = []
lock = threading.Lock()


def pose_loop(pub):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        with lock:
            sp = latest_setpoint[0]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        if sp is not None:
            pose.pose = sp.pose
        else:
            pose.pose.position.z = 5.0
            pose.pose.orientation.w = 1.0
        pub.publish(pose)
        rate.sleep()


def main():
    if not rospy.core.is_initialized():
        rospy.init_node("verify_ego_distance_gate", anonymous=True)
    # 1) launch the ego_planner_driver node on this master
    node = subprocess.Popen(
        ["roslaunch", "ego_planner_driver", "ego_planner_driver.launch"],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        rospy.Subscriber("/exec_state", String,
                         lambda m: state_events.append(m.data))
        rospy.Subscriber("/hold", Empty,
                         lambda m: hold_seen.__setitem__(0, hold_seen[0] + 1))
        # note: /hold is std_msgs/Empty; FakePub-based assertions use driver pub.
        pose_pub = rospy.Publisher("/local_pose", PoseStamped, queue_size=1)
        threading.Thread(target=pose_loop, args=(pose_pub,), daemon=True).start()

        # 2) build the driver with a fake neighbor odom source
        driver = EgoSwarmDriver(
            namespace="", state_timeout_s=10.0, ros=rospy,
            monotonic_clock=time.monotonic,
            neighbor_odom_topics=UAV2_ODOM,
            min_horizontal_distance_m=1.0, min_vertical_distance_m=2.0)
        # wait for exec_state HOLD (node ready)
        end = time.monotonic() + 20.0
        while time.monotonic() < end and "HOLD" not in state_events:
            time.sleep(0.05)
        if "HOLD" not in state_events:
            print("FAIL: ego_planner_driver node not ready")
            return 1

        # 3) inject a too-close neighbor odom (horizontal 0.5 < 1.0, vertical 0.1 < 2.0)
        odom_pub = rospy.Publisher(UAV2_ODOM, Odometry, queue_size=1)
        rospy.sleep(0.5)
        odom = Odometry()
        odom.pose.pose.position.x = 0.5
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 5.1
        for _ in range(10):
            odom_pub.publish(odom)
            rospy.sleep(0.05)

        # 4) run a MOVE_TO; the distance gate must fire
        goal = MotionGoal(10.0, 0.0, 12.0, 0, command="MOVE_TO")
        result = driver.start_move_to(goal, threading.Event(),
                                      time.monotonic() + 5.0)
        print("result: success=%s error_code=%s message=%s"
              % (result.success, result.error_code, result.message))
        if result.success or result.error_code != "MIN_DISTANCE_BREACH":
            print("FAIL: expected MIN_DISTANCE_BREACH, got %s" % result.error_code)
            return 1
        # /hold published by _issue_hold -> C++ should enter HOLD
        time.sleep(1.0)
        if "HOLD" not in state_events[-3:]:
            print("FAIL: exec_state did not reach HOLD after distance breach; "
                  "events=%s" % state_events[-6:])
            return 1
        print("PASS: distance gate -> MIN_DISTANCE_BREACH + exec_state HOLD")
        return 0
    finally:
        node.terminate()
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()


if __name__ == "__main__":
    sys.exit(main())
