#!/usr/bin/env python3
"""Directly verify EgoSwarmDriver consumes neighbor odom for the distance gate.

Runs on the UAV6 master (11316) against the live bridge network: builds an
EgoSwarmDriver with UAV6's neighbor_odom_topics, waits for callbacks, prints
how many neighbor poses were received (freshness), then verifies _distance_safe
flags a too-close neighbor.
"""
from __future__ import annotations

import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from swarm_uav_executor.drivers.ego_swarm import EgoSwarmDriver

TOPICS = ",".join("/UAV%d/mavros/local_position/odom" % m
                  for m in list(range(1, 16)) if m != 6)


def main():
    rospy.init_node("verify_ego_receives", anonymous=True)
    driver = EgoSwarmDriver(
        namespace="", state_timeout_s=10.0, ros=rospy,
        monotonic_clock=time.monotonic,
        neighbor_odom_topics=TOPICS,
        min_horizontal_distance_m=1.0, min_vertical_distance_m=2.0)
    rospy.sleep(3.0)
    now = time.monotonic()
    with driver._lock:
        poses = dict(driver._neighbor_poses)
    print("received neighbor poses: %d (expect 14)" % len(poses))
    fresh = 0
    for t, (pose, stamp) in sorted(poses.items()):
        age = now - stamp
        if age <= 1.0:
            fresh += 1
        print("  %-40s age=%.3fs pos=(%.2f,%.2f,%.2f)"
              % (t, age, pose.position.x, pose.position.y, pose.position.z))
    print("fresh (<=1.0s): %d/14" % fresh)

    # own pose at a safe distance from all received neighbors -> safe True
    own = PoseStamped()
    own.pose.position.x = 0.0
    own.pose.position.y = 0.0
    own.pose.position.z = 12.0
    driver._on_pose(own)
    print("distance_safe(far from all) = %s" % driver._distance_safe(own.pose))

    # inject a too-close neighbor -> must be False
    odom = Odometry()
    odom.pose.pose.position.x = 0.5
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.position.z = 12.1
    driver._on_neighbor_odom(odom, "/FAKE_NEAR")
    print("distance_safe(after fake near 0.5m) = %s" % driver._distance_safe(own.pose))

    # verify _wait_for_terminal gate path returns MIN_DISTANCE_BREACH
    from swarm_uav_executor.models import MotionGoal
    import threading
    goal = MotionGoal(50.0, -12.5, 12.0, 0, command="MOVE_TO")
    result = driver.start_move_to(goal, threading.Event(), time.monotonic() + 3.0)
    print("start_move_to with near neighbor -> success=%s error=%s"
          % (result.success, result.error_code))


if __name__ == "__main__":
    main()
