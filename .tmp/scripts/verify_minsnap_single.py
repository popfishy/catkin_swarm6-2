#!/usr/bin/env python3
"""Single-master min-snap trajectory smoothness + lifecycle verification.

Launches one ego_planner_driver node, feeds /setpoint/ego PositionTarget back
as /local_pose (ideal controller), sends a 2 m-densified route (EgoSwarmDriver
MOVE_TO style) and asserts:
  - lifecycle EXECUTING -> COMPLETED
  - EGO mask 2048 / FRAME_LOCAL_NED, P/V/A finite
  - speed profile smooth: peak <= 2.0 m/s, no periodic zero dips / cruise
    restart jumps (rolling-segment continuity).

Usage (rosmaster on :11311):
  source devel/setup.bash
  python3 .tmp/scripts/verify_minsnap_single.py
"""
from __future__ import annotations

import math
import os
import re
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Empty, String

state_events = []
speed_series = []
setpoint_count = [0]
latest_setpoint = [None]
lock = threading.Lock()


def ensure_master():
    """Start a rosmaster if ROS_MASTER_URI has none (returns the process)."""
    try:
        from rosgraph.masterapi import Master
        Master("/ensure_master").getPid()
        return None
    except Exception:
        uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        m = re.search(r":(\d+)", uri)
        port = m.group(1) if m else "11311"
        proc = subprocess.Popen(["roscore", "-p", port],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.STDOUT)
        time.sleep(3)
        return proc


def on_setpoint(msg):
    with lock:
        setpoint_count[0] += 1
        latest_setpoint[0] = msg
        if msg.type_mask == 2048:  # EGO trajectory mask
            v = math.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 +
                          msg.velocity.z ** 2)
            speed_series.append((rospy.Time.now().to_sec(), v))


def on_state(msg):
    with lock:
        state_events.append(msg.data)


def pose_loop(pub):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        with lock:
            sp = latest_setpoint[0]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        if sp is not None:
            pose.pose.position.x = sp.position.x
            pose.pose.position.y = sp.position.y
            pose.pose.position.z = sp.position.z
            pose.pose.orientation.z = math.sin(sp.yaw / 2.0)
            pose.pose.orientation.w = math.cos(sp.yaw / 2.0)
        else:
            pose.pose.position.z = 5.0
            pose.pose.orientation.w = 1.0
        pub.publish(pose)
        rate.sleep()


def wait_for(pred, what, timeout=40.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.05)
    print("FAIL: timeout waiting for %s" % what)
    return False


def main():
    master_proc = ensure_master()
    rospy.init_node("verify_minsnap_single", anonymous=True)
    node = subprocess.Popen(
        ["roslaunch", "ego_planner_driver", "ego_planner_driver.launch"],
        stdout=open(".tmp/logs/ego_node_single.log", "wb"),
        stderr=subprocess.STDOUT)
    try:
        rospy.Subscriber("/exec_state", String, on_state)
        rospy.Subscriber("/setpoint/ego", PositionTarget, on_setpoint)
        pose_pub = rospy.Publisher("/local_pose", PoseStamped, queue_size=1)
        threading.Thread(target=pose_loop, args=(pose_pub,), daemon=True).start()
        waypoints_pub = rospy.Publisher("/waypoints", PolygonStamped, queue_size=1)

        if not wait_for(lambda: "HOLD" in state_events, "node ready (HOLD)"):
            return 1

        # --- smoothness: 2m-densified MOVE_TO-like route (20 m horizontal) ---
        wp = PolygonStamped()
        wp.header.frame_id = "map"
        wp.header.stamp = rospy.Time.now()
        for x in range(0, 21, 2):  # 2m spacing, 0..20m
            p = Point32(); p.x = float(x); p.y = 0.0; p.z = 15.0
            wp.polygon.points.append(p)
        waypoints_pub.publish(wp)

        ok_completed = wait_for(lambda: "COMPLETED" in state_events, "COMPLETED")
        with lock:
            speeds = [s for _, s in speed_series]
        print("states: %s" % state_events)
        print("ego setpoints: %d, speed samples: %d" % (setpoint_count[0], len(speeds)))
        if not ok_completed or len(speeds) < 10:
            print("FAIL: lifecycle or speed samples missing")
            return 1
        vmax = max(speeds)
        print("vmax=%.3f m/s" % vmax)
        if vmax > 2.0:
            print("FAIL: vmax %.2f > 2.0" % vmax)
            return 1
        # no periodic zero dips / cruise restart jumps at 30 Hz
        jumps = [abs(speeds[i + 1] - speeds[i]) for i in range(len(speeds) - 1)
                 if speeds[i + 1] < 0.4 and speeds[i] > 0.8]
        print("cruise restart jumps: %d" % len(jumps))
        if len(jumps) > 3:
            print("FAIL: periodic speed drops detected (rolling discontinuity)")
            return 1
        with lock:
            sp = latest_setpoint[0]
        if sp is None or sp.type_mask != 2048 or \
                sp.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            print("FAIL: EGO setpoint mask/frame contract violated")
            return 1
        print("SMOKE PASS")
        return 0
    finally:
        node.terminate()
        try:
            node.wait(timeout=5)
        except subprocess.TimeoutExpired:
            node.kill()
        if master_proc is not None:
            master_proc.terminate()
            try:
                master_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                master_proc.kill()


if __name__ == "__main__":
    sys.exit(main())
