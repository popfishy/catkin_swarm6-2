#!/usr/bin/env python3
"""给 UAV1 发 FOLLOW_ROUTE（折线 keypoints，leader 单机执行）。

折线（12m 层）：当前正上方 (17.5,50,12) → (22.5,50,12) → (22.5,45,12) → (17.5,45,12)。
"""
from __future__ import annotations

import sys

import rospy
from std_msgs.msg import String
from swarm_uav_interfaces.msg import Pose3DYaw, TaskAssignment
from swarm_uav_interfaces.srv import (UavTask, UavTaskControl,
                                      UavTaskControlRequest, UavTaskRequest)

WAYPOINTS = [
    (17.5, 50.0, 12.0),
    (22.5, 50.0, 12.0),
    (22.5, 45.0, 12.0),
    (17.5, 45.0, 12.0),
]


def main():
    rospy.init_node("single_follow_uav1", anonymous=True)
    rospy.wait_for_service("/UAV1/uav_task", timeout=10)
    task = rospy.ServiceProxy("/UAV1/uav_task", UavTask)
    ctrl = rospy.ServiceProxy("/UAV1/uav_task_control", UavTaskControl)

    req = UavTaskRequest()
    req.protocol_version = "1.0"
    req.mission_id = "single-follow"
    req.group_id = "GroupA"
    req.command_id = "tcp-single-follow-001"
    req.uav_id = "UAV1"
    req.exec_target = "UAV1"
    req.command = "FOLLOW_ROUTE"
    req.timeout_s = 60.0
    req.leader_id = ""
    a = TaskAssignment()
    a.uav_id = "UAV1"
    a.formation_follow = False
    a.target_id = ""
    a.target_pose = Pose3DYaw()
    a.target_pose.x, a.target_pose.y, a.target_pose.z = WAYPOINTS[-1]
    a.target_pose.yaw = 0.0
    a.waypoints = []
    for x, y, z in WAYPOINTS:
        wp = Pose3DYaw()
        wp.x, wp.y, wp.z, wp.yaw = x, y, z, 0.0
        a.waypoints.append(wp)
    req.assignment = a

    resp = task(req)
    print("PREPARE: accepted=%s status=%s error=%s msg=%s"
          % (resp.accepted, resp.status, resp.error_code, resp.message))
    if not resp.accepted:
        return 1

    c = UavTaskControlRequest()
    c.protocol_version = "1.0"
    c.mission_id = "single-follow"
    c.command_id = "tcp-single-follow-001"
    c.uav_id = "UAV1"
    c.exec_target = "UAV1"
    c.operation = "START"
    c.reason = "single UAV1 FOLLOW_ROUTE test"
    resp2 = ctrl(c)
    print("START: accepted=%s status=%s error=%s msg=%s"
          % (resp2.accepted, resp2.status, resp2.error_code, resp2.message))

    # 监控 exec_state
    last = None
    end = rospy.Time.now() + rospy.Duration(60)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        try:
            es = rospy.wait_for_message("/exec_state", String, timeout=2)
            if es.data != last:
                print("exec_state:", es.data)
                last = es.data
            if es.data in ("COMPLETED", "EGO_PLAN_FAILED", "EGO_EXEC_TIMEOUT", "HOLD"):
                break
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
