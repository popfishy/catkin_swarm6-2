#!/usr/bin/env python3
"""给 UAV1 (A01) 发单机 MOVE_TO：PREPARE + START，走 MOVE_TO 分层逻辑。

目标 = prepare 中 A01 目标（任务坐标 17.5, 50, 12；executor 转 ENU）。
"""
from __future__ import annotations

import sys
import time

import rospy
from swarm_uav_interfaces.msg import Pose3DYaw, TaskAssignment
from swarm_uav_interfaces.srv import (UavTask, UavTaskControl,
                                      UavTaskControlRequest, UavTaskRequest)


def main():
    rospy.init_node("single_move_uav1", anonymous=True)
    rospy.wait_for_service("/UAV1/uav_task", timeout=10)
    task = rospy.ServiceProxy("/UAV1/uav_task", UavTask)
    ctrl = rospy.ServiceProxy("/UAV1/uav_task_control", UavTaskControl)

    req = UavTaskRequest()
    req.protocol_version = "1.0"
    req.mission_id = "single-move"
    req.group_id = "GroupA"
    req.command_id = "tcp-single-move-001"
    req.uav_id = "UAV1"
    req.exec_target = "UAV1"
    req.command = "MOVE_TO"
    req.timeout_s = 60.0
    req.leader_id = ""
    a = TaskAssignment()
    a.uav_id = "UAV1"
    a.formation_follow = False
    a.target_id = ""
    a.target_pose = Pose3DYaw()
    a.target_pose.x = 17.5
    a.target_pose.y = 50.0
    a.target_pose.z = 12.0
    a.target_pose.yaw = 0.0
    req.assignment = a

    resp = task(req)
    print("PREPARE: accepted=%s status=%s error=%s msg=%s"
          % (resp.accepted, resp.status, resp.error_code, resp.message))
    if not resp.accepted:
        return 1

    c = UavTaskControlRequest()
    c.protocol_version = "1.0"
    c.mission_id = "single-move"
    c.command_id = "tcp-single-move-001"
    c.uav_id = "UAV1"
    c.exec_target = "UAV1"
    c.operation = "START"
    c.reason = "single UAV1 MOVE_TO test"
    resp2 = ctrl(c)
    print("START: accepted=%s status=%s error=%s msg=%s"
          % (resp2.accepted, resp2.status, resp2.error_code, resp2.message))

    # 监控 exec_state / uav_task_state
    end = time.monotonic() + 60
    last = None
    while time.monotonic() < end:
        try:
            st = rospy.wait_for_message("/uav_task_state",
                                        rospy.AnyMsg, timeout=2)
        except Exception:
            st = None
        # 简化：直接读 exec_state
        from std_msgs.msg import String
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
