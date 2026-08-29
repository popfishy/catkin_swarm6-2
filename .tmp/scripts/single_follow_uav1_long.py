#!/usr/bin/env python3
"""给 UAV1 发超长 FOLLOW_ROUTE（leader 单机，合并 coverage-segment-1+2 蛇形航线）。

21 航点 S 形蛇形扫描（12m 层），参考 references/runtime 中 coverage-segment-1
（y=12..75）与 coverage-segment-2（y=75..138）leader A01 的任务航线：
  (15,12)->(80,12)->(80,26)->(15,26)->(15,40)->(80,40)->(80,54)->(15,54)
  ->(15,68)->(80,68)->(80,75)->(80,82)->(15,82)->(15,96)->(80,96)->(80,110)
  ->(15,110)->(15,124)->(80,124)->(80,138)->(15,138)
任务坐标即 ENU（world.json: mission_enu，field 0..100 x 0..150）；总长约 776 m，
@1m/s 巡航 + 转弯减速 ≈ 13-15 分钟。executor state_timeout_s 需 >= 1200s
（uav_offboard_ego.launch ego_state_timeout_s:=1200），否则 DRIVER_TIMEOUT。
"""
from __future__ import annotations

import sys

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from swarm_uav_interfaces.msg import Pose3DYaw, TaskAssignment
from swarm_uav_interfaces.srv import (UavTask, UavTaskControl,
                                      UavTaskControlRequest, UavTaskRequest)

# (x, y, z, yaw)：coverage-segment-1 (11 点) + coverage-segment-2 (11 点)，
# 交界重复点 (80,75) 合并为一次（yaw 1.570796 取 coverage-2 起点同值）。
WAYPOINTS = [
    (15.0, 12.0, 12.0, 0.0),
    (80.0, 12.0, 12.0, 0.0),
    (80.0, 26.0, 12.0, 3.141593),
    (15.0, 26.0, 12.0, 3.141593),
    (15.0, 40.0, 12.0, 0.0),
    (80.0, 40.0, 12.0, 0.0),
    (80.0, 54.0, 12.0, 3.141593),
    (15.0, 54.0, 12.0, 3.141593),
    (15.0, 68.0, 12.0, 0.0),
    (80.0, 68.0, 12.0, 0.0),
    (80.0, 75.0, 12.0, 1.570796),
    (80.0, 82.0, 12.0, 3.141593),
    (15.0, 82.0, 12.0, 3.141593),
    (15.0, 96.0, 12.0, 0.0),
    (80.0, 96.0, 12.0, 0.0),
    (80.0, 110.0, 12.0, 3.141593),
    (15.0, 110.0, 12.0, 3.141593),
    (15.0, 124.0, 12.0, 0.0),
    (80.0, 124.0, 12.0, 0.0),
    (80.0, 138.0, 12.0, 3.141593),
    (15.0, 138.0, 12.0, 3.141593),
]

# HOLD 不列终端态：命令前的稳态就是 HOLD，需见过 EXECUTING 后再遇 HOLD 才算异常。
TERMINAL = frozenset(("COMPLETED", "EGO_PLAN_FAILED", "EGO_EXEC_TIMEOUT",
                      "POSE_STALE"))
TIMEOUT_S = 1200.0


def _total_length() -> float:
    length = 0.0
    for left, right in zip(WAYPOINTS, WAYPOINTS[1:]):
        length += ((left[0] - right[0]) ** 2
                   + (left[1] - right[1]) ** 2) ** 0.5
    return length


def main():
    rospy.init_node("single_follow_uav1_long", anonymous=True)
    rospy.wait_for_service("/UAV1/uav_task", timeout=15)
    task = rospy.ServiceProxy("/UAV1/uav_task", UavTask)
    ctrl = rospy.ServiceProxy("/UAV1/uav_task_control", UavTaskControl)

    req = UavTaskRequest()
    req.protocol_version = "1.0"
    req.mission_id = "single-follow-long"
    req.group_id = "GroupA"
    req.command_id = "tcp-single-follow-long-001"
    req.uav_id = "UAV1"
    req.exec_target = "UAV1"
    req.command = "FOLLOW_ROUTE"
    req.timeout_s = TIMEOUT_S
    req.leader_id = ""
    a = TaskAssignment()
    a.uav_id = "UAV1"
    a.formation_follow = False
    a.target_id = ""
    a.target_pose = Pose3DYaw()
    a.target_pose.x, a.target_pose.y, a.target_pose.z = WAYPOINTS[-1][:3]
    a.target_pose.yaw = WAYPOINTS[-1][3]
    a.waypoints = []
    for x, y, z, yaw in WAYPOINTS:
        wp = Pose3DYaw()
        wp.x, wp.y, wp.z, wp.yaw = x, y, z, yaw
        a.waypoints.append(wp)
    req.assignment = a

    resp = task(req)
    print("PREPARE: accepted=%s status=%s error=%s msg=%s"
          % (resp.accepted, resp.status, resp.error_code, resp.message))
    if not resp.accepted:
        return 1

    c = UavTaskControlRequest()
    c.protocol_version = "1.0"
    c.mission_id = "single-follow-long"
    c.command_id = "tcp-single-follow-long-001"
    c.uav_id = "UAV1"
    c.exec_target = "UAV1"
    c.operation = "START"
    c.reason = "single UAV1 long FOLLOW_ROUTE coverage test"
    resp2 = ctrl(c)
    print("START: accepted=%s status=%s error=%s msg=%s"
          % (resp2.accepted, resp2.status, resp2.error_code, resp2.message))

    print("route: %d waypoints, length ~%.0f m, timeout %.0f s"
          % (len(WAYPOINTS), _total_length(), TIMEOUT_S))

    last = None
    last_pos_print = 0.0
    seen_executing = False
    end = rospy.Time.now() + rospy.Duration(TIMEOUT_S)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        try:
            es = rospy.wait_for_message("/exec_state", String, timeout=2)
            if es.data == "EXECUTING":
                seen_executing = True
            if es.data != last:
                print("[%8.1f] exec_state: %s" % (now - end.to_sec() + TIMEOUT_S,
                                                  es.data), flush=True)
                last = es.data
            if es.data in TERMINAL or (es.data == "HOLD" and seen_executing):
                break
        except Exception:
            pass
        if now - last_pos_print >= 10.0:
            last_pos_print = now
            try:
                pose = rospy.wait_for_message("/mavros/local_position/pose",
                                              PoseStamped, timeout=1)
                p = pose.pose.position
                print("[%8.1f] pos: (%.2f, %.2f, %.2f)"
                      % (now - end.to_sec() + TIMEOUT_S, p.x, p.y, p.z),
                      flush=True)
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
