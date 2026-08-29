#!/usr/bin/env python3
"""给 UAV2 发送一个前方 FOLLOW_ROUTE 任务（配合 emergency 注入使用）。

用法：ROS_MASTER_URI=http://localhost:11310 python3 .tmp/scripts/send_task_uav2.py <x> <y> <z>
"""
import sys
import time
import uuid

import rospy
from swarm_uav_interfaces.msg import Pose3DYaw, TaskAssignment, UavTaskState
from swarm_uav_interfaces.srv import UavTask, UavTaskControl, UavTaskControlRequest, UavTaskRequest

PROTOCOL = "1.0"
GROUP = "GroupA"


def main():
    x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    rospy.init_node("send_task_uav2", anonymous=True)
    mission = "inj-%s" % uuid.uuid4().hex[:8]
    cmd = "cmd-%s" % uuid.uuid4().hex[:8]
    req = UavTaskRequest()
    req.protocol_version = PROTOCOL
    req.mission_id = mission
    req.group_id = GROUP
    req.command_id = cmd
    req.uav_id = "UAV2"
    req.exec_target = "UAV2"
    req.command = "FOLLOW_ROUTE"
    req.timeout_s = 90.0
    req.leader_id = ""
    a = TaskAssignment()
    a.uav_id = "UAV2"
    a.formation_follow = False
    a.target_id = ""
    a.target_pose = Pose3DYaw(x=x, y=y, z=z, yaw=0.0)
    a.waypoints = [a.target_pose]
    req.assignment = a
    srv = rospy.ServiceProxy("/UAV2/uav_task", UavTask)
    r = srv(req)
    print("PREPARE accepted:", r.accepted, r.error_code, r.message, flush=True)
    ctrl = rospy.ServiceProxy("/UAV2/uav_task_control", UavTaskControl)
    rc = ctrl(UavTaskControlRequest(
        protocol_version=PROTOCOL, operation="START", mission_id=mission,
        command_id=cmd, uav_id="UAV2", exec_target="UAV2",
        reason="emergency injection task"))
    print("START accepted:", rc.accepted, rc.error_code, rc.message, flush=True)

    def on_state(msg):
        if msg.mission_id == mission and msg.command_id == cmd:
            print("task_state:", msg.status, msg.error_code, flush=True)
            if msg.status in ("COMPLETED", "FAILED", "HELD"):
                rospy.signal_shutdown("terminal")

    rospy.Subscriber("/UAV2/uav_task_state", UavTaskState, on_state, queue_size=10)
    deadline = time.time() + 100
    while not rospy.is_shutdown() and time.time() < deadline:
        rospy.sleep(0.5)


if __name__ == "__main__":
    main()
