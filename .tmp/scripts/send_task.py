#!/usr/bin/env python3
"""给指定 UAV 发送 FOLLOW_ROUTE 任务并等待终态。

用法：ROS_MASTER_URI=http://localhost:11310 python3 .tmp/scripts/send_task.py UAV1 2 5 12 [timeout_s]
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
    uav = sys.argv[1]
    x, y, z = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    timeout = float(sys.argv[5]) if len(sys.argv) > 5 else 90.0
    rospy.init_node("send_task", anonymous=True)
    mission = "t-%s" % uuid.uuid4().hex[:8]
    cmd = "c-%s" % uuid.uuid4().hex[:8]
    req = UavTaskRequest()
    req.protocol_version = PROTOCOL
    req.mission_id = mission
    req.group_id = GROUP
    req.command_id = cmd
    req.uav_id = uav
    req.exec_target = uav
    req.command = "FOLLOW_ROUTE"
    req.timeout_s = timeout
    req.leader_id = ""
    a = TaskAssignment()
    a.uav_id = uav
    a.formation_follow = False
    a.target_id = ""
    a.target_pose = Pose3DYaw(x=x, y=y, z=z, yaw=0.0)
    a.waypoints = [a.target_pose]
    req.assignment = a
    r = rospy.ServiceProxy("/%s/uav_task" % uav, UavTask)(req)
    print("PREPARE accepted:", r.accepted, r.error_code, flush=True)
    ctrl = rospy.ServiceProxy("/%s/uav_task_control" % uav, UavTaskControl)
    rc = ctrl(UavTaskControlRequest(
        protocol_version=PROTOCOL, operation="START", mission_id=mission,
        command_id=cmd, uav_id=uav, exec_target=uav, reason="manual task"))
    print("START accepted:", rc.accepted, rc.error_code, flush=True)

    def on_state(msg):
        if msg.mission_id == mission and msg.command_id == cmd:
            print("task_state:", msg.status, msg.error_code, flush=True)
            if msg.status in ("COMPLETED", "FAILED", "HELD"):
                rospy.signal_shutdown("terminal")

    rospy.Subscriber("/%s/uav_task_state" % uav, UavTaskState, on_state, queue_size=10)
    deadline = time.time() + timeout + 10
    while not rospy.is_shutdown() and time.time() < deadline:
        rospy.sleep(0.5)


if __name__ == "__main__":
    main()
