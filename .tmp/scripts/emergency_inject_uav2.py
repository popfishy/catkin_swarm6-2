#!/usr/bin/env python3
"""EMERGENCY 注入（方案 Y 端到端验证）：在 UAV2 机载 master 上把本机 trajectory_intent
改写成 UAV1 的同刻重叠 intent 发布到 /neighbor_intent，触发监督层 EMERGENCY →
replanOnce 常规 replan 失败 + ttc 紧急 → 制动 EMERGENCY_BRAKE → BRAKE_HOLD。

用法：ROS_MASTER_URI=http://localhost:11312 python3 .tmp/scripts/emergency_inject_uav2.py
"""
import math
import time

import rospy
from geometry_msgs.msg import Pose
from swarm_uav_interfaces.msg import UavTrajectoryIntent


def main():
    rospy.init_node("emergency_inject", anonymous=True)
    pub = rospy.Publisher("/neighbor_intent", UavTrajectoryIntent, queue_size=10)
    sub_msg = {"intent": None, "t": None}

    def on_intent(msg):
        # 只处理本机（UAV2）ACTIVE intent 作为模板
        if msg.uav_id == "UAV2" and msg.phase == UavTrajectoryIntent.PHASE_ACTIVE:
            sub_msg["intent"] = msg
            sub_msg["t"] = rospy.Time.now().to_sec()

    rospy.Subscriber("/trajectory_intent", UavTrajectoryIntent, on_intent,
                     queue_size=10)

    rospy.loginfo("waiting for a UAV2 ACTIVE intent template...")
    while sub_msg["intent"] is None and not rospy.is_shutdown():
        rospy.sleep(0.2)

    # 构造 UAV1 的同刻重叠 intent（完全复制本机轨迹 → 任何本机轨迹都侵入 1m 硬门）
    for attempt in range(200):
        base = sub_msg["intent"]
        if base is None:
            break
        msg = UavTrajectoryIntent()
        msg.protocol_version = "2.0"
        msg.phase = UavTrajectoryIntent.PHASE_ACTIVE
        msg.uav_id = "UAV1"
        msg.exec_target = "UAV1"
        msg.frame_id = "map"
        msg.stamp = rospy.Time.now().to_sec()  # 与当前执行同刻（激活 epoch）
        msg.clearance = 0.5
        msg.traj_id = 9999 + attempt
        msg.t = list(base.t)
        for p in base.sampled_traj:
            msg.sampled_traj.append(p)
        pub.publish(msg)
        rospy.sleep(0.25)
    rospy.loginfo("injection done")


if __name__ == "__main__":
    main()
