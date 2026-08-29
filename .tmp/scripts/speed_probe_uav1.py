#!/usr/bin/env python3
"""记录 UAV1 setpoint vs odom 速度峰值，判断超速来自轨迹（SLOW 加速档）还是 PX4 超调。"""
import math
import time

import rospy
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("speed_probe", anonymous=True)
    peaks = {"sp": 0.0, "odom": 0.0, "n": 0}

    def on_sp(msg):
        v = math.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 + msg.velocity.z ** 2)
        peaks["sp"] = max(peaks["sp"], v)

    def on_odom(msg):
        v = msg.twist.twist.linear
        n = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        peaks["odom"] = max(peaks["odom"], n)
        peaks["n"] += 1

    rospy.Subscriber("/setpoint/ego", PositionTarget, on_sp, queue_size=50)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, on_odom, queue_size=50)
    rospy.loginfo("speed probe ready; run reciprocal now, ctrl-c to print peaks")
    rospy.spin()


if __name__ == "__main__":
    main()
