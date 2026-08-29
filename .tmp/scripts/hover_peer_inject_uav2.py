#!/usr/bin/env python3
"""悬停 peer 注入（验证 LEFT/RIGHT 候选）：在 UAV2 机载 master 上注入一个 UAV1
悬停在指定点（默认 (10,45,12)）的静态 intent，触发 collisionCheckTick 冲突 →
replanOnce 常规 replan 失败 → tryYieldReplan（UAV2 高 ID）→ LEFT/RIGHT 绕行候选提交。

用法：ROS_MASTER_URI=http://localhost:11312 python3 .tmp/scripts/hover_peer_inject_uav2.py [x y z] [seconds]
"""
import sys
import time

import rospy
from geometry_msgs.msg import Pose
from swarm_uav_interfaces.msg import UavTrajectoryIntent


def main():
    args = sys.argv[1:]
    if len(args) >= 3:
        hover = (float(args[0]), float(args[1]), float(args[2]))
    else:
        hover = (10.0, 45.0, 12.0)
    seconds = float(args[3]) if len(args) >= 4 else 90.0
    rospy.init_node("hover_peer_inject", anonymous=True)
    pub = rospy.Publisher("/neighbor_intent", UavTrajectoryIntent, queue_size=10)
    rospy.loginfo("injecting hover peer at %s for %.0fs", hover, seconds)
    end = time.time() + seconds
    i = 0
    while not rospy.is_shutdown() and time.time() < end:
        msg = UavTrajectoryIntent()
        msg.protocol_version = "2.0"
        msg.phase = UavTrajectoryIntent.PHASE_ACTIVE
        msg.uav_id = "UAV1"
        msg.exec_target = "UAV1"
        msg.frame_id = "map"
        msg.stamp = rospy.Time.now().to_sec()
        msg.clearance = 0.5
        msg.traj_id = 7777 + int(rospy.Time.now().to_sec())
        msg.t = [0.0, 60.0]
        p0, p1 = Pose(), Pose()
        for p in (p0, p1):
            p.position.x = hover[0]
            p.position.y = hover[1]
            p.position.z = hover[2]
        msg.sampled_traj = [p0, p1]
        pub.publish(msg)
        i += 1
        rospy.sleep(0.4)
    rospy.loginfo("hover injection done (%d msgs at %s)", i, (hover,))


if __name__ == "__main__":
    main()

