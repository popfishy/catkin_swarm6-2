#!/usr/bin/env python3
"""纯监控 UAV1 长轨迹执行：订阅 /exec_state 与位置，直到终端态。

不调用任何服务/不发布任何消息，安全附加在运行中任务之上。
"""
from __future__ import annotations

import sys

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

TERMINAL = frozenset(("COMPLETED", "EGO_PLAN_FAILED", "EGO_EXEC_TIMEOUT",
                      "POSE_STALE"))
WINDOW_S = 1500.0


def main():
    rospy.init_node("monitor_uav1_flight", anonymous=True)
    last = None
    seen_exec = False
    last_pos = 0.0
    end = rospy.Time.now() + rospy.Duration(WINDOW_S)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        try:
            es = rospy.wait_for_message("/exec_state", String, timeout=2)
            if es.data == "EXECUTING":
                seen_exec = True
            if es.data != last:
                print("[%8.1f] exec_state: %s"
                      % (now - end.to_sec() + WINDOW_S, es.data), flush=True)
                last = es.data
            if es.data in TERMINAL or (es.data == "HOLD" and seen_exec):
                print("TERMINAL reached: %s" % es.data, flush=True)
                return 0
        except Exception:
            pass
        if now - last_pos >= 15.0:
            last_pos = now
            try:
                pose = rospy.wait_for_message("/mavros/local_position/pose",
                                              PoseStamped, timeout=1)
                p = pose.pose.position
                print("[%8.1f] pos: (%.2f, %.2f, %.2f)"
                      % (now - end.to_sec() + WINDOW_S, p.x, p.y, p.z),
                      flush=True)
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
