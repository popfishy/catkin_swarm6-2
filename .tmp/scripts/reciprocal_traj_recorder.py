#!/usr/bin/env python3
"""Record both test UAV odometry + exec_state + task states during reciprocal run."""
import atexit
import csv
import time
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from swarm_uav_interfaces.msg import UavTaskState

OUT = Path("/home/ub20tg/catkin_swarm6-2/.tmp/logs/reciprocal_26082621_traj.csv")

rows = []
started = time.monotonic()


def flush():
    with OUT.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "uav", "kind", "x", "y", "z", "vx", "vy", "vz", "extra1", "extra2"])
        w.writerows(rows)
    print("wrote %d rows to %s" % (len(rows), OUT))


atexit.register(flush)


def on_odom(msg, name):
    rows.append([round(time.monotonic() - started, 3), name, "odom",
                 msg.pose.pose.position.x, msg.pose.pose.position.y,
                 msg.pose.pose.position.z, msg.twist.twist.linear.x,
                 msg.twist.twist.linear.y, msg.twist.twist.linear.z, "", ""])


def on_state(msg, name):
    rows.append([round(time.monotonic() - started, 3), name, "state",
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, msg.data, ""])


def on_task(msg, name):
    rows.append([round(time.monotonic() - started, 3), name, "task",
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, msg.status, msg.error_code])


def main():
    rospy.init_node("reciprocal_traj_recorder", anonymous=True)
    for name in ("UAV1", "UAV2"):
        rospy.Subscriber("/%s/mavros/local_position/odom" % name, Odometry,
                         on_odom, callback_args=name, queue_size=100)
        rospy.Subscriber("/%s/exec_state" % name, String,
                         on_state, callback_args=name, queue_size=10)
        rospy.Subscriber("/%s/uav_task_state" % name, UavTaskState,
                         on_task, callback_args=name, queue_size=20)
    rospy.loginfo("recorder ready, recording until shutdown")
    rospy.spin()


if __name__ == "__main__":
    main()

