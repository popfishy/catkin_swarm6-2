#!/usr/bin/env python3
"""安全收口：切所有 UAV 到 AUTO.RTL，等待落地并 disarm。

用法：python3 return_land_all.py            # 全部 1..15
      python3 return_land_all.py 1 2 3      # 指定
      python3 return_land_all.py --wait 60   # 指定落地等待秒数（默认 90）
"""
import argparse, subprocess, sys, time

WS = "/home/ub20tg/catkin_swarm6-2"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("uavs", nargs="*", type=int)
    ap.add_argument("--wait", type=float, default=90.0)
    ap.add_argument("--skip-wait", action="store_true")
    args = ap.parse_args()
    idxs = args.uavs or list(range(1, 16))

    ok, failed = [], []
    for idx in idxs:
        cmd = [
            "bash", "-c",
            "source /opt/ros/noetic/setup.bash && source %s/devel/setup.bash && "
            "export ROS_MASTER_URI=http://localhost:%d; export ROS_HOSTNAME=localhost && "
            "timeout 8 rosservice call /mavros/set_mode "
            "'{base_mode: 0, custom_mode: \"AUTO.RTL\"}' 2>&1" % (WS, 11310 + idx),
        ]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=12)
            out = (r.stdout + r.stderr).strip()
            if "mode_sent: True" in out or "success: True" in out:
                ok.append(idx)
            else:
                failed.append((idx, out[-160:]))
        except subprocess.TimeoutExpired:
            failed.append((idx, "service timeout"))
    print("=== RTL 切换汇总 ===")
    print("success: %s" % ok)
    for idx, err in failed:
        print("FAIL UAV%d: %s" % (idx, err))

    if args.skip_wait:
        return 0 if not failed else 2

    # 等待所有目标 UAV disarm（间隔 3s 轮询，最长 --wait 秒）。
    deadline = time.monotonic() + args.wait
    landed = []
    pending = [i for i in idxs if i in ok] + [i for i, _ in failed if i not in ok]
    while time.monotonic() < deadline and pending:
        still = []
        for idx in pending:
            cmd = [
                "bash", "-c",
                "source /opt/ros/noetic/setup.bash && source %s/devel/setup.bash && "
                "export ROS_MASTER_URI=http://localhost:%d; export ROS_HOSTNAME=localhost && "
                "timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null "
                "| grep -E '^armed:' " % (WS, 11310 + idx),
            ]
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
                armed = "armed: True" in r.stdout
            except subprocess.TimeoutExpired:
                armed = True
            if not armed:
                landed.append(idx)
            else:
                still.append(idx)
        pending = still
        if pending:
            time.sleep(3)
    print("=== 落地/disarm 汇总 ===")
    print("disarmed: %s" % sorted(landed))
    print("still_armed: %s" % sorted(pending))
    return 0 if not pending else 3


if __name__ == "__main__":
    sys.exit(main())
