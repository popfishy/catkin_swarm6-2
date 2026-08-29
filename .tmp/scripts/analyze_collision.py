#!/usr/bin/env python3
"""Analyze the headless prepare ulogs for collision evidence.

Reads sitl_iris_{0..14}/log/2026-08-24/*.ulg (UAV1..UAV15), extracts
local_position (x/y/z) and estimates min inter-UAV distance over time.
Prints per-pair minima below 1.5 m, plus per-UAV max speed and time span.
"""
from __future__ import annotations

import glob
import math
import os
import sys

from pyulog import ULog

WS = "/home/ub20tg/catkin_swarm6-2"


def read_traj(idx):
    files = sorted(glob.glob(
        "%s/.ros_home/sitl_iris_%d/log/2026-08-24/*.ulg" % (WS, idx)))
    if not files:
        return None, None, None, None
    ulog = ULog(files[0])
    t = None
    px = py = pz = None
    for data in ulog.data_list:
        if data.name == "vehicle_local_position":
            if "x" in data.data and "y" in data.data and "z" in data.data:
                t = [v / 1e6 for v in data.data["timestamp"]]  # us -> s
                px = data.data["x"]
                py = data.data["y"]
                pz = data.data["z"]
                break
    return t, px, py, pz


def main():
    trajs = {}
    for idx in range(15):
        t, px, py, pz = read_traj(idx)
        if px is None:
            print("UAV%d: NO ulog" % (idx + 1))
            continue
        # speed from consecutive samples (m/s)
        spd = [0.0]
        for i in range(1, len(px)):
            d = math.hypot(px[i] - px[i - 1], py[i] - py[i - 1], pz[i] - pz[i - 1])
            dt = max(t[i] - t[i - 1], 1e-6)
            spd.append(d / dt)
        vmax = max(spd)
        t_start, t_end = t[0], t[-1]
        # z range
        zmin, zmax = min(pz), max(pz)
        print("UAV%d: dur=%.0fs vmax=%.2f z[%.1f..%.1f] start=(%.1f,%.1f,%.1f) end=(%.1f,%.1f,%.1f)"
              % (idx + 1, t_end - t_start, vmax, zmin, zmax,
                 px[0], py[0], pz[0], px[-1], py[-1], pz[-1]))
        trajs[idx] = (t, px, py, pz)

    print("\n=== 机间最小距离（飞行段 z>0.5m，<1.5m 报警）===")
    found = False
    for a in range(15):
        if a not in trajs:
            continue
        for b in range(a + 1, 15):
            if b not in trajs:
                continue
            ta, xa, ya, za = trajs[a]
            tb, xb, yb, zb = trajs[b]
            n = min(len(xa), len(xb))
            dmin = 1e9
            tmin = 0.0
            for i in range(0, n, 5):  # subsample 5 to speed up
                # NED: negative z = altitude. Skip pre-takeoff ground samples
                # (z near 0) and the gp_origin re-origin jump at takeoff.
                if za[i] > -0.5 or zb[i] > -0.5:
                    continue
                d = math.hypot(xa[i] - xb[i], ya[i] - yb[i], za[i] - zb[i])
                if d < dmin:
                    dmin = d
                    tmin = ta[i]
            if dmin < 1.5:
                found = True
                print("UAV%d-UAV%d min=%.3fm @t=%.1fs" % (a + 1, b + 1, dmin, tmin))
    if not found:
        print("(no pair below 1.5 m during flight)")


if __name__ == "__main__":
    sys.exit(main())
