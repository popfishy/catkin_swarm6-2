#!/usr/bin/env python3
"""聚焦 UAV3 prepare 执行段：与邻机（UAV4/UAV5）距离 + 速度，验证距离门禁触发。"""
import glob
import math
import os

import numpy as np
from pyulog import ULog

ROS_HOME = "/home/ub20tg/catkin_swarm6-2/.ros_home"


def load(n):
    pat = os.path.join(ROS_HOME, f"sitl_iris_{n-1}", "log", "2026-08-24", "*.ulg")
    fs = sorted(glob.glob(pat))
    if not fs:
        return None
    ulog = ULog(fs[-1])
    d = next((x for x in ulog.data_list if x.name == "vehicle_local_position"), None)
    if d is None:
        return None
    ts = np.asarray(d.data["timestamp"], dtype=np.float64) / 1e6
    X = np.asarray(d.data["y"], dtype=np.float64)
    Y = np.asarray(d.data["x"], dtype=np.float64)
    Z = -np.asarray(d.data["z"], dtype=np.float64)
    VX = np.asarray(d.data["vy"], dtype=np.float64)
    VY = np.asarray(d.data["vx"], dtype=np.float64)
    VZ = -np.asarray(d.data["vz"], dtype=np.float64)
    return ts, X, Y, Z, VX, VY, VZ


def main():
    u3 = load(3)
    if u3 is None:
        print("UAV3 no ulog"); return
    ts3, X3, Y3, Z3, VX3, VY3, VZ3 = u3
    sp3 = np.sqrt(VX3**2 + VY3**2 + VZ3**2)
    # prepare 执行段：z>6（离开 5m 悬停）首次起 45s
    idx = np.where(Z3 > 6.0)[0]
    if not len(idx):
        print("UAV3 未起飞"); return
    t0 = ts3[idx[0]]
    sel3 = (ts3 >= t0 - 2.0) & (ts3 - t0 <= 45.0)
    t_rel = ts3[sel3] - t0
    X3s, Y3s, Z3s, sp3s = X3[sel3], Y3[sel3], Z3[sel3], sp3[sel3]

    print("=== UAV3 prepare 执行段（z>6 起 45s）===")
    print(f"UAV3: vmax={sp3s.max():.2f} m/s, 轨迹末位=({X3s[-1]:.1f},{Y3s[-1]:.1f},{Z3s[-1]:.1f})")
    # 每 5s 采样 UAV3 轨迹
    for i in range(0, len(t_rel), max(1, int(5/(t_rel[1]-t_rel[0])))):
        print(f"  t={t_rel[i]:5.1f}s pos=({X3s[i]:7.1f},{Y3s[i]:7.1f},{Z3s[i]:5.1f}) sp={sp3s[i]:4.2f}")

    print("\n=== UAV3 与邻机距离（prepare 段，采样对齐）===")
    for n in (4, 5, 6, 2):
        r = load(n)
        if r is None:
            continue
        tsn, Xn, Yn, Zn, _, _, _ = r
        # 对齐 UAV3 的 t0：邻机同一时刻
        Xni = np.interp(t_rel + t0, tsn, Xn)
        Yni = np.interp(t_rel + t0, tsn, Yn)
        Zni = np.interp(t_rel + t0, tsn, Zn)
        dh = np.sqrt((X3s - Xni)**2 + (Y3s - Yni)**2)
        dv = np.abs(Z3s - Zni)
        d3 = np.sqrt(dh**2 + dv**2)
        i = int(np.argmin(d3))
        # 门禁判定：水平<1 且垂直<2
        gate = dh < 1.0
        gate2 = (dh < 1.0) & (dv < 2.0)
        print(f"UAV3-UAV{n}: min3D={d3[i]:.3f}m @{t_rel[i]:.1f}s  "
              f"min_horiz={dh.min():.3f}m  gate(hz<1)命中={gate.sum()}/{(dh<1.0).sum() if False else len(dh)}  "
              f"full-gate(hz<1&vz<2)={gate2.sum()}次")
        # 最近时刻细节
        if gate2.any():
            j = int(np.argmax(gate2))
            print(f"    最近命中 @{t_rel[j]:.1f}s UAV3=({X3s[j]:.1f},{Y3s[j]:.1f},{Z3s[j]:.1f}) "
                  f"UAV{n}=({Xni[j]:.1f},{Yni[j]:.1f},{Zni[j]:.1f}) dh={dh[j]:.2f} dv={dv[j]:.2f}")


if __name__ == "__main__":
    main()
