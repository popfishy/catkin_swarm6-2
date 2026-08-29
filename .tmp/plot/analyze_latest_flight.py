#!/usr/bin/env python3
"""Offline PX4 ULog analysis for the latest coherent swarm flight."""

import argparse
import csv
import json
import math
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
from pyulog import ULog

DATASETS = [
    "vehicle_local_position",
    "vehicle_gps_position",
    "trajectory_setpoint",
    "vehicle_status",
]
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
PEER_RE = re.compile(r"peer=\{([^}]*)\}")
STAMP_RE = re.compile(r"\[(\d{10}(?:\.\d+)?)\]")


@dataclass
class VehicleData:
    name: str
    ulog_path: Path
    utc_offset_s: float
    t: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    sp_t: np.ndarray
    sp_pos: np.ndarray
    sp_vel: np.ndarray
    sp_acc: np.ndarray
    status_t: np.ndarray
    nav_state: np.ndarray
    arming_state: np.ndarray
    failsafe: np.ndarray


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (np.floating, float)):
        return finite_float(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, Path):
        return str(value)
    return value


def parse_ulog_start(path: Path) -> datetime:
    date = datetime.strptime(path.parent.name, "%Y-%m-%d").date()
    time = datetime.strptime(path.stem, "%H_%M_%S").time()
    return datetime.combine(date, time, tzinfo=timezone.utc)


def discover_cohort(workspace: Path, date_filter: Optional[str]) -> Dict[str, Path]:
    roots = sorted(
        workspace.glob(".ros_home/sitl_iris_*"),
        key=lambda p: int(p.name.rsplit("_", 1)[1]),
    )
    if not roots:
        raise RuntimeError("no .ros_home/sitl_iris_* directories found")

    candidates: Dict[int, List[Path]] = {}
    for root in roots:
        index = int(root.name.rsplit("_", 1)[1])
        pattern = f"log/{date_filter}/*.ulg" if date_filter else "log/*/*.ulg"
        candidates[index] = sorted(root.glob(pattern), key=parse_ulog_start)
        if not candidates[index]:
            raise RuntimeError(f"no ULog files found for {root.name}")

    anchors = reversed(candidates[min(candidates)])
    for anchor in anchors:
        anchor_time = parse_ulog_start(anchor).timestamp()
        cohort = {"UAV1": anchor}
        coherent = True
        for index in sorted(candidates):
            if index == 0:
                continue
            nearest = min(
                candidates[index],
                key=lambda p: abs(parse_ulog_start(p).timestamp() - anchor_time),
            )
            if abs(parse_ulog_start(nearest).timestamp() - anchor_time) > 3.0:
                coherent = False
                break
            cohort[f"UAV{index + 1}"] = nearest
        if coherent:
            return cohort
    raise RuntimeError("no coherent ULog cohort found within 3 seconds")


def dataset(ulog: ULog, name: str):
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError):
        return None


def array_field(data, name: str, length: int, fill=np.nan):
    if data is None or name not in data:
        return np.full(length, fill, dtype=float)
    return np.asarray(data[name], dtype=float)


def ned_to_enu(vectors: np.ndarray) -> np.ndarray:
    if vectors.size == 0:
        return vectors.reshape((-1, 3))
    return np.column_stack((vectors[:, 1], vectors[:, 0], -vectors[:, 2]))


def valid_utc_offset(gps, warnings: List[str], vehicle: str) -> float:
    if gps is None:
        warnings.append(f"{vehicle}: vehicle_gps_position missing; boot time cannot align to epoch")
        return 0.0
    timestamp = np.asarray(gps.get("timestamp", []), dtype=float)
    utc = np.asarray(gps.get("time_utc_usec", []), dtype=float)
    mask = np.isfinite(timestamp) & np.isfinite(utc) & (utc > 1e15)
    if not np.any(mask):
        warnings.append(f"{vehicle}: no valid GPS UTC samples; boot time cannot align to epoch")
        return 0.0
    offsets = (utc[mask] - timestamp[mask]) / 1e6
    return float(np.median(offsets))


def load_vehicle(name: str, path: Path, warnings: List[str]) -> VehicleData:
    ulog = ULog(str(path), message_name_filter_list=DATASETS)
    local = dataset(ulog, "vehicle_local_position")
    if local is None:
        raise RuntimeError(f"{name}: vehicle_local_position missing in {path}")
    gps = dataset(ulog, "vehicle_gps_position")
    offset = valid_utc_offset(gps, warnings, name)

    local_n = len(local["timestamp"])
    t = np.asarray(local["timestamp"], dtype=float) / 1e6 + offset
    pos = ned_to_enu(np.column_stack([
        array_field(local, "x", local_n),
        array_field(local, "y", local_n),
        array_field(local, "z", local_n),
    ]))
    vel = ned_to_enu(np.column_stack([
        array_field(local, "vx", local_n),
        array_field(local, "vy", local_n),
        array_field(local, "vz", local_n),
    ]))
    acc = ned_to_enu(np.column_stack([
        array_field(local, "ax", local_n),
        array_field(local, "ay", local_n),
        array_field(local, "az", local_n),
    ]))

    sp = dataset(ulog, "trajectory_setpoint")
    if sp is None:
        warnings.append(f"{name}: trajectory_setpoint missing")
        sp_t = np.empty(0)
        sp_pos = sp_vel = sp_acc = np.empty((0, 3))
    else:
        sp_n = len(sp["timestamp"])
        sp_t = np.asarray(sp["timestamp"], dtype=float) / 1e6 + offset
        sp_pos = ned_to_enu(np.column_stack([
            array_field(sp, "x", sp_n), array_field(sp, "y", sp_n), array_field(sp, "z", sp_n)
        ]))
        sp_vel = ned_to_enu(np.column_stack([
            array_field(sp, "vx", sp_n), array_field(sp, "vy", sp_n), array_field(sp, "vz", sp_n)
        ]))
        sp_acc = ned_to_enu(np.column_stack([
            array_field(sp, "acceleration[0]", sp_n),
            array_field(sp, "acceleration[1]", sp_n),
            array_field(sp, "acceleration[2]", sp_n),
        ]))

    status = dataset(ulog, "vehicle_status")
    if status is None:
        warnings.append(f"{name}: vehicle_status missing")
        status_t = nav_state = arming_state = failsafe = np.empty(0)
    else:
        status_n = len(status["timestamp"])
        status_t = np.asarray(status["timestamp"], dtype=float) / 1e6 + offset
        nav_state = array_field(status, "nav_state", status_n)
        arming_state = array_field(status, "arming_state", status_n)
        failsafe = array_field(status, "failsafe", status_n)

    return VehicleData(
        name, path, offset, t, pos, vel, acc,
        sp_t, sp_pos, sp_vel, sp_acc,
        status_t, nav_state, arming_state, failsafe,
    )


def window_vehicle(vehicle: VehicleData, start: float, end: float) -> VehicleData:
    local_mask = (vehicle.t >= start) & (vehicle.t <= end)
    sp_mask = (vehicle.sp_t >= start) & (vehicle.sp_t <= end)
    status_mask = (vehicle.status_t >= start) & (vehicle.status_t <= end)
    return VehicleData(
        vehicle.name, vehicle.ulog_path, vehicle.utc_offset_s,
        vehicle.t[local_mask], vehicle.pos[local_mask], vehicle.vel[local_mask], vehicle.acc[local_mask],
        vehicle.sp_t[sp_mask], vehicle.sp_pos[sp_mask], vehicle.sp_vel[sp_mask], vehicle.sp_acc[sp_mask],
        vehicle.status_t[status_mask], vehicle.nav_state[status_mask],
        vehicle.arming_state[status_mask], vehicle.failsafe[status_mask],
    )


def load_prepare_targets(plan_path: Path, heading: float) -> Dict[str, np.ndarray]:
    document = json.loads(plan_path.read_text())
    prepare = next(plan for plan in document["plans"] if plan["plan_id"] == "prepare-a")
    targets = {}
    for task_id, assignment in prepare["robot_assignments"].items():
        pose = assignment["payload"]["target_pose"]
        right, forward, up = float(pose["x"]), float(pose["y"]), float(pose["z"])
        east = right * math.sin(heading) + forward * math.cos(heading)
        north = -right * math.cos(heading) + forward * math.sin(heading)
        targets[f"UAV{int(task_id[1:])}"] = np.array([east, north, up], dtype=float)
    return targets


def contiguous_intervals(t: np.ndarray, condition: np.ndarray) -> List[Dict[str, float]]:
    if len(t) == 0 or not np.any(condition):
        return []
    indices = np.flatnonzero(condition)
    if len(t) > 1:
        typical_dt = float(np.nanmedian(np.diff(t)))
    else:
        typical_dt = 0.0
    breaks = np.flatnonzero(np.diff(indices) > 1)
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, len(indices) - 1]
    result = []
    for first_i, last_i in zip(starts, ends):
        first = indices[first_i]
        last = indices[last_i]
        duration = max(0.0, float(t[last] - t[first] + typical_dt))
        result.append({"start_epoch": float(t[first]), "end_epoch": float(t[last]), "duration_s": duration})
    return result


def norm_rows(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return np.empty(0)
    finite = np.all(np.isfinite(values), axis=1)
    output = np.full(len(values), np.nan)
    output[finite] = np.linalg.norm(values[finite], axis=1)
    return output


def first_pva_to_hold(vehicle: VehicleData, goal_epoch: float) -> Optional[float]:
    if len(vehicle.sp_t) < 2:
        return None
    pos_finite = np.all(np.isfinite(vehicle.sp_pos), axis=1)
    vel_finite = np.all(np.isfinite(vehicle.sp_vel), axis=1)
    candidates = np.flatnonzero(vel_finite[:-1] & ~vel_finite[1:] & pos_finite[1:]) + 1
    candidates = candidates[vehicle.sp_t[candidates] >= goal_epoch]
    return float(vehicle.sp_t[candidates[0]]) if len(candidates) else None


def interpolate_positions(vehicle: VehicleData, grid: np.ndarray) -> np.ndarray:
    finite = np.isfinite(vehicle.t) & np.all(np.isfinite(vehicle.pos), axis=1)
    if np.count_nonzero(finite) < 2:
        return np.full((len(grid), 3), np.nan)
    t = vehicle.t[finite]
    pos = vehicle.pos[finite]
    output = np.column_stack([np.interp(grid, t, pos[:, axis]) for axis in range(3)])
    output[(grid < t[0]) | (grid > t[-1])] = np.nan
    return output


def parse_key_values(text: str, separator: str = "=") -> Dict[str, str]:
    values = {}
    for token in text.split():
        if separator in token:
            key, value = token.split(separator, 1)
            values[key] = value.strip(",")
    return values


def parse_structured_logs(
    workspace: Path, vehicles: Sequence[str], start: float, end: float
) -> Tuple[List[Dict], Dict[str, Dict], Dict[str, List[Tuple[float, str]]]]:
    events: List[Dict] = []
    summaries: Dict[str, Dict] = {}
    level_series: Dict[str, List[Tuple[float, str]]] = {}
    for vehicle in vehicles:
        path = workspace / "runtime_logs/ego_planner" / f"{vehicle}-ego-planner.log"
        if not path.exists():
            continue
        records = []
        peer_min = {}
        for raw_line in path.open(errors="replace"):
            line = raw_line.strip()
            top = parse_key_values(PEER_RE.sub("", line))
            stamp = finite_float(top.get("ros_s"))
            if stamp is None or not (start <= stamp <= end):
                continue
            record = {
                "time_epoch": stamp,
                "state": top.get("state", "UNKNOWN"),
                "level": top.get("level", "UNKNOWN"),
                "primary_peer": top.get("primary_peer", "none"),
                "episode": top.get("episode", "none"),
                "action": top.get("action", "none"),
            }
            records.append(record)
            for peer_text in PEER_RE.findall(line):
                peer = parse_key_values(peer_text.replace(",", " "), separator=":")
                peer_id = peer.get("id")
                min_d = finite_float(peer.get("min_d"))
                if peer_id and min_d is not None:
                    current = peer_min.get(peer_id)
                    if current is None or min_d < current["distance_m"]:
                        peer_min[peer_id] = {
                            "distance_m": min_d,
                            "time_epoch": stamp,
                            "level": peer.get("level"),
                            "warning_m": finite_float(peer.get("warning_d")),
                            "emergency_m": finite_float(peer.get("emergency_d")),
                        }
        level_series[vehicle] = [(r["time_epoch"], r["level"]) for r in records]
        if records:
            interval_start = records[0]["time_epoch"]
            previous = tuple(records[0][key] for key in ("state", "level", "primary_peer", "episode", "action"))
            for record in records[1:]:
                current = tuple(record[key] for key in ("state", "level", "primary_peer", "episode", "action"))
                if current != previous:
                    events.append({
                        "source": "ego_structured_measured",
                        "vehicle": vehicle,
                        "start_epoch": interval_start,
                        "end_epoch": record["time_epoch"],
                        "state": previous[0], "level": previous[1], "peer": previous[2],
                        "episode": previous[3], "action": previous[4],
                    })
                    interval_start = record["time_epoch"]
                    previous = current
            events.append({
                "source": "ego_structured_measured",
                "vehicle": vehicle,
                "start_epoch": interval_start,
                "end_epoch": records[-1]["time_epoch"],
                "state": previous[0], "level": previous[1], "peer": previous[2],
                "episode": previous[3], "action": previous[4],
            })
            summaries[vehicle] = {
                "first_epoch": records[0]["time_epoch"],
                "last_epoch": records[-1]["time_epoch"],
                "levels": sorted(set(r["level"] for r in records)),
                "states": sorted(set(r["state"] for r in records)),
                "peer_minima": peer_min,
            }
    return events, summaries, level_series


def parse_console_events(workspace: Path, vehicles: Iterable[str], start: float, end: float) -> List[Dict]:
    keywords = (
        "yield committed", "yield replan", "replan failed", "EGO_PLAN_FAILED",
        "EMERGENCY", "BRAKE_HOLD", "MIN_DISTANCE", "RESOURCE_LIMIT", "bad_alloc",
    )
    events = []
    for vehicle in vehicles:
        path = workspace / ".tmp/logs" / f"{vehicle}_offboard_ego.log"
        if not path.exists():
            continue
        for raw_line in path.open(errors="replace"):
            line = ANSI_RE.sub("", raw_line.strip())
            if not any(keyword in line for keyword in keywords):
                continue
            match = STAMP_RE.search(line)
            if not match:
                continue
            stamp = float(match.group(1))
            if start <= stamp <= end:
                events.append({
                    "source": "ros_console_measured", "vehicle": vehicle,
                    "start_epoch": stamp, "end_epoch": stamp, "message": line,
                })
    return events


def vehicle_metrics(vehicle: VehicleData, target: Optional[np.ndarray], goal: float) -> Dict:
    speed = norm_rows(vehicle.vel)
    acceleration = norm_rows(vehicle.acc)
    metrics = {
        "ulog": str(vehicle.ulog_path),
        "utc_offset_s": vehicle.utc_offset_s,
        "samples": len(vehicle.t),
        "start_epoch": float(vehicle.t[0]) if len(vehicle.t) else None,
        "end_epoch": float(vehicle.t[-1]) if len(vehicle.t) else None,
        "max_speed_mps": float(np.nanmax(speed)) if np.any(np.isfinite(speed)) else None,
        "max_acceleration_mps2": float(np.nanmax(acceleration)) if np.any(np.isfinite(acceleration)) else None,
        "pva_to_position_hold_epoch": first_pva_to_hold(vehicle, goal),
    }
    if len(vehicle.sp_t) > 1:
        gaps = np.diff(vehicle.sp_t)
        finite_gaps = gaps[np.isfinite(gaps)]
        metrics["setpoint_intervals"] = {
            "count": int(len(finite_gaps)),
            "median_s": float(np.median(finite_gaps)) if len(finite_gaps) else None,
            "p95_s": float(np.percentile(finite_gaps, 95)) if len(finite_gaps) else None,
            "max_s": float(np.max(finite_gaps)) if len(finite_gaps) else None,
            "gaps_ge_0_2_s": int(np.count_nonzero(finite_gaps >= 0.2)),
        }
    if target is not None and len(vehicle.t):
        error = np.linalg.norm(vehicle.pos - target, axis=1)
        finite = np.isfinite(error)
        if np.any(finite):
            finite_indices = np.flatnonzero(finite)
            min_index = finite_indices[np.argmin(error[finite])]
            metrics.update({
                "target_enu_m": target.tolist(),
                "minimum_target_error_m": float(error[min_index]),
                "minimum_target_error_epoch": float(vehicle.t[min_index]),
                "final_target_error_m": float(error[finite_indices[-1]]),
                "within_0_2_m_intervals": contiguous_intervals(vehicle.t, finite & (error <= 0.2)),
                "within_0_5_m_intervals": contiguous_intervals(vehicle.t, finite & (error <= 0.5)),
            })
    return metrics


def build_pair_metrics(
    vehicles: Dict[str, VehicleData], grid: np.ndarray
) -> Tuple[Dict[str, Dict], np.ndarray, List[str], Dict[str, np.ndarray]]:
    interpolated = {name: interpolate_positions(vehicle, grid) for name, vehicle in vehicles.items()}
    pair_metrics = {}
    envelope = np.full(len(grid), np.nan)
    envelope_pair = np.full(len(grid), "", dtype=object)
    traces = {}
    for first, second in combinations(sorted(vehicles, key=lambda name: int(name[3:])), 2):
        delta = interpolated[first] - interpolated[second]
        horizontal = np.linalg.norm(delta[:, :2], axis=1)
        distance = np.linalg.norm(delta, axis=1)
        valid = np.isfinite(distance)
        if not np.any(valid):
            continue
        indices = np.flatnonzero(valid)
        minimum_index = indices[np.argmin(distance[valid])]
        key = f"{first}-{second}"
        finite_distance = distance[valid]
        pair_metrics[key] = {
            "minimum_3d_m": float(distance[minimum_index]),
            "minimum_horizontal_m": float(horizontal[minimum_index]),
            "minimum_epoch": float(grid[minimum_index]),
            "duration_below_1_0_s": float(np.count_nonzero(finite_distance < 1.0) * (grid[1] - grid[0])),
            "duration_below_1_1_s": float(np.count_nonzero(finite_distance < 1.1) * (grid[1] - grid[0])),
            "duration_below_1_35_s": float(np.count_nonzero(finite_distance < 1.35) * (grid[1] - grid[0])),
        }
        traces[key] = np.column_stack((horizontal, distance))
        candidate = np.where(valid, distance, np.inf)
        current = np.where(np.isfinite(envelope), envelope, np.inf)
        replace = candidate < current
        envelope[replace] = distance[replace]
        envelope_pair[replace] = key
    return pair_metrics, envelope, envelope_pair.tolist(), traces


def write_events(path: Path, events: List[Dict]):
    fields = [
        "source", "vehicle", "start_epoch", "end_epoch", "relative_start_s",
        "relative_end_s", "state", "level", "peer", "episode", "action", "message",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(events)


def plot_overview(
    path: Path, vehicles: Dict[str, VehicleData], targets: Dict[str, np.ndarray],
    metrics: Dict, grid: np.ndarray, envelope: np.ndarray, envelope_pair: Sequence[str],
    pair_traces: Dict[str, np.ndarray], goal: float
):
    fig = plt.figure(figsize=(16, 13))
    ax_xy = fig.add_subplot(2, 2, 1)
    ax_3d = fig.add_subplot(2, 2, 2, projection="3d")
    ax_error = fig.add_subplot(2, 2, 3)
    ax_distance = fig.add_subplot(2, 2, 4)
    colors = plt.cm.tab20(np.linspace(0, 1, len(vehicles)))

    for color, name in zip(colors, sorted(vehicles, key=lambda item: int(item[3:]))):
        vehicle = vehicles[name]
        valid = np.all(np.isfinite(vehicle.pos), axis=1)
        if np.any(valid):
            width = 2.4 if name in ("UAV1", "UAV2") else 1.0
            ax_xy.plot(vehicle.pos[valid, 0], vehicle.pos[valid, 1], color=color, lw=width, label=name)
            ax_3d.plot(vehicle.pos[valid, 0], vehicle.pos[valid, 1], vehicle.pos[valid, 2], color=color, lw=width)
        if name in targets:
            target = targets[name]
            ax_xy.scatter(target[0], target[1], marker="x", color=color, s=50)
            ax_3d.scatter(target[0], target[1], target[2], marker="x", color=color, s=45)

    ax_xy.set(title="15-UAV prepare trajectories and A01-A12 targets", xlabel="East [m]", ylabel="North [m]")
    ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(ncol=3, fontsize=8, loc="upper right")
    ax_3d.set(title="ENU trajectories", xlabel="East [m]", ylabel="North [m]", zlabel="Up [m]")

    target_names = sorted(targets, key=lambda item: int(item[3:]))
    minimums = [metrics["vehicles"][name].get("minimum_target_error_m", np.nan) for name in target_names]
    finals = [metrics["vehicles"][name].get("final_target_error_m", np.nan) for name in target_names]
    x = np.arange(len(target_names))
    ax_error.bar(x - 0.2, minimums, width=0.4, label="minimum")
    ax_error.bar(x + 0.2, finals, width=0.4, label="window-end")
    ax_error.axhline(0.2, color="tab:red", ls="--", label="completion 0.2 m")
    ax_error.axhline(0.5, color="tab:orange", ls=":", label="advance 0.5 m")
    ax_error.set(title="Target error by participant", ylabel="3D error [m]", xticks=x, xticklabels=target_names)
    ax_error.tick_params(axis="x", rotation=45)
    ax_error.grid(True, axis="y", alpha=0.3)
    ax_error.legend()

    relative = grid - goal
    # 所有任意两架无人机之间的 3D 距离-时间轨迹（不加图例，浅灰弱化）
    for trace_key, trace in pair_traces.items():
        ax_distance.plot(relative, trace[:, 1], color="0.75", lw=0.5, alpha=0.6)
    # nearest actual 3D distance 包络：黑色普通线宽（区分浅灰对轨迹）
    ax_distance.plot(relative, envelope, color="black", lw=1.2, label="nearest actual pair")
    for threshold, color, label in [(1.35, "tab:orange", "warning 1.35"), (1.1, "tab:red", "emergency 1.1"), (1.0, "darkred", "runtime 1.0")]:
        ax_distance.axhline(threshold, color=color, ls="--", label=label)
    finite = np.isfinite(envelope)
    if np.any(finite):
        index = np.nanargmin(envelope)
        ax_distance.scatter(relative[index], envelope[index], color="red", zorder=5)
        ax_distance.annotate(f"{envelope[index]:.3f} m\n{envelope_pair[index]}", (relative[index], envelope[index]), xytext=(8, 8), textcoords="offset points")
    ax_distance.set(title="Swarm pairwise actual 3D distance (all pairs + nearest)", xlabel="Seconds from backend goal", ylabel="Distance [m]")
    ax_distance.set_ylim(bottom=0.0, top=7.0)
    ax_distance.grid(True, alpha=0.3)
    ax_distance.legend(fontsize=8)

    fig.suptitle("Latest coherent flight cohort: measured PX4 ULog data", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_focus(
    path: Path, vehicles: Dict[str, VehicleData], targets: Dict[str, np.ndarray],
    metrics: Dict, grid: np.ndarray, pair_trace: np.ndarray,
    level_series: Dict[str, List[Tuple[float, str]]], events: List[Dict], goal: float,
    inferred_failure: float, focus: Tuple[str, str]
):
    first_name, second_name = focus
    first, second = vehicles[first_name], vehicles[second_name]
    fig, axes = plt.subplots(4, 2, figsize=(17, 18))
    ax_xy, ax_error, ax_distance, ax_speed, ax_acc, ax_gap, ax_safety, ax_event = axes.flat
    colors = {first_name: "tab:blue", second_name: "tab:orange"}

    for name, vehicle in ((first_name, first), (second_name, second)):
        rel = vehicle.t - goal
        valid = np.all(np.isfinite(vehicle.pos), axis=1)
        ax_xy.plot(vehicle.pos[valid, 0], vehicle.pos[valid, 1], color=colors[name], label=name)
        target = targets[name]
        ax_xy.scatter(target[0], target[1], color=colors[name], marker="x", s=80)
        error = np.linalg.norm(vehicle.pos - target, axis=1)
        ax_error.plot(rel, error, color=colors[name], label=name)
        ax_speed.plot(rel, norm_rows(vehicle.vel), color=colors[name], label=name)
        ax_acc.plot(rel, norm_rows(vehicle.acc), color=colors[name], label=name)
        if len(vehicle.sp_t) > 1:
            ax_gap.plot(vehicle.sp_t[1:] - goal, np.diff(vehicle.sp_t), color=colors[name], marker=".", ms=2, lw=0.7, label=name)
        hold_epoch = metrics["vehicles"][name].get("pva_to_position_hold_epoch")
        if hold_epoch is not None:
            for axis in (ax_error, ax_speed, ax_acc, ax_gap, ax_safety, ax_event):
                axis.axvline(hold_epoch - goal, color=colors[name], alpha=0.45, ls=":")

    ax_xy.set(title=f"{first_name}/{second_name} ENU trajectories", xlabel="East [m]", ylabel="North [m]")
    ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend()

    ax_error.axhline(0.2, color="tab:red", ls="--", label="completion 0.2 m")
    ax_error.axhline(0.5, color="tab:orange", ls=":", label="advance 0.5 m")
    ax_error.set(title="Actual target error", xlabel="Seconds from goal", ylabel="3D error [m]", ylim=(0, 3.0))
    ax_error.grid(True, alpha=0.3)
    ax_error.legend(ncol=2, fontsize=8)

    relative = grid - goal
    ax_distance.plot(relative, pair_trace[:, 0], label="horizontal", color="tab:green")
    ax_distance.plot(relative, pair_trace[:, 1], label="3D", color="black")
    for threshold, color, label in [(1.35, "tab:orange", "warning"), (1.1, "tab:red", "emergency"), (1.0, "darkred", "runtime")]:
        ax_distance.axhline(threshold, color=color, ls="--", label=f"{label} {threshold}")
    ax_distance.set(title=f"{first_name}-{second_name} actual separation", xlabel="Seconds from goal", ylabel="Distance [m]", ylim=(0.8, 8.0))
    ax_distance.grid(True, alpha=0.3)
    ax_distance.legend(fontsize=8)

    ax_speed.set(title="Actual speed", xlabel="Seconds from goal", ylabel="Speed [m/s]")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()
    ax_acc.set(title="Actual filtered acceleration", xlabel="Seconds from goal", ylabel="Acceleration [m/s²]")
    ax_acc.grid(True, alpha=0.3)
    ax_acc.legend()
    ax_gap.axhline(0.2, color="tab:red", ls="--", label="0.2 s gate")
    ax_gap.set(title="PX4 trajectory_setpoint intervals (not ROS relay proof)", xlabel="Seconds from goal", ylabel="Interval [s]", ylim=(0, 0.25))
    ax_gap.grid(True, alpha=0.3)
    ax_gap.legend(fontsize=8)

    level_map = {"INSUFFICIENT_DATA": 0, "SAFE": 1, "WARNING": 2, "EMERGENCY": 3}
    for name in focus:
        series = level_series.get(name, [])
        if series:
            times = np.array([item[0] - goal for item in series])
            levels = np.array([level_map.get(item[1], -1) for item in series])
            ax_safety.step(times, levels, where="post", color=colors[name], label=name)
    ax_safety.set(title="Structured SafetyPredictor level", xlabel="Seconds from goal", ylabel="Level", yticks=list(level_map.values()), yticklabels=list(level_map.keys()))
    ax_safety.grid(True, alpha=0.3)
    ax_safety.legend()

    event_rows = [event for event in events if event.get("vehicle") in focus and event["source"] != "ego_structured_measured"]
    y_labels = [first_name, second_name, "inferred"]
    for event in event_rows:
        y = 0 if event["vehicle"] == first_name else 1
        x = event["start_epoch"] - goal
        ax_event.scatter(x, y, marker="|", s=150, color=colors[event["vehicle"]])
        message = event.get("message", "")
        short = "yield" if "yield" in message else "replan fail" if "replan failed" in message else "event"
        ax_event.annotate(short, (x, y), xytext=(3, 5), textcoords="offset points", rotation=35, fontsize=7)
    inferred_rel = inferred_failure - goal
    ax_event.scatter(inferred_rel, 2, marker="X", s=80, color="red")
    ax_event.annotate("inferred result: ticks/20 Hz\n(no task-state rosbag)", (inferred_rel, 2), xytext=(-90, -30), textcoords="offset points", fontsize=8)
    ax_event.set(title="Measured ROS events and inferred task-result time", xlabel="Seconds from goal", yticks=[0, 1, 2], yticklabels=y_labels, ylim=(-0.7, 2.7))
    ax_event.grid(True, axis="x", alpha=0.3)

    fig.suptitle(f"{focus[0]}/{focus[1]} focus investigation — measured facts separated from inference", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=170)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/home/ub20tg/catkin_swarm6-2"))
    parser.add_argument("--ulog-date", help="restrict cohort discovery to YYYY-MM-DD")
    parser.add_argument("--heading-rad", type=float, default=-0.006)
    parser.add_argument("--goal-epoch", type=float, default=1787900129.041)
    parser.add_argument("--task-ticks", type=int, default=1605)
    parser.add_argument("--tick-rate", type=float, default=20.0)
    parser.add_argument("--window-before-s", type=float, default=5.0)
    parser.add_argument("--window-after-s", type=float, default=160.0)
    parser.add_argument("--grid-step-s", type=float, default=0.05)
    parser.add_argument("--whole-flight", action="store_true")
    parser.add_argument("--uavs", default="UAV4,UAV6", help="focus vehicles for the pair-focus plot")
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    workspace = args.workspace.resolve()
    output_dir = (args.output_dir or workspace / ".tmp/plot").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    warnings: List[str] = []

    cohort = discover_cohort(workspace, args.ulog_date)
    print(f"Selected coherent cohort with {len(cohort)} ULogs")
    loaded = {}
    for name in sorted(cohort, key=lambda item: int(item[3:])):
        print(f"Loading {name}: {cohort[name]}")
        loaded[name] = load_vehicle(name, cohort[name], warnings)

    if args.whole_flight:
        start = max(min(vehicle.t) for vehicle in loaded.values() if len(vehicle.t))
        end = min(max(vehicle.t) for vehicle in loaded.values() if len(vehicle.t))
    else:
        start = args.goal_epoch - args.window_before_s
        end = args.goal_epoch + args.window_after_s
    vehicles = {name: window_vehicle(vehicle, start, end) for name, vehicle in loaded.items()}
    empty = [name for name, vehicle in vehicles.items() if len(vehicle.t) < 2]
    if empty:
        raise RuntimeError(f"no local-position data in selected window for: {', '.join(empty)}")

    plan_path = workspace / "references/runtime/examples/joint_mission/group_a/plans/plans.json"
    targets = load_prepare_targets(plan_path, args.heading_rad)
    inferred_failure = args.goal_epoch + args.task_ticks / args.tick_rate

    common_start = max(start, max(vehicle.t[0] for vehicle in vehicles.values()))
    common_end = min(end, min(vehicle.t[-1] for vehicle in vehicles.values()))
    if common_end <= common_start:
        raise RuntimeError("vehicles do not share a common analysis window")
    grid = np.arange(common_start, common_end + args.grid_step_s * 0.5, args.grid_step_s)
    pair_metrics, envelope, envelope_pair, pair_traces = build_pair_metrics(vehicles, grid)

    focus = tuple(name.strip().upper() for name in args.uavs.split(",") if name.strip())
    if len(focus) != 2 or len(set(focus)) != 2:
        raise RuntimeError("--uavs must contain exactly two distinct names, e.g. UAV4,UAV6")
    missing_focus = [name for name in focus if name not in vehicles]
    missing_targets = [name for name in focus if name not in targets]
    if missing_focus:
        raise RuntimeError(f"focus ULogs missing: {', '.join(missing_focus)}")
    if missing_targets:
        raise RuntimeError(f"focus prepare targets missing: {', '.join(missing_targets)}")
    focus_pair_key = "-".join(sorted(focus, key=lambda item: int(item[3:])))
    focus_first = focus[0]

    all_vehicle_names = sorted(vehicles, key=lambda item: int(item[3:]))
    structured_events, structured_summary, level_series = parse_structured_logs(
        workspace, all_vehicle_names, start, end
    )
    console_events = parse_console_events(workspace, focus, start, end)
    events = structured_events + console_events
    events.append({
        "source": "task_result_inferred",
        "vehicle": f"A{int(focus_first[3:]):02d}/UAV{focus_first[3:]}",
        "start_epoch": inferred_failure, "end_epoch": inferred_failure,
        "message": f"inferred from {args.task_ticks} ticks / {args.tick_rate:g} Hz; no task-state rosbag",
    })
    for event in events:
        event["relative_start_s"] = event["start_epoch"] - args.goal_epoch
        event["relative_end_s"] = event["end_epoch"] - args.goal_epoch
    events.sort(key=lambda event: (event["start_epoch"], event.get("vehicle", "")))

    vehicle_results = {
        name: vehicle_metrics(vehicle, targets.get(name), args.goal_epoch)
        for name, vehicle in vehicles.items()
    }
    uav12_pair = pair_metrics.get("UAV1-UAV2", {})  # 全局 UAV1-UAV2 对指标（与 focus 无关）
    focus_intervals = vehicle_results[focus_first].get("within_0_2_m_intervals", [])
    stable_before_inferred_failure = {}
    for name in sorted(targets, key=lambda item: int(item[3:])):
        intervals = vehicle_results[name].get("within_0_2_m_intervals", [])
        stable_before_inferred_failure[name] = any(
            interval["start_epoch"] <= inferred_failure
            and min(interval["end_epoch"], inferred_failure) - interval["start_epoch"] >= 1.0
            for interval in intervals
        )
    focus_at_failure = interpolate_positions(vehicles[focus_first], np.array([inferred_failure]))[0]
    focus_error_at_failure = (
        float(np.linalg.norm(focus_at_failure - targets[focus_first]))
        if np.all(np.isfinite(focus_at_failure)) else None
    )
    predicted_uav1_uav2 = None
    for observer, peer in (("UAV1", "UAV2"), ("UAV2", "UAV1")):
        candidate = structured_summary.get(observer, {}).get("peer_minima", {}).get(peer)
        if candidate and (predicted_uav1_uav2 is None or candidate["distance_m"] < predicted_uav1_uav2["distance_m"]):
            predicted_uav1_uav2 = dict(candidate, observer=observer, peer=peer)

    global_predicted_minimum = None
    for observer, summary in structured_summary.items():
        for peer, candidate in summary.get("peer_minima", {}).items():
            if global_predicted_minimum is None or candidate["distance_m"] < global_predicted_minimum["distance_m"]:
                global_predicted_minimum = dict(candidate, observer=observer, peer=peer)
    structured_any_emergency = any(
        "EMERGENCY" in summary.get("levels", []) for summary in structured_summary.values()
    )

    diagnosis = {
        "actual_uav1_uav2_minimum_3d_m": uav12_pair.get("minimum_3d_m"),
        "actual_uav1_uav2_minimum_epoch": uav12_pair.get("minimum_epoch"),
        "actual_runtime_1m_breached": bool(uav12_pair and uav12_pair["minimum_3d_m"] < 1.0),
        "predicted_uav1_uav2_minimum": predicted_uav1_uav2,
        "global_predicted_minimum": global_predicted_minimum,
        "structured_any_emergency": structured_any_emergency,
        "focus_0_2m_intervals": focus_intervals,
        "stable_0_2m_for_1s_before_inferred_failure": stable_before_inferred_failure,
        "focus_target_error_at_inferred_failure_m": focus_error_at_failure,
        "focus_structured_last_epoch": structured_summary.get(focus_first, {}).get("last_epoch"),
        "inferred_task_result_epoch": inferred_failure,
        "inference_basis": f"goal + {args.task_ticks} ticks / {args.tick_rate:g} Hz; exact task-state timestamp unavailable",
        "console_has_emergency_or_distance_breach": any(
            any(token in event.get("message", "") for token in ("EMERGENCY", "BRAKE_HOLD", "MIN_DISTANCE"))
            for event in console_events
        ),
        "most_supported_explanation": (
            f"{focus_first} did not continuously satisfy the 0.2 m / 1.0 s completion latch before the "
            "inferred task result; endpoint HOLD was mapped to COMMAND_HELD, while physical settling continued afterward."
        ),
    }

    metrics = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "provenance": {
            "workspace": str(workspace),
            "cohort": {name: str(path) for name, path in cohort.items()},
            "cohort_start_utc": parse_ulog_start(cohort["UAV1"]).isoformat(),
            "goal_epoch": args.goal_epoch,
            "goal_cst": datetime.fromtimestamp(args.goal_epoch, timezone.utc).astimezone().isoformat(),
            "analysis_window_epoch": [start, end],
            "target_plan": str(plan_path),
            "target_plan_id": "prepare-a",
            "target_transform": "E=right*sin(h)+forward*cos(h), N=-right*cos(h)+forward*sin(h), U=up",
            "heading_rad": args.heading_rad,
            "target_participants": sorted(targets, key=lambda item: int(item[3:])),
            "note": "A13-A15 have physical ULogs but no prepare-a target assignment",
        },
        "warnings": warnings + [
            "PX4 trajectory_setpoint cadence is not definitive proof of the ROS relay publication rate.",
            f"No complete task-state rosbag exists for this cohort; the {focus_first} result time is inferred.",
            "headless_test/07_task_prepare.log belongs to an older command and was excluded.",
        ],
        "vehicles": vehicle_results,
        "pairs": pair_metrics,
        "structured_planner": structured_summary,
        "diagnosis": diagnosis,
    }

    metrics_path = output_dir / "latest_flight_metrics.json"
    metrics_path.write_text(json.dumps(json_safe(metrics), ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    write_events(output_dir / "latest_flight_events.csv", events)
    plot_overview(output_dir / "latest_flight_overview.png", vehicles, targets, metrics, grid, envelope, envelope_pair, pair_traces, args.goal_epoch)
    if focus_pair_key not in pair_traces:
        raise RuntimeError(f"{focus_pair_key} pair trace missing")
    focus_path = output_dir / (
        "uav1_uav2_focus.png" if focus == ("UAV1", "UAV2")
        else f"{focus[0].lower()}_{focus[1].lower()}_focus.png"
    )
    plot_focus(
        focus_path, vehicles, targets, metrics, grid,
        pair_traces[focus_pair_key], level_series, events, args.goal_epoch, inferred_failure, focus,
    )

    print(json.dumps(json_safe({
        "metrics": metrics_path,
        "uav1_uav2_actual_min_3d_m": diagnosis["actual_uav1_uav2_minimum_3d_m"],
        "uav1_uav2_predicted_min": diagnosis["predicted_uav1_uav2_minimum"],
        "focus_0_2m_intervals": diagnosis["focus_0_2m_intervals"],
        "warnings": warnings,
    }), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
