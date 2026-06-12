#!/usr/bin/env python3
"""Analyze an apl20 tracking-performance CSV (written by TrackingRecorder).

The reference is the shaped setpoint the controller is tracking, so this measures
how well the airframe follows the *feasible* trajectory during a mission. Prints
overall and per-waypoint position/heading error statistics (RMS, max, mean).

    python3 track_analyze.py /tmp/track.csv
"""
import csv
import sys
from collections import defaultdict
from math import sqrt


def stats(vals):
    n = len(vals)
    rms = sqrt(sum(v * v for v in vals) / n)
    return rms, max(abs(v) for v in vals), sum(vals) / n


def main(path):
    with open(path) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    if not rows:
        print("empty log")
        return 1

    pos = [r["pos_err"] for r in rows]
    yaw = [abs(r["yaw_err"]) for r in rows]
    dur = rows[-1]["t"] - rows[0]["t"]
    n_wp = int(max(r["wp"] for r in rows))
    print(f"samples={len(rows)}  duration={dur:.1f}s  waypoints={n_wp}")
    rms, mx, mn = stats(pos)
    print(f"position error [m]  : rms={rms:.3f}  max={mx:.3f}  mean={mn:.3f}")
    rms, mx, mn = stats(yaw)
    print(f"heading error [rad] : rms={rms:.3f}  max={mx:.3f}  mean={mn:.3f}")

    by_wp = defaultdict(list)
    for r in rows:
        by_wp[int(r["wp"])].append(r["pos_err"])
    print("per-waypoint position error [m]:")
    for wp in sorted(by_wp):
        rms, mx, _ = stats(by_wp[wp])
        print(f"  wp {wp:2d}: rms={rms:.3f}  max={mx:.3f}  (n={len(by_wp[wp])})")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: track_analyze.py <track.csv>")
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
