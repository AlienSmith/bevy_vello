#!/usr/bin/env python3
"""Vertical-band + horizontal summary for one walk telemetry CSV.

Recipe calibrated to reproduce the §3d/§3e table in `plans/hip-foot-strut.md`
on `fs_strut.csv` (air 46.7 %, net +502, stall 14.1 s, bands 26 43 24 17 45 127
over windows 1.5-3 / 3-5 / 5-8 / 8-11 / 11-15 / 15-21).

Fields:
  air %    fraction of ticks with vphase != 0 (non-Stand vertical phase)
  band     reality-hip (tP3y) peak-to-peak per window, after subtracting the
           window's linear trend? No — plain max-min (matches the table).
  net_x    tP3x(last) - tP3x(first)
  stall    first time the filtered measured velocity collapses (v_filt < -20
           sustained) — the walk-backwards onset
"""
import csv
import sys

WINDOWS = [(1.5, 3.0), (3.0, 5.0), (5.0, 8.0), (8.0, 11.0), (11.0, 15.0), (15.0, 21.0)]


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = [float(r["t"]) for r in rows]
    p3x = [float(r["tP3x"]) for r in rows]
    p3y = [float(r["tP3y"]) for r in rows]
    cmd = [float(r["cmd_vx"]) for r in rows]
    vf = [float(r["v_filt"]) for r in rows]
    vph = [int(float(r["vphase"])) for r in rows]
    return t, p3x, p3y, cmd, vf, vph


def summarize(path):
    t, p3x, p3y, cmd, vf, vph = load(path)
    air = 100.0 * sum(1 for p in vph if p != 0) / len(vph)
    bands = []
    for (a, b) in WINDOWS:
        ys = [y for ti, y in zip(t, p3y) if a <= ti < b]
        bands.append(int(round(max(ys) - min(ys))) if ys else -1)
    net = int(round(p3x[-1] - p3x[0]))
    # stall: first sustained (>=0.3 s) sub-(-20) v_filt
    stall = float("inf")
    for i in range(len(t)):
        if vf[i] < -20.0:
            j = i
            while j < len(t) and vf[j] < -20.0:
                j += 1
            if t[j - 1] - t[i] >= 0.3:
                stall = t[i]
                break
    # cmd_vx lower vertex in cruise (first 8 s, exclude boot ramp)
    cd = [c for ti, c in zip(t, cmd) if 2.0 <= ti <= 8.0 and c > 0]
    cd_min = min(cd) if cd else -1
    v_min = min(vf)
    print(
        f"{path.split('/')[-1]:16s} air%={air:5.1f} net_x={net:+6d} "
        f"stall={stall:5.1f}s bands={bands} cmd_min={cd_min:6.1f} v_min={v_min:7.1f}"
    )


if __name__ == "__main__":
    for p in sys.argv[1:]:
        summarize(p)
