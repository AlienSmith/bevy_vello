#!/usr/bin/env python3
"""Quantify the commanded-vs-realized foot divergence from a headless walk run.

Reads the `<csv>.leg.csv` sidecar produced by `headless_arm_pd` in whole-body
(walking) mode. Each row is `t,left_err_px,right_err_px` where `err_px` is the
Euclidean distance between each leg controller's commanded IK foot target and the
realized foot particle. It answers:

  * how bad does the divergence get (peak per leg, and combined),
  * when it peaks (which gait time),
  * is it transient or does it grow over the run (divergence, not lag),
  * how much of the run spends time above a threshold.

Usage:
  analyze_leg_divergence.py <leg.csv> [--threshold px]

Prints a compact report. Exit code 0 = no divergence beyond the threshold,
1 = divergence found (peak > threshold).
"""
import argparse
import csv
import math
import sys


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                t = float(r["t"])
            except (ValueError, KeyError):
                continue
            def g(key):
                try:
                    return float(r[key])
                except (ValueError, KeyError):
                    return float("nan")
            rows.append((t, g("left_err_px"), g("right_err_px")))
    return rows


def peak(rows, idx):
    best, best_t = 0.0, None
    for t, l, r in rows:
        v = l if idx == 0 else r
        if not math.isnan(v) and v > best:
            best, best_t = v, t
    return best, best_t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("leg_csv")
    ap.add_argument("--threshold", type=float, default=25.0,
                    help="px; divergence beyond this flags the run (exit 1)")
    args = ap.parse_args()

    rows = load(args.leg_csv)
    if not rows:
        print("no rows")
        sys.exit(2)

    t0 = rows[0][0]
    t1 = rows[-1][0]
    dur = t1 - t0

    peak_l, peak_l_t = peak(rows, 0)
    peak_r, peak_r_t = peak(rows, 1)

    # Combined peak (max of the two at each row).
    comb = 0.0
    comb_t = None
    for t, l, r in rows:
        v = max(l, r) if not (math.isnan(l) or math.isnan(r)) else 0.0
        if v > comb:
            comb, comb_t = v, t
    # Early (first third) vs late (last third) combined mean: growth ⇒ divergence.
    n = len(rows)
    def seg_mean(lo, hi):
        vs = []
        for t, l, r in rows:
            if not (math.isnan(l) or math.isnan(r)) and lo <= t < hi:
                vs.append(max(l, r))
        return sum(vs) / len(vs) if vs else float("nan")
    early_mean = seg_mean(t0, t0 + dur / 3.0)
    mid_mean = seg_mean(t0 + dur / 3.0, t0 + 2 * dur / 3.0)
    late_mean = seg_mean(t0 + 2 * dur / 3.0, t1 + 1e-9)

    # Fraction of the run above the threshold (combined err).
    over = sum(1 for t, l, r in rows
               if not (math.isnan(l) or math.isnan(r)) and max(l, r) > args.threshold)
    over_frac = over / n

    growth = late_mean - early_mean

    print(f"run:          {dur:.1f}s, {n} ticks ({args.leg_csv})")
    print(f"peak left:    {peak_l:7.2f}px @ t={peak_l_t:.2f}")
    print(f"peak right:   {peak_r:7.2f}px @ t={peak_r_t:.2f}")
    print(f"combined:     peak={comb:7.2f}px @ t={comb_t}")
    print(f"combined err: early={early_mean:6.2f}px  mid={mid_mean:6.2f}px  "
          f"late={late_mean:6.2f}px  (growth={growth:+.2f}px)")
    print(f"time above {args.threshold:g}px: {over_frac*100:5.1f}%")

    diverged = comb > args.threshold  # combined peak
    if diverged:
        print(f"RESULT: DIVERGED (peak combined > {args.threshold:g}px)")
        sys.exit(1)
    print("RESULT: OK (no divergence beyond threshold)")
    sys.exit(0)


if __name__ == "__main__":
    main()
