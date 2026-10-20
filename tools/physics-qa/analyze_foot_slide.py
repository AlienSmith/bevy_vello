#!/usr/bin/env python3
"""Measure foot sliding during the stance phase from a walk telemetry CSV.

Reads the main telemetry CSV produced by `headless_arm_pd` (or `visual_arm_pd`)
in whole-body walk mode. The relevant columns are the realized foot particles
(PLLL = left foot, PRLL = right foot) and their commanded IK targets:

  PLLLx/PLLLy, PRLLx/PRLLy     realized foot position (vello y-down)
  tPLLLx/tPLLLy, tPRLLx/tPRLLy commanded IK foot target (same frame)

A *planted* foot must not move in world space while the body advances. Sliding is
the horizontal drift of a realized foot during a stance interval.

Stance detection is deliberately geometric and independent of the gait model:
in the vello y-down frame the floor is at the *largest* y (smaller y = higher
up), so a foot is in stance whenever its realized y sits within `--floor-tol` of
the run's floor height (the ~99th percentile of that foot's y). A foot that
never lifts (a known failure mode) is therefore classified as stance for the
whole run and its horizontal drift is reported as sliding — which is exactly the
symptom we want to catch. Short touchdown blips (shorter than `--min-frames`)
are ignored so transient contacts do not count as stance phases.

Usage:
  analyze_foot_slide.py <telemetry.csv> [--floor-tol px] [--gate px]
                        [--min-frames N] [--json]

Exit codes: 0 = no stance slide beyond the gate, 1 = sliding, 2 = bad input.
"""
import argparse
import csv
import json
import math
import sys

# vello y-down frame: floor is at the largest y.
FLOOR_PERCENTILE = 99.0
# Per-foot stance-phase tolerance above the floor height (px).
DEFAULT_FLOOR_TOL = 8.0
# A stance run shorter than this many ticks is treated as a touchdown blip.
DEFAULT_MIN_FRAMES = 5
# Stance drift beyond this (px) flags the run as SLIDING (exit 1).
DEFAULT_GATE = 2.0
# Sampling cadence for px/s scaling (matches FIXED_HZ=90).
HZ = 90.0


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            def g(key):
                try:
                    return float(r[key])
                except (ValueError, KeyError):
                    return float("nan")
            rows.append({
                "t": g("t"),
                # realized feet
                "lrx": g("PLLLx"), "lry": g("PLLLy"),
                "rrx": g("PRLLx"), "rry": g("PRLLy"),
                # commanded targets (same frame)
                "tlx": g("tPLLLx"), "tly": g("tPLLLy"),
                "trx": g("tPRLLx"), "try": g("tPRLLy"),
            })
    return rows


def floor_height(rows, key):
    """Floor = the largest y the foot reaches (99th percentile)."""
    ys = [r[key] for r in rows if not math.isnan(r[key])]
    if not ys:
        return math.nan
    ys.sort()
    idx = min(len(ys) - 1, int(len(ys) * FLOOR_PERCENTILE / 100.0))
    return ys[idx]


def stance_segments(rows, floor, key_y, key_x, key_t, tol, min_frames):
    """Return the contiguous stance intervals as (t0, t1, drift_px, max_speed).

    A row is in stance when its realized y is within `tol` of the floor. Runs
    shorter than `min_frames` are dropped. `drift_px` is the total horizontal
    displacement of the realized foot across the run; `max_speed` is the peak
    per-frame horizontal speed in px/s.
    """
    segs = []
    cur = []
    for r in rows:
        y = r[key_y]
        in_stance = not math.isnan(y) and y >= floor - tol
        if in_stance:
            cur.append(r)
        else:
            if len(cur) >= min_frames:
                segs.append(finalize_segment(cur, key_x, key_t))
            cur = []
    if len(cur) >= min_frames:
        segs.append(finalize_segment(cur, key_x, key_t))
    return segs


def finalize_segment(seg, key_x, key_t):
    t0, t1 = seg[0][key_t], seg[-1][key_t]
    # Total horizontal drift = end-to-end displacement of the realized foot.
    drift = abs(seg[-1][key_x] - seg[0][key_x])
    # Peak horizontal speed, px/s, from per-frame |dx| (dt = 1/HZ).
    max_speed = 0.0
    prev = seg[0][key_x]
    for r in seg[1:]:
        dx = abs(r[key_x] - prev)
        prev = r[key_x]
        max_speed = max(max_speed, dx * HZ)
    return {"t0": t0, "t1": t1, "drift": drift, "max_speed": max_speed,
            "n": len(seg)}


def summarize(segs, label):
    """Collapse a foot's stance segments into a report row."""
    if not segs:
        return {"foot": label, "segs": 0, "dur": 0.0, "max_drift": 0.0,
                "mean_drift": 0.0, "total_drift": 0.0, "max_speed": 0.0,
                "worst_t0": math.nan, "worst_t1": math.nan}
    max_drift = max(s["drift"] for s in segs)
    worst = max(segs, key=lambda s: s["drift"])
    mean_drift = sum(s["drift"] for s in segs) / len(segs)
    total_drift = sum(s["drift"] for s in segs)
    max_speed = max(s["max_speed"] for s in segs)
    dur = sum(s["t1"] - s["t0"] for s in segs)
    return {"foot": label, "segs": len(segs), "dur": dur,
            "max_drift": max_drift, "mean_drift": mean_drift,
            "total_drift": total_drift, "max_speed": max_speed,
            "worst_t0": worst["t0"], "worst_t1": worst["t1"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("telemetry_csv")
    ap.add_argument("--floor-tol", type=float, default=DEFAULT_FLOOR_TOL,
                    help="px; a foot within this of the floor is in stance")
    ap.add_argument("--gate", type=float, default=DEFAULT_GATE,
                    help="px; stance drift beyond this flags the run (exit 1)")
    ap.add_argument("--min-frames", type=int, default=DEFAULT_MIN_FRAMES,
                    help="shortest stance run counted as a real stance phase")
    ap.add_argument("--json", action="store_true", help="compact output")
    args = ap.parse_args()

    rows = load(args.telemetry_csv)
    if not rows:
        print("no rows")
        sys.exit(2)
    if all(math.isnan(r["lry"]) and math.isnan(r["rry"]) for r in rows):
        print("missing PLLL/PRLL columns (not a whole-body walk telemetry CSV)")
        sys.exit(2)

    # Floor per foot from its own realized trajectory.
    floor_l = floor_height(rows, "lry")
    floor_r = floor_height(rows, "rry")
    segs_l = stance_segments(rows, floor_l, "lry", "lrx", "t", args.floor_tol,
                             args.min_frames)
    segs_r = stance_segments(rows, floor_r, "rry", "rrx", "t", args.floor_tol,
                             args.min_frames)

    rep_l = summarize(segs_l, "left ")
    rep_r = summarize(segs_r, "right")

    dur = rows[-1]["t"] - rows[0]["t"] if not math.isnan(rows[-1]["t"]) else 0.0
    worst_drift = max(rep_l["max_drift"], rep_r["max_drift"])
    sliding = worst_drift > args.gate

    if args.json:
        out = {
            "file": args.telemetry_csv,
            "dur_s": round(dur, 2),
            "floor_tol_px": args.floor_tol,
            "gate_px": args.gate,
            "feet": [rep_l, rep_r],
            "worst_stance_drift_px": round(worst_drift, 3),
            "verdict": "SLIDING" if sliding else "OK",
        }
        print(json.dumps(out, indent=2))
    else:
        print(f"run:          {dur:.1f}s, {len(rows)} ticks ({args.telemetry_csv})")
        print(f"floor height: left={floor_l:.1f}px  right={floor_r:.1f}px "
              f"(tol {args.floor_tol}px)")
        for rep in (rep_l, rep_r):
            print(f"foot {rep['foot']}: stances={rep['segs']} "
                  f"dur={rep['dur']:.1f}s  "
                  f"drift mean={rep['mean_drift']:.3f}px "
                  f"max={rep['max_drift']:.3f}px  "
                  f"total={rep['total_drift']:.3f}px  "
                  f"peak_speed={rep['max_speed']:.1f}px/s")
            if rep["segs"]:
                print(f"   worst stance t={rep['worst_t0']:.2f}-"
                      f"{rep['worst_t1']:.2f}s drift {rep['max_drift']:.3f}px")
        print(f"worst stance drift: {worst_drift:.3f}px (gate {args.gate}px)")

    if sliding:
        print("RESULT: SLIDING (a stance foot moved more than the gate)")
        sys.exit(1)
    print("RESULT: OK (stance feet held within the gate)")
    sys.exit(0)


if __name__ == "__main__":
    main()
