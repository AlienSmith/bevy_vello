#!/usr/bin/env python3
"""Measure foot sliding during the stance phase from a walk telemetry CSV.

Reads the main telemetry CSV produced by `headless_arm_pd` (or `visual_arm_pd`)
in whole-body walk mode. The relevant columns are the realized foot particles
(PLLL = left foot, PRLL = right foot) and their commanded IK targets:

  PLLLx/PLLLy, PRLLx/PRLLy     realized foot position (vello y-down)
  tPLLLx/tPLLLy, tPRLLx/tPRLLy commanded IK foot target (same frame)
  gait_l/gait_r                per-leg resolver gait phase (0 = Stance, 1 = Swing)

A *planted* foot must not move in world space while the body advances. Sliding is
the horizontal drift of a realized foot during a stance interval.

Stance is segmented by the **event signal** the resolver exports (`gait_l` /
`gait_r`, 0 = Stance, 1 = Swing) rather than by a geometric floor tolerance.
This is the correct signal because the realized foot lift is small (~5 px) and a
geometric tolerance would swallow it, classifying a whole run as one continuous
stance phase (a false "SLIDING" verdict equal to the foot's forward travel).

For older CSVs that predate the `gait_*` columns, the analyzer falls back to the
geometric test (foot y within `--floor-tol` of the run's floor height).

The startup and stop transients are a different phenomenon from steady-state
sliding, so `--skip-start` / `--skip-end` drop stance segments that fall outside
the steady window (a segment is kept only when `t0 >= skip-start` and
`t1 <= skip-end`). Both default to "no trimming" so existing verdicts are
unchanged.

Usage:
  analyze_foot_slide.py <telemetry.csv> [--floor-tol px] [--gate px]
                        [--min-frames N] [--skip-start s] [--skip-end s]
                        [--json]

Exit codes: 0 = no stance slide beyond the gate, 1 = sliding, 2 = bad input.
"""
import argparse
import csv
import json
import math
import sys

# vello y-down frame: floor is at the largest y.
FLOOR_PERCENTILE = 99.0
# Per-foot stance-phase tolerance above the floor height (px). Only used as a
# fallback when the `gait_l`/`gait_r` event columns are absent.
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
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return rows
        has_gait = "gait_l" in reader.fieldnames and "gait_r" in reader.fieldnames
        for r in reader:
            def g(key):
                try:
                    return float(r[key])
                except (ValueError, KeyError):
                    return float("nan")
            row = {
                "t": g("t"),
                # realized feet
                "lrx": g("PLLLx"), "lry": g("PLLLy"),
                "rrx": g("PRLLx"), "rry": g("PRLLy"),
                # commanded targets (same frame)
                "tlx": g("tPLLLx"), "tly": g("tPLLLy"),
                "trx": g("tPRLLx"), "try": g("tPRLLy"),
                # event signal: per-leg gait phase (0 = Stance, 1 = Swing).
                # If the columns are missing, both read as NaN and the caller
                # falls back to the geometric floor test.
                "gl": g("gait_l"), "gr": g("gait_r"),
                "has_gait": has_gait,
            }
            rows.append(row)
    return rows


def floor_height(rows, key):
    """Floor = the largest y the foot reaches (99th percentile)."""
    ys = [r[key] for r in rows if not math.isnan(r[key])]
    if not ys:
        return math.nan
    ys.sort()
    idx = min(len(ys) - 1, int(len(ys) * FLOOR_PERCENTILE / 100.0))
    return ys[idx]


def stance_segments(rows, floor, key_y, key_x, key_t, key_gait, tol, min_frames):
    """Return the contiguous stance intervals as (t0, t1, drift_px, max_speed).

    A row is in stance when its per-leg gait phase reads Stance (0). When the
    `gait_*` event columns are absent (all NaN), fall back to the geometric
    test: realized y within `tol` of the floor. Runs shorter than `min_frames`
    are dropped. `drift_px` is the total horizontal displacement of the
    realized foot across the run; `max_speed` is the peak per-frame horizontal
    speed in px/s.
    """
    segs = []
    cur = []
    use_event = all(not math.isnan(r[key_gait]) for r in rows)
    for r in rows:
        if use_event:
            in_stance = r[key_gait] == 0
        else:
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


def filter_window(segs, skip_start, skip_end):
    """Keep only stance segments fully inside the steady-state window."""
    return [s for s in segs if s["t0"] >= skip_start and s["t1"] <= skip_end]


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
                    help="px; fallback stance test when gait_* columns are absent")
    ap.add_argument("--gate", type=float, default=DEFAULT_GATE,
                    help="px; stance drift beyond this flags the run (exit 1)")
    ap.add_argument("--min-frames", type=int, default=DEFAULT_MIN_FRAMES,
                    help="shortest stance run counted as a real stance phase")
    ap.add_argument("--skip-start", type=float, default=0.0,
                    help="s; drop stance segments starting earlier (startup "
                         "transient)")
    ap.add_argument("--skip-end", type=float, default=float("inf"),
                    help="s; drop stance segments ending later (stop transient)")
    ap.add_argument("--json", action="store_true", help="compact output")
    args = ap.parse_args()

    rows = load(args.telemetry_csv)
    if not rows:
        print("no rows")
        sys.exit(2)
    if all(math.isnan(r["lry"]) and math.isnan(r["rry"]) for r in rows):
        print("missing PLLL/PRLL columns (not a whole-body walk telemetry CSV)")
        sys.exit(2)

    # Preferred signal: per-leg gait phase from the resolver event columns.
    # Fallback: floor height per foot from its own realized trajectory.
    use_event_l = rows[0]["has_gait"] and all(
        not math.isnan(r["gl"]) for r in rows)
    use_event_r = rows[0]["has_gait"] and all(
        not math.isnan(r["gr"]) for r in rows)

    floor_l = floor_height(rows, "lry")
    floor_r = floor_height(rows, "rry")
    segs_l = stance_segments(rows, floor_l, "lry", "lrx", "t", "gl",
                             args.floor_tol, args.min_frames)
    segs_r = stance_segments(rows, floor_r, "rry", "rrx", "t", "gr",
                             args.floor_tol, args.min_frames)

    # Steady-state window: the startup (`t < ~1.1 s`) and stop transients are a
    # different phenomenon from steady-state sliding, so allow them to be
    # excluded from the reported drift.
    steady_l = filter_window(segs_l, args.skip_start, args.skip_end)
    steady_r = filter_window(segs_r, args.skip_start, args.skip_end)

    rep_l = summarize(steady_l, "left ")
    rep_r = summarize(steady_r, "right")

    dur = rows[-1]["t"] - rows[0]["t"] if not math.isnan(rows[-1]["t"]) else 0.0
    worst_drift = max(rep_l["max_drift"], rep_r["max_drift"])
    sliding = worst_drift > args.gate

    seg_signal = "gait-phase event columns"
    if not (use_event_l and use_event_r):
        seg_signal = "geometric floor tolerance (no gait_* columns present)"

    window = {"skip_start_s": args.skip_start, "skip_end_s": args.skip_end}
    dropped = (len(segs_l) + len(segs_r)) - (len(steady_l) + len(steady_r))

    if args.json:
        out = {
            "file": args.telemetry_csv,
            "dur_s": round(dur, 2),
            "segmentation": seg_signal,
            "window": window,
            "segments_dropped_by_window": dropped,
            "gate_px": args.gate,
            "feet": [rep_l, rep_r],
            "worst_stance_drift_px": round(worst_drift, 3),
            "verdict": "SLIDING" if sliding else "OK",
        }
        print(json.dumps(out, indent=2))
    else:
        print(f"run:          {dur:.1f}s, {len(rows)} ticks ({args.telemetry_csv})")
        print(f"segmentation: {seg_signal}")
        print(f"window:       t >= {args.skip_start:.2f}s and t <= "
              f"{args.skip_end:.2f}s ({dropped} stance segment(s) trimmed)")
        print(f"floor height: left={floor_l:.1f}px  right={floor_r:.1f}px "
              f"(fallback tol {args.floor_tol}px)")
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
