#!/usr/bin/env python3
"""Check that a walking swing actually *places* the foot: does it leave from
behind the opposite foot and land ahead of it?

A walk is not "the body advances while the legs cycle in place". Per swing the
foot must

  1. **lift**      — leave the floor (vello y-down, so `floor_y - min(y)`),
  2. **advance**   — gain ground *relative to the body* (`x - body_x` grows);
                     otherwise the foot is only travelling with the body,
  3. **overtake**  — at touchdown be ahead of the opposite (stance) foot, having
                     started behind it, i.e. the foot-to-foot lead must *gain* a
                     step length (`lead_gain`).

A foot that only does (1) is the "wobble while creeping forward" gait: the body
advances, both feet ride along, the feet stay splayed at a fixed separation and
the swing never passes the stance foot. The tool also reports the **tracking
error** (realized foot vs its commanded IK target at liftoff/touchdown): if the
realized feet sit far outside their commanded span, the leg is not reaching its
own foot placement and that, not friction, is the walk bug.

Reads the whole-body walk telemetry CSV written by `headless_arm_pd` /
`visual_arm_pd`:

  t                           time (s)
  gait_l / gait_r             per-leg resolver phase (0 = Stance, 1 = Swing)
  PLLLx/PLLLy, PRLLx/PRLLy    realized foot particles (vello y-down)
  tPLLLx/tPRLLx               commanded foot IK targets
  P3x                         spine end / body reference x
  cmd_vx                      commanded hip velocity (walk intent)

Usage:
  analyze_swing.py <telemetry.csv> [--gate-lead px] [--gate-gain px]
                   [--gate-lift px] [--min-frames N]
                   [--skip-start s] [--skip-end s] [--json] [--per-swing]

Exit codes: 0 = every swing lifts, advances and overtakes; 1 = at least one
swing fails a gate; 2 = bad input.
"""
import argparse
import csv
import json
import math
import sys

# A swing run shorter than this many ticks is a phase blip, not a step.
DEFAULT_MIN_FRAMES = 3
# Minimum floor clearance of a swing (px). ~4 px is what this rig currently does.
DEFAULT_GATE_LIFT = 2.0
# Minimum lead of the swing foot over the opposite foot at touchdown (px).
DEFAULT_GATE_LEAD = 0.0
# Minimum *gain* in that lead across the swing (px): the foot must pass the other
# foot, so this is the real "did it step" test.
DEFAULT_GATE_GAIN = 0.0
# Startup / stop transients of the 11.5 s walk.
DEFAULT_SKIP_START = 1.6
DEFAULT_SKIP_END = 10.5
# vello y-down: the floor is the largest y a foot reaches.
FLOOR_PERCENTILE = 99.0

LEGS = (
    # name, x key, y key, gait key, target-x key
    ("left", "PLLLx", "PLLLy", "gait_l", "tPLLLx"),
    ("right", "PRLLx", "PRLLy", "gait_r", "tPRLLx"),
)


def load(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return rows
        for r in reader:
            def g(key):
                try:
                    return float(r[key])
                except (KeyError, ValueError):
                    return float("nan")
            row = {
                "t": g("t"),
                "body_x": g("P3x"),
                "cmd_vx": g("cmd_vx"),
                "left": {"x": g("PLLLx"), "y": g("PLLLy"), "gait": g("gait_l"),
                         "tx": g("tPLLLx")},
                "right": {"x": g("PRLLx"), "y": g("PRLLy"), "gait": g("gait_r"),
                          "tx": g("tPRLLx")},
            }
            rows.append(row)
    return rows


def mean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else math.nan


def floor_height(rows, leg):
    ys = [r[leg]["y"] for r in rows if not math.isnan(r[leg]["y"])]
    if not ys:
        return math.nan
    ys.sort()
    return ys[min(len(ys) - 1, int(len(ys) * FLOOR_PERCENTILE / 100.0))]


def swing_segments(rows, leg, other, skip_start, skip_end, min_frames):
    """Contiguous `gait == 1` runs of `leg`, clipped to the steady window."""
    runs = []
    cur = []
    for r in rows:
        if r[leg]["gait"] == 1:
            cur.append(r)
        else:
            if len(cur) >= min_frames:
                runs.append(cur)
            cur = []
    if len(cur) >= min_frames:
        runs.append(cur)

    segs = []
    for seg in runs:
        first, last = seg[0], seg[-1]
        t0, t1 = first["t"], last["t"]
        if t0 < skip_start or t1 > skip_end:
            continue
        x0, x1 = first[leg]["x"], last[leg]["x"]
        o0, o1 = first[other]["x"], last[other]["x"]
        b0, b1 = first["body_x"], last["body_x"]
        segs.append({
            "t0": t0, "t1": t1, "dur": t1 - t0, "ticks": len(seg),
            "x0": x0, "x1": x1,
            "dx_world": x1 - x0,
            "body_dx": b1 - b0,
            "dx_body_rel": (x1 - b1) - (x0 - b0),
            # foot-to-foot lead: negative = swing foot is behind
            "lead0": x0 - o0,
            "lead1": x1 - o1,
            "lead_gain": (x1 - o1) - (x0 - o0),
            # realized foot vs its own commanded target
            "track0": x0 - first[leg]["tx"],
            "track1": x1 - last[leg]["tx"],
            # absolute foot separation vs commanded target separation
            "sep0": abs(x0 - o0), "sep1": abs(x1 - o1),
            "tsep0": abs(first[leg]["tx"] - first[other]["tx"]),
            "tsep1": abs(last[leg]["tx"] - last[other]["tx"]),
            "y_min": min(r[leg]["y"] for r in seg),
            "y_max": max(r[leg]["y"] for r in seg),
        })
    return segs


def classify(seg, floor, args):
    seg["lift"] = floor - seg["y_min"] if not math.isnan(floor) else math.nan
    fails = []
    if math.isnan(seg["lift"]) or seg["lift"] < args.gate_lift:
        fails.append("NO_LIFT")
    if seg["dx_body_rel"] <= 0.0:
        fails.append("NO_ADVANCE")
    if seg["lead1"] < args.gate_lead:
        fails.append("NO_OVERTAKE")
    if seg["lead_gain"] < args.gate_gain:
        fails.append("NO_LEAD_GAIN")
    seg["fails"] = fails
    return seg


def summarize(segs, label, floor, rows, args):
    base = {"leg": label, "swings": len(segs), "floor_y": round(floor, 2)}
    if not segs:
        base.update({"pass": 0, "fail": 0, "verdict": "NO_SWING"})
        return base
    for s in segs:
        classify(s, floor, args)
    passed = [s for s in segs if not s["fails"]]
    codes = {}
    for s in segs:
        for f in s["fails"]:
            codes[f] = codes.get(f, 0) + 1

    # Whole-run context: did the body actually travel at the commanded speed?
    i0, i1 = 0, len(rows) - 1
    dur = rows[i1]["t"] - rows[i0]["t"]
    body_speed = (rows[i1]["body_x"] - rows[i0]["body_x"]) / dur if dur > 0 else 0.0

    base.update({
        "pass": len(passed),
        "fail": len(segs) - len(passed),
        "verdict": "PASS" if not (len(segs) - len(passed)) else "FAIL",
        "fail_codes": codes,
        "dur_s": round(mean([s["dur"] for s in segs]), 3),
        "lift_mean": round(mean([s["lift"] for s in segs]), 2),
        "lift_min": round(min(s["lift"] for s in segs), 2),
        "lift_max": round(max(s["lift"] for s in segs), 2),
        "advance_mean": round(mean([s["dx_body_rel"] for s in segs]), 2),
        "advance_min": round(min(s["dx_body_rel"] for s in segs), 2),
        "lead0_mean": round(mean([s["lead0"] for s in segs]), 2),
        "lead1_mean": round(mean([s["lead1"] for s in segs]), 2),
        "lead1_min": round(min(s["lead1"] for s in segs), 2),
        "lead_gain_mean": round(mean([s["lead_gain"] for s in segs]), 2),
        "lead_gain_min": round(min(s["lead_gain"] for s in segs), 2),
        "track0_mean": round(mean([s["track0"] for s in segs]), 2),
        "track1_mean": round(mean([s["track1"] for s in segs]), 2),
        "sep_mean": round(mean([s["sep0"] for s in segs]), 2),
        "tsep_mean": round(mean([s["tsep0"] for s in segs]), 2),
        "body_speed_px_s": round(body_speed, 1),
        "cmd_vx_mean": round(mean([r["cmd_vx"] for r in rows]), 1),
    })
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("telemetry_csv")
    ap.add_argument("--gate-lift", type=float, default=DEFAULT_GATE_LIFT,
                    help="px; minimum floor clearance of a swing")
    ap.add_argument("--gate-lead", type=float, default=DEFAULT_GATE_LEAD,
                    help="px; minimum lead over the opposite foot at touchdown")
    ap.add_argument("--gate-gain", type=float, default=DEFAULT_GATE_GAIN,
                    help="px; minimum gain in that lead across the swing "
                         "(the swing must pass the stance foot)")
    ap.add_argument("--min-frames", type=int, default=DEFAULT_MIN_FRAMES,
                    help="shortest swing run counted as a real swing")
    ap.add_argument("--skip-start", type=float, default=DEFAULT_SKIP_START,
                    help="s; ignore swings starting earlier (startup)")
    ap.add_argument("--skip-end", type=float, default=DEFAULT_SKIP_END,
                    help="s; ignore swings ending later (stop)")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    ap.add_argument("--per-swing", action="store_true",
                    help="list every swing segment")
    args = ap.parse_args()

    rows = load(args.telemetry_csv)
    if not rows:
        print("no rows")
        sys.exit(2)
    if any(math.isnan(rows[0][leg]["gait"]) for leg, *_ in LEGS):
        print("missing gait_l/gait_r columns (not a whole-body walk CSV)")
        sys.exit(2)
    if all(math.isnan(rows[0][leg]["x"]) for leg, *_ in LEGS):
        print("missing PLLLx/PRLLx columns (not a whole-body walk CSV)")
        sys.exit(2)

    reports = {}
    for leg, _xk, _yk, _gk, _tk in LEGS:
        other = "right" if leg == "left" else "left"
        floor = floor_height(rows, leg)
        segs = swing_segments(rows, leg, other, args.skip_start, args.skip_end,
                              args.min_frames)
        rep = summarize(segs, leg, floor, rows, args)
        rep["segments"] = segs
        reports[leg] = rep

    verdict = "PASS"
    for rep in reports.values():
        if rep["verdict"] != "PASS":
            verdict = "FAIL"

    if args.json:
        out = {
            "file": args.telemetry_csv,
            "window": {"skip_start_s": args.skip_start,
                       "skip_end_s": args.skip_end},
            "gates": {"lift_px": args.gate_lift, "lead_px": args.gate_lead,
                      "lead_gain_px": args.gate_gain},
            "legs": [{k: v for k, v in rep.items() if k != "segments"}
                     for rep in reports.values()],
            "verdict": verdict,
        }
        print(json.dumps(out, indent=2))
        sys.exit(0 if verdict == "PASS" else 1)

    print(f"run:    {rows[-1]['t']:.1f}s, {len(rows)} ticks ({args.telemetry_csv})")
    print(f"window: swings fully inside t=[{args.skip_start:.2f}, "
          f"{args.skip_end:.2f}]s")
    print(f"gates:  lift >= {args.gate_lift:.2f}px, lead at touchdown >= "
          f"{args.gate_lead:.2f}px, lead gain >= {args.gate_gain:.2f}px")

    any_rep = next(iter(reports.values()))
    if "body_speed_px_s" in any_rep:
        print(f"body:   travelled {any_rep['body_speed_px_s']:.1f}px/s against "
              f"cmd_vx={any_rep['cmd_vx_mean']:.1f}px/s "
              f"({100.0 * any_rep['body_speed_px_s'] / any_rep['cmd_vx_mean']:.0f}% "
              f"of the commanded speed)" if any_rep["cmd_vx_mean"] else "")

    for leg in ("left", "right"):
        rep = reports[leg]
        print()
        if not rep["swings"]:
            print(f"{leg:5s}: no swing segments in the window")
            continue
        print(f"{leg:5s}: swings={rep['swings']} pass={rep['pass']} "
              f"fail={rep['fail']}  verdict={rep['verdict']}")
        print(f"       floor y={rep['floor_y']:.1f}px  lift mean/min/max="
              f"{rep['lift_mean']:.2f}/{rep['lift_min']:.2f}/"
              f"{rep['lift_max']:.2f}px  dur={rep['dur_s']:.3f}s")
        print(f"       advance rel. body mean/min={rep['advance_mean']:+.2f}/"
              f"{rep['advance_min']:+.2f}px")
        print(f"       lead liftoff={rep['lead0_mean']:+.2f}px -> touchdown="
              f"{rep['lead1_mean']:+.2f}px (min {rep['lead1_min']:+.2f})  "
              f"gain mean/min={rep['lead_gain_mean']:+.2f}/"
              f"{rep['lead_gain_min']:+.2f}px")
        print(f"       tracking err liftoff={rep['track0_mean']:+.2f}px "
              f"touchdown={rep['track1_mean']:+.2f}px   "
              f"foot sep mean={rep['sep_mean']:.1f}px vs target sep "
              f"{rep['tsep_mean']:.1f}px")
        if rep["fail_codes"]:
            codes = ", ".join(f"{k}x{v}"
                              for k, v in sorted(rep["fail_codes"].items()))
            print(f"       failures: {codes}")
        if args.per_swing:
            print(f"       {'liftoff':>8}{'touchdn':>9}{'lift':>7}{'adv':>7}"
                  f"{'lead0':>8}{'lead1':>8}{'gain':>7}{'trk1':>8}  verdict")
            for s in rep["segments"]:
                print(f"       {s['t0']:8.2f}{s['t1']:9.2f}{s['lift']:7.2f}"
                      f"{s['dx_body_rel']:+7.2f}{s['lead0']:+8.2f}"
                      f"{s['lead1']:+8.2f}{s['lead_gain']:+7.2f}"
                      f"{s['track1']:+8.2f}  "
                      f"{'FAIL:' + ','.join(s['fails']) if s['fails'] else 'PASS'}")

    print()
    print("RESULT: " + ("PASS (every swing lifts, advances and overtakes)"
                        if verdict == "PASS" else
                        "FAIL (a swing did not lift / advance / overtake)"))
    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
