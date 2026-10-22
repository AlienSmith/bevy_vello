#!/usr/bin/env python3
"""Describe the *walking gesture* of the v10 character from headless telemetry.

The character is a rigid stick (the torso) on top of two two-segment legs; the
hip (`P3`) is far heavier than every other particle. To explain the persistent
"one leg steps but the other does not" asymmetry we track, per gait cycle:

  hip    P3      the torso/body reference (heavy CoM proxy)
  knees  P40/P41 left/right knee particles (intermediate leg joint)
  feet   PLLL/PRLL the two foot particles (vello y-down: bigger y = lower)
  head   PH      one upper-body particle (top of head) -> body bobbing + tilt

For each detected swing segment (a contiguous `gait_* == 1` run) the tool scores
the action that actually constitutes "stepping": does the foot *lift* off the
floor, *advance* relative to the body, and *overtake* the opposite foot? On top
of that it reports the **knee flexion** used to make the step and the **head bob**
during the step. The whole run is then summarised left vs right to expose the
asymmetry (the body is supposed to advance while alternate legs step under it).

Telemetry columns (v10, position+velocity):
  t, cmd_vx, vphase, gait_l, gait_r, body_theta, ...
  P3x/P3y/P3vx/P3vy, P40x/y, P41x/y, PLLLx/y, PRLLx/y, PHx/y

Usage:
  analyze_gesture.py <walk.csv> [--gate-lift px] [--gate-gain px]
                     [--floor-tol px] [--skip-start s] [--skip-end s]
                     [--min-frames N] [--json] [--plot out.png] [--per-cycle]

Foot physics per swing (why the step missed):
  stuck   the foot never left the floor (floor too sticky / leg too weak to lift)
  midair  the foot left but stayed off the floor (body hopped so it can't reach down)
  lifted  the foot rose and landed, failed only on advance (stride/timing) — the
          normal-looking non-step
  airFr   fraction of the swing the foot spends above the floor (vello y-down)

Exit codes: 0 = gesture described (gait recognised); 1 = no usable gait found;
2 = bad input / missing columns.
"""
import argparse
import csv
import json
import math
import sys

# A swing run shorter than this many ticks is a phase blip, not a step.
DEFAULT_MIN_FRAMES = 3
# Minimum floor clearance of a swing (px).
GATE_LIFT = 2.0
# Minimum *gain* in foot-to-foot lead across the swing (px): the swing foot must
# pass the opposite (stance) foot for it to be a real "step".
GATE_GAIN = 0.0
# Skip startup/finish transients (same defaults as analyze_swing.py).
DEFAULT_SKIP_START = 1.6
DEFAULT_SKIP_END = 10.5
# vello y-down: the floor is the largest y a foot reaches.
FLOOR_PERCENTILE = 99.0

# name, foot-x, foot-y, knee-x, knee-y, gait key
LEGS = (
    ("left", "PLLLx", "PLLLy", "P40x", "P40y", "gait_l"),
    ("right", "PRLLx", "PRLLy", "P41x", "P41y", "gait_r"),
)


def load(path):
    """Read the CSV into a list of dicts with typed values + derived per-leg.

    Returns (rows, cols) where cols is the set of present column names so the
    caller can warn about missing channels and still degrade gracefully.
    """
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return rows, set()
        cols = set(reader.fieldnames)
        for r in reader:
            def g(key):
                try:
                    v = r[key]
                    return float(v) if v not in ("", "nan", "NaN") else float("nan")
                except (KeyError, ValueError):
                    return float("nan")

            row = {
                "t": g("t"),
                "cmd_vx": g("cmd_vx"),
                "gait_l": g("gait_l"),
                "gait_r": g("gait_r"),
                "hip":  {"x": g("P3x"),    "y": g("P3y"),
                         "vx": g("P3vx"), "vy": g("P3vy")},
                "head": {"x": g("PHx"),    "y": g("PHy"),
                         "vx": g("PHvx"), "vy": g("PHvy")},
            }
            for name, fx, fy, kx, ky, gk in LEGS:
                row[name] = {
                    "foot": {"x": g(fx), "y": g(fy)},
                    "knee": {"x": g(kx), "y": g(ky)},
                    "gait": g(gk),
                }
            rows.append(row)
    return rows, cols


def mean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else math.nan


def floor_height(rows, leg):
    """vello y-down floor: largest y the foot reaches (99th percentile)."""
    ys = [r[leg]["foot"]["y"] for r in rows if not math.isnan(r[leg]["foot"]["y"])]
    if not ys:
        return math.nan
    ys.sort()
    return ys[min(len(ys) - 1, int(len(ys) * FLOOR_PERCENTILE / 100.0))]


def knee_flexion(hip, knee, foot):
    """Legged knee angle in degrees (0 = fully extended, 180 = folded).

    Hip, knee, foot form a triangle; the knee is the vertex. Fully extended means
    hip-knee-foot are collinear with the knee between hip and foot -> 180 deg at
    the knee -> 0 flexion. As the heel lifts / the shank folds the angle shrinks
    and flexion grows.
    """
    if any(math.isnan(v) for p in (hip, knee, foot) for v in (p["x"], p["y"])):
        return math.nan
    hk = (hip["x"] - knee["x"], hip["y"] - knee["y"])
    fk = (foot["x"] - knee["x"], foot["y"] - knee["y"])
    hl = math.hypot(*hk)
    fl = math.hypot(*fk)
    if hl == 0.0 or fl == 0.0:
        return math.nan
    cos_a = (hk[0] * fk[0] + hk[1] * fk[1]) / (hl * fl)
    cos_a = max(-1.0, min(1.0, cos_a))
    angle_deg = math.degrees(math.acos(cos_a))
    return 180.0 - angle_deg


def lead_gain(seg, leg, other):
    """Swing-foot-over-stance-foot lead *gain* across the segment (px).

    At each tick the swing foot's x minus the opposite foot's x; the gain is the
    last minus the first. Positive => the swing foot physically passes the stance
    foot during the swing (the real "did it step" test).
    """
    first = last = None
    for r in seg:
        lx = r[leg]["foot"]["x"]
        ox = r[other]["foot"]["x"]
        if math.isnan(lx) or math.isnan(ox):
            continue
        v = lx - ox
        if first is None:
            first = v
        last = v
    return (last - first) if (first is not None and last is not None) else math.nan


def analyze_swing(seg, leg, other, floor, args):
    """Score one swing segment of `leg`. Returns a dict or None if degenerate."""
    if len(seg) < args.min_frames:
        return None

    t0 = seg[0]["t"]
    t1 = seg[-1]["t"]

    # -- lift: max clearance of the foot above the floor during the swing --
    lifts = [floor - r[leg]["foot"]["y"] for r in seg
             if not math.isnan(r[leg]["foot"]["y"])]
    max_lift = max(lifts) if lifts else math.nan

    # -- advance: gain of (foot.x - body_x) across the swing --
    def foot_rel_body(r):
        fx, bx = r[leg]["foot"]["x"], r["hip"]["x"]
        return fx - bx if (not math.isnan(fx) and not math.isnan(bx)) else math.nan
    a0 = foot_rel_body(seg[0])
    a1 = foot_rel_body(seg[-1])
    advance = (a1 - a0) if (a0 is not None and a1 is not None
                            and not math.isnan(a0) and not math.isnan(a1)) else math.nan

    # -- overtake: lead gain over the other foot --
    gain = lead_gain(seg, leg, other)

    # -- knee flexion: how much the shank folds to make the step --
    flex = [knee_flexion(seg[i]["hip"], seg[i][leg]["knee"], seg[i][leg]["foot"])
            for i in range(len(seg))]
    flex_ok = [f for f in flex if not math.isnan(f)]
    max_flex = max(flex_ok) if flex_ok else math.nan
    mean_flex = mean(flex_ok)

    # -- head bob: vertical range of the head (upper body) across the step --
    hys = [r["head"]["y"] for r in seg if not math.isnan(r["head"]["y"])]
    bob = (max(hys) - min(hys)) if len(hys) >= 2 else math.nan

    # -- body / hip vertical drop across the step (squash / bounce) --
    hps = [r["hip"]["y"] for r in seg if not math.isnan(r["hip"]["y"])]
    hip_drop = (max(hps) - min(hps)) if len(hps) >= 2 else math.nan

    # -- FOOT PHYSICS: physical state of the foot during the swing --.
    # vello y-down: clear = floor - foot.y => >0 means foot is above the floor.
    # `floor_tol` px of the floor counts as "grounded"; above that is airborne.
    clears = [floor - r[leg]["foot"]["y"] for r in seg
              if not math.isnan(r[leg]["foot"]["y"])]
    nticks = len(clears)
    air_ticks = sum(1 for c in clears if c > args.floor_tol)
    air_frac = (air_ticks / nticks) if nticks else math.nan
    min_clear = min(clears) if clears else math.nan   # deepest the foot reaches (near-land)
    end_clear = clears[-1] if clears else math.nan    # clearance at the swing's last tick
    # Hip height above the floor (vello: floor is a big y, hip a small y).
    hy = [floor - r["hip"]["y"] for r in seg
          if not math.isnan(r["hip"]["y"])]
    hip_gap = mean(hy)                                # how high the hip rides this swing

    # Classify the PHYSICAL cause of this swing (what the foot actually did),
    # regardless of whether it passed:
    #   "stuck"   never left the floor (floor too sticky / leg too weak to lift)
    #   "midair"  left the floor but never came back to it (min_clear still above
    #             tolerance) — body hopped up so the foot can't reach the floor
    #   "lifted"  rose AND touched the floor again; if it failed it was purely on
    #             advance (stride/timing), not on reaching the floor
    if max_lift < args.gate_lift:
        phys = "stuck"
    elif min_clear > args.floor_tol:
        phys = "midair"   # foot airborne the whole swing, never touches floor
    else:
        phys = "lifted"   # foot lifted and re-contacted the floor

    passed = (
        max_lift >= args.gate_lift and advance > 0.0 and gain >= args.gate_gain
    )
    return {
        "leg": leg,
        "t0": t0, "t1": t1,
        "dur": t1 - t0,
        "max_lift": max_lift, "advance": advance, "lead_gain": gain,
        "max_flex": max_flex, "mean_flex": mean_flex,
        "head_bob": bob, "hip_drop": hip_drop,
        "air_frac": air_frac, "min_clear": min_clear,
        "end_clear": end_clear, "hip_gap": hip_gap,
        "phys": phys,
        "passed": bool(passed),
    }


def split_swings(rows, leg, other, skip_start, skip_end):
    """Contiguous gait==1 runs of `leg` clipped to the steady window."""
    runs = []
    cur = []
    for r in rows:
        if r["t"] < skip_start or r["t"] > skip_end:
            # Outside the steady window: don't score, terminate any open run.
            if cur and len(cur) >= DEFAULT_MIN_FRAMES:
                runs.append((cur, leg, other))
            cur = []
            continue
        if r[leg]["gait"] == 1:
            cur.append(r)
        else:
            if cur and len(cur) >= DEFAULT_MIN_FRAMES:
                runs.append((cur, leg, other))
            cur = []
    if cur and len(cur) >= DEFAULT_MIN_FRAMES:
        runs.append((cur, leg, other))
    return runs


def summarize_runs(runs):
    """Aggregate per-leg statistics across all scored swings."""
    per_leg = {}
    for seg in runs:
        s = per_leg.setdefault(seg["leg"], {
            "n": 0, "passed": 0,
            "max_lift": [], "advance": [], "lead_gain": [],
            "max_flex": [], "mean_flex": [], "head_bob": [], "hip_drop": [], "dur": [],
            "air_frac": [], "hip_gap": [], "phys_counts": {"stuck": 0,
                                                            "midair": 0,
                                                            "lifted": 0},
        })
        s["n"] += 1
        s["passed"] += int(seg["passed"])
        for k in ("max_lift", "advance", "lead_gain", "max_flex", "mean_flex",
                  "head_bob", "hip_drop", "dur", "air_frac", "hip_gap"):
            s[k].append(seg[k])
        s["phys_counts"][seg["phys"]] += 1

    return per_leg


def body_speed(rows):
    """Average body forward speed over the steady window (px/s), vello x."""
    xs = [(r["t"], r["hip"]["x"]) for r in rows if not math.isnan(r["hip"]["x"])]
    if len(xs) < 2:
        return math.nan
    return (xs[-1][1] - xs[0][1]) / (xs[-1][0] - xs[0][0]) if xs[-1][0] > xs[0][0] else math.nan


def render_text(per_leg, meta):
    lines = []
    lines.append("=== Walking gesture: per-leg step summary ===")
    lines.append(f"body speed~{meta['body_speed']:.1f} px/s  "
                 f"window=[{meta['skip_start']}s, {meta['skip_end']}s]  "
                 f"floor_l={meta['floor_l']:.1f}  floor_r={meta['floor_r']:.1f}")
    lines.append("")
    hdr = (f"{'leg':<5}{'n':>3}{'pass':>5} | {'maxLift':>8}{'adv':>8}{'gain':>8}"
           f" | {'airFr':>6}{'stck':>5}{'air':>4}{'lift':>6} | "
           f"{'maxFlex':>8}{'bob':>7}")
    lines.append(hdr)
    lines.append("-" * len(hdr))
    for leg in ("left", "right"):
        if leg not in per_leg:
            continue
        s = per_leg[leg]
        pc = s["phys_counts"]
        lines.append(
            f"{leg:<5}{s['n']:>3}{s['passed']:>5} | {mean(s['max_lift']):>8.1f}"
            f"{mean(s['advance']):>8.1f}{mean(s['lead_gain']):>8.1f}"
            f" | {mean(s['air_frac']):>6.2f}{pc['stuck']:>5}{pc['midair']:>4}"
            f"{pc['lifted']:>6} | "
            f"{mean(s['max_flex']):>8.1f}{mean(s['head_bob']):>7.1f}"
        )
    lines.append("")
    lines.append("Columns: n=swings; pass=swings that lift & advance & overtake; "
                 "maxLift=max foot clearance (px);")
    lines.append("adv=gain of foot-before-body across swing (px); "
                 "gain=swing-over-stance foot lead gain (px);")
    lines.append("airFr=fraction of the swing the foot spends off the floor; "
                 "stck/air/lift = swings classified stuck-on-floor / airborne-whole /")
    lines.append("lifted-and-advanced (the last is the 'fails only on advance' case); "
                 "maxFlex=peak knee flexion deg; bob=head vertical range (px).")
    lines.append("")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--gate-lift", type=float, default=GATE_LIFT)
    ap.add_argument("--gate-gain", type=float, default=GATE_GAIN)
    ap.add_argument("--min-frames", type=int, default=DEFAULT_MIN_FRAMES)
    ap.add_argument("--floor-tol", type=float, default=2.0,
                    help="clearance (px) above the floor that still counts as "
                         "'grounded' when classifying foot physics")
    ap.add_argument("--skip-start", type=float, default=DEFAULT_SKIP_START)
    ap.add_argument("--skip-end", type=float, default=DEFAULT_SKIP_END)
    ap.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    ap.add_argument("--plot", metavar="OUT.png",
                    help="optionally render overlaid left/right gesture plot")
    ap.add_argument("--per-cycle", action="store_true",
                    help="print each scored swing on its own line")
    args = ap.parse_args()

    rows, cols = load(args.csv)
    if not rows:
        print(f"error: no rows read from {args.csv}", file=sys.stderr)
        return 2

    # Sanity-check required columns.
    required = {"P3x", "P3y", "P40x", "P40y", "P41x", "P41y",
                "PLLLx", "PLLLy", "PRLLx", "PRLLy", "PHx", "PHy",
                "gait_l", "gait_r"}
    missing = required - cols
    if missing:
        print(f"error: missing telemetry columns: {', '.join(sorted(missing))}",
              file=sys.stderr)
        return 2

    floor_l = floor_height(rows, "left")
    floor_r = floor_height(rows, "right")

    runs = []
    for leg, other in (("left", "right"), ("right", "left")):
        runs += split_swings(rows, leg, other, args.skip_start, args.skip_end)

    scored = [analyze_swing(*run, floor_l if run[1] == "left" else floor_r, args)
              for run in runs]
    scored = [s for s in scored if s is not None]

    if not scored:
        print("error: no gait swings found in window", file=sys.stderr)
        return 1

    per_leg = summarize_runs(scored)
    meta = {
        "body_speed": body_speed(rows),
        "skip_start": args.skip_start, "skip_end": args.skip_end,
        "floor_l": floor_l, "floor_r": floor_r,
    }

    if args.json:
        print(json.dumps({"meta": meta, "swings": scored}, indent=2))
    else:
        print(render_text(per_leg, meta))
        # Per-cycle line-by-line when requested or when few cycles.
        if args.per_cycle or len(scored) <= 8:
            print("\n--- per-cycle detail ---")
            for s in scored:
                print(
                    f"{s['leg']:<6} t=[{s['t0']:7.2f},{s['t1']:7.2f}] "
                    f"air={s['air_frac']:.2f} lift={s['max_lift']:5.1f} "
                    f"adv={s['advance']:+.1f} gain={s['lead_gain']:+.1f} "
                    f"flex={s['max_flex']:.0f}deg bob={s['head_bob']:.1f} "
                    f"hipDrop={s['hip_drop']:.1f} phys={s['phys']:<6} "
                    f"-> {'STEP' if s['passed'] else 'NO'}"
                )

    if args.plot:
        try:
            render_plot(rows, per_leg, args.plot)
        except Exception as e:  # noqa: BLE001
            print(f"warning: could not render plot: {e}", file=sys.stderr)

    ay = per_leg.get("left", {}).get("n", 0)
    by = per_leg.get("right", {}).get("n", 0)
    return 0 if (ay and by) else 0


def render_plot(rows, per_leg, out):
    """Overlaid left/right gesture traces. Requires matplotlib (optional)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot", file=sys.stderr)
        return

    ts = [r["t"] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)

    names = {
        "left":  {"foot": "PLLL", "knee": "P40"},
        "right": {"foot": "PRLL", "knee": "P41"},
    }
    colors = {"left": "#1f77b4", "right": "#d62728"}

    # Panel 1: foot + hip x trajectories (forward travel / step placement)
    ax = axes[0]
    ax.plot(ts, [r["hip"]["x"] for r in rows], color="k", lw=2, label="hip (P3)")
    for leg in ("left", "right"):
        ax.plot(ts, [r[leg]["foot"]["x"] for r in rows],
                color=colors[leg], ls="--", lw=1.5, label=f"{names[leg]['foot']} x")
    ax.axhline(0, color="grey", lw=0.5)
    ax.set_ylabel("x") 
    ax.set_title("Forward travel: hip vs feet (x)")
    ax.legend(loc="upper left", fontsize=8)

    # Panel 2: foot + hip y (floor contact / lift / body bob)
    ax = axes[1]
    ax.plot(ts, [r["hip"]["y"] for r in rows], color="k", lw=2, label="hip (P3)")
    for leg in ("left", "right"):
        ax.plot(ts, [r[leg]["foot"]["y"] for r in rows],
                color=colors[leg], ls="-", lw=1.2, label=f"{names[leg]['foot']} y")
        ax.plot(ts, [r[leg]["knee"]["y"] for r in rows],
                color=colors[leg], ls=":", lw=1.0, label=f"{names[leg]['knee']} y")
    ax.set_ylabel("y (vello down)")
    ax.set_title("Vertical: knees / feet vs hip (lift + bob)")
    ax.legend(loc="upper right", fontsize=8)

    # Panel 3: knee flexion per leg (the "how it steps" angle)
    ax = axes[2]
    for leg in ("left", "right"):
        flex = [knee_flexion(r["hip"], r[leg]["knee"], r[leg]["foot"]) for r in rows]
        ax.plot(ts, flex, color=colors[leg], lw=1.3, label=f"{leg} knee flexion")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("flexion (deg)")
    ax.set_title("Knee flexion 0=straight, 180=folded")
    ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        sys.exit(1)