#!/usr/bin/env python3
"""Stance delivery analysis — reproduce `plans/stance-delivery-is-the-brake.md`.

Why this exists: the walk plateaus at ~200 px/s and cannot be slowed to the
commanded speed by tuning `HIP_V_MIX`, `V_MEAS_TAU`, `ADAPT_TC` or `DCM_TC`.
Those knobs change what the placement law *asks for*; they cannot change what
the physics *delivers*. This script measures the delivery channel directly, so
the argument is a table of numbers rather than a plot impression.

It answers four questions, per leg, from a `run_walk.sh` telemetry CSV:

  1. clock    — are the stance/swing windows the configured durations?
  2. swing    — how much of the swing's target advance does the foot really
                travel, in the ground frame?
  3. stance   — how far does the supposedly PINNED stance foot slide, and does
                the slide size track the swing shortfall?
  4. pendulum — how much relative sweep (hip-close on foot) does stance
                actually produce, versus the velocity change it delivers, and
                versus the LIPM's own prediction?

Reading the output: if `ratio` (delivery) is well under 1.0 and the stance
`drag` is comparable to the `shortfall`, the stride is being paid for by the
foot sliding forward, the relative sweep stays near zero, and the LIPM never
brakes — which is the reported plateau. `corr(drag, shortfall)` near +0.7 is
the fingerprint of that chain.

This is read-only: no env switches, no asset edits, no physics tuning.

Usage:
  analyze_stance_delivery.py <telemetry.csv> [--json]

Units: px, px/s, seconds. World is y-down; x is shared with the resolver.

Exit codes: 0 ok, 2 bad input.
"""
import argparse
import csv
import json
import math
import sys

# Gravity and the nominal CoM height used for the LIPM check. The walk runs
# gravity 980 px/s²; z is measured per stance from hip→foot height when the
# columns allow it, and falls back to the model standing height.
GRAVITY = 980.0
FALLBACK_Z = 90.0

# gait_l / gait_r codes (report.rs `gait_code`, GaitPhase declaration order).
STANCE = 0
SWING = 1

LEGS = (
    ("L", "PLLLx", "PLLLy", "gait_l"),
    ("R", "PRLLx", "PRLLy", "gait_r"),
)


def fnum(v):
    """Parse a telemetry cell, returning None when the row is not numeric."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(path):
    """Rows that carry a numeric `P3x` — the per-tick leg rows."""
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            if fnum(row.get("P3x")) is None:
                continue
            out.append(row)
    if not out:
        raise SystemExit(f"no numeric rows in {path}")
    return out


def median(xs):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    return xs[len(xs) // 2]


def quantile(xs, q):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    i = min(len(xs) - 1, max(0, int(q * len(xs))))
    return xs[i]


def corr(a, b):
    """Pearson r; 0.0 when either series is degenerate."""
    n = len(a)
    if n < 2:
        return float("nan")
    ma, mb = sum(a) / n, sum(b) / n
    va = math.sqrt(sum((x - ma) ** 2 for x in a))
    vb = math.sqrt(sum((y - mb) ** 2 for y in b))
    if va * vb < 1e-9:
        return 0.0
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (va * vb)


def runs(values, ts):
    """Contiguous spans of equal `values` as (value, t_start, t_end)."""
    spans = []
    start, cur = ts[0], values[0]
    for i in range(1, len(values)):
        if values[i] != cur:
            spans.append((cur, start, ts[i]))
            start, cur = ts[i], values[i]
    spans.append((cur, start, ts[-1]))
    return spans


def stance_and_swing(rows, t, xcol, gcol):
    """Collect per-swing and per-stance records for one leg."""
    gait = [int(float(r[gcol])) for r in rows]
    swings, stances = [], []

    # --- swing records: lift-off → touchdown -------------------------------
    for i in range(1, len(rows) - 1):
        if not (gait[i] == SWING and gait[i - 1] == STANCE):
            continue
        j = i
        while j + 1 < len(rows) and gait[j + 1] == SWING:
            j += 1
        n = j - i
        if n < 8:
            continue
        T = t[j] - t[i]
        tgt_from, tgt_to = fnum(rows[i]["t" + xcol]), fnum(rows[j]["t" + xcol])
        act_from, act_to = fnum(rows[i][xcol]), fnum(rows[j][xcol])
        if None in (tgt_from, tgt_to, act_from, act_to):
            continue
        speed = [
            abs(fnum(rows[k + 1][xcol]) - fnum(rows[k][xcol])) / (t[k + 1] - t[k])
            for k in range(i, j)
        ]
        swings.append(
            {
                "t0": t[i],
                "T": T,
                "tgt_adv": tgt_to - tgt_from,
                "act_adv": act_to - act_from,
                "tgt_speed": (tgt_to - tgt_from) / T if T > 0 else 0.0,
                "act_speed": (act_to - act_from) / T if T > 0 else 0.0,
                "peak_speed": max(speed) if speed else 0.0,
                "shortfall": tgt_to - act_to,
                "lag_profile": [
                    fnum(rows[i + int(b * n / 10)]["t" + xcol])
                    - fnum(rows[i + int(b * n / 10)][xcol])
                    for b in range(10)
                ],
            }
        )

    # --- stance records: touchdown → lift-off ------------------------------
    for i in range(1, len(rows) - 1):
        if not (gait[i] == STANCE and gait[i - 1] == SWING):
            continue
        j = i
        while j + 1 < len(rows) and gait[j + 1] == STANCE:
            j += 1
        if j - i < 8:
            continue
        T = t[j] - t[i]
        if T <= 0.05:
            continue
        v0, v1 = fnum(rows[i]["P3vx"]), fnum(rows[j]["P3vx"])
        vd = fnum(rows[i]["v_des"])
        hip_adv = fnum(rows[j]["P3x"]) - fnum(rows[i]["P3x"])
        drag = fnum(rows[j][xcol]) - fnum(rows[i][xcol])
        if None in (v0, v1, vd, hip_adv, drag):
            continue
        hy = fnum(rows[i]["P3y"])
        # z: hip-above-foot where both columns exist, else the model height.
        # The leg PVs are y-down, so a foot BELOW the hip has the LARGER y.
        z = FALLBACK_Z
        yy = fnum(rows[i][xcol.replace("x", "y")])
        if hy is not None and yy is not None:
            z = max(1.0, yy - hy)
        w = math.sqrt(GRAVITY / z)
        s = w * T
        rel_sweep = (fnum(rows[i]["P3x"]) - fnum(rows[i][xcol])) - (
            fnum(rows[j]["P3x"]) - fnum(rows[j][xcol])
        )
        stance = {
            "t0": t[i],
            "T": T,
            "v0": v0,
            "v1": v1,
            "vd": vd,
            "dv": v1 - v0,
            "demand": vd - v0,
            "hip_adv": hip_adv,
            "drag": drag,
            "rel_sweep": rel_sweep,
            # Deadbeat relative demand: the hip→foot offset that would deliver
            # v_desired exactly over this stance.
            "rel_demand": (vd - v0 * math.cos(s)) / (w * math.sin(s))
            if abs(math.sin(s)) > 1e-6
            else 0.0,
            "lipm_dv": -w * math.sin(s) * rel_sweep,
            "z": z,
            "omega": w,
        }
        # Pair the stance with the swing that delivered it (same touchdown
        # time), so shortfall and drag line up in time.
        near = [x for x in swings if abs(x["t0"] + x["T"] - t[i]) <= 0.05]
        if near:
            stance["shortfall"] = min(near, key=lambda x: abs(x["t0"] + x["T"] - t[i]))[
                "shortfall"
            ]
        stances.append(stance)

    return swings, stances


def summarize(leg, swings, stances):
    d = {"leg": leg, "n_swings": len(swings), "n_stances": len(stances)}

    # 1. clock
    d["stance_T"] = median([s["T"] for s in stances]) if stances else float("nan")
    d["swing_T"] = median([s["T"] for s in swings]) if swings else float("nan")

    # 2. swing delivery
    if swings:
        d["tgt_adv"] = median([s["tgt_adv"] for s in swings])
        d["act_adv"] = median([s["act_adv"] for s in swings])
        ratio = [
            s["act_adv"] / s["tgt_adv"] for s in swings if abs(s["tgt_adv"]) > 10.0
        ]
        d["ratio"] = median(ratio) if ratio else float("nan")
        d["tgt_speed"] = median([s["tgt_speed"] for s in swings])
        d["act_speed"] = median([s["act_speed"] for s in swings])
        d["peak_speed"] = median([s["peak_speed"] for s in swings])
        d["shortfall"] = median([s["shortfall"] for s in swings])
        # median lag by swing decile
        d["lag_profile"] = [
            median([s["lag_profile"][b] for s in swings]) for b in range(10)
        ]

    # 3. stance drag
    if stances:
        d["drag"] = median([s["drag"] for s in stances])
        d["hip_adv"] = median([s["hip_adv"] for s in stances])
        d["peak_stance_speed"] = max(
            abs(s["drag"]) / s["T"] for s in stances if s["T"] > 0
        )
        paired = [s for s in stances if "shortfall" in s]
        d["corr_drag_shortfall"] = corr(
            [s["drag"] for s in paired], [s["shortfall"] for s in paired]
        )

        # 4. pendulum
        d["rel_sweep"] = median([s["rel_sweep"] for s in stances])
        d["rel_demand"] = median([s["rel_demand"] for s in stances])
        d["dv"] = median([s["dv"] for s in stances])
        d["lipm_dv"] = median([s["lipm_dv"] for s in stances])
        d["omega"] = median([s["omega"] for s in stances])
        d["corr_rel_dv"] = corr(
            [s["rel_sweep"] for s in stances], [s["dv"] for s in stances]
        )
        d["corr_hip_dv"] = corr(
            [s["hip_adv"] for s in stances], [s["dv"] for s in stances]
        )
        d["corr_T_dv"] = corr([s["T"] for s in stances], [s["dv"] for s in stances])
        frac = [
            s["dv"] / s["demand"] for s in stances if abs(s["demand"]) > 5.0
        ]
        d["delivered_fraction"] = median(frac) if frac else float("nan")

    return d


def report(tag, summary, rows):
    g = "%.3f"
    print(f"== {tag}: {len(rows)} rows")
    for d in summary:
        leg = d["leg"]
        print(f"  [{leg}] swings={d['n_swings']} stances={d['n_stances']}")
        print(
            f"    clock    stance_T {g % d.get('stance_T', float('nan'))} s   "
            f"swing_T {g % d.get('swing_T', float('nan'))} s"
        )
        print(
            f"    swing    target {d['tgt_adv']:6.1f}px @{d['tgt_speed']:6.1f}px/s   "
            f"actual {d['act_adv']:6.1f}px @{d['act_speed']:6.1f}px/s   "
            f"ratio {d['ratio']:.2f}   peak {d['peak_speed']:.0f}px/s"
        )
        print(f"    shortfall at touchdown {d['shortfall']:.1f}px")
        print(
            "    lag by swing decile "
            + " ".join(f"{v:.1f}" for v in d["lag_profile"])
        )
        print(
            f"    stance   drag {d['drag']:.1f}px (peak {d['peak_stance_speed']:.0f}px/s)   "
            f"hip_adv {d['hip_adv']:.1f}px   "
            f"corr(drag,shortfall) {d['corr_drag_shortfall']:+.2f}"
        )
        print(
            f"    pendulum rel_sweep {d['rel_sweep']:.1f}px (demand {d['rel_demand']:+.1f}px)  "
            f"dv {d['dv']:+.1f}px/s   lipm {d['lipm_dv']:+.1f}px/s"
        )
        print(
            f"    corr(rel_sweep,dv) {d['corr_rel_dv']:+.3f}   "
            f"corr(hip_adv,dv) {d['corr_hip_dv']:+.3f}   "
            f"corr(T,dv) {d['corr_T_dv']:+.3f}   "
            f"delivered_fraction {d['delivered_fraction']:+.2f}"
        )

    # Verdict: the chain from the plan document.
    for d in summary:
        drag_ok = abs(d.get("drag", 0.0)) < 5.0
        corr_ok = d.get("corr_rel_dv", 0.0) > 0.5
        if drag_ok and corr_ok:
            verdict = "PINNED: stance drag small and the sweep predicts dv"
        elif corr_ok:
            verdict = "PARTIAL: sweep predicts dv, but the foot still slides"
        else:
            verdict = "SLIDING: the stance foot leaks and the pendulum cannot brake"
        print(f"  [{d['leg']}] {verdict}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", help="telemetry CSV from run_walk.sh")
    ap.add_argument("--json", action="store_true", help="emit JSON instead of text")
    args = ap.parse_args()

    rows = load(args.csv)
    t = [fnum(r["t"]) for r in rows]
    summary = []
    for leg, xcol, _ycol, gcol in LEGS:
        swings, stances = stance_and_swing(rows, t, xcol, gcol)
        summary.append(summarize(leg, swings, stances))

    if args.json:
        print(json.dumps({"csv": args.csv, "rows": len(rows), "legs": summary}, indent=2))
    else:
        report(args.csv, summary, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
