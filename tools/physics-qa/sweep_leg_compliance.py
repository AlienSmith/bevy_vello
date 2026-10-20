#!/usr/bin/env python3
"""Sweep the leg angular-constraint compliance (`LEG_COMPLIANCE`) on the walk.

Why an env var and not the asset: the hip pin **owns** the leg angular
compliance and rewrites it every frame
(`right_leg.config.angular_compliance = params.leg_compliance` in
`game_lib/src/lower_body/hip_pin.rs`), so the asset's `Angular` compliance for
`P2_P3_P40/P41` and `P3_P40/P41_PLLL/PRLL` is dead at runtime. The pin default
lives in `HipPinParams::default()`; `LEG_COMPLIANCE` overrides it.

Each candidate is scored on three things at once, because stiffening the legs
moves all of them:

  swing   — does the swing advance relative to the body and gain lead over the
            stance foot (`analyze_swing.py`),
  stance  — does the planted foot stay put (`analyze_foot_slide.py`),
  rig     — does the pinned frame stay rigid (the harness's own
            `rig=…max_rel_dev=` line, parsed from stdout).

Usage:
  sweep_leg_compliance.py [--values 1e-6,5e-7,1e-7,1e-8,1e-9] [--repeat 1]
                          [--skip-start 1.6] [--skip-end 10.5]
                          [--out-root DIR] [--binary PATH] [--script PATH]

Exit codes: 0 = sweep completed (read the table), 2 = bad input, 3 = a run failed.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import analyze_foot_slide as afs  # noqa: E402
import analyze_swing as aswing  # noqa: E402

DEFAULT_VALUES = "1e-6,5e-7,1e-7,5e-8,1e-8,5e-9,1e-9"
RIG_RE = re.compile(r"rig=(\w+) max_rel_dev=([0-9.]+)")
LEG_RE = re.compile(r"leg: peak left=([0-9.]+)px@t([0-9.]+) right=([0-9.]+)px@t([0-9.]+)")


def repo_root():
    return HERE.parents[1]


def default_binary(root):
    exe = "headless_arm_pd.exe" if os.name == "nt" else "headless_arm_pd"
    return root / "target" / "release" / "examples" / exe


def run_walk(binary, root, asset_root, script, out_csv, extra_env):
    env = os.environ.copy()
    env.update({
        "ASSET_ROOT": str(asset_root),
        "SCRIPT": str(script),
        "OUT_CSV": str(out_csv),
        "COLLISION": "1",
    })
    env.update(extra_env)
    env.setdefault("RUST_LOG", "info")
    started = time.time()
    proc = subprocess.run([str(binary)], cwd=str(root), env=env,
                          capture_output=True, text=True,
                          encoding="utf-8", errors="replace")
    return proc, time.time() - started


def swing_stats(csv_path, skip_start, skip_end):
    rows = aswing.load(str(csv_path))
    if not rows:
        return None
    out = {"lift_mean": 0.0, "adv_mean": 0.0, "gain_mean": 0.0,
           "body_speed": 0.0, "cmd_vx": 0.0, "lead0": 0.0, "lead1": 0.0}
    acc = {k: [] for k in ("lift_mean", "adv_mean", "gain_mean", "lead0", "lead1")}
    for leg, _xk, _yk, _gk, _tk in aswing.LEGS:
        other = "right" if leg == "left" else "left"
        floor = aswing.floor_height(rows, leg)
        segs = aswing.swing_segments(rows, leg, other, skip_start, skip_end,
                                     aswing.DEFAULT_MIN_FRAMES)
        if not segs:
            continue
        for k, key in (("lift_mean", "lift"), ("adv_mean", "dx_body_rel"),
                       ("gain_mean", "lead_gain"), ("lead0", "lead0"),
                       ("lead1", "lead1")):
            if key == "lift":
                vals = [floor - s["y_min"] for s in segs]
                acc[k].append(sum(vals) / len(vals))
            else:
                acc[k].append(sum(s[key] for s in segs) / len(segs))
    for k, vals in acc.items():
        out[k] = round(sum(vals) / len(vals), 2) if vals else 0.0
    dur = rows[-1]["t"] - rows[0]["t"]
    if dur > 0:
        out["body_speed"] = round((rows[-1]["body_x"] - rows[0]["body_x"]) / dur, 1)
    out["cmd_vx"] = round(aswing.mean([r["cmd_vx"] for r in rows]), 1)
    return out


def stance_stats(csv_path, gate, skip_start, skip_end):
    try:
        m = afs.load(str(csv_path))
    except OSError:
        return None
    if not m:
        return None
    floor_l = afs.floor_height(m, "lry")
    floor_r = afs.floor_height(m, "rry")
    segs_l = afs.stance_segments(m, floor_l, "lry", "lrx", "t", "gl",
                                 afs.DEFAULT_FLOOR_TOL, afs.DEFAULT_MIN_FRAMES)
    segs_r = afs.stance_segments(m, floor_r, "rry", "rrx", "t", "gr",
                                 afs.DEFAULT_FLOOR_TOL, afs.DEFAULT_MIN_FRAMES)
    rep_l = afs.summarize(afs.filter_window(segs_l, skip_start, skip_end), "left ")
    rep_r = afs.summarize(afs.filter_window(segs_r, skip_start, skip_end), "right")
    return max(rep_l["max_drift"], rep_r["max_drift"])


def render_markdown(results, meta):
    lines = [
        "# Leg angular-compliance sweep (`LEG_COMPLIANCE`)",
        "",
        f"- binary: `{meta['binary']}`  script: `{meta['script']}`",
        f"- window: t >= {meta['skip_start']:.2f}s, t <= {meta['skip_end']:.2f}s",
        f"- asset: {meta['asset']}",
        "",
        "| LEG_COMPLIANCE | body px/s (cmd) | lift mean | swing adv | lead gain "
        "| lead liftoff→touchdown | stance drift max | rig dev |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for r in results:
        s, k = r["swing"], r["stance"]
        lines.append(
            f"| {r['value']:g} | {s['body_speed']:.1f} ({s['cmd_vx']:.0f}) "
            f"({100.0 * s['body_speed'] / s['cmd_vx']:.0f}%) | "
            f"{s['lift_mean']:.1f}px | {s['adv_mean']:+.1f}px | "
            f"{s['gain_mean']:+.1f}px | {s['lead0']:+.1f}→{s['lead1']:+.1f}px | "
            f"{k:.2f}px | {r['rig']} {r['rig_dev']:.4f} |")
    lines.append("")
    return "\n".join(lines)


def main():
    root = repo_root()
    ap = argparse.ArgumentParser()
    ap.add_argument("--values", default=DEFAULT_VALUES)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--gate", type=float, default=2.0,
                    help="px; stance-drift gate for the report")
    ap.add_argument("--skip-start", type=float, default=1.6)
    ap.add_argument("--skip-end", type=float, default=10.5)
    ap.add_argument("--binary", default=str(default_binary(root)))
    ap.add_argument("--script", default=str(root / "plans" / "walk_script.json"))
    ap.add_argument("--asset-root",
                    default=str(root / "examples" / "collision_detection" / "assets"))
    ap.add_argument("--out-root", default=str(root / "tmp" / "headless"
                                              / "leg_compliance_sweep"))
    ap.add_argument("--extra-env", default="",
                    help="extra K=V pairs for every run, comma-separated "
                         "(e.g. LEG_MAX_ANG_RATE=20,LEG_ALPHA=0.5)")
    args = ap.parse_args()

    extra_env = {}
    for pair in args.extra_env.split(","):
        pair = pair.strip()
        if not pair:
            continue
        if "=" not in pair:
            print(f"bad --extra-env pair: {pair}", file=sys.stderr)
            sys.exit(2)
        key, val = pair.split("=", 1)
        extra_env[key.strip()] = val.strip()

    binary = Path(args.binary).resolve()
    script = Path(args.script).resolve()
    asset_root = Path(args.asset_root).resolve()
    out_root = Path(args.out_root).resolve()
    for p, label in ((binary, "binary"), (script, "script"),
                     (asset_root, "asset root")):
        if not p.exists():
            print(f"{label} not found: {p}", file=sys.stderr)
            sys.exit(2)

    values = []
    for tok in args.values.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            values.append(float(tok))
        except ValueError:
            print(f"bad value: {tok}", file=sys.stderr)
            sys.exit(2)
    if not values:
        sys.exit(2)

    out_root.mkdir(parents=True, exist_ok=True)
    meta = {"binary": str(binary), "script": str(script),
            "asset": str(asset_root), "repeat": args.repeat,
            "skip_start": args.skip_start, "skip_end": args.skip_end}
    results, failures = [], []
    for v in values:
        runs = []
        for rep in range(1, args.repeat + 1):
            tag = f"leg_{v:g}" + (f"_r{rep}" if args.repeat > 1 else "")
            tag += "".join(f"_{k}{val}" for k, val in extra_env.items())
            csv_path = out_root / f"{tag}.csv"
            env = {"LEG_COMPLIANCE": f"{v:g}"}
            env.update(extra_env)
            proc, secs = run_walk(binary, root, asset_root, script, csv_path, env)
            if proc.returncode != 0 or not csv_path.exists():
                failures.append({"value": v, "rep": rep,
                                 "rc": proc.returncode,
                                 "tail": (proc.stderr or "")[-500:]})
                print(f"[{tag}] RUN FAILED rc={proc.returncode}", file=sys.stderr)
                continue
            rig_m = RIG_RE.search(proc.stderr + proc.stdout)
            leg_m = LEG_RE.search(proc.stderr + proc.stdout)
            swing = swing_stats(csv_path, args.skip_start, args.skip_end)
            stance = stance_stats(csv_path, args.gate, args.skip_start, args.skip_end)
            if swing is None or stance is None:
                failures.append({"value": v, "rep": rep, "error": "no telemetry"})
                continue
            run = {
                "value": v,
                "rig": "PASS" if rig_m and rig_m.group(1) == "PASS" else "FAIL",
                "rig_dev": float(rig_m.group(2)) if rig_m else math.nan,
                "leg_peak": float(leg_m.group(1)) if leg_m else math.nan,
                "swing": swing,
                "stance": stance,
                "secs": round(secs, 1),
                "csv": str(csv_path),
            }
            runs.append(run)
            print(f"[{tag}] {secs:5.1f}s body {swing['body_speed']:+7.1f}px/s "
                  f"lift {swing['lift_mean']:5.1f} adv {swing['adv_mean']:+6.1f} "
                  f"gain {swing['gain_mean']:+6.1f} stance {stance:6.2f}px "
                  f"rig {run['rig']} {run['rig_dev']:.4f}")
        if not runs:
            continue
        # Worst-case across repeats on the two gates.
        worst_stance = max(runs, key=lambda r: r["stance"])
        merged = dict(worst_stance)
        merged["swing"] = runs[-1]["swing"]
        merged["runs"] = len(runs)
        results.append(merged)

    if not results:
        print("no successful runs", file=sys.stderr)
        sys.exit(3)

    print()
    table = render_markdown(results, meta)
    print(table)
    (out_root / "leg_compliance_sweep.md").write_text(table, encoding="utf-8")
    (out_root / "leg_compliance_sweep.json").write_text(
        json.dumps({"meta": meta, "results": results, "failures": failures},
                   indent=2), encoding="utf-8")
    print(f"wrote {out_root / 'leg_compliance_sweep.md'}")
    if any(f.get("rc") for f in failures):
        sys.exit(3)


if __name__ == "__main__":
    main()
