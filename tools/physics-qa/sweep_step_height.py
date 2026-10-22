#!/usr/bin/env python3
"""Sweep STEP_HEIGHT on the headless walk and classify the *foot physics* of
every swing with analyze_gesture's stuck / midair / lifted classifier.

Physics knobs (friction, collision scale) are already settled — stand wobble
validated collision, the foot-slide tool validated friction. This isolates the
one remaining gait knob we can tune without a rebuild: the swing-arc apex
`STEP_HEIGHT`, which also sets the hop's foot lift (FLIGHT_SWING_LIFT).

Usage:
  sweep_step_height.py [--heights 28,50,70,90,110] [--compliance 5e-9]
                       [--binary PATH] [--script PATH] [--out-root DIR]

Exit codes: 0 = done, 2 = bad input / missing binary, 3 = a run failed.
"""
import argparse
import csv
import math
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import analyze_gesture as ag  # noqa: E402


def repo_root():
    return HERE.parents[1]


def default_binary(root):
    exe = "headless_arm_pd.exe" if os.name == "nt" else "headless_arm_pd"
    return root / "target" / "release" / "examples" / exe


def default_script(root):
    return root / "plans" / "walk_script.json"


def default_asset_root(root):
    return root / "examples" / "collision_detection" / "assets"


def run_walk(binary, root, asset_root, script, out_csv, compliance, step_height):
    env = os.environ.copy()
    env.update({
        "ASSET_ROOT": str(asset_root),
        "SCRIPT": str(script),
        "OUT_CSV": str(out_csv),
        "COLLISION": "1",
        "LEG_COMPLIANCE": str(compliance),
        "STEP_HEIGHT": str(step_height),
    })
    env.setdefault("RUST_LOG", "info")
    proc = subprocess.run(
        [str(binary)], cwd=str(root), env=env,
        capture_output=True, text=True, encoding="utf-8", errors="replace")
    return proc


def classify(csv_path, skip_start, skip_end):
    rows, cols = ag.load(str(csv_path))
    if not rows:
        raise RuntimeError(f"no rows in {csv_path}")
    floor_l = ag.floor_height(rows, "left")
    floor_r = ag.floor_height(rows, "right")
    runs = []
    for leg, other in (("left", "right"), ("right", "left")):
        runs += ag.split_swings(rows, leg, other, skip_start, skip_end)
    scored = [s for s in
              (ag.analyze_swing(*run, floor_l if run[1] == "left" else floor_r,
                                argparse.Namespace(gate_lift=ag.GATE_LIFT,
                                                   gate_gain=ag.GATE_GAIN,
                                                   min_frames=ag.DEFAULT_MIN_FRAMES,
                                                   floor_tol=2.0))
               for run in runs) if s is not None]
    per_leg = ag.summarize_runs(scored)
    return per_leg, scored, {"body_speed": ag.body_speed(rows),
                             "floor_l": floor_l, "floor_r": floor_r}


def main():
    root = repo_root()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--heights", default="28,50,70,90,110")
    ap.add_argument("--compliance", type=float, default=5e-9)
    ap.add_argument("--skip-start", type=float, default=ag.DEFAULT_SKIP_START)
    ap.add_argument("--skip-end", type=float, default=ag.DEFAULT_SKIP_END)
    ap.add_argument("--binary", default=str(default_binary(root)))
    ap.add_argument("--script", default=str(default_script(root)))
    ap.add_argument("--out-root", default=str(root / "tmp" / "headless" / "step_height_sweep"))
    args = ap.parse_args()

    binary = Path(args.binary).resolve()
    script = Path(args.script).resolve()
    asset_root = default_asset_root(root).resolve()
    out_root = Path(args.out_root).resolve()
    for p, label in ((binary, "binary"), (script, "script"), (asset_root, "asset_root")):
        if not p.exists():
            print(f"{label} not found: {p}", file=sys.stderr)
            sys.exit(2)

    heights = []
    for tok in args.heights.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            heights.append(float(tok))
        except ValueError:
            print(f"bad step_height: {tok}", file=sys.stderr)
            sys.exit(2)
    if not heights:
        print("no step heights", file=sys.stderr)
        sys.exit(2)

    out_root.mkdir(parents=True, exist_ok=True)

    print(f"STEP_HEIGHT sweep  LEG_COMPLIANCE={args.compliance:g}  "
          f"(physics settled: collision via stand, friction via foot-slide)")
    print(f"window=[{args.skip_start}s,{args.skip_end}s]")
    print()
    print(f"{'stepHt':>6} | {'body':>6} | {'left pass/n: stck/air/lift':>26} | "
          f"{'right pass/n: stck/air/lift':>26}")
    print("-" * 80)
    failures = 0
    for sh in heights:
        tag = f"sh{sh:g}"
        csv_path = out_root / f"{tag}.csv"
        proc = run_walk(binary, root, asset_root, script, csv_path,
                        args.compliance, sh)
        if proc.returncode != 0 or not csv_path.exists():
            failures += 1
            print(f"{tag:>6} | RUN FAILED rc={proc.returncode}",
                  file=sys.stderr)
            if proc.stderr:
                print((proc.stderr or "")[-600:], file=sys.stderr)
            continue
        try:
            per_leg, scored, meta = classify(csv_path, args.skip_start, args.skip_end)
        except RuntimeError as exc:
            failures += 1
            print(f"{tag:>6} | {exc}", file=sys.stderr)
            continue

        def fmt(leg):
            s = per_leg.get(leg)
            if not s:
                return "  n/a"
            pc = s["phys_counts"]
            return f"{s['passed']}/{s['n']}: {pc['stuck']}/{pc['midair']}/{pc['lifted']}"
        print(f"{sh:>6.0f} | {meta['body_speed']:>6.1f} | "
              f"{fmt('left'):>26} | {fmt('right'):>26}")

    print()
    print("key: 'pass/n' = swings passing lift+advance+overtake; "
          "'stck/air/lift' = foot stuck / midair / landed-only-advance failures")
    sys.exit(3 if failures else 0)


if __name__ == "__main__":
    sys.exit(main())