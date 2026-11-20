#!/usr/bin/env python3
"""standing_pose.py — turn a walker_qa *standing* CSV into a start pose.

The trainer's own start state (factory rest pose + a 45-tick settle) is not the
pose the game actually idles in, and it even flips the tiny left/right foot
asymmetry, so a checkpoint deployed next to an idle character hopped off the
other leg (measured: trainer lifts the LEFT foot 17 px, QA the RIGHT). This tool
captures the *game's* idle pose instead and writes it for `walker_ppo POSE_IN=`.

    tools/physics-qa/run_walker_qa.sh stand SCRIPT=tmp/walker_qa/stand.json
    python3 tools/physics-qa/standing_pose.py tmp/walker_qa/stand.csv \
        plans/walker_standing_pose.json

The CSV already carries every rig particle (13 of them) with x, y, vx, vy, so
nothing extra has to be logged. The pose is the **average over a window** of the
idle stand, which also averages the slow standing rock down to a near-zero
velocity — a repeatable start rather than one instant of a wobble.

Text/momentum channels are deliberately not averaged: only the particles are
the state the trainer teleports.
"""

import argparse
import json
import sys

# The 13 rig particles the QA telemetry resolves (see `game_lib::headless::channels`).
PARTS = ["P0", "P1", "P2", "P3", "P40", "P41", "P10", "P13",
         "PH", "PLLA", "PRLA", "PRLL", "PLLL"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="walker_qa telemetry CSV from a standing run")
    ap.add_argument("out", help="output pose JSON")
    ap.add_argument("--from", dest="t0", type=float, default=0.75,
                    help="window start, seconds (default 0.75: after the spawn drop)")
    ap.add_argument("--to", dest="t1", type=float, default=1.15,
                    help="window end, seconds (default 1.15: before any jump)")
    args = ap.parse_args()

    import csv as csvmod
    rows = list(csvmod.DictReader(open(args.csv)))
    if not rows:
        sys.exit(f"{args.csv}: empty")

    win = [r for r in rows if args.t0 <= float(r["t"]) <= args.t1]
    if len(win) < 10:
        sys.exit(f"{args.csv}: only {len(win)} rows in [{args.t0}, {args.t1}] s")

    # The window must be a *stand*: reject a run that already jumped.
    modes = {r["mode"] for r in win}
    if modes != {"standing"}:
        sys.exit(f"{args.csv}: window is not all standing (modes={sorted(modes)})")

    particles = {}
    for name in PARTS:
        cols = [f"{name}{s}" for s in ("x", "y", "vx", "vy")]
        missing = [c for c in cols if c not in win[0]]
        if missing:
            sys.exit(f"{args.csv}: missing columns {missing}")
        avg = [sum(float(r[c]) for r in win) / len(win) for c in cols]
        particles[name] = [round(v, 4) for v in avg]

    pose = {
        "source": args.csv,
        "window_s": [args.t0, args.t1],
        "ticks": len(win),
        "note": (
            "Average idle standing pose + velocity, captured from walker_qa. "
            "Applied by walker_ppo (POSE_IN=) at every respawn so training "
            "starts where the game idles."
        ),
        "particles": particles,
    }
    with open(args.out, "w") as f:
        json.dump(pose, f, indent=2)
        f.write("\n")

    hip = particles["P3"]
    dlr = particles["PLLL"][1] - particles["PRLL"][1]
    print(f"wrote {args.out}: {len(win)} ticks [{args.t0}, {args.t1}] s, "
          f"hip=({hip[0]:.1f}, {hip[1]:.1f}) v=({hip[2]:+.2f}, {hip[3]:+.2f}) px/s, "
          f"d(L-R)={dlr:+.2f} px")
    for name in ("PLLL", "PRLL"):
        p = particles[name]
        print(f"  {name}: ({p[0]:.1f}, {p[1]:.1f}) v=({p[2]:+.2f}, {p[3]:+.2f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
