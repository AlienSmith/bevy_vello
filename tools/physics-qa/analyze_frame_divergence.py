#!/usr/bin/env python3
"""Quantify P2/P3 (hip frame) divergence from the Raibert resolver command.

The CSV columns (see headless_arm_pd.rs limb_sample + channels.rs v10) are:
  cmd_vx  = resolver commanded hip vx (pin.state.hip_vel.x, px/s, x frame-indep)
  P2x/P2y/P2vx/P2vy, P3x/P3y/P3vx/P3vy = realized frame particles (Vello y-down,
  but x is directly comparable; y needs a sign flip to match the y-up resolver)
  PHx/PHy/PHvx/PHvy = the character's own "PH" spine particle (NOT resolver-driven)

Position divergence is seed-independent: the pin maps the resolver hip delta
onto P3 (and rigidly onto P2), so
    commanded_delta(t) = integral of cmd_vx from move start
    residual(t)        = (realized_x - realized_x@start) - commanded_delta(t)
A zero residual over the whole walk = the frame tracks the resolver exactly.

Usage:
  python3 analyze_frame_divergence.py <file.csv> [<file2.csv> ...]
"""
import csv
import sys


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("cmd_vx", "P2x", "P2vx", "P3x", "P3vx", "P3y", "P2y",
                  "PHvx", "PHx", "PHy", "t"):
            try:
                r[k] = float(r[k])
            except (KeyError, ValueError):
                r[k] = float("nan")
    return rows


def fmt(x, nd=2):
    if x != x:  # nan
        return "    nan"
    return f"{x:9.{nd}f}"


def report(path, rows):
    # Split into wait (cmd_vx ~ 0) and move windows by the action column 'act'
    # changing to a move, or simpler: cmd_vx becomes nonzero in the walk.
    move = [r for r in rows if r["cmd_vx"] != 0.0]
    wait = [r for r in rows if r["cmd_vx"] == 0.0]
    print(f"=== {path} ===")
    print(f"rows total={len(rows)}  move(cmd!=0)={len(move)}  wait(cmd==0)={len(wait)}")

    # ---------- WAIT: commanded stationary, is the frame still? ----------
    if wait:
        pk3 = max(abs(r["P3vx"]) for r in wait)
        pk2 = max(abs(r["P2vx"]) for r in wait)
        pk3h = max(abs(r["PHvx"]) for r in wait)
        # rocking = differential x-velocity between the two frame particles
        rock = max(abs(r["P3vx"] - r["P2vx"]) for r in wait)
        # growth check: peak in first half vs second half of wait
        half = max(1, len(wait) // 2)
        pk3_a = max(abs(r["P3vx"]) for r in wait[:half])
        pk3_b = max(abs(r["P3vx"]) for r in wait[half:])
        print("  [wait cmd_vx=0]  peak |P3vx|=", fmt(pk3),
              " peak |P2vx|=", fmt(pk2),
              " peak |P3vx-P2vx|(rocking)=", fmt(rock),
              " PH peak|vx|=", fmt(pk3h))
        print("      P3|vx| first-half peak=", fmt(pk3_a),
              " second-half peak=", fmt(pk3_b),
              " ->", "GROWING (energy added)" if pk3_b > pk3_a * 1.5 else "settling/stable")

    # ---------- MOVE: velocity tracking ----------
    if move:
        lag3 = [abs(r["P3vx"] - r["cmd_vx"]) for r in move]
        lag2 = [abs(r["P2vx"] - r["cmd_vx"]) for r in move]
        lagr = [abs(r["P3vx"] - r["P2vx"]) for r in move]  # internal rocking
        mean3 = sum(lag3) / len(lag3)
        mean2 = sum(lag2) / len(lag2)
        meanr = sum(lagr) / len(lagr)
        print("  [move] velocity tracking (px/s)")
        print("      P3vx vs cmd_vx : mean|err|=", fmt(mean3), " peak|err|=", fmt(max(lag3)))
        print("      P2vx vs cmd_vx : mean|err|=", fmt(mean2), " peak|err|=", fmt(max(lag2)))
        print("      internal rock  : mean|P3vx-P2vx|=", fmt(meanr), " peak=", fmt(max(lagr)))

        # ---------- MOVE: position tracking (integrated command) ----------
        # Integrate cmd_vx with the actual per-row dt (t is in seconds).
        dt = []
        for i in range(1, len(rows)):
            dt.append(rows[i]["t"] - rows[i - 1]["t"])
        dt = dt or [1 / 90.0]
        step = sum(dt) / len(dt)
        t0 = move[0]["t"]
        p2x0 = move[0]["P2x"]
        p3x0 = move[0]["P3x"]
        p3y0 = move[0]["P3y"]
        p2y0 = move[0]["P2y"]
        integral = 0.0
        prev_cmd = move[0]["cmd_vx"]
        max_res_p3 = 0.0
        max_res_p2 = 0.0
        max_pitch = 0.0  # differential vertical displacement P3y vs P2y drift
        for r in move:
            integral += 0.5 * (prev_cmd + r["cmd_vx"]) * step
            prev_cmd = r["cmd_vx"]
            res_p3 = (r["P3x"] - p3x0) - integral
            res_p2 = (r["P2x"] - p2x0) - integral
            max_res_p3 = max(max_res_p3, abs(res_p3))
            max_res_p2 = max(max_res_p2, abs(res_p2))
            pitch = (r["P3y"] - p3y0) - (r["P2y"] - p2y0)
            max_pitch = max(max_pitch, abs(pitch))
        # final residuals
        integral_f = integral
        res_p3_f = (move[-1]["P3x"] - p3x0) - integral_f
        res_p2_f = (move[-1]["P2x"] - p2x0) - integral_f
        print(f"  [move] position tracking  (dt~={step:.4f}s, {len(move)} rows)")
        print("      integrated cmd distance   =", fmt(integral_f, 1), "px")
        print("      P3 residual : peak=", fmt(max_res_p3), " final=", fmt(res_p3_f))
        print("      P2 residual : peak=", fmt(max_res_p2), " final=", fmt(res_p2_f))
        print("      P3-P2 vertical pitch drift peak =", fmt(max_pitch))
        print("      cmd_vx range:", fmt(min(r["cmd_vx"] for r in move)),
              "..", fmt(max(r["cmd_vx"] for r in move)))
    print()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    for path in sys.argv[1:]:
        try:
            rows = load(path)
        except FileNotFoundError:
            print(f"NOT FOUND: {path}")
            continue
        report(path, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
