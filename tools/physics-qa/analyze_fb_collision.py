#!/usr/bin/env python3
"""Refined A/B: is the FEEDBACK closure stable at the TRACKING level?

What "divergence" MEANS here
----------------------------
The only meaningful divergence is the WITHIN-RUN distance between what the
Raibert resolver commands (`tP3/tP40/tP41/tPLLL/tPRLL` targets) and what the
physics simulation actually produces (`P3/P40/P41/PLLL/PRLL` realized
particles), for the 5 lower-body particles:

  P3    = hip    (pinned frame anchor)
  P40   = left  knee
  P41   = right knee
  PLLL  = left  foot
  PRLL  = right foot

Both channels are emitted in the SAME vello y-down frame, so `Px - tPx` is
directly the signed tracking error. A bounded, non-growing error = the loop
tracks its own command and is stable. A growing error = real instability.

The cross-run fb1-vs-fb0 absolute-position difference is NOT divergence: the
resolver coupling is delta-based (its absolute frame never leaks into the
physics pose), so two runs can cruise at a slightly different realized speed
(e.g. +1.25%) and their absolute positions drift apart linearly forever while
each run still tracks its own command perfectly. That cross-run shift is
reported below only as CONTEXT and never contributes to the verdict.
"""
import csv
import os
import sys

BASE = "/home/ubuntu/bender/tmp/headless/att_fb_collision"

# name -> (target_x, target_y, realized_x, realized_y) columns
# Order matches the user's 5 lower-body particles.
TRACK_PAIRS = [
    ("P3   (hip)",      "tP3x",   "tP3y",   "P3x",   "P3y"),
    ("P40  (L knee)",   "tP40x",  "tP40y",  "P40x",  "P40y"),
    ("P41  (R knee)",   "tP41x",  "tP41y",  "P41x",  "P41y"),
    ("PLLL (L foot)",   "tPLLLx", "tPLLLy", "PLLLx", "PLLLy"),
    ("PRLL (R foot)",   "tPRLLx", "tPRLLy", "PRLLx", "PRLLy"),
]


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def lin_slope(ys):
    """Least-squares slope of ys over its own row indices (per-frame rate)."""
    n = len(ys)
    if n < 2:
        return 0.0
    xs = range(n)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    return sxy / sxx if sxx > 0.0 else 0.0


def within_run_tracking(rows, label):
    """Max/mean per-frame tracking error per particle + growth (slope)."""
    n = len(rows)
    print(f"\n  [{label}] within-run tracking error (Raibert target vs physics):")
    worst = 0.0
    worst_particle = None
    for name, tx, ty, px, py in TRACK_PAIRS:
        errs = []
        for i in range(n):
            ex = float(rows[i].get(px) or 0.0) - float(rows[i].get(tx) or 0.0)
            ey = float(rows[i].get(py) or 0.0) - float(rows[i].get(ty) or 0.0)
            errs.append((ex * ex + ey * ey) ** 0.5)
        mx = max(errs)
        mean = sum(errs) / n
        # Growth check on the magnitude series: bounded means stable.
        slope = lin_slope(errs)
        # Rate in px per 1000 frames = px/s at 90Hz.
        rate = slope * 90.0
        grow = "  <-- growing" if abs(rate) > 1.0 and mx > 2.0 else ""
        print(f"    {name}: max={mx:7.3f}  mean={mean:6.3f}  "
              f"trend={rate:+6.2f} px/s{grow}")
        if mx > worst:
            worst = mx
            worst_particle = name
    return worst, worst_particle


def cross_run_context(r0, r1):
    """fb1-vs-fb0 absolute shift — CONTEXT ONLY, never the verdict."""
    print("\n  [A/B context] cross-run absolute shift fb1-fb0 (NOT divergence,")
    print("    delta-based coupling: linear shift = benign realized-speed diff):")
    for name, tx, ty, px, py in TRACK_PAIRS[:2]:
        n = min(len(r0), len(r1))
        # per-frame shift of the realized channel, then slope
        shift = []
        for i in range(n):
            shift.append(float(r1[i].get(px) or 0.0) - float(r0[i].get(px) or 0.0))
        start = shift[min(30, n - 1)]
        end = shift[-1]
        slope = lin_slope(shift) * 90.0  # px/s
        print(f"    {name}: start={start:+7.3f} end={end:+7.3f} "
              f"linear rate={slope:+6.2f} px/s")


def main():
    for name in ("walk", "jump"):
        p0 = os.path.join(BASE, f"{name}_fb0.csv")
        p1 = os.path.join(BASE, f"{name}_fb1.csv")
        r0, r1 = load(p0), load(p1)
        n = min(len(r0), len(r1))
        print(f"\n=== {name} ({n} rows) ===")

        w0, wp0 = within_run_tracking(r0, "fb0")
        w1, wp1 = within_run_tracking(r1, "fb1")
        cross_run_context(r0, r1)

        worst = max(w0, w1)
        if worst < 3.0:
            v = "STABLE (tracks its own command within ~1 frame; no divergence)"
        elif worst < 20.0:
            v = "BOUNDED (moderate peak tracking error; check particle detail)"
        else:
            v = "DIVERGENCE (target and physics drift apart)"
        print(f"  verdict: worst within-run tracking |err| = {worst:.3f}px -> {v}")

    print("\nSUMMARY: feedback closure is judged on within-run target-vs-physics")
    print("tracking (bounded = stable); cross-run absolute shift is context only.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
