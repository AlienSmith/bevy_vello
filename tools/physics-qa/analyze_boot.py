#!/usr/bin/env python3
"""Boot-run hip metrics from a walk CSV (tP3x / body_theta columns).

Usage:  analyze_boot.py <csv> [<csv> ...]

Prints, per CSV:
  net      = hip_x(last) - hip_x(first)            (px, signed forward travel)
  backward = max drop from the running max, after the move starts  (px)
  onset    = first t where hip_x >= start + 30 px  (s, "" if never)
  fall     = peak |theta_deg| (deg) and the t it happened — a body past ~35-40°
             of measured torso lean has gone down

This isolates the startup question (how fast + how far forward, does it fall)
without the swing/slide gates that fire on the end-of-run settle.
"""
import sys


def load(path):
    rows = []
    with open(path) as f:
        header = f.readline().strip().split(",")

        def col(name):
            return header.index(name)

        ti, hi, ai = col("t"), col("tP3x"), col("body_theta")
        for line in f:
            p = line.split(",")
            if len(p) <= max(ti, hi, ai):
                continue
            rows.append((float(p[ti]), float(p[hi]), float(p[ai])))
    return rows


def report(path):
    rows = load(path)
    if not rows:
        print(f"{path}: EMPTY")
        return
    start = rows[0][1]
    run_max = start
    backward = 0.0
    onset = ""
    peak_abs_theta = 0.0
    peak_theta_t = 0.0
    for t, hx, th in rows:
        if hx > run_max:
            run_max = hx
        # backward travel: how far below the running max we have fallen
        backward = max(backward, run_max - hx)
        if onset == "" and hx >= start + 30.0:
            onset = f"{t:.2f}s"
        if abs(th) > abs(peak_abs_theta):
            peak_abs_theta, peak_theta_t = th, t
    net = rows[-1][1] - start
    # body_theta is the measured torso lean from vertical, radians. Past
    # ~0.6 rad (~34 deg) the body is going down, not just trimming.
    fall = abs(peak_abs_theta) > 0.6
    print(
        f"{path.split('/')[-1]:<12} net={net:+7.1f}px  "
        f"backward={backward:6.1f}px  "
        f"onset={onset or 'NEVER':>7}  "
        f"peak|lean|={abs(peak_abs_theta):5.2f}rad@{peak_theta_t:.2f}s  "
        f"{'FALL' if fall else 'upright'}"
    )


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    for p in sys.argv[1:]:
        report(p)


if __name__ == "__main__":
    main()
