"""Is the foot arriving at the floor vertically, or kicking it horizontally?

For each stance window (gait==0), report the foot velocity vector at:
  - touchdown (first stance frame): vx, vy, speed, incidence angle from vertical
  - stance mean and the tangential/normal speed ratio
The incidence angle from vertical: 0 = straight down onto the floor,
90 = pure horizontal football kick.

Usage: py tmp/headless/touchdown_angle.py tmp/headless/walks/default_win.csv [more...]
"""
import csv
import math
import sys

VPM = 90.0  # px per frame at 90 Hz -> velocity px/s = (dx/frame)*90


def main() -> int:
    for path in sys.argv[1:] or ["tmp/headless/walks/default_win.csv"]:
        rows = list(csv.DictReader(open(path, newline="")))
        print(f"\n== {path}")
        for leg, vx, vy in (("l", "PLLLvx", "PLLLvy"), ("r", "PRLLx", "PRLLy")):
            vx, vy = f"{'PLLL' if leg=='l' else 'PRLL'}vx", f"{'PLLL' if leg=='l' else 'PRLL'}vy"
            gk, ck = f"gait_{leg}", f"contact_{leg}"
            runs, cur = [], []
            for r in rows:
                if r[gk] == "0":
                    cur.append(r)
                elif len(cur) >= 10:
                    runs.append(cur)
                    cur = []
            if len(cur) >= 10:
                runs.append(cur)

            td_ang, td_vx, td_vy = [], [], []
            ratio = []
            for run in runs:
                # touchdown = first frame; average over first 2 frames for robustness
                ax = sum(float(r[vx]) for r in run[:2]) / 2
                ay = sum(float(r[vy]) for r in run[:2]) / 2
                td_vx.append(ax)
                td_vy.append(ay)
                # incidence from vertical (the floor normal is y)
                # use the velocity toward the floor = max(ay,0) vs |ax|
                down = max(ay, 0.0)
                ang = math.degrees(math.atan2(abs(ax), down)) if down > 1e-3 else 90.0
                td_ang.append(ang)
                # stance: tangential (x) vs normal (y) speed, contact frames only
                ct = [r for r in run if r[ck] == "1"]
                if ct:
                    t = sum(abs(float(r[vx])) for r in ct) / len(ct)
                    n = sum(abs(float(r[vy])) for r in ct) / len(ct)
                    if n > 1e-3:
                        ratio.append(t / n)

            def m(xs):
                return sum(xs) / len(xs) if xs else float("nan")
            print(f" leg {leg}: touchdown vx_mean={m(td_vx):+.1f} vy_mean={m(td_vy):+.1f} "
                  f"(px/s; vy+ = down)  incidence_from_vertical={m(td_ang):.0f} deg")
            print(f"   stance contact-frames tangent/normal speed ratio = {m(ratio):.2f} "
                  f"(>1 means sliding faster along the floor than pressing into it)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
