"""What lifts the stance foot off the floor mid-stance?

Per stance decile, print the MEAN of:
  - realized foot y (PLLLy / PRLLy, vello y-down; floor surface at ~278.97)
  - commanded IK target foot y (tPLLLy / tPRLLy)
  - realized foot vy (negative = moving UP)
  - debounced contact fraction

Usage: py tmp/headless/foot_y_profile.py tmp/headless/walks/default_win.csv [more.csv...]
"""
import csv
import sys

FLOOR_Y = 278.97  # vello y of the floor surface (run log, both walks)


def main() -> int:
    for path in sys.argv[1:] or ["tmp/headless/walks/default_win.csv"]:
        rows = list(csv.DictReader(open(path, newline="")))
        print(f"\n== {path}")
        for leg, fx, fy, fvy, tx, ty in (
            ("l", "PLLLx", "PLLLy", "PLLLvy", "tPLLLx", "tPLLLy"),
            ("r", "PRLLx", "PRLLy", "PRLLvy", "tPRLLx", "tPRLLy"),
        ):
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

            print(f" leg {leg} ({len(runs)} stances)   decile: realized_y  target_y  realized_vy  contact%")
            for d in range(10):
                frames = [r for run in runs for i, r in enumerate(run)
                          if min(9, int(i * 10 / len(run))) == d]
                if not frames:
                    continue
                ry = sum(float(r[fy]) for r in frames) / len(frames)
                ty_ = sum(float(r[ty]) for r in frames) / len(frames)
                vy = sum(float(r[fvy]) for r in frames) / len(frames)
                ct = 100 * sum(int(r[ck]) for r in frames) / len(frames)
                print(f"   {d*10:>3}-{d*10+10:<3}      {ry:8.1f}  {ty_:8.1f}  {vy:9.1f}  {ct:7.0f}")
            print(f"   (floor surface y = {FLOOR_Y}; realized_y below it = foot ABOVE floor)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
