#!/usr/bin/env python3
"""swing_clock.py — does the swing CURVE span the swing WINDOW?

    python3 tools/physics-qa/swing_clock.py <csv> [...] [--from T0] [--to T1]

The gait says "swing" from t0 to t1 (`gait_*` flips: 1 = swing). If the pacing
were sound the profile would span exactly that window: the foot leaves the
floor AT t0 (so stance friction stops being fought) and arrives at the aim AT
t1. This measures how badly each swing misses both ends:

  win       t1 - t0, the actual flight window
  start_dly lift_at - t0   >0 -> the command still sits on the floor: the foot
                                  DRAGS and keeps making friction
  end_hold  t1 - sat_at    >0 -> the command reached (and parks at / retreats
                                  from) its aim early

Floor line = median command y while that foot is NOT swinging.
"""
import csv
import statistics as stats
import sys


def load(path):
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            row = {}
            for k, v in r.items():
                if v in (None, ""):
                    continue
                try:
                    row[k] = float(v)
                except ValueError:
                    pass
            out.append(row)
    return out


SIDE = {
    "L": ("gait_l", "tPLLLy", "tPLLLx", "planLx"),
    "R": ("gait_r", "tPRLLy", "tPRLLx", "planRx"),
}


def runs(rows, gait):
    out, cur = [], []
    for r in rows:
        if r.get(gait, 0.0) > 0.5:
            cur.append(r)
        else:
            if len(cur) >= 4:
                out.append(cur)
            cur = []
    return out


def report(path, t_from=1.4, t_to=13.9):
    rows = load(path)
    print(f"\n=== {path} ===")
    all_sd, all_eh = [], []
    for side, (gait, cy, cx, dcm) in SIDE.items():
        floor = stats.median(
            r[cy] for r in rows if r.get(gait, 0.0) < 0.5 and r[cy] > 1.0
        )
        print(f" {side}  (stance command y ~ {floor:.1f})")
        print("   t0    t1   win | start_dly  end_hold | cmd_adv aim_l  aim_td")
        sd, eh = [], []
        for sw in runs(rows, gait):
            t0, t1 = sw[0]["t"], sw[-1]["t"]
            if t0 < t_from or t0 > t_to:
                continue
            lift = sat = None
            for r in sw:
                if lift is None and r[cy] < floor - 1.5:
                    lift = r["t"]
                if sat is None and abs(r[cx] - r[dcm]) < 0.06:
                    sat = r["t"]
            s_d = (lift - t0) if lift is not None else float("nan")
            e_h = (t1 - sat) if sat is not None else float("nan")
            if s_d == s_d:
                sd.append(s_d)
            if e_h == e_h:
                eh.append(e_h)
            print(
                f" {t0:5.2f}{t1:6.2f}{t1 - t0:6.3f} |"
                f"{s_d:10.3f}{e_h:10.3f} |"
                f"{sw[-1][cx] - sw[0][cx]:9.1f}"
                f"{sw[0][dcm] - sw[0]['P3x']:6.1f}"
                f"{sw[-1][dcm] - sw[-1]['P3x']:7.1f}"
            )
        if sd or eh:
            def d(v):
                return f"med {stats.median(v):+.3f} max {max(v):+.3f}" if v else "n/a"
            print(f"   n={len(sd)}  start_delay {d(sd)}   end_hold {d(eh)}")
        all_sd += sd
        all_eh += eh
    if all_sd:
        print(
            f" ALL: start_delay med {stats.median(all_sd):+.3f} s  "
            f"end_hold med {stats.median(all_eh):+.3f} s  "
            f"(swing_duration_ts = 0.280 s)"
        )


argv = sys.argv[1:]
t_from, t_to = 1.4, 13.9
if "--from" in argv:
    i = argv.index("--from"); t_from = float(argv[i + 1]); argv = argv[:i] + argv[i + 2:]
if "--to" in argv:
    i = argv.index("--to"); t_to = float(argv[i + 1]); argv = argv[:i] + argv[i + 2:]
for p in argv:
    report(p, t_from, t_to)
