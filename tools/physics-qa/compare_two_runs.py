#!/usr/bin/env python3
"""compare_two_runs.py — find WHERE two walker_qa runs diverge, and by how much.

    python3 tools/physics-qa/compare_two_runs.py A.csv B.csv [--from T0]

Row-aligned by index (both harnesses emit one row per fixed tick, so the
index is a valid join key as long as neither run dropped frames). Reports:

  1. the first tick at which each interesting channel differs by > tol
  2. a per-tick table of the derived quantities that decide walk-vs-fall:
       live stance height (foot_y - hip_y, vello y-down)
       hip-to-foot |dx| for both feet, and the model's x_max
       whether the 4c stance hip clamp is ACTIVE (|dx| pinned at x_max)
       the deadbeat aim (planLx/planRx) relative to the hip
       v_filt (reality), cmd_vx (command), v_des
  3. a coarse cycle summary (stride, cadence, slide) so a fall is visible as
     stride collapse rather than a single bad tick.

Everything is in VELLO frame (y DOWN, +x right) because that is what the CSV
stores; the solver frame is y-up, hence `stance_h = foot_y - hip_y`.
"""
import csv
import math
import sys

LEG = 111.0        # thigh_len + calf_len for v10 at scale 1/2
MARGIN = 0.85      # LowerBodyConfig::safety_margin default


def x_max_model(hip_height_cfg):
    inner = LEG * LEG - hip_height_cfg * hip_height_cfg
    return math.sqrt(inner) * MARGIN if inner > 0 else 0.0


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


CHANNELS = [
    ("tP3y", 0.5, "hip pin target y"),
    ("P3y", 0.5, "hip y"),
    ("P3x", 0.5, "hip x"),
    ("v_filt", 1.0, "reality vx"),
    ("cmd_vx", 1.0, "command vx"),
    ("body_theta", 0.01, "lean"),
    ("grounded", 0.5, "grounded"),
    ("mode", 0.5, "mode"),
    ("vphase", 0.5, "vphase"),
    ("act", 0.5, "act"),
]


def first_divergence(a, b):
    print("--- first divergence per channel ---")
    n = min(len(a), len(b))
    for key, tol, label in CHANNELS:
        for i in range(n):
            ra, rb = a[i], b[i]
            if key not in ra or key not in rb:
                continue
            if abs(ra[key] - rb[key]) > tol:
                print(f"  {label:>16} ({key:11}) first differs at row {i:5d}  "
                      f"t={ra['t']:6.2f}  A={ra[key]:9.3f}  B={rb[key]:9.3f}")
                break
        else:
            print(f"  {label:>16} ({key:11}) never differs > {tol}")


def derived(row):
    hip_x, hip_y = row["P3x"], row["P3y"]
    lx, ly = row["PLLLx"], row["PLLLy"]
    rx, ry = row["PRLLx"], row["PRLLy"]
    stance_h = max(ly, ry) - hip_y
    dl, dr = lx - hip_x, rx - hip_x
    aim_l = row.get("planLx", float("nan")) - hip_x
    aim_r = row.get("planRx", float("nan")) - hip_x
    true_reach = math.sqrt(max(LEG * LEG - stance_h * stance_h, 0.0))
    return stance_h, dl, dr, aim_l, aim_r, true_reach


def table(a, b, xa, xb, t_from=0.0, step=0.1):
    print(f"\nmodel x_max: A={xa:.1f}  B={xb:.1f}   "
          f"(true reach at 105 px stance = "
          f"{math.sqrt(max(LEG*LEG - 105*105, 0.0)):.1f})")
    hdr = (f"{'t':>5} | {'A: st_h  dl    dr   aimL  aimR  vfl  lean gr' } "
           f"| {'B: st_h  dl    dr   aimL  aimR  vfl  lean gr'}")
    print(hdr)
    n = min(len(a), len(b))
    ia, ib = 0, 0
    tnext = t_from
    for i in range(n):
        if a[i]["t"] < tnext:
            continue
        tnext += step
        ra, rb = a[i], b[i]
        sh, dl, dr, al, ar, tr = derived(ra)
        sh2, dl2, dr2, al2, ar2, tr2 = derived(rb)
        def fmt(r, sh, dl, dr, al, ar):
            lean = r["body_theta"]
            return (f"{sh:5.1f} {dl:6.1f} {dr:6.1f} {al:6.1f} {ar:6.1f} "
                    f"{r['v_filt']:6.1f} {lean:6.3f} {r['grounded']:2.0f}")
        print(f"{ra['t']:5.2f} | A {fmt(ra, sh, dl, dr, al, ar)} "
              f"| B {fmt(rb, sh2, dl2, dr2, al2, ar2)}")
        if ra["t"] > 6.0:
            break


def main():
    argv = sys.argv[1:]
    t_from = 0.0
    if "--from" in argv:
        i = argv.index("--from")
        t_from = float(argv[i + 1])
        argv = argv[:i] + argv[i + 2:]
    hipA = hipB = None
    for a in list(argv):
        if a.startswith("hipA="):
            hipA = float(a[5:]); argv.remove(a)
        if a.startswith("hipB="):
            hipB = float(a[5:]); argv.remove(a)
    if len(argv) < 2:
        sys.exit(__doc__)
    a, b = load(argv[0]), load(argv[1])
    hipA = 97.0 if hipA is None else hipA
    hipB = 96.0 if hipB is None else hipB
    print(f"A={argv[0]} ({len(a)} rows)  B={argv[1]} ({len(b)} rows)")
    first_divergence(a, b)
    table(a, b, x_max_model(hipA), x_max_model(hipB), t_from)


if __name__ == "__main__":
    main()
