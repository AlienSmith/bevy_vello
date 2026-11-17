#!/usr/bin/env python3
"""three_level_trace.py — dcm aim vs actuator target vs physical foot, in hip-relative px.

    python3 tools/physics-qa/three_level_trace.py <csv> [--range T0 T1] [--step S]

For every sampled tick it prints, for BOTH feet, the horizontal offset from the
hip at three levels of the command chain:

    dcm   = planLx/planRx  - hip   (placement law's aim; NaN outside WalkingCtrl)
    act   = tPLLLx/tPRLLx  - hip   (the swing PROFILE command the actuator chases)
    real  = PLLLx/PRLLx    - hip   (the physical foot particle)

plus the four signals that decide whether the gait is being believed:

    v_des   the raw command speed
    cmd_vx  the fiction's ramped reference (what the laws mostly read)
    v_filt  the filtered MEASURED speed (what `PLAN_MEAS_V` feeds the deadbeat)
    gr      grounded flag

Sign: +x is travel direction, so a positive foot offset means the foot is AHEAD
of the hip. A stance foot should sit near the dcm/act value it was promised and
stay put; if `real` drifts away from `act` while the leg is in stance, that is
the slide. If `real` is positive for BOTH feet, the hip has fallen behind its
own support polygon.
"""
import csv
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


def f(x, w=6, nd=1):
    if x != x:
        return " " * (w - 3) + "nan"
    return f"{x:{w}.{nd}f}"


def main():
    argv = sys.argv[1:]
    t0, t1, step = 0.0, 1e9, 0.05
    path = argv[0]
    if "--range" in argv:
        i = argv.index("--range")
        t0, t1 = float(argv[i + 1]), float(argv[i + 2])
    if "--step" in argv:
        i = argv.index("--step")
        step = float(argv[i + 1])
    rows = load(path)
    print(f"=== {path}  window [{t0}, {t1}] ===")
    print(
        "    t  v_des cmd_vx vfil |   L: dcm   act  real  |   R: dcm   act  real "
        "| lean  gr"
    )
    next_t = t0
    for r in rows:
        t = r["t"]
        if t < next_t or t > t1:
            continue
        next_t = t + step
        hx = r["P3x"]
        dl = r.get("planLx", float("nan")) - hx
        dr = r.get("planRx", float("nan")) - hx
        al = r["tPLLLx"] - hx
        ar = r["tPRLLx"] - hx
        rl = r["PLLLx"] - hx
        rr = r["PRLLx"] - hx
        print(
            f"{t:6.2f} {f(r['v_des'], 5)} {f(r['cmd_vx'], 6)} {f(r['v_filt'], 4)} |"
            f"    {f(dl)} {f(al)} {f(rl)} |"
            f"    {f(dr)} {f(ar)} {f(rr)} |"
            f" {f(r['body_theta'], 5, 3)} {r['grounded']:2.0f}"
        )


if __name__ == "__main__":
    main()
