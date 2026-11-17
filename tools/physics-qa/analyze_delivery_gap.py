#!/usr/bin/env python3
"""analyze_delivery_gap.py — three-level delivery + stance slide, in WORLD x.

    python3 tools/physics-qa/analyze_delivery_gap.py <csv> [...] [--tail T0]

Walks one STANCE episode per foot (contiguous ticks where `gait_l`/`gait_r`
report stance) and reports, all in the WORLD frame so a retreating hip cannot
masquerade as a slipping foot:

  dcm_td    the placement law's aim at touchdown   (planLx / planRx)
  act_td    the pose/actuator target at touchdown  (tPLLLx / tPRLLx)
  real_td   the physical foot particle             (PLLLx / PRLLx)
  ERR       real_td - dcm_td     (>0 = foot landed AHEAD of what the law aimed)
  slide     real x drift over the stance (true friction slip)
  hip_mov   hip x drift over the same window
  rel       slide - hip_mov      (foot vs hip: what the pendulum actually sees)
  end_gap   real_end - dcm_end   (how far the law's aim is from the foot it
                                  believes it owns, when the next step is planned)

`slide` near zero with a large negative `hip_mov` means the foot was HELD and
the body backed away from it — an aim/delivery failure, not a friction failure.
A large positive `ERR` is the delivery shortfall that `plans/stance-delivery-
is-the-brake.md` §2 measured at 74-80 %.
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


COLS = {
    "L": {"gait": "gait_l", "real": "PLLLx", "act": "tPLLLx", "dcm": "planLx"},
    "R": {"gait": "gait_r", "real": "PRLLx", "act": "tPRLLx", "dcm": "planRx"},
}


def stances(rows, c, t_from, min_n=5):
    """Contiguous stance runs for one foot."""
    eps, cur = [], []
    for r in rows:
        # NOTE the frozen encoding: `gait_*` is 0 for STANCE and 1 for swing
        # (telemetry.rs `gait_code`). Reading it as a grounded flag inverts the
        # whole analysis.
        in_stance = r.get(c["gait"], 1.0) < 0.5
        if in_stance:
            cur.append(r)
        else:
            if len(cur) >= min_n and cur[0]["t"] >= t_from:
                eps.append(cur)
            cur = []
    if len(cur) >= min_n and cur[0]["t"] >= t_from:
        eps.append(cur)
    return eps


def report(path, t_from=1.6):
    rows = load(path)
    print(f"\n=== {path} ===")
    print(
        " foot  t_td   dur |   dcm   act  real    ERR |  slide  hip_mov    rel "
        "| end_gap  vfil  v_des"
    )
    acc = []
    for side, c in COLS.items():
        for ep in stances(rows, c, t_from):
            a, b = ep[0], ep[-1]
            dcm, act, real = a[c["dcm"]], a[c["act"]], a[c["real"]]
            err = real - dcm
            slide = b[c["real"]] - real
            hip_mov = b["P3x"] - a["P3x"]
            end_gap = b[c["real"]] - b[c["dcm"]]
            print(
                f"  {side}  {a['t']:5.2f}{b['t'] - a['t']:6.2f} |"
                f"{dcm:6.1f}{act:6.1f}{real:6.1f}{err:6.1f}  |"
                f"{slide:7.1f}{hip_mov:8.1f}{slide - hip_mov:7.1f}  |"
                f"{end_gap:7.1f} {b['v_filt']:6.1f} {b['v_des']:5.0f}"
            )
            acc.append((err, slide, hip_mov, end_gap))
    if acc:
        n = len(acc)
        for i, label in enumerate(("ERR", "slide", "hip_mov", "end_gap")):
            vals = sorted(a[i] for a in acc)
            print(
                f"  n={n}  {label:>8}: median {vals[n // 2]:+7.1f}   "
                f"mean {sum(vals) / n:+7.1f}   "
                f"|max| {max(abs(v) for v in vals):6.1f}"
            )


def main():
    argv = sys.argv[1:]
    t_from = 1.6
    if "--tail" in argv:
        i = argv.index("--tail")
        t_from = float(argv[i + 1])
        argv = argv[:i] + argv[i + 2:]
    for p in argv:
        report(p, t_from)


if __name__ == "__main__":
    main()
