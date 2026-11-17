#!/usr/bin/env python3
"""analyze_swing_pacing.py — when does the swing PROFILE finish, vs when the swing ENDS?

    python3 tools/physics-qa/analyze_swing_pacing.py <csv> [...] [--step-height H]

The three-phase profile (`stepping.rs::swing_profile_foot`) maps progress
`s ∈ [0,1]` from `swing_from` to `swing_to`, and `s` is paced *deadbeat*:

    t_target = (|swing_to.x - plant.x| / max(|v|,1)).min(swing_duration_ts)
    s        = 1 - phase_timer / t_target

but the swing only ENDS on measured touchdown (`GaitAuthority::Full`). If the
contact arrives after `t_target`, `s` has already clamped at 1 and the command
is pinned to `swing_to` — which is re-planned every tick and falls toward the
hip as the remaining flight time shrinks. The last part of the swing is then a
RETREAT, not an advance.

Detects saturation without the internal `s`: `act == dcm` exactly means the
profile has arrived at the live target (`s = 1`). Reports, per swing:

  t_lift  t_td    swing length, and t_target at liftoff
  sat_t   first tick where act == dcm  (NaN if it never saturates)
  frac    (sat_t - t_lift) / (t_td - t_lift)   <-- 1.0 = healthy
  aim_l / aim_td   the aim at liftoff / touchdown
  peak_cmd         max(act - hip) during the swing
  real_td          foot - hip at touchdown
  retreat          peak_cmd - aim_td  (how far the command walked BACK)
  miss             aim_td - real_td   (how far the foot fell short)
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


SWING = 0.28   # config.swing_duration_ts
TS = 1.0 / 90.0

SIDE = {
    "L": ("gait_l", "tPLLLx", "planLx", "PLLLx"),
    "R": ("gait_r", "tPRLLx", "planRx", "PRLLx"),
}


def swings(rows, gait, act, dcm, real):
    """Contiguous swing runs (gait_* == 1 means SWING)."""
    runs, cur = [], []
    for r in rows:
        if r.get(gait, 0.0) > 0.5:
            cur.append(r)
        else:
            if len(cur) >= 4:
                runs.append(cur)
            cur = []
    return runs


def report(path):
    rows = load(path)
    print(f"\n=== {path} ===")
    print(
        " foot  t_lift  t_td  dur  t_tgt |  sat_t  frac |  aim_l aim_td "
        "peak_cmd real_td  retreat  miss"
    )
    acc = []
    for side, (gait, act, dcm, real) in SIDE.items():
        for sw in swings(rows, gait, act, dcm, real):
            a, b = sw[0], sw[-1]
            hip_a, hip_b = a["P3x"], b["P3x"]
            aim_l = a[dcm] - hip_a
            aim_td = b[dcm] - hip_b
            # world-frame aim at liftoff, for the pacing estimate
            dist = abs(a[act] - a[dcm])  # rough: cmd start vs its aim
            t_tgt = min(dist / max(abs(a["cmd_vx"]), 1.0), SWING)
            sat = None
            peak = None
            for r in sw:
                off_act = r[act] - r["P3x"]
                if peak is None or off_act > peak[1]:
                    peak = (r["t"], off_act)
                if abs(r[act] - r[dcm]) < 0.05 and sat is None:
                    sat = r["t"]
            dur = b["t"] - a["t"]
            frac = (sat - a["t"]) / dur if sat is not None else 1.0
            retreat = peak[1] - aim_td
            miss = aim_td - (b[real] - hip_b)
            print(
                f"  {side}  {a['t']:6.2f}{b['t']:6.2f}{dur:6.3f}{t_tgt:6.3f} |"
                f"{'  none' if sat is None else f'{sat:7.3f}'}{frac:6.2f} |"
                f"{aim_l:7.1f}{aim_td:7.1f}{peak[1]:9.1f}{b[real] - hip_b:8.1f}"
                f"{retreat:9.1f}{miss:6.1f}"
            )
            acc.append((frac, retreat, miss, dur, t_tgt))
    if acc:
        n = len(acc)
        srt = lambda i: sorted(a[i] for a in acc)
        print(
            f"  n={n}  median frac-to-saturation {srt(0)[n // 2]:.2f}  "
            f"retreat {srt(1)[n // 2]:+.1f} px  miss {srt(2)[n // 2]:+.1f} px  "
            f"dur {srt(3)[n // 2]:.3f} s  t_target {srt(4)[n // 2]:.3f} s"
        )


for p in sys.argv[1:]:
    report(p)
