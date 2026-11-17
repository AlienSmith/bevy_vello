#!/usr/bin/env python3
"""Speed-tracking analysis for walker_qa telemetry (one CSV in, one report out).

The one question this answers: **does the commanded speed `v_des` reach the
body, and if not, which link of the chain eats it?**

The chain a working speed controller needs:

    v_des ──► placement law (stride length) ──► realized plant ──► body speed
              ▲                                                │
              └──────────── measured v_com (PLAN_MEAS_V) ◄──────┘

So the report is three layered tables:

1. **Per command segment** (split on `v_des`/`kind` changes) — the three speed
   channels side by side:
     - `v_des`   the command (px/s)
     - `cmd_vx`  the controller's own hip velocity (walker_math fiction `v`)
     - `v_filt`  observe's EMA of the measured whole-body CoM vx
     - `v_real`  central difference of the REAL hip particle `P3x`
   A controller "tracking the command" needs `v_real ≈ v_des`. `cmd_vx ≈
   v_des` while `v_real` sits at a constant is the signature of an **open
   speed loop**: the fiction obeys, the body does not.

2. **Per gait cycle, per foot** — what the command is *supposed* to change:
     - stride length (plant-to-plant, same foot) and cycle time
       (`speed = stride / cycle_time`, so if neither moves, speed cannot move)
     - stance slide (foot x drift while `gait == 0`), swing advance
     - touchdown quality: foot-behind-hip offset and the swing delivery ratio
       (realized plant vs the commanded target column `tPLLLx`)

3. **Command sensitivity** — least-squares slopes of realized speed, stride,
    and cycle time against `v_des`. A tracking controller has slope(speed) ≈ 1;
    a "stuck at 150 px/s" body has slope(speed) ≈ 0 with the *stride* or the
    *cadence* also at ≈ 0, which localizes the missing authority.

Usage:

    python3 tools/physics-qa/analyze_speed_tracking.py tmp/walker_qa/seeit.csv \
        [out.png]

`out.png` is optional (matplotlib, Agg); the text report always prints.
Columns are read by name, so an older CSV without a column degrades to a
sub-header rather than dying.
"""

from __future__ import annotations

import csv
import math
import statistics
import sys

# ---------------------------------------------------------------------------
# Loading + small numerics
# ---------------------------------------------------------------------------


def load(path: str):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = [r for r in reader]
    if not rows:
        sys.exit(f"empty csv: {path}")
    return rows


def col(rows, name: str):
    """Column as floats, or `None` when the CSV predates that column."""
    if name not in rows[0]:
        return None
    out = []
    for r in rows:
        v = r[name]
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            out.append(float("nan"))
    return out


def cen_diff(x, t):
    """Central difference d x / d t (px/s), one-sided at the ends."""
    n = len(x)
    out = [0.0] * n
    for i in range(n):
        a = max(i - 1, 0)
        b = min(i + 1, n - 1)
        dt = t[b] - t[a]
        out[i] = (x[b] - x[a]) / dt if dt > 0.0 else 0.0
    return out


def smooth(x, n: int):
    """Centered moving average over `n` samples (`n` odd-ish; 1 = identity)."""
    if n <= 1:
        return list(x)
    half = n // 2
    out = []
    for i in range(len(x)):
        a = max(i - half, 0)
        b = min(i + half + 1, len(x))
        vals = [v for v in x[a:b] if math.isfinite(v)]
        out.append(sum(vals) / len(vals) if vals else float("nan"))
    return out


def mean(xs):
    xs = [x for x in xs if math.isfinite(x)]
    return sum(xs) / len(xs) if xs else float("nan")


def sd(xs):
    xs = [x for x in xs if math.isfinite(x)]
    return statistics.pstdev(xs) if len(xs) > 1 else 0.0


def linfit(xs, ys):
    """Least-squares slope + intercept, skipping NaN pairs. `(None, None)`
    when there is no usable spread."""
    pts = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pts) < 3:
        return None, None
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    if sxx <= 1e-12:
        return None, None
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    m = sxy / sxx
    return m, my - m * mx


# ---------------------------------------------------------------------------
# Segments: split where the command (v_des) or the script cell kind changes
# ---------------------------------------------------------------------------


def segments(rows, v_des):
    """`[(i0, i1, v_des_key, kind)]`, adjacent equal keys merged."""
    keys = []
    for i, r in enumerate(rows):
        v = v_des[i] if v_des is not None else float("nan")
        # 0.5 px/s buckets: the rate-limited ramp inside one cell is one segment.
        keys.append((round(v / 0.5) if math.isfinite(v) else None, r.get("kind", "")))
    segs = []
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or keys[i] != keys[start]:
            segs.append((start, i, keys[start], rows[start].get("kind", "")))
            start = i
    return segs


# ---------------------------------------------------------------------------
# Gait events from the stance/swing flag columns (0 = stance, 1 = swing)
# ---------------------------------------------------------------------------


def edges(gait):
    """Indices where `gait` changes. Returns (liftoffs, touchdowns).

    `gait` is 0 = stance, 1 = swing, so 0→1 is liftoff and 1→0 touchdown.
    """
    lift, touch = [], []
    for i in range(1, len(gait)):
        if gait[i - 1] == 0 and gait[i] == 1:
            lift.append(i)
        elif gait[i - 1] == 1 and gait[i] == 0:
            touch.append(i)
    return lift, touch


def stance_intervals(gait):
    """`[(start, end)]` sample ranges where gait == 0."""
    out = []
    n = len(gait)
    i = 0
    while i < n:
        if gait[i] == 0:
            j = i
            while j + 1 < n and gait[j + 1] == 0:
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def swing_intervals(gait):
    """`[(start, end)]` sample ranges where gait == 1."""
    out = []
    n = len(gait)
    i = 0
    while i < n:
        if gait[i] == 1:
            j = i
            while j + 1 < n and gait[j + 1] == 1:
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    png = sys.argv[2] if len(sys.argv) > 2 else None

    rows = load(path)
    t = col(rows, "t")
    if t is None:
        sys.exit("no `t` column")
    n = len(rows)
    dt = statistics.median([t[i + 1] - t[i] for i in range(n - 1)])

    v_des = col(rows, "v_des")
    cmd_vx = col(rows, "cmd_vx")
    v_filt = col(rows, "v_filt")
    kinds = [r.get("kind", "") for r in rows]

    # --- the body's real speed: central difference of the hip particle ---------
    hip_x = col(rows, "P3x")
    if hip_x is None:
        sys.exit("no `P3x` column: not a walker_qa telemetry CSV")
    # The controller's own hip target (`tP3x`): the frame the commanded foot
    # columns (`tPLLLx`/`tPRLLx`) are expressed around. In this architecture the
    # pose hip IS the measured hip, so `tP3x == P3x` and either works.
    thip = col(rows, "tP3x")
    if thip is None:
        thip = hip_x
    v_raw = cen_diff(hip_x, t)
    # 2 gait cycles ≈ 0.56 s of smoothing: the per-cycle mean the DCM should see.
    n_smooth = max(1, int(round(0.56 / dt)) | 1)
    v_real = smooth(v_raw, n_smooth)

    print(f"file      : {path}")
    print(f"rows      : {n}  ({t[-1] - t[0]:.2f} s, dt = {dt * 1000:.2f} ms)")
    print(f"v_real    : d(P3x)/dt, smoothed over {n_smooth} samples "
          f"({n_smooth * dt:.2f} s)")

    # ------------------------------------------------------------------
    # 1. Per command segment
    # ------------------------------------------------------------------
    print("\n== 1. speed channels per command segment")
    head = (f"{'t0':>6} {'t1':>6} {'kind':>8} {'n':>5} "
            f"{'v_des':>7} {'cmd_vx':>7} {'v_filt':>7} {'v_real':>7} "
            f"{'real/des':>8} {'sd(real)':>8} {'stance%':>7}")
    print(head)
    print("-" * len(head))
    gait_l = col(rows, "gait_l")
    gait_r = col(rows, "gait_r")
    seg_rows = []
    for (i0, i1, _, kind) in segments(rows, v_des):
        if i1 - i0 < 10:
            continue
        # Skip the pre-gait stand cell: it says nothing about speed control.
        gait = []
        if gait_l is not None and gait_r is not None:
            gait = gait_l[i0:i1] + gait_r[i0:i1]
        stance_pct = 100.0 * (1.0 - mean(gait)) if gait else float("nan")
        vd = mean(v_des[i0:i1]) if v_des else float("nan")
        vr = mean(v_real[i0:i1])
        ratio = vr / vd if vd else float("nan")
        print(f"{t[i0]:6.2f} {t[i1 - 1]:6.2f} {kind:>8} {i1 - i0:5d} "
              f"{vd:7.1f} {mean(cmd_vx[i0:i1]):7.1f} {mean(v_filt[i0:i1]):7.1f} "
              f"{vr:7.1f} {ratio:8.2f} {sd(v_real[i0:i1]):8.1f} {stance_pct:7.1f}")
        seg_rows.append((t[i0], t[i1 - 1], kind, vd, vr))

    # ------------------------------------------------------------------
    # 2. Per gait cycle: what would move the speed
    # ------------------------------------------------------------------
    print("\n== 2. per-cycle stride / slide / delivery (per foot)")
    if gait_l is None or gait_r is None:
        print("  no gait_l/gait_r columns")
        cycles = []
    else:
        cycles = []  # dicts per touchdown-to-touchdown stride
        for name, gait, foot_x, hip, tgt_x in (
            ("L", gait_l, col(rows, "PLLLx"), hip_x, col(rows, "tPLLLx")),
            ("R", gait_r, col(rows, "PRLLx"), hip_x, col(rows, "tPRLLx")),
        ):
            _, touch = edges(gait)
            for k in range(len(touch) - 1):
                a, b = touch[k], touch[k + 1]
                stride = foot_x[b] - foot_x[a]
                cycle_t = t[b] - t[a]
                hip_adv = hip[b] - hip[a]
                # stance that ended at `a` = the stance interval starting at a
                st = [iv for iv in stance_intervals(gait) if iv[0] <= a <= iv[1]]
                slide = float("nan")
                if st:
                    s0, s1 = st[0]
                    slide = foot_x[s1] - foot_x[s0]
                sw = [iv for iv in swing_intervals(gait) if iv[0] <= b <= iv[1]]
                swing_adv = float("nan")
                if sw:
                    w0, w1 = sw[0]
                    swing_adv = foot_x[w1] - foot_x[w0]
                deliv = float("nan")
                if tgt_x is not None:
                    deliv = foot_x[b] / tgt_x[b] if tgt_x[b] else float("nan")
                cycles.append(dict(
                    foot=name, t=t[b], stride=stride, cycle_t=cycle_t,
                    speed=stride / cycle_t if cycle_t > 0 else float("nan"),
                    hip_adv=hip_adv, slide=slide, swing_adv=swing_adv,
                    behind=hip[b] - foot_x[b],
                    shortfall=(tgt_x[b] - foot_x[b]) if tgt_x is not None else float("nan"),
                    delivery=deliv,
                    v_des=v_des[b] if v_des is not None else float("nan"),
                ))

        if cycles:
            ch = (f"{'t_td':>6} {'foot':>4} {'v_des':>6} {'stride':>7} "
                  f"{'cyc_t':>6} {'speed':>6} {'hip_adv':>7} {'slide':>6} "
                  f"{'swing':>6} {'behind':>7} {'short':>6}")
            print(ch)
            print("-" * len(ch))
            step = max(1, len(cycles) // 24)
            for c in cycles[::step]:
                print(f"{c['t']:6.2f} {c['foot']:>4} {c['v_des']:6.1f} "
                      f"{c['stride']:7.1f} {c['cycle_t']:6.3f} {c['speed']:6.1f} "
                      f"{c['hip_adv']:7.1f} {c['slide']:6.1f} {c['swing_adv']:6.1f} "
                      f"{c['behind']:7.1f} {c['shortfall']:6.1f}")

            print(f"\n  medians over {len(cycles)} cycles (all feet, all commands):")
            for key, unit in (("stride", "px"), ("cycle_t", "s"), ("speed", "px/s"),
                              ("hip_adv", "px"), ("slide", "px"), ("swing_adv", "px"),
                              ("behind", "px"), ("shortfall", "px")):
                vals = [c[key] for c in cycles if math.isfinite(c[key])]
                if vals:
                    print(f"    {key:>10}: {statistics.median(vals):8.2f} {unit}")

    # ------------------------------------------------------------------
    # 2b. Swing delivery in the WORLD frame (the plant the law asked for vs
    #     the plant physics delivered) + the reach clamp it saturates on.
    # ------------------------------------------------------------------
    print("\n== 2b. swing delivery, world frame (target vs realized foot advance)")
    if gait_l is not None and gait_r is not None:
        deliv = []  # per-swing dicts
        for name, gait, foot_x, tgt_x in (
            ("L", gait_l, col(rows, "PLLLx"), col(rows, "tPLLLx")),
            ("R", gait_r, col(rows, "PRLLx"), col(rows, "tPRLLx")),
        ):
            if tgt_x is None:
                continue
            for (j0, j1) in swing_intervals(gait):
                if j1 - j0 < 8 or t[j0] < 2.5:
                    continue  # skip the spawn/settle artefacts
                adv_t = tgt_x[j1] - tgt_x[j0]
                adv_r = foot_x[j1] - foot_x[j0]
                if abs(adv_t) < 1e-6:
                    continue
                offs = [tgt_x[k] - thip[k] for k in (j0, j1)]
                spread = [tgt_x[k] - thip[k] for k in range(j0, j1 + 1)]
                deliv.append(dict(
                    foot=name, t=t[j1], v_des=v_des[j1] if v_des else float("nan"),
                    adv_t=adv_t, adv_r=adv_r, ratio=adv_r / adv_t,
                    off_lo=offs[0], off_td=offs[1],
                    clamp=max(spread),
                    foot_td=foot_x[j1] - hip_x[j1],
                ))
        if deliv:
            ch = (f"{'t_td':>6} {'foot':>4} {'v_des':>6} {'tgt_adv':>7} "
                  f"{'real_adv':>8} {'ratio':>6} {'off_lo':>7} {'off_td':>7} "
                  f"{'clamp':>6} {'foot_td':>8}")
            print(ch)
            print("-" * len(ch))
            stepn = max(1, len(deliv) // 18)
            for d in deliv[::stepn]:
                print(f"{d['t']:6.2f} {d['foot']:>4} {d['v_des']:6.1f} "
                      f"{d['adv_t']:7.1f} {d['adv_r']:8.1f} {d['ratio']:6.2f} "
                      f"{d['off_lo']:7.1f} {d['off_td']:7.1f} {d['clamp']:6.1f} "
                      f"{d['foot_td']:8.1f}")
            print("\n  medians by commanded speed (swing target advance is the plant"
                  " pattern the law asks for):")
            buckets = {}
            for d in deliv:
                key = round(d["v_des"] / 10.0) * 10 if math.isfinite(d["v_des"]) else 0
                buckets.setdefault(key, []).append(d)
            bh = (f"    {'v_des':>5} {'n':>4} {'tgt_adv':>8} {'real_adv':>8} "
                  f"{'ratio':>6} {'clamp':>6} {'foot_td':>8} {'implied v':>9}")
            print(bh)
            for key in sorted(buckets):
                ds = buckets[key]
                dt_t = statistics.median([d["adv_t"] for d in ds])
                dt_r = statistics.median([d["adv_r"] for d in ds])
                cyc = statistics.median([c["cycle_t"] for c in cycles]) if cycles else float("nan")
                # one step = half a cycle, so the target advance per step implies
                # a target cruising speed of 2*tgt_adv/cycle.
                implied = 2.0 * dt_t / cyc if cyc and math.isfinite(cyc) else float("nan")
                print(f"    {key:5.0f} {len(ds):4d} {dt_t:8.1f} {dt_r:8.1f} "
                      f"{statistics.median([d['ratio'] for d in ds]):6.2f} "
                      f"{statistics.median([d['clamp'] for d in ds]):6.1f} "
                      f"{statistics.median([d['foot_td'] for d in ds]):8.1f} "
                      f"{implied:9.1f}")
            print("  (a `clamp` that is flat across v_des = the reach limit is what"
                  " the placement is pinned to;\n   `implied v` ≫ v_des = the plant"
                  " pattern itself is asking for the wrong speed)")

    # ------------------------------------------------------------------
    # 2c. Swing tracking per decile: where does the foot lose its target?
    #     The command column is the pose foot the actuator is actually asked
    #     for (`tPLLLx`), the realized column is the physics foot (`PLLLx`).
    #     `gap` is realized − commanded, so a negative slope means the foot is
    #     falling BEHIND the re-planned target as the swing progresses.
    # ------------------------------------------------------------------
    print("\n== 2c. swing tracking per decile (medians over swings)")
    if gait_l is not None and gait_r is not None:
        bins = [i / 10 for i in range(1, 11)]
        acc = {b: {"cmd": [], "real": [], "gap": [], "hipv": []} for b in bins}
        nsw = 0
        for gait, foot_x, tgt_x in (
            (gait_l, col(rows, "PLLLx"), col(rows, "tPLLLx")),
            (gait_r, col(rows, "PRLLx"), col(rows, "tPRLLx")),
        ):
            if tgt_x is None or foot_x is None:
                continue
            for (i, j) in swing_intervals(gait):
                if j - i < 8 or t[i] < 2.5:
                    continue
                nsw += 1
                span = t[j] - t[i]
                if span <= 0:
                    continue
                for k in range(i, j + 1):
                    s = (t[k] - t[i]) / span
                    b = min(1.0, max(0.1, round(s * 10) / 10))
                    a, c = max(k - 4, 0), min(k + 4, len(t) - 1)
                    acc[b]["cmd"].append(tgt_x[k] - hip_x[k])
                    acc[b]["real"].append(foot_x[k] - hip_x[k])
                    acc[b]["gap"].append(foot_x[k] - tgt_x[k])
                    acc[b]["hipv"].append((hip_x[c] - hip_x[a]) / (t[c] - t[a])
                                          if t[c] > t[a] else 0.0)
        if nsw:
            print(f"  {nsw} swings (t > 2.5 s, both feet, all commands)")
            ch = (f"  {'s':>4} {'cmd_rel':>8} {'real_rel':>9} {'gap':>7} "
                  f"{'d(gap)':>7} {'hip v':>7}")
            print(ch)
            print("  " + "-" * (len(ch) - 2))
            prev = None
            for b in bins:
                d = acc[b]
                if not d["cmd"]:
                    continue
                gap = statistics.median(d["gap"])
                dg = "" if prev is None else f"{gap - prev:+7.1f}"
                prev = gap
                print(f"  {b:4.1f} {statistics.median(d['cmd']):8.1f} "
                      f"{statistics.median(d['real']):9.1f} {gap:7.1f} "
                      f"{dg:>7} {statistics.median(d['hipv']):7.1f}")
            print(f"  commanded relative sweep "
                  f"{statistics.median(acc[bins[-1]]['cmd']) - statistics.median(acc[bins[0]]['cmd']):.1f} px, "
                  f"realized {statistics.median(acc[bins[-1]]['real']) - statistics.median(acc[bins[0]]['real']):.1f} px")
            print("  (a `gap` that grows across the swing = the leg cannot follow the"
                  " re-planned target rate;\n   a constant gap = a fixed initial offset,"
                  " not a rate limit)")

    # ------------------------------------------------------------------
    # 2d. STANCE tracking: how far is the physics stance foot from the plant
    #     the algorithm commanded? During stance `tPLLLx` is a world-pinned
    #     constant (the plant), so `real − cmd` IS the plant error the whole
    #     control chain has to live with. If it drifts, the pendulum load the
    #     placement law assumes does not exist.
    # ------------------------------------------------------------------
    print("\n== 2d. stance tracking: physics foot vs commanded plant (world x)")
    if gait_l is not None and gait_r is not None:
        per_dec = {i / 10: [] for i in range(0, 11)}
        stances = []
        for gait, foot_x, tgt_x in (
            (gait_l, col(rows, "PLLLx"), col(rows, "tPLLLx")),
            (gait_r, col(rows, "PRLLx"), col(rows, "tPRLLx")),
        ):
            if tgt_x is None or foot_x is None:
                continue
            for (i, j) in stance_intervals(gait):
                if j - i < 8 or t[i] < 2.5:
                    continue
                span = float(j - i)
                for k in range(i, j + 1):
                    d = round((k - i) / span * 10) / 10
                    per_dec[min(1.0, d)].append(foot_x[k] - tgt_x[k])
                # The FIRST stance tick still carries the touchdown plant (the
                # law's demand); the second tick already reflects the one-shot
                # `plant_correction` re-anchoring the plant onto the measured
                # foot. Measure the hold from the second tick so the re-anchor
                # jump is not mistaken for command drift.
                i2 = min(i + 1, j)
                stances.append(dict(
                    t=t[i], dur=t[j] - t[i],
                    real_drift=foot_x[j] - foot_x[i2],
                    cmd_drift=tgt_x[j] - tgt_x[i2],
                    d0=foot_x[i] - tgt_x[i], d1=foot_x[j] - tgt_x[j],
                    d0b=foot_x[i2] - tgt_x[i2],
                    hip_adv=hip_x[j] - hip_x[i],
                ))
        if stances:
            ch = f"  {'s':>4} {'real−cmd':>9}"
            print(f"  {len(stances)} stances (t > 2.5 s, both feet)")
            print(ch)
            print("  " + "-" * (len(ch) - 2))
            for d in sorted(per_dec):
                if per_dec[d]:
                    print(f"  {d:4.1f} {statistics.median(per_dec[d]):9.1f}")
            print(f"\n  medians per stance:")
            for key, unit in (("dur", "s"), ("real_drift", "px"),
                              ("cmd_drift", "px"), ("d0", "px"), ("d0b", "px"),
                              ("d1", "px"), ("hip_adv", "px")):
                vals = [s[key] for s in stances]
                print(f"    {key:>11}: {statistics.median(vals):8.2f} {unit}")
            errs = [abs(s["d1"]) for s in stances]
            print(f"    |plant error| at stance end: median {statistics.median(errs):6.1f} px,"
                  f" max {max(errs):6.1f} px")
            print("  (cmd_drift ≈ 0 means the plant really is commanded fixed; a"
                  " nonzero real_drift is\nThe slip the stance leg must resist, and d1 is"
                  " the geometry the next swing inherits)")

    # ------------------------------------------------------------------
    # 3. Command sensitivity: what does v_des actually change?
    # ------------------------------------------------------------------
    print("\n== 3. what the command changes (slope vs v_des; 1.0 = tracking)")
    if cycles:
        xs = [c["v_des"] for c in cycles]
        for key, unit in (("speed", "px/s per px/s"), ("stride", "px per px/s"),
                          ("cycle_t", "s per px/s"), ("slide", "px per px/s"),
                          ("behind", "px per px/s")):
            m, b = linfit(xs, [c[key] for c in cycles])
            if m is None:
                print(f"  {key:>10}: no spread in v_des (single command) — "
                      f"median {statistics.median([c[key] for c in cycles if math.isfinite(c[key])]):.2f}")
            else:
                print(f"  {key:>10}: slope {m:+.4f} {unit:<18}"
                      f" (at v_des=0 → {b:+.1f})")
        # Realized speed vs the command it was asked for, per cycle.
        pts = [(c["v_des"], c["speed"]) for c in cycles
               if math.isfinite(c["v_des"]) and math.isfinite(c["speed"])]
        if pts:
            errs = [s - v for v, s in pts]
            print(f"  speed error (realized − v_des): mean {mean(errs):+.1f} px/s,"
                  f" sd {sd(errs):.1f}, min {min(errs):+.1f}, max {max(errs):+.1f}")

    # ------------------------------------------------------------------
    # Verdict
    # ------------------------------------------------------------------
    print("\n== verdict")
    if seg_rows:
        vds = [s[3] for s in seg_rows if s[3] > 1.0]
        vrs = [s[4] for s in seg_rows if s[3] > 1.0]
        if len(set(round(v, 0) for v in vds)) > 1:
            spread_des = max(vds) - min(vds)
            spread_real = max(vrs) - min(vrs)
            print(f"  command span {min(vds):.0f}..{max(vds):.0f} px/s "
                  f"({spread_des:.0f} px/s); realized span "
                  f"{min(vrs):.0f}..{max(vrs):.0f} px/s ({spread_real:.0f} px/s)")
            if spread_des > 5.0 and spread_real < 0.25 * spread_des:
                print("  => speed is STUCK: the body barely responds to the command.")
            elif spread_real >= 0.75 * spread_des:
                print("  => speed tracks the command.")
            else:
                print("  => partial tracking: the body responds, but attenuated.")
    if cycles:
        m, _ = linfit([c["v_des"] for c in cycles], [c["stride"] for c in cycles])
        mc, _ = linfit([c["v_des"] for c in cycles], [c["cycle_t"] for c in cycles])
        if m is not None and mc is not None:
            print(f"  stride sensitivity {m:+.3f} px per px/s, "
                  f"cycle-time sensitivity {mc:+.4f} s per px/s")
            print("  (stride must grow and/or cycle time shrink for speed to rise)")

    # ------------------------------------------------------------------
    # Optional plot
    # ------------------------------------------------------------------
    if png:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print(f"  (matplotlib missing: skipped {png})")
        else:
            fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
            ax[0].plot(t, v_des, "k--", lw=2, label="v_des (command)")
            ax[0].plot(t, cmd_vx, "tab:blue", lw=1, label="cmd_vx (fiction)")
            ax[0].plot(t, v_filt, "tab:orange", lw=1, label="v_filt (measured EMA)")
            ax[0].plot(t, v_real, "tab:green", lw=2, label="v_real (dP3x/dt)")
            ax[0].set_ylabel("px/s")
            ax[0].legend(loc="upper right", fontsize=8)
            ax[0].set_title("speed channels")
            if cycles:
                ax[1].plot([c["t"] for c in cycles], [c["speed"] for c in cycles],
                           "o-", ms=3, label="per-cycle speed (stride/cycle)")
                ax[1].plot([c["t"] for c in cycles], [c["v_des"] for c in cycles],
                           "k--", lw=2, label="v_des")
                ax[1].set_ylabel("px/s")
                ax[1].legend(loc="upper right", fontsize=8)
                ax[1].set_title("realized cycle speed vs command")
            ax[1].set_xlabel("t (s)")
            fig.tight_layout()
            fig.savefig(png, dpi=120)
            print(f"\nwrote {png}")


if __name__ == "__main__":
    main()
