#!/usr/bin/env python3
"""Compare headless probe runs: baseline vs one-way coupling.

Metrics (per run):
  walk: center x displacement during D_held; lateral sigma of PH and P3
        (waggle proxy); collider-follow lag (mean collider pos - frame center).
  glide: time for |vx| to fall to 10% of release value; residual drift after 2 s.
  Optional synthetic-hit section if SYNTH rows present (phase == "synth").
"""
import csv, sys, math

def load(path):
    rows = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    return rows

def f(row, k):
    v = row.get(k, "nan")
    try:
        return float(v)
    except ValueError:
        return float("nan")

def stats(rows):
    out = {}
    baseline_end, input_end = 2.0, 4.4
    held = [r for r in rows if baseline_end <= f(r, "t") < input_end]
    after = [r for r in rows if f(r, "t") >= input_end]
    # walk displacement
    if held:
        out["walk_dx"] = f(held[-1], "center_x") - f(held[0], "center_x")
        out["walk_dy"] = f(held[-1], "center_y") - f(held[0], "center_y")
    # lateral waggle sigma during hold (perpendicular axis = y)
    for name in ("PH", "P3"):
        ys = [f(r, f"{name}_y") for r in held]
        ys = [y for y in ys if not math.isnan(y)]
        if len(ys) > 2:
            m = sum(ys) / len(ys)
            out[f"{name}_sigma_y"] = math.sqrt(sum((y - m) ** 2 for y in ys) / len(ys))
    # collider follow lag during hold
    lags = []
    for r in held:
        cx, cy, ccx, ccy = f(r, "center_x"), f(r, "center_y"), f(r, "col_cx"), f(r, "col_cy")
        if not any(math.isnan(v) for v in (cx, cy, ccx, ccy)):
            lags.append(math.hypot(ccx - cx, ccy - cy))
    if lags:
        out["follow_lag_mean"] = sum(lags) / len(lags)
        out["follow_lag_max"] = max(lags)
    # glide: vx of center decay after release
    if after:
        vxs = [(f(r, "t"), f(r, "center_x")) for r in after]
        if len(vxs) > 2:
            (t0, x0), _ = vxs[0], vxs[-1]
            (t1, x1) = next(((t, x) for t, x in vxs if t >= t0 + 2.0), vxs[-1])
            out["drift_2s_after_release"] = x1 - x0
    # raw center speed at release
    if len(rows) > 2:
        r0 = rows[max(0, len(rows) - 25)]
        r1 = rows[-1]
        out["end_pos"] = (f(r1, "center_x"), f(r1, "center_y"))
    return out

def main():
    a, b = sys.argv[1], sys.argv[2]
    ra, rb = load(a), load(b)
    sa, sb = stats(ra), stats(rb)
    print(f"{'metric':<28}{'baseline':>16}{'one-way':>16}")
    for k in sorted(set(sa) | set(sb)):
        va, vb = sa.get(k), sb.get(k)
        fa = f"{va:.3f}" if isinstance(va, float) else str(va)
        fb = f"{vb:.3f}" if isinstance(vb, float) else str(vb)
        print(f"{k:<28}{fa:>16}{fb:>16}")

if __name__ == "__main__":
    main()
