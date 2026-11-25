#!/usr/bin/env python3
"""compare_runs.py — frame-by-frame diff of two walker_qa telemetry CSVs.

Both runs are fixed-timestep, so row i of A is the same instant as row i of B.
Use it to answer "what differs between the expert and a policy (or between two
checkpoints / spawn positions), and WHEN does it first show up?"

    python3 tools/physics-qa/compare_runs.py A.csv B.csv [--rel 1e-3] [--abs 1e-6]
                                                         [--top 25] [--window 12]

Reports:
  1. the takeover frame — first row where `mode` becomes `jump` (the residual
     policy only acts from there; before it both runs should be identical);
  2. columns that already differ BEFORE takeover (should be none for the same
     spawn x; nonzero means the starts were not the same);
  3. per-column first divergence after takeover, earliest first;
  4. the first `--window` frames after takeover: the columns with the largest
     scale-normalised difference, so you can see which channel moves first.

A column "differs" at a frame when |a-b| > max(--abs, --rel * range_of_column_in_A).

Note: the CSV records physical state only (particle pos/vel/acc, lean, ω, ang
momentum, grounded). It has NO policy action / residual column, so the policy's
strategy has to be read off the state it produces.
"""
import argparse
import csv
import math


def load(path):
    rows = list(csv.DictReader(open(path)))
    cols = list(rows[0].keys())
    num, cat = {}, {}
    for c in cols:
        vals = [r[c] for r in rows]
        try:
            num[c] = [float(v) for v in vals]
        except ValueError:
            cat[c] = vals
    return rows, num, cat


def takeover(cat, name="mode", target="jump"):
    vals = cat.get(name, [])
    return next((i for i, v in enumerate(vals) if v == target), None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--rel", type=float, default=1e-3)
    ap.add_argument("--abs", dest="abs_", type=float, default=1e-6)
    ap.add_argument("--top", type=int, default=25)
    ap.add_argument("--window", type=int, default=12)
    args = ap.parse_args()

    ra, na, ca = load(args.a)
    rb, nb, cb = load(args.b)
    n = min(len(ra), len(rb))
    t = na["t"]
    print(f"A={args.a} ({len(ra)} rows)\nB={args.b} ({len(rb)} rows)  comparing first {n}")

    ta, tb = takeover(ca), takeover(cb)
    print(f"takeover (mode->jump): A row {ta} (t={t[ta]:.3f})  B row {tb} (t={nb['t'][tb]:.3f})")
    k = ta if ta is not None else 0
    if ta != tb:
        print("  !! takeover frames differ; using A's")

    cols = [c for c in na if c in nb and c != "t"]
    scale = {}
    for c in cols:
        vs = [v for v in na[c][:n] if not math.isnan(v)]
        scale[c] = (max(vs) - min(vs)) if vs else 0.0

    def diff(c, i):
        a, b = na[c][i], nb[c][i]
        if math.isnan(a) and math.isnan(b):
            return 0.0
        if math.isnan(a) or math.isnan(b):
            return float("inf")
        return abs(a - b)

    def thr(c):
        return max(args.abs_, args.rel * scale[c])

    # categorical columns (mode, ext_pin, ...) — report if they ever differ
    for c in ca:
        if c in cb:
            d = next((i for i in range(n) if ca[c][i] != cb[c][i]), None)
            if d is not None:
                print(f"categorical '{c}' first differs at row {d} (t={t[d]:.3f}): {ca[c][d]!r} vs {cb[c][d]!r}")

    # 2. before takeover
    pre = []
    for c in cols:
        first = next((i for i in range(k) if diff(c, i) > thr(c)), None)
        if first is not None:
            pre.append((first, c, max(diff(c, i) for i in range(k))))
    print(f"\n== differences BEFORE takeover (rows 0..{k - 1}, t<{t[k]:.3f}) ==")
    if not pre:
        print("none: the two runs are identical up to the frame the policy takes over.")
    else:
        for first, c, mx in sorted(pre)[: args.top]:
            print(f"  {c:10s} first differs row {first:4d} (t={t[first]:.3f})  max|diff|={mx:.4g}  (range {scale[c]:.3g})")
        print(f"  ({len(pre)} columns differ before takeover)")

    # 3. first divergence after takeover
    post = []
    for c in cols:
        first = next((i for i in range(k, n) if diff(c, i) > thr(c)), None)
        if first is not None:
            post.append((first, c))
    print(f"\n== first divergence AFTER takeover (earliest {args.top}) ==")
    for first, c in sorted(post)[: args.top]:
        print(
            f"  row {first:4d} (+{first - k:3d} frames, t={t[first]:.3f})  {c:10s} "
            f"A={na[c][first]:10.4f} B={nb[c][first]:10.4f}  diff={diff(c, first):.4g}"
        )
    print(f"  ({len(post)} of {len(cols)} columns diverge after takeover)")

    # 4. which channel moves first, scale-normalised, frame by frame
    print(f"\n== first {args.window} frames after takeover: top normalised diffs per frame ==")
    for i in range(k, min(n, k + args.window)):
        sc = []
        for c in cols:
            if scale[c] <= 0:
                continue
            d = diff(c, i)
            if d == float("inf"):
                continue
            sc.append((d / scale[c], c, d))
        sc.sort(reverse=True)
        tops = "  ".join(f"{c}:{d:.3g}" for _, c, d in sc[:4])
        print(f"  +{i - k:2d} t={t[i]:.3f}  {tops}")


if __name__ == "__main__":
    main()
