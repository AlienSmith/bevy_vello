#!/usr/bin/env python3
"""walk_ab_metrics.py — one consistent A/B metric row per walk CSV.

Physical velocity is the finite difference of the resolved hip particle
(`P3x`), NOT the sampled `tP3vx` (which is the teleport-corrected source
velocity and over-reports). Windows are 0.5 s medians of that difference.

Reported per run:
  onset    first window with v > +15 px/s (s)
  fwd      seconds with v > +15
  back     seconds with v < -15      (the failure we care about)
  net      P3x(last) - P3x(first) after the settle skip
  vmax/vmin  extreme window medians
  tilt     max |body_theta| in degrees

Usage:  python3 tools/physics-qa/walk_ab_metrics.py a.csv [b.csv ...]
"""

from __future__ import annotations

import csv
import io
import os
import statistics
import sys

SKIP_START = 1.0   # s, ignore the settle transient
SKIP_END = 0.4     # s, ignore the run-out tail
WIN = 0.5          # s, window length
FWD_GATE = 15.0    # px/s
BACK_GATE = -15.0  # px/s


def load(path: str):
    with open(path, newline="", encoding="utf-8", errors="replace") as fh:
        reader = csv.DictReader(fh)
        rows = [r for r in reader if r]
    t, x, th = [], [], []
    for r in rows:
        try:
            t.append(float(r["t"]))
            x.append(float(r["P3x"]))
            th.append(abs(float(r["body_theta"])))
        except (KeyError, ValueError):
            continue
    return t, x, th


def windowed_velocity(t, x, win=WIN):
    """Median physical velocity per contiguous window of `win` seconds."""
    if len(t) < 4:
        return []
    out = []
    i = 0
    while i < len(t):
        j = i
        while j < len(t) and t[j] - t[i] < win:
            j += 1
        if j - i >= 3 and t[j - 1] > t[i]:
            # per-sample velocities inside the window
            vs = []
            for k in range(i + 1, j):
                dt = t[k] - t[k - 1]
                if dt > 1e-9:
                    vs.append((x[k] - x[k - 1]) / dt)
            if vs:
                out.append((0.5 * (t[i] + t[j - 1]), statistics.median(vs)))
        i = j
    return out


def summarize(path: str):
    t, x, th = load(path)
    if len(t) < 20:
        return {"tag": os.path.basename(path), "error": "too few rows"}
    t0, t1 = t[0] + SKIP_START, t[-1] - SKIP_END
    keep = [i for i in range(len(t)) if t0 <= t[i] <= t1] or list(range(len(t)))
    i0, i1 = keep[0], keep[-1]
    net = x[i1] - x[i0]

    wv = windowed_velocity(t, x)
    wv = [p for p in wv if t0 <= p[0] <= t1]

    fwd = back = 0.0
    onset_t = None
    for wt, v in wv:
        if onset_t is None and v > FWD_GATE:
            onset_t = wt
        if v > FWD_GATE:
            fwd += WIN
        if v < BACK_GATE:
            back += WIN

    vs = [v for _, v in wv]
    return {
        "tag": os.path.basename(path).replace(".csv", ""),
        "rows": len(t),
        "dur": round(t[-1] - t[0], 2),
        "onset": None if onset_t is None else round(onset_t, 2),
        "fwd": round(fwd, 1),
        "back": round(back, 1),
        "net": round(net, 1),
        "vmax": None if not vs else round(max(vs), 1),
        "vmin": None if not vs else round(min(vs), 1),
        "tilt": round(max(th), 2),
    }


FIELDS = ["tag", "rows", "dur", "onset", "fwd", "back", "net", "vmax", "vmin", "tilt"]


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    paths = [p for p in argv[1:] if os.path.exists(p)]
    missing = [p for p in argv[1:] if not os.path.exists(p)]
    if missing:
        print(f"# missing: {', '.join(missing)}", file=sys.stderr)

    table = [summarize(p) for p in paths]
    widths = {f: max(len(f), *(len(str(r.get(f, ""))) for r in table)) for f in FIELDS}
    line = "  ".join(f.ljust(widths[f]) for f in FIELDS)
    print(line)
    print("  ".join("-" * widths[f] for f in FIELDS))
    for r in table:
        print("  ".join(str(r.get(f, "")).ljust(widths[f]) for f in FIELDS))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
