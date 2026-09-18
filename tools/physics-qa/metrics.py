#!/usr/bin/env python3
"""Objective stability metrics for a captured frame burst.

The vision judge explains *why* a run looks wrong; these numbers decide *whether*
it is wrong. Vision verdicts flip on near-identical frames, so the tuning loop
must not gate on them alone.

Two properties of these captures shape the design:

1. A debug UI panel occupies the left ~25% of the window. It is static and would
   swamp the statistics, so it is detected and excluded.
2. The camera is fixed and the arena is drawn with a lot of static detail. A
   simple "differs from the background" mask is therefore ~98% unchanging arena
   geometry, not character. Foreground area is *not* a usable character measure;
   what moves between frames is. The gates below are temporal.

Usage:
  metrics.py frame_01.png frame_02.png frame_03.png
  metrics.py --json shots_run3/frame_*.png

Prints one JSON object. The fields that gate are `mean_rmse`, `motion_frac` and
`motion_growth`; the rest are diagnostic.

Reference points measured on recorded runs:

  run    mean_rmse  motion_frac  verdict
  -----  ---------  -----------  -------
  2          6.56        0.030   stable (vision: OK)
  3         23.61        0.332   particle VFX covering the body (vision: EXPLOSION)
  quiet      2.54        0.008   stable, no collision in the capture window
"""
import argparse
import json
import sys

import numpy as np
from PIL import Image

# Gates. The stable runs sit at mean_rmse 2.5-6.6 and the VFX run at 23.6, so the
# RMSE gate is set between them with margin on both sides.
GATE_RMSE = 12.0
# Fraction of arena pixels that change across the burst. Stable runs stay under
# 0.03; a burst of particles or a body flying apart covers far more.
GATE_MOTION_FRAC = 0.15
# How much the changed-pixel bounding box grows between consecutive pairs. A body
# holding its shape keeps this near 1; a diverging one keeps expanding.
GATE_MOTION_GROWTH = 2.5

# A pixel counts as changed when its channel range across the burst exceeds this.
# Below it is renderer dithering and anti-aliasing noise.
MOTION_TOL = 20


def load(paths):
    imgs = [np.asarray(Image.open(p).convert("RGB"), dtype=np.float64) for p in paths]
    shapes = {im.shape for im in imgs}
    if len(shapes) != 1:
        raise SystemExit(f"frames differ in size: {sorted(shapes)}")
    if len(imgs) < 2:
        raise SystemExit("need at least 2 frames to measure motion")
    return np.stack(imgs)


def panel_width(stack, dark=(24, 24, 24), tol=20, thresh=0.2, sustain=40):
    """Width of the static dark UI panel at the left of the frame, or 0 if none.

    The panel is drawn with a small margin and contains text and widgets, so it is
    neither flush to column 0 nor uniformly dark. The column darkness profile is
    smoothed to bridge those gaps, then the panel is taken to end at the first
    column whose smoothed darkness drops below `thresh` and stays there for
    `sustain` columns. Detected on the mean frame so a transient effect cannot
    move the edge.
    """
    mean = stack.mean(axis=0)
    w = mean.shape[1]
    is_dark = np.abs(mean - np.array(dark, dtype=np.float64)).max(axis=2) < tol
    cf = is_dark.mean(axis=0)
    k = 25
    smooth = np.convolve(cf, np.ones(k) / k, mode="same")

    limit = int(w * 0.6)  # the panel never occupies most of the frame
    for c in range(1, limit):
        if smooth[c] < thresh and smooth[c:c + sustain].max(initial=0.0) < thresh:
            return c if c > w * 0.02 else 0
    return 0


def bbox_of(mask):
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    return [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]


def bbox_area(b):
    return (b[2] - b[0] + 1) * (b[3] - b[1] + 1) if b else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true", help="compact output")
    ap.add_argument("--gate-rmse", type=float, default=GATE_RMSE)
    ap.add_argument("--gate-motion-frac", type=float, default=GATE_MOTION_FRAC)
    ap.add_argument("--gate-motion-growth", type=float, default=GATE_MOTION_GROWTH)
    ap.add_argument("frames", nargs="+")
    args = ap.parse_args()

    stack = load(args.frames)
    n, h, w, _ = stack.shape

    pw = panel_width(stack)
    arena = stack[:, :, pw:, :]
    ah, aw = arena.shape[1], arena.shape[2]
    arena_px = ah * aw

    # --- temporal change --------------------------------------------------
    dev = arena.max(axis=0) - arena.min(axis=0)
    motion = dev.max(axis=2) > MOTION_TOL
    motion_px = int(motion.sum())
    motion_frac = motion_px / arena_px
    motion_bbox = bbox_of(motion)

    # Whether the disturbance reaches the arena border: a body leaving the arena
    # shows up here.
    border = np.zeros((ah, aw), dtype=bool)
    border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
    edge_frac = int((motion & border).sum()) / motion_px if motion_px else 0.0

    # Per-pair changed area, to see the disturbance growing frame over frame.
    pair_fills = []
    for i in range(n - 1):
        d = np.abs(arena[i] - arena[i + 1]).max(axis=2) > MOTION_TOL
        pair_fills.append(bbox_area(bbox_of(d)) / arena_px)
    nz = [f for f in pair_fills if f > 0]
    motion_growth = (max(nz) / min(nz)) if len(nz) > 1 and min(nz) > 0 else 1.0

    # --- frame-to-frame difference ---------------------------------------
    rmse = []
    for i in range(n - 1):
        d = arena[i] - arena[i + 1]
        rmse.append(float(np.sqrt((d * d).mean())))
    mean_rmse = float(np.mean(rmse))

    # --- diagnostics ------------------------------------------------------
    # Dominant arena colour, reported so a capture can be sanity-checked (a black
    # or blank frame is obvious here). Deliberately not gated.
    mean_frame = stack.mean(axis=0)[:, pw:, :]
    bg_q = (mean_frame.reshape(-1, 3) // 4) * 4
    uniq, counts = np.unique(bg_q, axis=0, return_counts=True)
    bg = uniq[counts.argmax()]

    suspect = []
    if motion_px == 0:
        suspect.append("no_motion")
    else:
        if mean_rmse > args.gate_rmse:
            suspect.append(f"mean_rmse={mean_rmse:.1f}>{args.gate_rmse}")
        if motion_frac > args.gate_motion_frac:
            suspect.append(f"motion_frac={motion_frac:.3f}>{args.gate_motion_frac}")
        if motion_growth > args.gate_motion_growth:
            suspect.append(f"motion_growth={motion_growth:.2f}>{args.gate_motion_growth}")
        if edge_frac > 0.02:
            suspect.append(f"edge_frac={edge_frac:.3f}>0.02")

    result = {
        "ok": not suspect,
        "frames": n,
        "panel_cols": [0, pw - 1] if pw else None,
        "arena_cols": [pw, w - 1],
        "mean_rmse": round(mean_rmse, 2),
        "frame_rmse": [round(r, 2) for r in rmse],
        "motion_px": motion_px,
        "motion_frac": round(motion_frac, 4),
        "motion_bbox": motion_bbox,
        "motion_growth": round(motion_growth, 3),
        "edge_frac": round(edge_frac, 4),
        "bg_color": [int(c) for c in bg],
        "suspect": suspect,
    }
    print(json.dumps(result, indent=None if args.json else 2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
