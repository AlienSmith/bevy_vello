#!/usr/bin/env python3
"""Plot the hip (P3) and both feet (PLLL/PRLL) position vs time from a
headless walk CSV.

Two panels: x and y (flipped so up = positive), with gait-phase shading
(pink = left swing, blue = right swing). Hip is black, left foot is a
darker pink, right foot a darker blue, so foot/phase colors pair up.
Usage:

    python3 tools/physics-qa/plot_p3.py tmp/headless/viz/walk.csv [out.png]
"""

import csv
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path, targets):
    t, hip, lft, rft, gl, gr = [], [], [], [], [], []
    lt, rt = [], []  # commanded foot targets (x, y) if wanted
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            hip.append((float(row["P3x"]), -float(row["P3y"])))  # up is +
            lft.append((float(row["PLLLx"]), -float(row["PLLLy"])))
            rft.append((float(row["PRLLx"]), -float(row["PRLLy"])))
            if targets:
                lt.append((float(row["tPLLLx"]), -float(row["tPLLLy"])))
                rt.append((float(row["tPRLLx"]), -float(row["tPRLLy"])))
            gl.append(int(row["gait_l"]))
            gr.append(int(row["gait_r"]))
    hip = np.array(hip)
    lft = np.array(lft)
    rft = np.array(rft)
    return (
        np.array(t),
        hip,
        lft,
        rft,
        np.array(gl),
        np.array(gr),
        np.array(lt) if targets else None,
        np.array(rt) if targets else None,
    )


def main():
    args = [a for a in sys.argv[1:] if a != "--targets"]
    targets = "--targets" in sys.argv
    if len(args) < 1:
        sys.exit(__doc__)
    path = args[0]
    out = args[1] if len(args) > 1 else path.rsplit(".", 1)[0] + ".p3.png"
    t, hip, lft, rft, gl, gr, lt, rt = load(path, targets)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    def shade(ax):
        for sig, c in ((gl, "#ffc2d1"), (gr, "#bfd4ff")):
            d = np.diff(np.concatenate(([0], sig, [0])))
            for a, b in zip(np.where(d == 1)[0], np.where(d == -1)[0]):
                ax.axvspan(t[a], t[min(b, len(t) - 1)], color=c, alpha=0.35, lw=0)

    def curves(ax, col, ylab, title=None):
        shade(ax)  # bands first, so the curves draw on top
        ax.plot(t, hip[:, col], lw=1.4, color="k", label="hip P3")
        ax.plot(t, lft[:, col], lw=1.0, color="#d6336c", label="L foot PLLL")
        ax.plot(t, rft[:, col], lw=1.0, color="#1971c2", label="R foot PRLL")
        if targets:
            ax.plot(
                t,
                lt[:, col],
                lw=0.8,
                ls="--",
                color="#d6336c",
                alpha=0.7,
                label="L target",
            )
            ax.plot(
                t,
                rt[:, col],
                lw=0.8,
                ls="--",
                color="#1971c2",
                alpha=0.7,
                label="R target",
            )
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", ncol=3)
        ax.set_ylabel(ylab)
        if title:
            ax.set_title(title)

    curves(ax1, 0, "x (px)", f"Hip + feet position vs time — {path}")
    curves(ax2, 1, "y (px, up +)")
    ax2.set_xlabel("t (s)")

    fig.tight_layout()
    fig.savefig(out, dpi=110)

    dur = t[-1] - t[0]
    print(f"wrote {out}")
    for nm, a in (("hip", hip), ("L foot", lft), ("R foot", rft)):
        print(
            f"{nm}: x {a[0,0]:7.1f} -> {a[-1,0]:7.1f}  "
            f"y min {a[:,1].min():7.1f} max {a[:,1].max():6.1f}"
        )
    print(f"avg hip speed {(hip[-1,0]-hip[0,0])/dur:.1f} px/s over {dur:.1f} s")


if __name__ == "__main__":
    main()
