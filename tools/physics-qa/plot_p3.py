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
    vx, vcmd = [], []  # measured hip vx (P3vx) + fiction commanded vx (cmd_vx)
    v_filt, v_des = [], []  # persistent filtered reality + desired velocity (new columns)
    has_filt = False  # column present only in CSVs from the HIP_V_MIX harness build
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            hip.append((float(row["P3x"]), -float(row["P3y"])))  # up is +
            lft.append((float(row["PLLLx"]), -float(row["PLLLy"])))
            rft.append((float(row["PRLLx"]), -float(row["PRLLy"])))
            vx.append(float(row.get("P3vx") or 0.0))
            vcmd.append(float(row.get("cmd_vx") or 0.0))
            if "v_filt" in row:
                has_filt = True
                v_filt.append(float(row.get("v_filt") or 0.0))
                v_des.append(float(row.get("v_des") or 0.0))
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
        np.array(vx),
        np.array(vcmd),
        np.array(v_filt) if has_filt else None,
        np.array(v_des) if has_filt else None,
    )


def main():
    args = [a for a in sys.argv[1:] if a != "--targets"]
    targets = "--targets" in sys.argv
    if len(args) < 1:
        sys.exit(__doc__)
    path = args[0]
    out = args[1] if len(args) > 1 else path.rsplit(".", 1)[0] + ".p3.png"
    t, hip, lft, rft, gl, gr, lt, rt, vx, vcmd, v_filt, v_des = load(path, targets)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 11), sharex=True)

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
    # Hip x-velocity panel: measured P3vx (black) vs fiction command (green),
    # so speed changes and brake/launch transients read directly off the plot.
    # When the harness records them (HIP_V_MIX builds), also draw the
    # PERSISTENT signals the placement law brakes against: v_filt = EMA of the
    # measured CoM velocity (the stable reality the DCM sees) and v_des = the
    # rate-limited desired command. The transient per-frame fiction (cmd_vx) is
    # drawn thin/behind; the filtered reality leads visually (thick blue).
    shade(ax3)
    ax3.plot(t, vx, lw=1.4, color="k", alpha=0.5, label="P3vx (measured)")
    if v_filt is not None:
        ax3.plot(t, v_des, lw=1.0, ls="--", color="#e8590c", label="v_des (desired cmd)")
        ax3.plot(t, v_filt, lw=2.0, color="#1971c2", label="v_filt (filtered reality)")
        ax3.plot(t, vcmd, lw=0.7, color="#2f9e44", alpha=0.6, label="cmd_vx (transient fiction)")
    else:
        ax3.plot(t, vcmd, lw=1.0, color="#2f9e44", alpha=0.8, label="cmd_vx (fiction)")
    ax3.axhline(0.0, color="gray", lw=0.6)
    ax3.grid(alpha=0.3)
    ax3.legend(loc="upper left", ncol=2)
    ax3.set_ylabel("hip vx (px/s)")
    ax3.set_xlabel("t (s)")

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
