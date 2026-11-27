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
    plan_l, plan_r = [], []  # the placement LAW's swing target x (planLx/planRx)
    has_plan = False
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
            if "planLx" in row:
                has_plan = True
                # The law's target is a world POINT (plant x, floor y), so both
                # panels can draw the aim it actually asked for.
                plan_l.append((float(row.get("planLx") or "nan"), -float(row.get("planLy") or "nan")))
                plan_r.append((float(row.get("planRx") or "nan"), -float(row.get("planRy") or "nan")))
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
        np.array(plan_l) if has_plan else None,
        np.array(plan_r) if has_plan else None,
        np.array(vx),
        np.array(vcmd),
        np.array(v_filt) if has_filt else None,
        np.array(v_des) if has_filt else None,
    )


def parse_args(argv):
    """`--targets`, `--plan`, `--range T0 T1` (seconds, inclusive), PATH [OUT.png].

    `--targets` draws the profile command (`tPLLLx/y`, the old 5-curve panel);
    `--plan` additionally draws the placement law's own aim (`planLx/y`). They are
    separate switches so each panel can keep the legacy line count when that is
    what the reader expects.
    """
    targets = False
    show_plan = False
    t_range = None
    rest = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--targets":
            targets = True
            i += 1
        elif a == "--plan":
            show_plan = True
            i += 1
        elif a == "--range":
            if i + 2 >= len(argv):
                sys.exit("--range needs two numbers: --range T0 T1")
            t_range = (float(argv[i + 1]), float(argv[i + 2]))
            i += 3
        else:
            rest.append(a)
            i += 1
    return rest, targets, show_plan, t_range


def main():
    args, targets, show_plan, t_range = parse_args(sys.argv[1:])
    if len(args) < 1:
        sys.exit(__doc__)
    path = args[0]
    out = args[1] if len(args) > 1 else path.rsplit(".", 1)[0] + ".p3.png"
    (
        t,
        hip,
        lft,
        rft,
        gl,
        gr,
        lt,
        rt,
        plan_l,
        plan_r,
        vx,
        vcmd,
        v_filt,
        v_des,
    ) = load(path, targets)

    # Optional window: everything below (curves, shading, summary) sees only
    # [t0, t1]. Used to zoom on transients (e.g. the 0–2.5 s entry where the
    # body leaves the boost and the speed runs away).
    if t_range is not None:
        t0, t1 = t_range
        m = (t >= t0) & (t <= t1)
        if not m.any():
            sys.exit(f"no rows in {t_range[0]}..{t_range[1]} s (data {t[0]:.2f}..{t[-1]:.2f})")

        def win(a):
            return None if a is None else a[m]

        t, hip, lft, rft = t[m], hip[m], lft[m], rft[m]
        gl, gr = gl[m], gr[m]
        lt, rt = win(lt), win(rt)
        plan_l, plan_r = win(plan_l), win(plan_r)
        vx, vcmd, v_filt, v_des = win(vx), win(vcmd), win(v_filt), win(v_des)

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(14, 13), sharex=True)

    def shade(ax):
        for sig, c in ((gl, "#ffc2d1"), (gr, "#bfd4ff")):
            d = np.diff(np.concatenate(([0], sig, [0])))
            for a, b in zip(np.where(d == 1)[0], np.where(d == -1)[0]):
                ax.axvspan(t[a], t[min(b, len(t) - 1)], color=c, alpha=0.35, lw=0)

    def curves(ax, col, ylab, title=None):
        shade(ax)  # bands first, so the curves draw on top
        ax.plot(t, hip[:, col], lw=1.4, color="k", label="hip P3")
        ax.plot(t, lft[:, col], lw=1.0, color="#d6336c", label="L foot (physics)")
        ax.plot(t, rft[:, col], lw=1.0, color="#1971c2", label="R foot (physics)")
        if show_plan and plan_l is not None:
            # The placement law's OWN target (DebugInfo left/right_target_x at
            # the config floor), drawn on BOTH axes together with the commanded
            # profile target so "the law aimed wrong" and "the leg did not reach
            # the aim" are visually separable. Opt-in (`--plan`) so a plain
            # `--targets` plot keeps the legacy five curves per panel.
            # Same hue per side as the physics foot (left = pink family,
            # right = blue family); only the line style/weight distinguishes
            # physics / actuator / dcm, so a side is one colour.
            ax.plot(t, plan_l[:, col], lw=1.2, ls=":", color="#8a1c42", label="L dcm")
            ax.plot(t, plan_r[:, col], lw=1.2, ls=":", color="#0b3d66", label="R dcm")
        if targets:
            ax.plot(
                t,
                lt[:, col],
                lw=0.8,
                ls="--",
                color="#d6336c",
                alpha=0.7,
                label="L actuator",
            )
            ax.plot(
                t,
                rt[:, col],
                lw=0.8,
                ls="--",
                color="#1971c2",
                alpha=0.7,
                label="R actuator",
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

    # Foot penetration below the floor surface: how far each foot's physics y
    # (y-down) passes the floor at 312. In the plot's flipped frame the raw y is
    # `-lft[:,1]`, so depth = max(0, (y_phys) - 312) = max(0, -(y_plot + 312)).
    # A dedicated panel (rather than just the y trace) so the deep-contact
    # windows read directly off the run instead of by eye against the floor.
    FLOOR_Y = 312.0
    shade(ax4)
    dL = np.maximum(0.0, -(lft[:, 1] + FLOOR_Y))
    dR = np.maximum(0.0, -(rft[:, 1] + FLOOR_Y))
    ax4.plot(t, dL, lw=1.0, color="#d6336c", label="L foot depth below floor")
    ax4.plot(t, dR, lw=1.0, color="#1971c2", label="R foot depth below floor")
    ax4.axhline(0.0, color="gray", lw=0.6)
    ax4.grid(alpha=0.3)
    ax4.legend(loc="upper left", ncol=2)
    ax4.set_ylabel("depth below floor (px)")
    ax4.set_xlabel("t (s)")
    print(
        f"foot max depth below floor: L {dL.max():.1f} px, R {dR.max():.1f} px"
        f" (>2 px in {int(100 * (dL > 2).mean()):02d}% / {int(100 * (dR > 2).mean()):02d}%)"
    )
    if t_range is not None:
        ax1.set_xlim(t_range)
        ax1.set_title(
            f"Hip + feet position vs time — {path}  [{t_range[0]:.2f}..{t_range[1]:.2f} s]"
        )

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
