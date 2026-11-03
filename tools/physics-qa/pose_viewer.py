#!/usr/bin/env python3
"""Interactive pose scrubber for a walk telemetry CSV.

One window, two regions sharing the time axis:

  LEFT  — the time-series strip (like plot_p3.py): hip+feet x, hip+feet y,
          and the velocity panel. The x axis IS time, so clicking anywhere in
          the left region picks a frame.
  RIGHT — the floor line + stick-figure skeleton for the picked frame
          (hip/knees/feet solid, spine/head/arms thin). Contacting feet are
          drawn as filled dots; the vertical is shown up = +.

Controls:
  click / drag in the left strip  -> scrub time (cursor line + pose follow)
  LEFT / RIGHT arrow              -> step one tick
  SHIFT-LEFT / SHIFT-RIGHT        -> step 10 ticks

Usage:
  python3 tools/physics-qa/pose_viewer.py tmp/headless/walks/ts_base2.csv [START_T]

Headless smoke test (writes one pose PNG, no GUI):
  python3 tools/physics-qa/pose_viewer.py in.csv --probe T OUT.png
"""
import csv
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.widgets import Slider, Button

# Realized-particle columns (vello y-down; we flip y so up is +).
# Lower body (drawn solid) and a thin upper body for context.
HIP = "P3"
L_KNEE, R_KNEE = "P40", "P41"
L_FOOT, R_FOOT = "PLLL", "PRLL"
SPINE = ["P0", "P1", "P2"]  # P3 hip is the top of the spine chain
HEAD = "PH"
L_HAND, R_HAND = "P10", "P13"  # arm ends (shoulders approximated at P1)


def load(path):
    cols = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                cols.setdefault(k, []).append(v)
    def num(k):
        return np.array([float(x) if x not in ("", "nan", None) else 0.0 for x in cols[k]])
    def pfx(p):
        return (num(f"{p}x"), -num(f"{p}y"))  # y-up
    t = num("t")
    hip = pfx(HIP)
    lkn = pfx(L_KNEE)
    rkn = pfx(R_KNEE)
    lft = pfx(L_FOOT)
    rft = pfx(R_FOOT)
    d = {
        "t": t,
        "hip": hip,
        "lkn": lkn,
        "rkn": rkn,
        "lft": lft,
        "rft": rft,
        "spine": [pfx(p) for p in SPINE],
        "head": pfx(HEAD),
        "lhand": pfx(L_HAND),
        "rhand": pfx(R_HAND),
        "vph": num("vphase").astype(int),
        "gl": num("gait_l").astype(int),
        "gr": num("gait_r").astype(int),
        "cl": num("contact_l").astype(int),
        "cr": num("contact_r").astype(int),
        "hipvx": num("P3vx"),
        "cmd": num("cmd_vx"),
        "lftvx": num("PLLLvx"),
        "rftvx": num("PRLLvx"),
    }
    return d


def pose_edges(d):
    """Return list of ((x0,y0),(x1,y1)) polyline groups for one tick (index i)."""
    # built at draw time per-frame; this helper just documents the topology.
    return None


class Viewer:
    def __init__(self, d, start_t):
        self.d = d
        self.n = len(d["t"])
        # floor = deepest (lowest up-y) any foot reaches across the run.
        self.floor = min(d["lft"][1].min(), d["rft"][1].min())
        self.i = int(np.argmin(np.abs(d["t"] - start_t)))
        self._syncing = False  # guard: slider<->cursor updates don't loop
        self.playing = False
        self.timer = None
        self._build()

    def _build(self):
        gs = matplotlib.gridspec.GridSpec(
            4,
            2,
            width_ratios=[2, 1],
            height_ratios=[1, 1, 1, 0.16],
            hspace=0.55,
            wspace=0.18,
        )
        self.fig = plt.figure(figsize=(8, 6), dpi=100)  # ~800x600 px by default
        # Ask the GUI backend to open at 800x600 (harmless on backends that
        # ignore it; keeps a tiny window instead of maximized/fullscreen).
        try:
            self.fig.canvas.manager.resize(800, 600)
        except Exception:
            pass
        self.fig.canvas.manager.set_window_title(
            "pose scrubber — drag the time slider / click the left strip / arrow keys"
        )
        self.axX = self.fig.add_subplot(gs[0, 0])
        self.axY = self.fig.add_subplot(gs[1, 0])
        self.axV = self.fig.add_subplot(gs[2, 0])
        self.axPose = self.fig.add_subplot(gs[0:3, 1])
        self.left_axes = [self.axX, self.axY, self.axV]
        d = self.d
        # LEFT strip
        self._shade(self.axX)
        self._shade(self.axY)
        self._shade(self.axV)
        self.axX.plot(d["t"], d["hip"][0], "k", lw=1.4, label="hip x")
        self.axX.plot(d["t"], d["lft"][0], color="#d6336c", lw=1.0, label="L foot x")
        self.axX.plot(d["t"], d["rft"][0], color="#1971c2", lw=1.0, label="R foot x")
        self.axX.set_ylabel("x (px)"); self.axX.legend(loc="upper left", ncol=3, fontsize=8)
        self.axY.plot(d["t"], d["hip"][1], "k", lw=1.4, label="hip y")
        self.axY.plot(d["t"], d["lft"][1], color="#d6336c", lw=1.0, label="L foot y")
        self.axY.plot(d["t"], d["rft"][1], color="#1971c2", lw=1.0, label="R foot y")
        self.axY.axhline(self.floor, color="gray", ls=":", lw=1, label="floor")
        self.axY.set_ylabel("y (px, up +)"); self.axY.legend(loc="upper left", ncol=2, fontsize=8)
        self.axV.plot(d["t"], d["hipvx"], "k", lw=1.1, alpha=0.6, label="hip P3vx")
        self.axV.plot(d["t"], d["lftvx"], color="#d6336c", lw=0.6, alpha=0.6, label="L foot vx")
        self.axV.plot(d["t"], d["rftvx"], color="#1971c2", lw=0.6, alpha=0.6, label="R foot vx")
        self.axV.plot(d["t"], d["cmd"], color="#2f9e44", lw=1.0, alpha=0.8, label="cmd_vx")
        self.axV.axhline(0, color="gray", lw=0.5)
        self.axV.set_ylabel("vx (px/s)"); self.axV.set_xlabel("t (s)")
        self.axV.legend(loc="upper left", ncol=2, fontsize=8)
        # cursors (one per left axis) + pose artists
        self.curs = []
        for ax in self.left_axes:
            c = ax.axvline(d["t"][self.i], color="red", lw=0.9, alpha=0.8)
            self.curs.append(c)
        self._build_pose()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_drag)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self._dragging = False
        # explicit, discoverable controls: a time slider + a Play button.
        ax_sl = self.fig.add_subplot(gs[3, 0])
        self.slider = Slider(
            ax_sl, "time", 0, self.n - 1, valinit=self.i, valstep=1, valfmt="%d", color="#e8590c"
        )
        # show seconds under the slider thumb via the label
        ax_sl.set_xlabel(f"tick → time ({self.d['t'][0]:.1f}–{self.d['t'][-1]:.1f} s)", fontsize=8)
        self.slider.on_changed(self.on_slider)
        ax_btn = self.fig.add_subplot(gs[3, 1])
        self.btn_play = Button(ax_btn, "▶ Play")
        self.btn_play.on_clicked(self.toggle_play)
        self.fig.subplots_adjust(bottom=0.09, top=0.96, left=0.05, right=0.98)
        self.update()

    def _shade(self, ax):
        for sig, c in ((self.d["gl"], "#ffc2d1"), (self.d["gr"], "#bfd4ff")):
            dd = np.diff(np.concatenate(([0], sig, [0])))
            for a, b in zip(np.where(dd == 1)[0], np.where(dd == -1)[0]):
                ax.axvspan(self.d["t"][a], self.d["t"][min(b, self.n - 1)], color=c, alpha=0.3, lw=0)

    def _build_pose(self):
        ax = self.axPose
        ax.set_title("pose (floor + skeleton)")
        ax.set_xlabel("x (px, world)"); ax.set_ylabel("y (px, up +)")
        ax.grid(alpha=0.3)
        ax.axhline(self.floor, color="gray", lw=1.2)
        # skeleton polylines (created empty, set per frame)
        self.pose_lines = {
            "spine": Line2D([], [], color="#555", lw=1.6, solid_capstyle="round"),
            "torsoHead": Line2D([], [], color="#555", lw=1.2, alpha=0.8),
            "armL": Line2D([], [], color="#555", lw=1.0, alpha=0.7),
            "armR": Line2D([], [], color="#555", lw=1.0, alpha=0.7),
            "legL": Line2D([], [], color="#d6336c", lw=2.4, solid_capstyle="round"),
            "legR": Line2D([], [], color="#1971c2", lw=2.4, solid_capstyle="round"),
        }
        self.pose_joints = ax.plot([], [], "o", ms=4, color="k", zorder=5)[0]
        self.footL = ax.plot([], [], "o", ms=8, color="#d6336c", mec="k", zorder=6)[0]
        self.footR = ax.plot([], [], "o", ms=8, color="#1971c2", mec="k", zorder=6)[0]
        for ln in self.pose_lines.values():
            ax.add_line(ln)
        self.pose_txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=9,
                                bbox=dict(boxstyle="round", fc="white", alpha=0.85))

    def _chain(self, pts):
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        return xs, ys

    def update(self):
        d = self.d
        i = self.i
        t = d["t"][i]
        for c in self.curs:
            c.set_xdata([t, t])
        hip = (d["hip"][0][i], d["hip"][1][i])
        lkn = (d["lkn"][0][i], d["lkn"][1][i])
        rkn = (d["rkn"][0][i], d["rkn"][1][i])
        lft = (d["lft"][0][i], d["lft"][1][i])
        rft = (d["rft"][0][i], d["rft"][1][i])
        p0 = (d["spine"][0][0][i], d["spine"][0][1][i])
        p1 = (d["spine"][1][0][i], d["spine"][1][1][i])
        p2 = (d["spine"][2][0][i], d["spine"][2][1][i])
        head = (d["head"][0][i], d["head"][1][i])
        lh = (d["lhand"][0][i], d["lhand"][1][i])
        rh = (d["rhand"][0][i], d["rhand"][1][i])
        # spine chain P0-P1-P2-P3(hip)
        xs, ys = self._chain([p0, p1, p2, hip])
        self.pose_lines["spine"].set_data(xs, ys)
        # neck/head: head (PH) sits ABOVE the top-of-spine P0, not at the hip end.
        xs, ys = self._chain([p0, head])
        self.pose_lines["torsoHead"].set_data(xs, ys)
        xs, ys = self._chain([p1, lh])
        self.pose_lines["armL"].set_data(xs, ys)
        xs, ys = self._chain([p1, rh])
        self.pose_lines["armR"].set_data(xs, ys)
        xs, ys = self._chain([hip, lkn, lft])
        self.pose_lines["legL"].set_data(xs, ys)
        xs, ys = self._chain([hip, rkn, rft])
        self.pose_lines["legR"].set_data(xs, ys)
        # joints scatter: all key points
        jx = [hip[0], lkn[0], rkn[0], p0[0], p1[0], p2[0], head[0]]
        jy = [hip[1], lkn[1], rkn[1], p0[1], p1[1], p2[1], head[1]]
        self.pose_joints.set_data(jx, jy)
        self.footL.set_data([lft[0]], [lft[1]])
        self.footR.set_data([rft[0]], [rft[1]])
        # fill contacting feet solid vs hollow
        self.footL.set_alpha(1.0 if d["cl"][i] else 0.35)
        self.footR.set_alpha(1.0 if d["cr"][i] else 0.35)
        # keep x-window centered on the character (world x drifts a lot)
        cx = np.mean(jx)
        span = 260
        self.axPose.set_xlim(cx - span, cx + span)
        self.axPose.set_ylim(self.floor - 40, self.floor + 360)
        vph = d["vph"][i]
        vp_name = ["Stand", "Load", "Launch", "FLIGHT", "Land"][vph] if 0 <= vph < 5 else str(vph)
        self.pose_txt.set_text(
            f"t={t:5.2f}s  vphase={vp_name}\n"
            f"hipy={hip[1]:6.1f}  Lfoot={lft[1]:6.1f}{'C' if d['cl'][i] else ' '} "
            f"Rfoot={rft[1]:6.1f}{'C' if d['cr'][i] else ' '}\n"
            f"hipvx={d['hipvx'][i]:6.1f} cmd={d['cmd'][i]:5.1f}"
        )
        self.fig.canvas.draw_idle()

    def set_index(self, i):
        """Single source of truth for the current tick: moves the slider,
        the red cursor line and the pose together. Guard prevents slider
        on_changed feedback loops when we set the value programmatically."""
        self.i = int(np.clip(i, 0, self.n - 1))
        if not self._syncing:
            self._syncing = True
            self.slider.set_val(self.i)  # fires on_changed -> re-enter, guarded
            self._syncing = False
        self.update()

    def on_slider(self, _val):
        if self._syncing:
            return
        self.i = int(self.slider.val)
        self.update()

    def toggle_play(self, _event):
        if self.playing:
            self.playing = False
            if self.timer is not None:
                self.timer.stop()
                self.timer = None
            self.btn_play.label.set_text("▶ Play")
        else:
            self.playing = True
            self.btn_play.label.set_text("⏸ Pause")
            if self.i >= self.n - 1:
                self.set_index(0)
            # ~20 fps playback
            self.timer = self.fig.canvas.new_timer(interval=50)
            self.timer.add_callback(self._play_step)
            self.timer.start()
        self.fig.canvas.draw_idle()

    def _play_step(self):
        if not self.playing:
            return
        nxt = self.i + 1
        if nxt >= self.n:
            self.toggle_play(None)  # reached the end, stop
            return
        self.set_index(nxt)

    def on_click(self, event):
        if event.inaxes in self.left_axes and event.xdata is not None:
            self._dragging = True
            self.set_index(np.argmin(np.abs(self.d["t"] - event.xdata)))

    def on_drag(self, event):
        if self._dragging and event.inaxes in self.left_axes and event.xdata is not None:
            self.set_index(np.argmin(np.abs(self.d["t"] - event.xdata)))
        elif event.button is None:
            self._dragging = False

    def on_key(self, event):
        step = 10 if event.key in ("shift+left", "shift+right") else 1
        if event.key in ("left", "shift+left"):
            self.set_index(self.i - step)
        elif event.key in ("right", "shift+right"):
            self.set_index(self.i + step)
        elif event.key == " ":
            self.toggle_play(None)
        else:
            return


def main():
    args = [a for a in sys.argv[1:]]
    if not args:
        sys.exit(__doc__)
    path = args[0]
    d = load(path)
    if "--probe" in args:
        # headless: build, jump to T, save pose to OUT.png, exit
        k = args.index("--probe")
        start = float(args[k + 1])
        out = args[k + 2]
        matplotlib.use("Agg")
        v = Viewer(d, start)
        v.fig.savefig(out, dpi=90)
        print(f"probe ok: t~{start}s -> {out} (cursor idx={v.i}, t={d['t'][v.i]:.2f}s)")
        return
    start = float(args[1]) if len(args) > 1 else 0.0
    Viewer(d, start)
    plt.show()


if __name__ == "__main__":
    main()
