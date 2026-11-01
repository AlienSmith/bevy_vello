# Compare commanded vs realized knee/foot positions (vello y-down) per gait
# phase, and check whether the "t*" target channels are frozen at spawn.
import csv, sys

path = sys.argv[1] if len(sys.argv) > 1 else "tmp/headless/walks/kneecheck.csv"
rows = list(csv.DictReader(open(path)))

# True collider floor surface, vello y-down (measured: anchor_p3.y + stance_h).
FLOOR_Y = 279.0

def f(r, k):
    try:
        return float(r[k])
    except (ValueError, KeyError):
        return float("nan")

print(f"rows={len(rows)}")

# --- 1) Are the target channels frozen? Compare first vs last value.
print("\n== target channel motion (first -> last, min..max) ==")
for nm in ("tP3", "tP40", "tP41", "tPLLL", "tPRLL"):
    xs = [f(r, nm + "x") for r in rows]
    ys = [f(r, nm + "y") for r in rows]
    print(f"{nm:6s} x {xs[0]:8.1f} -> {xs[-1]:8.1f}  [{min(xs):8.1f}..{max(xs):8.1f}]   "
          f"y {ys[0]:8.1f} -> {ys[-1]:8.1f}  [{min(ys):8.1f}..{max(ys):8.1f}]")

# --- 2) Realized vs target errors, split by stance of that leg.
print("\n== |realized - target| by phase (stance vs swing of that leg) ==")
for real, tgt, gait_col in (("P40", "tP40", "gait_l"), ("PLLL", "tPLLL", "gait_l"),
                            ("P41", "tP41", "gait_r"), ("PRLL", "tPRLL", "gait_r")):
    buckets = {"stance": [], "swing": []}
    for r in rows:
        ph = r[gait_col].strip()
        d = ((f(r, real + "x") - f(r, tgt + "x")) ** 2
             + (f(r, real + "y") - f(r, tgt + "y")) ** 2) ** 0.5
        buckets["stance" if ph == "0" else "swing"].append(d)
    for k, v in buckets.items():
        if v:
            mean = sum(v) / len(v)
            mx = max(v)
            print(f"{real:5s} vs {tgt:6s} {k:6s}: mean={mean:7.1f}  max={mx:7.1f}  n={len(v)}")

# --- 3) The user's claim: realized knee BELOW foot during stance?
# (vello y-down: larger y = lower on screen)
print("\n== stance frames where realized knee y > foot y (knee BELOW foot) ==")
for leg in ("L", "R"):
    kn, ft, gk = ("P40", "PLLL", "gait_l") if leg == "L" else ("P41", "PRLL", "gait_r")
    n_st = below = 0
    worst = 0.0
    worst_t = 0.0
    times = []
    for r in rows:
        if r[gk].strip() != "0":
            continue
        n_st += 1
        dy = f(r, kn + "y") - f(r, ft + "y")
        if dy > 0:
            below += 1
            times.append(f(r, "t"))
            if dy > worst:
                worst, worst_t = dy, f(r, "t")
    span = f"[{min(times):.1f}..{max(times):.1f}]" if times else "-"
    print(f"{leg}: {below}/{n_st} stance frames knee below foot ({100.0*below/max(n_st,1):.0f}%), "
          f"worst gap {worst:.1f}px @t{worst_t:.2f}, occurred over t{span}")

# --- 3b) REACH check: realized bone lengths vs hip->target distance in stance.
# If |hip - target| > thigh+calf, the 2-bone IK ray-clamps the foot along the
# hip->target ray -> the commanded plant lifts OFF the floor and slides under
# the advancing hip. This is the user's claim (knee can't reach the plant).
print("\n== stance reach check (vello; hip=P3, thigh=|knee-hip|, calf=|foot-knee|) ==")
for leg in ("L", "R"):
    kn, ft, gk = ("P40", "PLLL", "gait_l") if leg == "L" else ("P41", "PRLL", "gait_r")
    th, ca, over, hip_t_dist, n = [], [], 0, [], 0
    knee_below_foot = 0
    for r in rows:
        if r[gk].strip() != "0":
            continue
        n += 1
        hip = (f(r, "P3x"), f(r, "P3y"))
        knee = (f(r, kn + "x"), f(r, kn + "y"))
        tgt = (f(r, "t" + ft + "x"), f(r, "t" + ft + "y"))
        thigh = ((knee[0]-hip[0])**2 + (knee[1]-hip[1])**2) ** 0.5
        calf = ((f(r, ft+"x")-knee[0])**2 + (f(r, ft+"y")-knee[1])**2) ** 0.5
        th.append(thigh); ca.append(calf)
        hd = ((tgt[0]-hip[0])**2 + (tgt[1]-hip[1])**2) ** 0.5
        hip_t_dist.append(hd)
        if hd > thigh + calf:
            over += 1
        if f(r, kn + "y") > f(r, ft + "y"):  # knee below foot (y-down)
            knee_below_foot += 1
    mean_th = sum(th)/n; mean_ca = sum(ca)/n; reach = mean_th + mean_ca
    print(f"{leg}: n_stance={n}  mean thigh={mean_th:.1f} calf={mean_ca:.1f} "
          f"=> full_ext~{reach:.1f}")
    print(f"   hip->target dist: mean={sum(hip_t_dist)/n:.1f} "
          f"max={max(hip_t_dist):.1f} | exceeds full_ext in {over}/{n} stance frames "
          f"({100.0*over/n:.0f}%)")

# --- 4) Foot tracking error over time (every 2s): is the divergence growing?
print("\n== foot |realized-target| (stance frames), per 2s window ==")
for leg in ("L", "R"):
    ft, gk = ("PLLL", "gait_l") if leg == "L" else ("PRLL", "gait_r")
    win = {}
    for r in rows:
        if r[gk].strip() != "0":
            continue
        d = ((f(r, ft + "x") - f(r, "t" + ft + "x")) ** 2
             + (f(r, ft + "y") - f(r, "t" + ft + "y")) ** 2) ** 0.5
        win.setdefault(int(f(r, "t") // 2), []).append(d)
    print(f"{leg}: " + "  ".join(f"{k*2}-{k*2+2}s:{sum(v)/len(v):5.1f}px" for k, v in sorted(win.items())))

# --- 5) Signed decomposition of stance foot tracking error (vello y-down):
# dy = realized - target: positive = foot ABOVE its plant target (floating),
# negative = below (plowing). dx negative = lagging the command.
print("\n== stance foot signed error: realized - target ==")
for leg in ("L", "R"):
    ft, gk = ("PLLL", "gait_l") if leg == "L" else ("PRLL", "gait_r")
    dxd, dyd = [], []
    for r in rows:
        if r[gk].strip() != "0":
            continue
        dxd.append(f(r, ft + "x") - f(r, "t" + ft + "x"))
        dyd.append(f(r, ft + "y") - f(r, "t" + ft + "y"))
    n = len(dxd)
    # vello y-down: realized_y - target_y < 0 => realized ABOVE target (floating)
    # > 0 => BELOW target (plowing). dx < 0 => lagging behind command.
    print(f"{leg}: dx mean={sum(dxd)/n:+6.1f} | dy mean={sum(dyd)/n:+6.1f} "
          f"| float(above target)={100.0*sum(1 for d in dyd if d<0)/n:.0f}% "
          f"plow(below)={100.0*sum(1 for d in dyd if d>0)/n:.0f}%")
    # split early vs late stance
    early = [d for d, r in zip(dyd, [x for x in rows if x[gk].strip() == '0']) if f(r, "t") < 6]
    late = [d for d, r in zip(dyd, [x for x in rows if x[gk].strip() == '0']) if f(r, "t") >= 6]
    if early and late:
        print(f"   dy early(t<6s) mean={sum(early)/len(early):+6.1f}  late(t>=6s) mean={sum(late)/len(late):+6.1f}")

# --- 6) How high does the stance PLANT TARGET float above the true floor,
# and does it SLIDE FORWARD during stance (a planted foot should not advance)?
print("\n== stance plant target vs true floor + forward-slide ==")
for leg in ("L", "R"):
    ft, gk = ("PLLL", "gait_l") if leg == "L" else ("PRLL", "gait_r")
    tgt_y = [FLOOR_Y - f(r, "t" + ft + "y") for r in rows if r[gk].strip() == "0"]
    real_y = [FLOOR_Y - f(r, ft + "y") for r in rows if r[gk].strip() == "0"]
    print(f"{leg} stance: target above floor mean={sum(tgt_y)/len(tgt_y):5.1f}px "
          f"| realized above floor mean={sum(real_y)/len(real_y):5.1f}px")

# Walk stance episodes (consecutive gait==0 runs) for the right leg; measure
# how far the target.x and realized.x each advance across one stance.
print("\n== per-stance forward advance (target.x drift, realized.x drift) ==")
def episodes(gk):
    ep, cur = [], []
    for r in rows:
        if r[gk].strip() == "0":
            cur.append(r)
        elif cur:
            ep.append(cur); cur = []
    if cur: ep.append(cur)
    return ep
for leg, ft, gk in (("L", "PLLL", "gait_l"), ("R", "PRLL", "gait_r")):
    drifts = []
    for ep in episodes(gk):
        if len(ep) < 5:
            continue
        tgt_dx = f(ep[-1], "t" + ft + "x") - f(ep[0], "t" + ft + "x")
        real_dx = f(ep[-1], ft + "x") - f(ep[0], ft + "x")
        hip_dx = f(ep[-1], "tP3x") - f(ep[0], "tP3x")
        drifts.append((tgt_dx, real_dx, hip_dx))
    if drifts:
        n = len(drifts)
        print(f"{leg} ({n} stances): tgt_dx={sum(d[0] for d in drifts)/n:+6.1f} "
              f"real_dx={sum(d[1] for d in drifts)/n:+6.1f} hip_dx={sum(d[2] for d in drifts)/n:+6.1f} "
              f"(a planted foot => tgt_dx~0, real_dx~0; hip_dx = stride length)")
