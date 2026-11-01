# Posture trajectory: is the body sinking (hip low) and pitching forward, the
# "about to fall but still moving" the user sees on screen? Vello y-down.
import csv, sys

path = sys.argv[1] if len(sys.argv) > 1 else "tmp/headless/walks/kneecheck.csv"
rows = list(csv.DictReader(open(path)))
FLOOR_Y = 279.0  # collider floor surface (vello y-down)

def f(r, k):
    try:
        return float(r[k])
    except (ValueError, KeyError):
        return float("nan")

print(f"file={path}")

# Knee/foot clearance split by gait phase. STEP_HEIGHT only lifts the SWING
# foot; a low KNEE during STANCE is support collapse (low hip), not a knob issue.
def st_rows(gk):
    return [r for r in rows if r[gk].strip() == "0"]
def sw_rows(gk):
    return [r for r in rows if r[gk].strip() == "1"]

print("\n== knee & foot clearance above floor (px), split by phase ==")
for leg, kn, ft, gk in (("L", "P40", "PLLL", "gait_l"), ("R", "P41", "PRLL", "gait_r")):
    st = st_rows(gk); sw = sw_rows(gk)
    for name, sel in (("stance", st), ("swing", sw)):
        if not sel:
            continue
        kn_h = [FLOOR_Y - f(r, kn + "y") for r in sel]
        ft_h = [FLOOR_Y - f(r, ft + "y") for r in sel]
        hip_h = [FLOOR_Y - f(r, "P3y") for r in sel]
        print(f"{leg} {name:6s}: knee mean={sum(kn_h)/len(kn_h):5.1f} min={min(kn_h):5.1f} | "
              f"foot mean={sum(ft_h)/len(ft_h):5.1f} max={max(ft_h):5.1f} | "
              f"hip mean={sum(hip_h)/len(hip_h):5.1f}")
print("(swing foot max ~= STEP_HEIGHT is what STEP_HEIGHT controls; stance knee "
      "min ~= 0 is support collapse, NOT STEP_HEIGHT.)")
first = rows[0]
hip0 = FLOOR_Y - f(first, "P3y")
print(f"stand-ref hip height ~{hip0:.1f}px (t0). straight-leg knee ~55px below hip.\n")
print("win    hipH   minKneeH   |  body_th meas_th (deg) | hipVx(px/s)")
for tlo in range(0, 12, 1):
    sel = [r for r in rows if tlo <= f(r, "t") < tlo + 1]
    if not sel:
        continue
    hip_h = [FLOOR_Y - f(r, "P3y") for r in sel]
    # tP3y = the resolver's COMMANDED hip (fiction, loop-1 Stand spring target).
    # P3y  = the realized hip particle. If commanded stays ~106 but realized
    # sinks, the height is commanded and NOT DELIVERED (pin/leg transmission).
    cmd_h = [FLOOR_Y - f(r, "tP3y") for r in sel]
    kn_h = [FLOOR_Y - f(r, "P40y") for r in sel] + [FLOOR_Y - f(r, "P41y") for r in sel]
    bt = sum(f(r, "body_theta") for r in sel) / len(sel)
    mt = sum(f(r, "meas_theta") for r in sel) / len(sel)
    vx = sum(f(r, "cmd_vx") for r in sel) / len(sel)
    # realized - commanded hip vertical velocity (px/s, y-down -> negate)
    hv = sum(f(r, "tP3vy") for r in sel) / len(sel)
    print(f"{tlo:2d}s  {sum(hip_h)/len(hip_h):6.1f}  {min(kn_h):7.1f}  "
          f"cmd_h={sum(cmd_h)/len(cmd_h):6.1f} gap={sum(cmd_h)/len(cmd_h)-sum(hip_h)/len(hip_h):+6.1f}  |  "
          f"{bt*57.3:6.1f} {mt*57.3:6.1f}  | {vx:6.1f}")

# min knee height (closest to floor) over the whole run + where hip is lowest
kn_all = [(FLOOR_Y - f(r, "P40y"), f(r, "t"), 'L') for r in rows]
kn_all += [(FLOOR_Y - f(r, "P41y"), f(r, "t"), 'R') for r in rows]
mk = min(kn_all)
print(f"\nlowest knee above floor: {mk[0]:.1f}px @t{mk[1]:.2f} ({mk[2]})")
hip_all = [(FLOOR_Y - f(r, "P3y"), f(r, "t")) for r in rows]
mh = min(hip_all)
print(f"lowest hip above floor:  {mh[0]:.1f}px @t{mh[1]:.2f}")
