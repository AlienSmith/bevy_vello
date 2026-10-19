# Loop-3 Attitude Regression — Problem Record (2026-10-19)

Status: **OPEN / WIP — not merged as verified**. The loop-3 (attitude/tilt) code is
committed as WIP, but the §7.1 regression **fails** its hip/leg-unchanged criterion.
This document records the problem and how to reproduce it so the work can be
picked up cleanly later.

## 1. What was built (committed WIP)

Third Raibert control loop (attitude/tilt) + upper-body point, per
[`raibert_sandbox_attitude_loop.md`](raibert_sandbox_attitude_loop.md):

- [`resolver.rs`](examples/game_lib/src/lower_body/resolver.rs) — `AttitudeState`
  (`theta`/`omega`), config knobs (`torso_len=70`, `attitude_kp=45`,
  `attitude_kd=11`, `attitude_accel_coupling=1.0`, `attitude_gravity_coupling=true`,
  `theta_max=1.30`), pure math `attitude_pd` + `tick_attitude` (inverted pendulum
  at the hip, semi-implicit Euler, flight = weightless keep-PD), wired into
  `step_lower_body` after the horizontal loop with `hip_accel_x = Δv_x/dt`
  (finite-difference of the resolver's own `hip_vel.x`). Upper-body point
  `upper = hip + L·(sinθ, cosθ)`.
- [`hip_pin.rs`](examples/game_lib/src/lower_body/hip_pin.rs) — `pin.theta =
  pin.state.attitude.theta` written before `rigid_frame_targets` so the P1/P2/P3
  frame physically tilts; `last_attitude_torque`/`last_hip_accel_x` latched for
  telemetry.
- [`headless_arm_pd.rs`](examples/collision_detection/examples/headless_arm_pd.rs) —
  `body_theta`/`body_omega`/`att_torque`/`hip_accel_x` added to the whole-body row.
- [`raibert_sandbox.rs`](examples/collision_detection/src/raibert_sandbox.rs) —
  torso line hip→upper, upper marker, loop-3 sliders/telemetry.
- 8 unit tests in `resolver.rs` (PD sign/restore, divergence w/o PD, settle-to-
  upright, accel-coupling lean+recovery, upper geometry, clamp, flight weightless).
  `cargo test -p game_lib` = 108 pass; clippy clean for new code; sandbox compiles.

## 2. The regression that fails (§7.1 DoD criterion (a))

Runs: [`walk_script.json`](plans/walk_script.json) and
[`jump_script.json`](plans/jump_script.json) through
[`headless_arm_pd.rs`](examples/collision_detection/examples/headless_arm_pd.rs)
whole-body mode with `FEEDBACK=0`, compared to the pre-loop-3 baseline
(`*_fb0.csv`, which was byte-identical to `*_base.csv`).

Artifacts in `tmp/headless/att_baseline/` (outside the repo; see repro below):

| run | rows | rows ok | NaN | DoD (a) hip/leg max diff vs fb0 |
|-----|------|---------|-----|---------------------------------|
| walk_att3 | 1036 | yes | 0 | **P3y/P40y/P41y/PLLLy/PRLLy ≈ 50 px at tail (row 1035)** |
| jump_att3 | 253  | yes | 0 | **P3y etc. ≈ 13 px at tail (row 252)** |

Details:

- The **resolver's own commanded target `tP3y`** differs from fb0
  (`max|d| = 50.77` walk, `13.04` jump) — so the hip *trajectory itself* changed,
  not just the physics settling. Loop 3 was supposed to add **no hip/leg
  translation term**.
- The **first telemetry row already differs**: fb0 `tP3y=173.00`, `P0vy=-1.46`
  (settle velocity), `theta_deg=0.511`; att3 `tP3y=172.88`, `P0vy=0.00`,
  `theta_deg=0.516` — with `body_theta=0.0000` at that instant. So the two runs
  are not even starting from the same spawn/settle state.
- The limb-classifier sidecar summaries disagree between runs (different hashes):
  walk fb0 `WAGGLE 114.54°` vs att3 `NO_SWING 3.67°`; jump fb0 `WAGGLE 173.34°`
  vs att3 `WAGGLE 107.00°` (reversals 1 → 3).
- DoD (b) behavior *is* healthy in walk: `body_theta` leans back on move start
  (−0.1107 rad peak), returns to ~0 during cruise, oscillates then recovers to
  0.0099; `att_torque` ≤ 10.93; `hip_accel_x` 685.7 at start/stop. Jump is
  inert (all zeros) as expected (zero-g harness + no move command).

## 3. Why the earlier "near-identical" reading was wrong

A prior column-wise diff looked at the **top-12 worst columns** — those were all
upper-body (arms/head) with hundreds-of-px diffs, so the hip/leg columns looked
"near-identical" by omission. They are **not** byte-identical; they differ at the
sub-pixel-to-pixel level throughout and diverge to ~50 px / ~13 px at the tail.
The correct test is a per-column max|Δ| over the DoD set, which is what the
`analyze_att3_regression.py` script in `tmp/headless/att_baseline/` does.

## 4. Likely causes (unresolved)

1. **Different spawn/settle state between runs.** The row-0 divergence (zeroed
   velocities, 0.1px y shift) suggests the att3 runs were taken under slightly
   different harness conditions than the fb0 baseline (e.g. env vars, asset
   load timing, tick alignment) — **or** the loop-3 wiring changed the pin's
   effect on spawn. Need to confirm both runs used identical env (`COLLISION`,
   `GRAVITY`, `LIMB`, `GRID`, `FEEDBACK`).
2. **Resolver hip-y perturbation.** If `rigid_frame_targets` or the upper-point
   change feeds back into `state.hip_pos.y` (even via `regulate_height` reacting
   to a tilted frame), the leg IK would shift the foot targets → tail drift.
   The upper point is render/telemetry only, but the *tilted frame* is the
   natural suspect for a vertical leak.
3. **Non-determinism in un-pinned particles** (arms) is real but should not move
   `tP3` (resolver state) — the fact that `tP3y` itself moves rules this out as
   the whole story.

## 5. Next steps when picking this up

- Re-run fb0 and att3 with **identical** env and capture the spawn-frame state
  (or add a `REPLAY`/seed determinism gate).
- Isolate loop 3: temporarily set `pin.theta = 0` (keep telemetry) and confirm
  `tP3`/hip/leg columns become byte-identical to fb0 again. If they do, the
  tilt-path is the leak; if not, the run conditions differ and the comparison is
  invalid.
- Trace `state.hip_pos.y` through `step_lower_body` to confirm loop 3 adds no
  vertical term.
- Decide: keep loop 3 behind a gate until DoD (a) passes, or redesign the
  hip-pin hook-up (e.g. apply θ only to the upper-body render + torso angular
  constraints later, per plan §8).

## 6. How to reproduce

Baseline-era (pre-loop-3) physics with feedback off:

```bash
# from /home/ubuntu/bender/bevy_vello
SCRIPT=$PWD/plans/walk_script.json OUT_CSV=/tmp/att_repro/walk_fb0.csv \
  FEEDBACK=0 cargo run -p collision_detection --example headless_arm_pd --release
SCRIPT=$PWD/plans/jump_script.json OUT_CSV=/tmp/att_repro/jump_fb0.csv \
  FEEDBACK=0 cargo run -p collision_detection --example headless_arm_pd --release
```

Loop-3 run (same flags; requires real GPU — the collision/physics path uses
compute shaders, Xvfb is not viable):

```bash
SCRIPT=$PWD/plans/walk_script.json OUT_CSV=/tmp/att_repro/walk_att3.csv \
  FEEDBACK=0 cargo run -p collision_detection --example headless_arm_pd --release
SCRIPT=$PWD/plans/jump_script.json OUT_CSV=/tmp/att_repro/jump_att3.csv \
  FEEDBACK=0 cargo run -p collision_detection --example headless_arm_pd --release
```

Check:

```bash
cd /tmp/att_repro
# per-column max|Δ| over the DoD set (P3/P40/P41/PLLL/PRLL + tP3/tP40/tP41/tPLLL/tPRLL)
python3 /home/ubuntu/bender/tmp/headless/att_baseline/analyze_att3_regression.py
```

Expected for DoD (a): every column in the set `max|d| < 0.001` (unchanged).
Actual: `~50 px` (walk) / `~13 px` (jump) at the tail → FAIL.
