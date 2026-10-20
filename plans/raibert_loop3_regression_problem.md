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

## 7. Implementation note — the missing actuator (2026-10-19)

The WIP computed `pin.theta` but never applied it: the θ-rotated
`rigid_frame_targets()` result was used only for `last_ref_p3`/`last_sag_y`
telemetry, while the actual constraints came from the translation-only
`delta_frame_targets()`. Loop 3 therefore had no physical authority.

The first actuator wiring is now in `drive_hip_pin`:

- The whole `[P1, P2, P3]` rigid frame rotates about **P2**. P2 carries the
  existing delta/velocity/gravity servo target; P1 and P3 are placed at
  `P2 + R(θ)·r` using the seeded rest offsets.
- This preserves every frame distance (P1–P2, P2–P3, P1–P3), so it does not
  over-constrain the rigid triangle. P3 remains hard-pinned, but to its
  attitude-adjusted target `P3* = P2* + R(θ)·r_hip`; the hip therefore follows
  the torso lean.
- The angular-joint path (`CharacterAngularConstraintEvent` on `P1_P2_P3`
  with only P1 free) was tried first and rejected: with P2/P3 pinned and the
  rigid P1–P3 frame distance, changing only the joint angle forces the solver
  through the collinear singularity, and the whole character spins about P3.
- `attitude_rest_basis()` is retained for telemetry (`att_rest_cos/sin`), but
  it no longer drives the actuator.
- New knobs: `ATTITUDE` (`0`/`1`), `ATT_COMPLIANCE`, `ATT_TARGET_DEG`,
  `ATT_MAX_DEG` (default 20°, caps the commanded lean so a saturated sandbox
  θ cannot drive the hard-pinned body horizontal). New telemetry:
  `att_enabled`, `att_target`, `att_rest_cos`, `att_rest_sin`, `meas_theta`.

Still open:

- The target attitude is still `pin.state.attitude.theta` (the hip-pivoted
  sandbox model) plus `ATT_TARGET_DEG`. The next step is to close the loop on
  the **measured** `P2→P1` attitude instead of replaying the model lean.
- Runtime A/B (`ATTITUDE=0` vs `1`) and the revised DoD still need to be run.
  Compare resolver commands (`pose.hip`/`pose.foot`), not realized P1/P2/P3.
- When the hard hip pin is removed, the same P2-centred rotation can carry P3
  without a pin target; only the P2 translation command remains.

## 8. Angular-only walking mode (2026-10-19)

Normal floor walking no longer needs the external `P1/P2/P3` position pin:

- `HipPinParams::external_pin` defaults **off** (`EXTERNAL_PIN=1` re-enables
  the debug hard pin).
- `HipPinParams::air_control` defaults **on** (`AIR_CONTROL=0` disables it).
  An external pin tick is allowed only when airborne **and** horizontal input
  is held, i.e. the mid-air direction-change assist:
  `external_pin || (air_control && !grounded && |input_x| > 0)`.
- When the external pin is off, `drive_hip_pin` still runs the resolver and
  writes the leg targets, but emits **no** external position constraints and
  sets `anchor_chain = false` on both leg controllers.
- [`tick_loop3_angular`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:806)
  runs after `drive_hip_pin` and before the constraint world step. It emits a
  `CharacterAngularConstraintEvent` for `P1_P2_P3` with `free_mask = 0b111`,
  so loop 3 is driven purely by the angular joint. The legs are driven by the
  existing leg-PD angular constraints; contacts/physics provide support.
- `att_rest_basis` / `att_target` / `meas_theta` remain in telemetry, and the
  new `ext_pin` column records whether the external pin ran that tick.

The tuning app exposes both switches plus the loop-3 compliance/zeta/bias/max
controls. The floor/contact path must be active for support (the current tuning
scene still has the static ground wall disabled for the old pin bridge).

## 9. Headless angular-only result (2026-10-19)

Run: `COLLISION=1 GRAVITY=on EXTERNAL_PIN=0 ATTITUDE=1 SUBSTEPS=1
SCRIPT=plans/walk_script.json` through `headless_arm_pd` (1036 rows, no NaN).

### What was fixed

The first angular-only run drove the leg targets from the **live hip y**.
When the body sank, the foot targets sank with it and the leg PD pushed the
feet through the floor:

| metric | v1 (live hip y) | v2 (resolver-relative hip y + foot clamp) |
|---|---|---|
| max foot penetration | **53–54 px** | **2.4 / 1.9 px** |
| rows with >10 px penetration | 149 / 184 | **0 / 0** |
| calf stretch | **33.8% / 30.0%** | **2.3% / 2.0%** |
| thigh stretch | 7.4% / 6.7% | 0.5% / 0.5% |
| P1–P3 frame diagonal | 29.2% | 2.8% |
| leg tracking peak | 126 px | **16.9 px** |

The fix in `drive_hip_pin`: the hip target x follows the live P3, but y follows
the resolver's vertical command relative to the seed; foot target y is clamped
to the seed floor line. This is still angular-only (no external position pin).

### Remaining serious blocker: the torso falls

With the stable-leg configuration, the physical torso attitude runs away:

- `meas_theta` reaches **−2.3 rad** at the tail (the classifier sees the body
  fold/roll); `att_target` stays within ±0.11 rad.
- The `P1_P2_P3` joint's required upright rest angle reaches **~3.06 rad**,
  while the live joint angle stays at ~0.3 rad: the authored **15° hard wall**
  is saturated, so the torso follows the falling lower body.
- Removing the wall is worse: the joint then tries to fold the torso ~175°
  while the lower body is inverted, tearing the frame (P1–P3 98%, leg peak
  122 px).
- Blending the leg hip-target reference toward world-upright is also unstable:
  blend 0.1 → 71% rigidity failure / 218 px leg peak; blend 0.2 → `DIVERGED`.
  Rate-limiting the rest-angle chase did not fix it (blend 0.1 → `DIVERGED`).

So this is **not a tuning-only problem**. The live-spine reference is what keeps
the legs/feet stable, but it deliberately makes the hip joint follow the torso;
without the external pin there is no absolute torso-orientation authority. The
15° wall only masks the failure by clamping the torso to the falling lower body.

### Next control decision

To make angular-only walking work, add an absolute torso/balance authority that
does not rely on the leg hip-target reference. Options:

1. A torque/force at P2 driven by a world-referenced attitude PD (the original
   loop-3 intent), with a bounded budget so it cannot tear the frame.
2. A lightweight external attitude constraint during ground walking (an
   exception to “no external constraints”, but only rotational).
3. A CoM/pitch balance controller that adjusts foot placement and hip height
   from `meas_theta`/`meas_omega`, i.e. a real loop-2 + loop-3 closure.

The height-aware leg targets and foot clamp should stay regardless; they reduced
penetration by ~20× and leg stretch by ~15×.

## 10. Breakthrough: world-upright reference + softer legs (2026-10-19)

The blend was unnecessary. The correct target is the resolver's **world-upright**
spine, and the XPBD angular solver handles the dynamics if the leg spring is not
over-stiff.

Run: `EXTERNAL_PIN=0 ATTITUDE=1 SUBSTEPS=1 SPINE_REF=upright` with
`LEG_COMPLIANCE` swept `1e-8 / 1e-7 / 1e-6`, `LEG_ZETA=1.0`, `LEG_ALPHA=1.0`,
`LEG_MAX_ANG_RATE=1e6`, `P1_P2_P3 max_angle=null`.

| `LEG_COMPLIANCE` | torso `meas_theta` | P1–P2 / P2–P3 / P1–P3 | calf stretch | max foot penetration | leg peak |
|---|---|---|---|---|---|
| `1e-8` | ±1.2 rad (falls) | 6.7 / 17.9 / 22.1% | 31.6 / 30.1% | 139 / 140 px | 147 px |
| `1e-7` | ±1.3 rad (falls) | 3.3 / 6.5 / 15.1% | 7.9 / 10.3% | 71 / 67 px | 177 px |
| **`1e-6`** | **−0.084 … +0.233 rad (upright)** | **0.7 / 0.3 / 3.5%** | **1.7 / 1.1%** | **3.7 / 3.8 px** | 131 px @ start |

At `1e-6`:

- The torso stays upright for the whole script; no row exceeds 0.5 rad.
- All bones are within ~3.5%; the frame diagonal P1–P3 is the worst at 3.5%.
- Foot penetration is <4 px, none >5 px.
- `P3` travels ~131 px in +x over the 11.5 s script, so it moves forward but the
  stride/speed is still far below the resolver command (`target_speed=120`).
- The remaining work is walking quality: leg tracking peak ~131 px at move
  start, tail WAGGLE rms ~7 px, and a hip sag (`P3y` up to 232 vs 173 rest).

So the three-loop architecture is sufficient; the previous failure was the
incorrect live-spine target plus an over-stiff leg spring. The blend and the
15° wall are not needed.

Next tuning matrix (all through `headless_arm_pd`):

1. `LEG_COMPLIANCE` around `1e-6`: `1.5e-6`, `2e-6`, `3e-6`, `1e-5`.
2. `LEG_ZETA`: `0.6`, `0.8`, `1.0`; `LEG_ALPHA`: `0.5`, `0.75`, `1.0`.
3. `SUBSTEPS`: `1`, `2`, `4` at the best compliance.
4. v10 particle `inv_mass` ratios for hip/knee/foot (the user's suggested
   weight ratios) once the compliance band is found.
5. Re-run the headless `walk` and `jump` scripts; the target is upright torso,
   no bone stretch >5%, penetration <2 px, and foot tracking under ~20 px.

### Mass-ratio result (v10)

The original v10 masses are `P1/P2/P3 = 10` and `P40/P41/PLLL/PRLL = 1`.
That 30:4 torso/leg ratio compresses the soft leg springs ~58 px (`P2y`
138→196, `P3y` 173→231), which is what reads as “standing on P2”.

Sweep at `LEG_COMPLIANCE=1e-6`, `LEG_ZETA=1.0`, torso/leg `inv_mass`:

| torso / leg `inv_mass` | torso mass / leg mass | `P2y` max | `P3y` max | `meas_theta` | penetration >5 px | `P3x` travel |
|---|---|---|---|---|---|---|
| `0.1 / 1.0` (original) | 10 / 1 | 198.7 | 232.5 | −0.084…0.233 | 0 | 130.9 |
| `0.5 / 0.5` | 2 / 2 | 189.6 | 221.0 | −0.022…0.088 | 0 | 104.2 |
| **`1.0 / 0.5` (chosen)** | **1 / 2** | **179.9** | **211.1** | **−0.028…0.121** | **0** | **124.7** |
| `0.5 / 0.25` | 2 / 4 | 195.5 | 230.0 | −0.033…0.091 | 0 | 46.2 |

Chosen v10 masses: `P1/P2/P3 inv_mass = 1.0`, `P40/P41/PLLL/PRLL inv_mass = 0.5`.
This cuts the waist/hip sag by ~20 px, keeps the torso within ±0.12 rad, and
removes all >5 px penetration. It still does not reach the commanded stride;
leg tracking peak remains ~129 px and the tail is `WAGGLE`, so the next pass is
compliance/zeta/alpha/substeps around this ratio.

## 11. Why compliance cannot make the legs support the body

The angular constraints are internal to the skeleton. Internal springs cannot
hold a body against gravity without an external reaction. The external reaction
must come from the floor, and right now it does not reach the skeleton:

- `skeleton_drives_body` makes the visual soft-body parts **follow** the
  skeleton one-way.
- Collision reaches the skeleton only through the kinematic channel
  (`apply_collision_correction`, with `collision_damping` /
  `rotation_resistance`), not as a load-bearing contact force.
- The floor normal force acts on the soft-body foot collider, not on the
  skeleton foot particle. The skeleton has no upward reaction, so the leg
  angular springs compress until the geometry bottoms out.

That is exactly what the numbers show: `LEG_COMPLIANCE=1e-6` gives a ~40 px
squat, `5e-7` collapses (P3y 281), and `SUBSTEPS=2/4` at `5e-7` still folds
(30% P1–P3) or NaNs. Compliance, mass ratio, and substeps only change how the
unsupported body deforms; they cannot create a support force.

The old external pin worked because it **was** the missing external reaction.
To remove it, the skeleton needs one of:

1. A P2-only vertical support constraint (soft, grounded-only) — the minimal
   change, and still angular-only for the pose.
2. Skeleton foot contact constraints, so the floor can push the actual
   `PLLL`/`PRLL` particles.
3. A two-way collision→skeleton channel that transmits a load-bearing normal
   force, not just a damped kinematic delta.

Until one of those exists, angular-only walking will always squat or collapse
under gravity, regardless of the compliance settings.

## 12. Substep convergence result (2026-10-19)

The tuning app and headless now share `Time<Fixed> = 90 Hz` and the same
`SUBSTEPS` env, default **2** in both. Sweep at mass B + `LEG_COMPLIANCE=1e-6`:

| SUBSTEPS | P2y max | P3y max | P3x travel | rigidity | leg peak |
|---|---|---|---|---|---|
| 1 | 179.9 | 211.1 | 124.7 | 3.0% | 126 px |
| **2** | 187.9 | 222.8 | **262.0** | **1.4% (pass)** | **101 px** |
| 4 | 194.0 | 228.4 | **506.1** | 2.2% | 152 px |
| 8 | — | — | — | NaN (now guarded) | — |

Substeps are a real convergence lever: `2` gives the best rigidity and roughly
doubles the traveled distance. But the squat does **not** disappear — it gets
slightly deeper as substeps increase, because the skeleton still lacks a
load-bearing floor reaction. So `SUBSTEPS=2` is the new shared default and the
best tuning baseline, but section 11’s support-path fix is still required.

High substeps exposed a NaN panic in `drive_hip_pin`; it is now guarded by
`clamp_attitude` plus finite-vector checks in `angle_basis`/`attitude_rest_basis`.
