# Grounded Walking & Substep Stability — Investigation Plan (2026-10-17)

> Status: **planning / next phase.** The kinematic foot-divergence problem is
> resolved (see [`foot_divergence_state_of_knowledge.md`](foot_divergence_state_of_knowledge.md)).
> This file records what to measure, what to instrument, and which hypotheses to
> kill first before reintroducing gravity, collision, and a half-pinned hip.

## 1. Where we are

Working baseline (headless walk, `GRAVITY=off`, no colliders):

| metric | value |
|---|---|
| foot divergence, peak hypot | **3.5 px** |
| foot divergence, move mean | **0.70 px** |
| late mean | **0.43 px** |
| tail verdict | `PASS (rms 0.394 px)` |
| pin frame P3 residual | **0.58 px** |
| determinism | identical hash across runs |
| leg config | compliance `1e-10`, `ζ=0.5`, `α=1`, `soft_scale=1` |
| pin | position-only (`kp=kd=0`), compliance `1e-7`, damping `0.01` |
| resolver | `thigh=calf=55`, `hip_height=90`, walk `target_speed=120` |

The rig is currently **kinematic**: `P1/P2/P3` are pinned, no gravity in the
gate, no floor, and the legs follow the resolver through angular constraints.
The next phase puts the character on the ground and lets contacts carry the
stance.

## 2. Immediate cleanup: make the projection truly off

The angle-derived reference is now in place, but the projection is not useful
(§11.6 of the state doc). Make the intent explicit:

- when `LimbConfig::sigma <= 0`, emit `angular_velocity: None` from
  [`calculate_leg_pd`](../examples/game_lib/src/character/leg_pd.rs:159) and
  skip the reference-rate computation;
- keep `LegPdMemory` for the future velocity-level implementation;
- keep the call in
  [`post_step`](../../study_vello/integrations/vello_physics/src/soft_body_connection.rs:387)
  (it returns immediately for `sigma < 1e-6`) or remove it until the velocity
  stage is redesigned.

Acceptance: identical walk numbers with `sig=0`, no `angular_velocity` in the
emitted configs, no dead computation.

## 3. Why do 4+ substeps explode?

The old state doc observed 1 → 45.7 px, 2 → 45.6 px, 4 → 933.7 px,
8 → 3.3e9 px (NaN). That was measured with the old target bridge, the old leg
config, and the velocity projection. **Retest before theorizing** — the current
rig may no longer explode. If it does, the mechanism is almost certainly one of
the frame/substep mismatches below.

### 3.1 How the step loop is actually ordered

[`ConstraintWorld::step`](../../study_vello/integrations/vello_physics/src/collision_response.rs:543):

```
for each substep (sub_dt = dt / N):
    body.predict_positions_frame(sub_dt, gravity)
    connection.step(sub_dt):          # skeleton particles
        predict_positions(sub_dt)
        solve_constraints(sub_dt):
            external_position_constraints once      # the HipPin
            8 × { distance..., angular... }         # dt * 0.25
    body.step(sub_dt)                            # collisions
    connection.post_step(sub_dt):
        external_position_constraint_damping()
        [velocity projection — disabled / σ=0]
        apply_collision_correction()
        update_particles_velocity(sub_dt)
```

`drive_hip_pin` runs **once per FixedUpdate** (1/90 s) and queues external
position constraints. Those constraints are then solved **once per substep**
with a target built from the *frame* dt:

- `g_dt2 = g * dt²` uses the frame dt ([`hip_pin.rs`](../examples/game_lib/src/lower_body/hip_pin.rs:333));
- `pd_target(...)` uses the frame `dt`;
- the pin damping runs every substep.

### 3.2 Ranked hypotheses

| rank | hypothesis | why it would explode with N |
|---|---|---|
| **H1** | Frame-level pin command applied per substep | `g·dt²` is applied N times per frame, so the pin injects N× the gravity feed-forward. The same for any frame-rate PD target. At N=4 this is a 4× upward bias. |
| **H2** | Pin solved once, angular constraints 8× per substep | The free chain walks away from P3 during 8 passes; the next substep's pin yanks it back. N substeps = N yanks per frame. |
| **H3** | `external_position_constraint_damping` per substep | A frame-rate damping coefficient applied N times per frame over-damps or injects an impulse at each substep boundary. |
| **H4** | `alpha`/`gamma` scaling with 8 passes | `alpha = compliance/dt²` grows as 1/N² and `gamma` as 1/N. The per-pass correction and damping both change with N; 8N passes can over-correct. |
| **H5** | `update_particles_velocity` 0.999 per substep | N× extra velocity loss per frame; should not explode, but biases the measured damping. |
| **H6** | Collision / transmission offsets | Not active in the current gate (no colliders); becomes relevant after §4. |

H1 and H2 are the ones that explain "more substeps = worse" rather than merely
"different".

### 3.3 Experiment matrix

Run each cell with `OUT_CSV` distinct and record peak/mean/NaN/max|v|:

| # | mode | pin | gravity | projection | substeps | purpose |
|---|---|---|---|---|---|---|
| 1 | limb (`LIMB=right_leg`, no `SCRIPT`) | no | off | off | 1,2,4,8 | isolates the general solver |
| 2 | body (`SCRIPT=walk_script.json`) | yes | off | off | 1,2,4,8 | reproduces the old explosion |
| 3 | body | yes | on | off | 1,2,4,8 | H1: gravity feed-forward |
| 4 | body, `LEGS=off` | yes | off | off | 1,2,4,8 | separates leg drive from the pin |
| 5 | body, pin disabled (new `PIN=0` lever) | no | off | off | 1,2,4,8 | H2: pin/angular alternation |

If test 1 is stable at 8 and test 2 is not, the pin is the cause. If test 3 is
much worse than test 2, H1 leads. If test 4 is stable, the leg PD's interaction
with the pin matters. If test 5 is stable, H2 leads.

### 3.4 Instrumentation to add (before changing anything)

- `STABILIZE_STEPS` env (default 8) and log it next to `SUBSTEPS`.
- Per-substep guard: max |velocity|, max |Δpos|, NaN count; abort with the
  substep index and the worst particle.
- Log the pin target error per particle (`P1/P2/P3`) *before* and *after* the
  external constraint solve, and the accumulated `g_dt2`.
- Log `alpha`, `gamma`, and the applied `delta_lambda` for the worst angular
  constraint.
- Optional: a `PIN_SUBSTEP=frame|substep` switch to solve the pin once per frame
  instead of per substep.

### 3.5 Candidate fixes, in order

1. **Scale the pin's frame-level feed-forward by `1/N`** (at minimum `g_dt2`),
   or move the pin command into the substep loop so it is recomputed with
   `sub_dt`.
2. **Solve the pin inside the same iteration loop as the angular constraints**
   (or at least re-solve it between the 8 passes) so it cannot be left behind
   and yanked.
3. **Make the pin velocity-level** (target velocity + damping) instead of a
   position target solved N times.
4. Revisit `STABLIZE_STEPS_COUNT=8` / `dt*0.25` only after 1–3.

Acceptance: substeps 1/2/4 are monotone or flat (no NaN, no >2× peak), and the
frame residual stays <1 px.

## 4. Grounding the character and reintroducing collision

### 4.1 Geometry to reconcile

- Resolver: `hip_height=90`, `thigh=calf=55`, foot at `hip.y - hip_height`.
- Character: hip `P3` at the pinned height; legs 55+55; feet currently follow
  the resolver kinematically.
- Floor: currently no collider; the "floor" is only the resolver's `floor_y`.
- The pin anchor (`anchor_p3`) and the leg bridge seed are captured **once**;
  any change to the spawn height or teleport invalidates both. Call
  `reseat_leg_bridge` (and reset the pin) after moving the body.

### 4.2 Staged plan

1. **Static stand, collision on.** Add a floor collider at the resolver's
   `floor_y`; add foot colliders sized to the visual feet. Spawn the feet 2–5 px
   above the floor, `GRAVITY=on`. Measure penetration, settle time, pitch, and
   the pin's vertical share.
2. **Match the hip height.** Lower/raise the pin anchor until the feet rest with
   zero penetration and the pin's vertical error is ~0. The resolver's
   `hip_height` must be adjusted to the contact-adjusted hip height; otherwise
   the leg targets command a pose the body cannot occupy.
3. **Weight shift.** Add a `stand → shift left/right` script. Measure lateral
   CoM travel, foot slip, and whether the pin or the contacts provide the
   support.
4. **One step.** A single `move` for one stance+swing cycle. Gate on
   penetration < 1 px, slip < a few px, no NaN.
5. **Walk.** Full walk script with contacts. Compare against the kinematic
   baseline (3.5 px peak) and track slip/penetration/pin share.

### 4.3 The stance-foot contract

Right now the stance foot is a **kinematic target** (the resolver pins it in
world space and the leg PD servos to it). With contacts, that becomes a
competition: the contact wants to support the body; the leg PD wants the foot
at the target. Three options:

| option | stance foot | pros | cons |
|---|---|---|---|
| A. Keep kinematic stance, collide only the swing foot/body | leg PD | preserves the current 3.5 px tracking | contact can fight the leg; friction not exercised |
| B. Contact-driven stance, leg PD only for swing | contact + friction | physically meaningful walking | needs the resolver to stop pinning the stance foot; harder tuning |
| C. Soft stance leg | leg PD with high compliance | lets the contact push the foot | reintroduces tracking lag; use the feed-forward work |

Recommended: start with **A** to get collision stable, then move the stance to
**C**, then **B**. Record which option each run uses.

### 4.4 Half-pin design

Goal: prevent falling and make tuning easier, while letting contacts carry the
body. Candidate implementations, from least to most invasive:

1. **Axis mask on the existing pin.** Pin `P1/P2`, and pin `P3` only on x (and
   optionally attitude); leave y free. Implement as a per-axis weight on
   `pd_target`/target delta in [`drive_hip_pin`](../examples/game_lib/src/lower_body/hip_pin.rs:261).
   This is the smallest change and directly gives "half pinned".
2. **Dead-band / soft pin.** Only correct when the error exceeds a radius;
   scale compliance with speed (the existing `softened_compliance` already does
   part of this). Lets the body move naturally inside the band.
3. **Velocity-only balance.** Replace the position target with a velocity/force
   that damps horizontal CoM velocity and pitch, while contacts provide the
   vertical support. This is the cleanest physical model but needs a CoM/pitch
   estimator and a force budget.
4. **Upper-body-only pin.** Pin `P1/P2`, leave `P3` to the legs/contacts entirely;
   the resolver's hip target becomes a velocity/force reference.

Metrics for the half-pin:

- **support share**: pin vertical force proxy / total support (pin + contacts).
  Target ≈ 0 in steady stance, non-zero only during balance recovery.
- **pitch/roll**: body attitude error vs vertical.
- **CoM height** and its variation.
- **settle time** after a push or a step.
- **leg tracking** (the existing `.leg.csv`) and **foot slip**.

Do not tune the half-pin against the foot-divergence metric alone; a pin that
holds the body rigidly will make the legs look good while the contacts do
nothing.

## 5. Instrumentation and gates to add

### 5.1 CSV columns

Extend the body telemetry with:

- `substeps`, `stabilize_steps`, `gravity`;
- per foot: `contact`, `normal_impulse`, `tangent_impulse`, `slip_x` (realized
  foot x minus the stance plant x), `penetration`;
- per pin particle: `pin_err_x/y`, `pin_target_x/y` (or the applied offset);
- `pin_share_y` (pin vertical proxy / (pin + contact vertical proxy));
- `pitch`, `roll`, `com_x/y`, `max_v`.

### 5.2 Guards (fail loudly, not with NaN)

Abort with a diagnostic when any of:

- non-finite position/velocity;
- |velocity| > a configurable bound (e.g. 10× the target speed);
- |position| outside the arena;
- penetration > threshold for more than K ticks;
- rigidity error > threshold;
- foot slip > threshold during stance.

The harness already checks rigidity and NaNs; add the rest.

### 5.3 Scripts and analyzer

- `plans/stand_script.json`: wait 2 s.
- `plans/weight_shift_script.json`: wait, move left, wait, move right, wait.
- `plans/walk_collision_script.json`: the existing walk.
- `tools/physics-qa/analyze_grounded.py`: penetration, slip, contact duty,
  pin share, pitch, settle; reuse the CLI style of the existing analyzers.

## 6. Prioritized backlog

| priority | task | acceptance |
|---|---|---|
| P0 | Emit `angular_velocity: None` when `sigma <= 0`; skip the reference computation | identical walk metrics, no dead work |
| P0 | Retest `SUBSTEPS=1,2,4,8` with the fixed config, projection off; pin on/off, gravity on/off | table of peak/mean/NaN/max\|v\|; no unexplained explosion |
| P0 | Add `STABILIZE_STEPS` + per-substep guard + pin error logging | a failing run reports the substep and worst particle |
| P1 | Fix the frame/substep scaling (H1/H2) and retest | substeps 1/2/4 monotone or flat |
| P1 | Floor + foot colliders; static stand gate | penetration < 1 px, settle < 1 s, no NaN |
| P1 | Match resolver `hip_height` to the contact-adjusted hip height | pin vertical error ≈ 0 at rest |
| P2 | Half-pin v1: axis mask (`P3` y free) | support share ≈ 0 at rest, no fall on a small push |
| P2 | Weight-shift and one-step scripts | slip and penetration within budget |
| P2 | Grounded analyzer and gate | one command reports all metrics |
| P3 | Contact-driven stance (option B) or soft stance (option C) | walk with friction, tracking comparable to baseline |
| P3 | Determinism of contact runs | identical hash across two runs |

## 7. Reference numbers

| item | value |
|---|---|
| old foot divergence (before fix) | 60.1 px peak, 32.1 px mean |
| fixed kinematic walk | 3.5 px peak, 0.70 px mean, `PASS 0.394` |
| pin residual (walk) | 0.58 px |
| substep explosion (old, unverified) | 4 → 933.7 px, 8 → 3.3e9 (NaN) |
| resolver defaults | thigh 55, calf 55, hip_height 90, floor_y 0 |
| pin defaults | compliance 1e-7, damping 0.01, kp=kd=0 |
| stabilization | 8 passes, `dt*0.25` |
| leg config | compliance 1e-10, ζ=0.5, α=1, soft_scale=1 |

## 8. Pitfalls

1. **Reseat after teleport.** `LegBridgeSeed` and `anchor_p3` are captured once.
   Changing the spawn height or moving the body requires
   `reseat_leg_bridge` + a pin reset, or the targets jump.
2. **Units/scale.** The resolver works in px; the tuning app applies a world
   scale. Keep the headless harness and the visual app on the same scale when
   comparing numbers.
3. **Do not tune compliance before the substep scaling is fixed.** H1/H2 can
   masquerade as a stiffness problem.
4. **Pin vs contact.** A hard position pin will absorb the contact forces and
   make friction invisible. Measure the support share; if the pin carries the
   body, the contact test is not testing anything.
5. **Foot collider radius.** A circle collider shifts the effective stance
   height by its radius; the resolver's `hip_height` and the pin anchor must
   account for it.
6. **Determinism.** Contacts and the parallel body step can introduce
   order-dependent results. Run every gate twice and compare the hash.
7. **The old "velocity projection" trap.** Do not re-enable it as a tuning knob.
   The reference is now clean, but the application stage is still wrong (§11.6
   of the state doc). Fix the stage first.

## 9. Open questions

1. Is the pin a **root-motion driver**, a **balance controller**, or a **spring**?
   The half-pin design depends on the answer.
2. Should the stance foot be **kinematic** (option A/B/C above), and what is the
   acceptable slip budget?
3. How much of the body's support should the pin provide at rest and during
   balance recovery?
4. Is the half-pin **per-axis** (free y, lock x/pitch) or **per-particle**
   (P1/P2 pinned, P3 free)?
5. Do we want the resolver's `hip_height` to be a *commanded* height or a
   *measured* one after contact settles?
6. What is the smallest contact set that makes friction meaningful: foot
   circles only, or foot + lower-leg capsules?

## 10. Collision bring-up status (2026-10-17)

Collision is now available in the headless harness behind `COLLISION=1`.

### What was added

- `COLLISION=1` env flag. Default off, so the kinematic gates stay
  byte-identical.
- A headless wgpu device + `vello::Renderer` (no surface/window) and
  `GpuCollisionRunner`, inserted at Startup.
- `VelloCollisionBroadPhase` + `run_broad_phase` (CPU BVH) and
  `run_gpu_collision` (GPU narrow phase), scheduled after
  `update_collider_from_soft_body` exactly like the visual plugin.
- `TransformPlugin`, needed for `GlobalTransform` propagation into the BVH.
- A static floor collider with its **top surface** on the foot line, spawned
  once the pin is seeded:
  `surface_y = -anchor_p3.y - live_stance_height`,
  `floor_y (center) = surface_y - 20` (the rect is 40 px thick). `live_stance_height`
  is the character's own hip→foot distance at the bridge seed (~106 px), **not**
  the resolver's `hip_height=90`: the pose-aware bridge preserves the live
  (nearly straight) leg, so the resolver's bent-stance height would put the
  target feet ~16 px below the floor.
- `HIP_DROP=<px>` lowers the whole pinned body and both leg targets relative to
  the fixed floor (positive = into the floor). `0` is the natural stance; it is
  a deliberate-penetration knob for the stroke round-end margin.

### Run

```powershell
cd D:\bender\bevy_vello
$env:ASSET_ROOT = "D:\bender\bevy_vello\examples\collision_detection\assets"
$env:SCRIPT     = "D:\bender\bevy_vello\plans\walk_script.json"
$env:COLLISION  = "1"
$env:GRAVITY    = "on"
$env:OUT_CSV    = "D:\bender\bevy_vello\tmp\headless\walk_collision.csv"
target\release\examples\headless_arm_pd.exe
```

### First measurements (walk, 120 px/s)

| run | floor surface | contacts | foot peak | foot mean | tail | rigidity |
|---|---|---|---|---|---|---|
| gravity, collision off | — | 0 | 3.46 px | 0.63 px | PASS 0.359 | ok (0.0011) |
| collision, resolver `hip_height` | 90 px | 1–4 | 7.79 px | ~5.1 px | WAGGLE 1.836 | FAIL 0.118 |
| collision, live stance, center at foot line | 126 px\* | 2 | 5.86 px | ~3.7 px | WAGGLE 0.720 | FAIL 0.093 |
| collision, live stance + `HIP_DROP=1.5` | 126 px\* | 2 | 6.16 px | ~3.9 px | WAGGLE 0.443 | FAIL 0.096 |
| collision, **live stance, top surface at foot line** (shipped) | **106 px** | 2 | **3.58 px** | **~1.2 px** | WAGGLE 0.298 | FAIL 0.033 |

\*The earlier runs placed the *center* of the 40 px-thick rect on the foot line,
so the rect's top surface was 20 px above the feet and the strokes were already
20 px penetrated at spawn. Subtracting the half-thickness fixes that: the same
walk now reports early/mid/late means of 1.16 / 1.26 / 1.25 px and a peak of
3.58 px (deterministic across reruns).

Interpretation:

- The pipeline works: the BVH sees 13 colliders (12 soft character parts + the
  floor) and the GPU returns contacts every frame; no NaN.
- The first collision runs used `anchor_p3 − resolver.hip_height` (90 px) as the
  floor. Because the pose-aware bridge keeps the live, nearly straight leg, the
  target feet were ~16 px below that floor. Switching to the **live stance
  height** (106 px) puts the foot particles on the floor line: at t=11,
  `PLLLy=279.23`, `PRLLy=278.31`, floor `y=279.0` (Vello). This is the "lower
  the character until the hip height matches the P3→floor distance" step.
- The remaining contacts are the leg collider's **stroke width + round end**
  below the foot particle. That margin is intentionally ignored for now; the
  foot particles are the reference.
- `rig=FAIL` (3.3 % bone-length deviation after the half-thickness fix, down
  from 9.3 %) is the soft-body contact deformation, not a kinematic failure.
  The worst bone is `P40-PLLL` (the foot stroke), exactly the contact site. Gate
  collision runs on penetration/slip/pin-share, not on the rigid-bone check.
- The visual `character_tuning_main` app now uses the same geometry:
  `GROUND_SURFACE_Y = -279`, rect center `-299` (top edge on the feet).
  Careful with the scale: `assemble_character` **bakes the root transform**
  (`scale=0.5`) into the particles via `transform_to_affine`, so the headless
  foot at Vello y=+279 is already the world physics position. Dividing by 0.5
  a second time (the first attempt used -139.5) puts the rect at the pelvis and
  leaves the character sunk to the hips.
- `HIP_DROP>0` pushes the body further into the floor and does not help; keep
  it at `0` until the pin is released vertically.

### Next steps

1. Release the pin vertically (half-pin v1) so the contacts carry the body.
2. Recompute the floor and hip height from the contact-settled pose.
3. Add penetration/slip/pin-share columns and the grounded analyzer.
4. Then tune friction and the stance contract (options A/B/C in §4.3).

## 11. Delta (state-relative) pin experiment (2026-10-17)

Goal: remove the absolute-pose projection that made the hip kinematic, and let
gravity + contacts own the vertical before adding any half-pin masking.

### 11.1 Control law

`PinMode::Delta` (`PIN_MODE=delta`) computes, per pinned particle:

```text
p*     = p + v·dt + g·dt²              // the free-flight prediction
target = p* + k_v·(v_ref − v)·dt      // k_v clamped to [0, 1]
```

No absolute pose term and no `+g·dt²` feed-forward: gravity is already in the
prediction the physics step makes, so the constraint adds only the velocity-error
correction. The resolver pose enters only through `v_ref` (its hip velocity).

Applying the resolver's **raw** delta instead (`p* + v_ref·dt`) would add `v_ref`
to the realized velocity every tick (`v_{n+1} = v_n + v_ref`) — the "constant
speed gets faster and faster" failure. The `− v` term is what turns it into a
servo; a unit test locks both behaviors.

`PinMode::Rigid` is the previous absolute-pose pin, kept selectable for A/B.
The law is only valid at `SUBSTEPS=1`: the position constraint carries a
frame-level prediction and is re-solved per substep.

### 11.2 Run

```powershell
$env:ASSET_ROOT = "D:\bender\bevy_vello\examples\collision_detection\assets"
$env:SCRIPT     = "D:\bender\bevy_vello\plans\walk_script.json"
$env:COLLISION  = "1"
$env:GRAVITY    = "on"
$env:PIN_MODE   = "delta"
$env:KV         = "0.3"          # sweep 0.1 / 0.3 / 0.6 / 1.0
$env:PIN_LOG    = "1"            # once-per-second pin line (optional)
$env:OUT_CSV    = "D:\bender\bevy_vello\tmp\headless\walk_delta_k03.csv"
target\release\examples\headless_arm_pd.exe
```

The pin log line reports `P3 actual`, the resolver `ref`, the emitted `cmd`, and
`sag_y = actual.y − ref.y` in the Vello y-down frame (positive = body lower than
the plan). The `PIN_MODE`, `KV` and `SUBSTEPS` are printed once at startup.

### 11.3 Results (walk, 120 px/s reference; 11.5 s)

| run | foot peak | hip lag mean/max | sag mean (min..max) | realized vx | rig dev | contacts |
|---|---|---|---|---|---|---|
| rigid baseline | **3.58 px** | 0 | ~0 | 115.5 | 0.033 | 2 |
| delta k_v=0.1 | 259.1 px | 121.5 / 258.1 px | −5.3 (−9.0 .. 2.2) | 92.3 | 0.0231 | 5 |
| delta k_v=0.3 | 52.2 px | 30.6 / 50.4 px | −5.9 (−10.7 .. 1.6) | 112.4 | 0.0233 | 5 |
| delta k_v=0.6 | 43.3 px | 24.5 / 41.3 px | −4.8 (−8.9 .. 1.2) | 113.3 | 0.0238 | 3 |
| delta k_v=1.0 | 34.8 px | 18.6 / 32.7 px | −4.3 (−8.7 .. 1.0) | 112.5 | 0.0252 | 3 |

All delta runs: `nan_rows=0`, deterministic, `tail=PASS` (the rigid tail was
WAGGLE).

### 11.4 Interpretation

- **No runaway.** Realized speed sits at 112–113 px/s against a 115.5 px/s
  reference for every usable `k_v`. The raw-delta accumulation is absent.
- **No sitting.** The frame does not collapse when the pin stops cancelling
  gravity. The mean sag is *negative* (a few px **above** the reference) and the
  worst downward excursion is ~1–2 px: contacts and the leg chain already carry
  the body in this configuration. The predicted "legs not strong enough" failure
  did not appear, so support authority is not the first problem.
- **The residual is horizontal position lag**, 18–30 px mean at usable `k_v`,
  because the law has no position feedback. Foot error (35–52 px) is mostly that
  lag: the leg targets are still built from the resolver hip (`pose.hip`), so the
  legs chase a point the body is not at.
- Rig deviation improved (2.3–2.5 % vs 3.3 %) and the tail verdict improved,
  consistent with contacts doing more of the work.

### 11.5 Next

1. Add a bounded position term (`k_p·clamp(p_ref − p, ±e_max)·dt²`) or retarget
   the stance foot from the **actual** hip, so the lag is corrected without
   restoring a hard absolute pose.
2. Then the stance/support work: `anchor_chain` off for the stance leg and a
   support-share force.
3. Keep `SUBSTEPS=1` until the command becomes a per-substep force/impulse.

### 11.6 Gravity gate: 100 px floor drop (2026-10-17)

Test: `HIPPIN=1`, `COLLISION=1`, `GRAVITY=on`, a 3 s stand script, and the floor
surface lowered 100 px below the seed foot line (`GROUND_DROP=100`, i.e. 1 m in
this world). The floor is still spawned from the live stance, so only its height
changes.

| run | P3 y at t=1 / 2 / 3 | sag at t=3 | contacts | verdict |
|---|---|---|---|---|
| rigid | 173.2 / 173.2 / 173.1 | 0.2 | 0 | **floats** 100 px above the lowered floor |
| delta k_v=0.3, vertical on | 218.3 / 264.6 / 273.1 | 100.2 | 0 / 0 / 4 | falls, but ~40 px/s |
| delta k_v=0.3, `PIN_VERTICAL=off` | 266.9 / 272.2 / 273.3 | 100.3 | 2 / 4 / 4 | **ballistic**, lands in <1 s |
| delta k_v=1.0, vertical on | 187.4 / 201.6 / 216.2 | 43.2 | 0 | slower still |

**Gravity is applied** — the rigid pin floats, the delta pin falls by exactly the
drop and lands on contacts. But the symmetric delta law is a **velocity**
matcher, not a force: with `v_ref,y = 0` the vertical equilibrium under gravity
is `v_y = g·dt/k_v` (980/90/0.3 ≈ 36 px/s; k_v=1 ⇒ 11 px/s). That is exactly the
measured fall rate. The `g·dt²` in `p*` is real, but the position constraint also
rewrites the realized velocity from the target, so gravity cannot accumulate
ballistically.

`PIN_VERTICAL=off` zeroes the y component of the velocity-error command; gravity
then integrates freely and the drop completes in under a second (94 px by t=1).
That is the first concrete "release the vertical" (half-pin) step. A unit test
locks both behaviours (masked y accumulates `10·g·dt`; commanded y stays under
half of it).

Consequences for the walk: with the vertical unactuated the body sinks further
and tracking degrades (delta k_v=0.3, vertical off: peak 82.1 px vs 52.2 px,
rig 0.0286, tail PASS, contacts 5). So the correct shape is **one-sided**: hold
height when the body is above/at the plan, but never damp free fall; or rely on
stance-leg support (the `anchor_chain`/support-share work).

### 11.7 Visual app

`character_tuning_main` now uses `HipPinParams::from_env()`, and its ground rect
honours `GROUND_DROP`. To see the drop:

```powershell
$env:PIN_MODE      = "delta"
$env:PIN_VERTICAL  = "off"
$env:KV            = "0.3"
$env:GROUND_DROP   = "100"
cargo run -p collision_detection --bin character_tuning_main --release
```

The startup logs print `tuning ground: surface y=...` and the pin controller
mode/vertical flag, so a run is self-describing.

### 11.8 Frame-attitude bug in the first delta implementation (2026-10-17)

Symptom (visual app): the upper body tips sideways until it rests on the ground.
Headless confirmed it numerically — the spine angle of `P1−P3` from vertical:

| run | tilt at t≈5.8 | tilt at t≈11.5 |
|---|---|---|
| rigid baseline | 0.0° | 0.0° |
| delta k_v=0.3, per-particle law | −110.2° | 171.8° |
| delta k_v=0.3, frame law (fixed) | 0.0° | 0.0° |

The `rig` gate only checks bone lengths, so a rotated frame passed it. The
distance constraints keep lengths, not attitude.

**Cause:** delta mode applied the servo **per particle**. Each particle was
predicted from its own velocity and corrected independently, so nothing held
the frame's orientation; gravity and contact torques rotated it freely. The
rigid pin held attitude implicitly by projecting each particle onto the
absolute (rotated) pose targets.

**Fix:** [`delta_frame_targets`](../examples/game_lib/src/lower_body/hip_pin.rs)
moves the **frame centroid** with the delta law and rebuilds all three targets
from the reference offsets, so shape and attitude are exact by construction.
The offsets must be **centroid-relative** — using `ref − P3` adds a constant
bias to the realized centroid every tick and launches the body (observed at
`y=-118k` before the correction).

**After the fix** (walk k_v=0.3, collision on): peak 63.1 px, tail PASS
(rms 0.62 px), rig 2.78 %, contacts 2, tilt 0.0° throughout. Drop test with
`PIN_VERTICAL=off`: lands by t=1 (contacts 4), tilt 0.0°; the remaining
119.8 px foot error is the old-floor target, i.e. the stance-retarget task.

A unit test (`delta_frame_keeps_attitude_under_differential_velocity`) locks the
behavior: one particle moving does not shear the frame, and the centroid still
translates.

### 11.9 Ground-gated gravity cancellation (replaces global `PIN_VERTICAL=off`)

Releasing the vertical axis permanently also removed jump authority: the hop
rise fell from 53.4 px to 28.3 px. The right shape is to gate authority on
**real ground contact**, not on a global switch.

**Design**

- `GroundSupport` is refreshed by `update_ground_support` from
  `CollisionEventBatch`: a mixed pair (soft character part vs static/other
  collider) whose contact normal is mostly vertical counts as floor contact.
  Register the system before `drive_hip_pin`. Without a collision pipeline the
  resource keeps its grounded default (legacy behaviour).
- `drive_hip_pin` then decides per tick:
  - vertical command allowed = `PIN_VERTICAL && (grounded || phase is
    Launch/Flight)`. In flight the resolver's velocity reference is ballistic, so
    tracking it preserves the jump instead of damping gravity;
  - `support_share` (default **0.5**) of gravity is cancelled **only while
    grounded**; airborne it is forced to 0.
- Walking keeps a stance foot down at all times (duty 0.5), so support is on
  during the whole gait; jumps and drops have no contact and are ballistic.

**Results**

| case | grounded | result |
|---|---|---|
| walk, k_v=0.3 | always | contacts 2, peak 44.4 px, rig **ok** 2.0 %, tail PASS 0.72 |
| jump | launch → flight | rise **57.1 px** (rigid 61.8; global vertical-off gave 28.3) |
| 100 px drop | false after the ledge | lands by t≈0.6 (contacts 2 at t=1), sag ≈100, tilt 0° |

The drop's remaining ~104 px foot error is the feet still targeting the old
floor line — the stance-retarget task, not a support problem.

**Knobs:** `SUPPORT_SHARE=0..1` (env) and the matching UI slider; `PIN_VERTICAL`
remains a master switch. Unit tests cover airborne ballistic release,
`support_share` cancelling 0/50/100 % of gravity, and the grounded upward
command preserving a jump.
