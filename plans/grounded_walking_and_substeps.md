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
