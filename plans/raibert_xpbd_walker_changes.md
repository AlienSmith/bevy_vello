# Raibert XPBD 2D Walker — Required Changes

Concrete changes needed to bring the current implementation up to the converged design in
[`raibert-xpbd-2d-walker.md`](raibert-xpbd-2d-walker.md). Each row is `Current → Change`.

> **Concepts reference** (capture point vs Raibert neutral point, hip vs CoM) — §0.

## Status — where to pick up

| phase | section | state | commit |
| --- | --- | --- | --- |
| A | §1 whole-body CoM measurement layer | **done** | `7031c6f` |
| B | §2 loop 1 capture point | **done** | `aea3f96` |
| C | §3 loop 1 scripted swing profile | **done** | `fb46b64` |
| D | §4 loop 2 virtual-leg thrust | **done** | `a14b1f9` |
| E | §5 loop 3 leg-delivered attitude | **done** | `6be7a63` |
| **F** | **§6 contact-gated gait clock authority** | **not started — next** | — |

Read §6 (note the Phase-E correction box inside it), then
[`step_leg`](../examples/game_lib/src/lower_body/resolver.rs:1230) for the timer that
needs gating and [`update_ground_support`](../examples/game_lib/src/lower_body/hip_pin.rs:189)
/ [`GroundSupport`](../examples/game_lib/src/lower_body/hip_pin.rs:171) for the contact
truth that already exists.

Baseline to measure Phase F against (best-walk config `LEG_COMPLIANCE=1e-9
STEP_HEIGHT=70`, from [`physics_tuning_progress.md`](../tools/physics-qa/physics_tuning_progress.md:517) §10):
swing **28/29** pass, body **99.5 px/s vs 103.8 cmd (96%)**, rig `max_rel_dev`
**0.1502**, stance slide **61.1 px worst**. Phase F's target is the slide — the swing
gate and forward progress should not regress.

---

## 1. Balance reference — the CoM and the P3-heavy hip coincide

- **Current:** resolver runs at the hip (`state.hip_pos`); there is no whole-body CoM
  anywhere in the code.
- **Change:** compute whole-body CoM from all character particles × mass
  (`m_i = 1/inv_mass`, including arms), plus EMA-filtered `v_CoM` and `z_CoM` (height
  above the floor). With the §7 **P3-heavy** budget the CoM lands essentially at the
  hip, so the resolver's hip reference and the measured CoM coincide — loops 1 & 2 can
  use the hip as the balance origin *and* the whole-body CoM as the exact quantity.
  `resolver.rs` / new measurement module + system; mass values from
  `v10.character.json`.

## 2. Loop 1 — capture point replaces the Raibert neutral point

- **Current:** `plan_target_x` uses `x_target = v·Ts/2 + kv·(v − v_desired)` (heuristic,
  hip-based).
- **Change:** `x_foot = x_CoM + (v_CoM − v_des) / ω₀`, `ω₀ = √(g/z_CoM)` — the capture
  point shifted by the desired CoM velocity, so speed regulation is implicit (a target
  shift, not a separate gain). Recompute every frame during swing; reachability clamp
  from leg geometry (clamping = "in trouble" signal, log it). `resolver.rs`.
  (Raw balance-only form is `x_capture = x_CoM + v_CoM/ω₀`; see §0.)

## 3. Loop 1 — scripted swing profile replaces the parabolic arc

- **Current:** swing is a parabolic `step_height` arc (`step_leg`: linear x/y lerp +
  `lift = (t·(1−t)·4)·step_height`, [`resolver.rs:1073`](../examples/game_lib/src/lower_body/resolver.rs:1073)).
- **Change:** scripted profile lift (knee flex) → advance (hip flex) → extend (knee
  extend) → plant; loop 1 sets only the touchdown x. `resolver.rs`.

**Default (chosen): 3-phase foot-space piecewise profile.** Split normalized swing phase
`s ∈ [0,1]` into three segments, each a foot-space path through the existing
`solve_leg_ik`:
- **Lift** (s 0 → ~0.25): knee flex, foot rises steeply, little x advance — clears the
  toe before moving forward.
- **Advance** (s ~0.25 → ~0.8): foot travels forward at roughly constant height (hip
  flexion), covering the step length.
- **Extend / plant** (s ~0.8 → 1.0): knee extends, foot descends to the touchdown x
  `x_foot = x_CoM + (v_CoM − v_des)/ω₀`, decelerating to a gentle plant.

**Why a single symmetric parabola fails:** (1) clearance can't lead advance — it is at
floor height at t=0 and starts moving forward while still scraping; (2) no extension/
plant phase — the foot arrives descending at full swing speed = the "random touchdown
kick"; (3) constant horizontal rate means a hard stop at the plant point; (4) foot-space
shape doesn't encode the knee-flex/hip-flex/knee-extend choreography that actually clears
the toe. Cautions: keep segments C¹-continuous (match position + velocity at junctions,
blend the parabolas) so segment joints don't reintroduce a kick; the cycloid foot path is
a one-segment alternative to piecewise parabolas.

## 4. Loop 2 — virtual-leg thrust replaces the displacement spring

- **Current:** `tick_vertical` is a resolver-internal displacement spring/integrator
  (`Δl = Δl_prev + k_p·(h_desired − h_apex)`) with a vertical FSM.
- **Change:** thrust = knee/ankle extension through angular constraints during stance
  (bones are rigid; extension pushes CoM up via ground reaction); apex-height feedback
  (predicted apex vs target height scales next stance's extension — energy-based,
  mass-adaptive); flight = knee flex/retract; landing = controlled flexion (absorb).
  `resolver.rs` + leg-PD angular channel.

### The law is unchanged — the actuator and the measurement change

The algorithm **is** exactly `Δl = Δl_prev + k_p·(h_desired − h_apex)`
([`update_delta_l`](../examples/game_lib/src/lower_body/resolver.rs:498)) — a per-hop
integrator that learns the baseline stroke needed to fight gravity (no mass, no
stiffness by construction). We keep that verbatim. What loop 2 actually changes is the
**delivery** of that stroke and the **measurement** that closes the loop:

- **Measurement — measured CoM apex, not the resolver's own ballistic model.** Today the
  whole vertical loop closes *inside* `tick_vertical`: `Flight` integrates
  `hip_vel.y -= g·dt` in the resolver's own model and latches `h_apex_prev` from that
  internal trajectory ([`resolver.rs:607`](../examples/game_lib/src/lower_body/resolver.rs:607)).
  The real XPBD CoM apex is never read, so when compliance / contact / terrain make the
  physics differ from the model, the loop has no idea. Change: measure the physics CoM
  apex (ties to §1) and feed that to the integrator.
- **Delivery — already angular-only, but a *hip-height target*, not a leg stroke.**
  There is **no external hip pin** in normal walking: the constraint queue is gated off
  and commented out ([`hip_pin.rs:722`](../examples/game_lib/src/lower_body/hip_pin.rs:722)
  `if !ext_pin { continue; }`, queue call at
  [`hip_pin.rs:739`](../examples/game_lib/src/lower_body/hip_pin.rs:739)); the pin is
  air-control assist only. What reaches the physics today: the vertical spring's
  `state.hip_pos.y` becomes the leg PD's `hip_target` y
  ([`hip_pin.rs:850`](../examples/game_lib/src/lower_body/hip_pin.rs:850),
  [`hip_pin.rs:881`](../examples/game_lib/src/lower_body/hip_pin.rs:881)), and the leg PD
  converts those foot/knee/hip targets into angular joint commands. So the vertical loop
  *is* delivered angular-only — but **indirectly**, by asking the leg's angular joints to
  hold a kinematic pose the resolver computed. In `Load`/`Launch` the stroke is banked as
  a crouch depth (`target = (stand − Δl)` through `regulate_height`,
  [`resolver.rs:562`](../examples/game_lib/src/lower_body/resolver.rs:562)) and spent as a
  fixed vertical hip acceleration (`hip_vel.y += (launch_accel − g)·dt`,
  [`resolver.rs:578`](../examples/game_lib/src/lower_body/resolver.rs:578)) — a kinematic
  hip trajectory. Change: spend `Δl` **directly** as knee/ankle **extension** during
  stance (an angular push-off), not as a hip setpoint the leg must chase; the leg pushes
  the ground, the ground pushes the CoM up, and the learned `Δl` scales *next stance's
  extension*.
- **Phase behavior — a virtual leg, not a hip-height setpoint.** `regulate_height` is a
  critically-damped spring on the resolver's *internal* `state.hip_pos.y`
  ([`resolver.rs:511`](../examples/game_lib/src/lower_body/resolver.rs:511)); it never
  creates an XPBD constraint — it only sets the hip target the leg PD chases. During
  stance it fights contact through that indirect chase, and in flight there is nothing to
  push on. Change: stance = controlled push-off (extend), flight = knee flex/retract
  (shorten the virtual leg so it clears), landing = controlled flexion (absorb the
  touchdown) — all as angular-constraint targets, no hip setpoint.

### Why these changes are needed

1. **The loop is closed on a model, not on the physics.** `h_apex_prev` is latched from
   the resolver's internal ballistic flight, so the integrator learns nothing about the
   real ground reaction. If the leg's compliance or contact timing makes the actual CoM
   apex differ, the stroke keeps being wrong and no signal exists to fix it. This is the
   same open-loop problem as the current walk's stance slide: the resolver *asks* for a
   height, the physics does something else, and nothing measures the difference.
2. **A fixed launch accel is not a leg stroke.** A real run's height is set by *how far*
   the leg extends into the ground during stance, not by a velocity kick applied to a hip
   setpoint the leg chases. The `Δl` integrator's currency is displacement, so spending
   it as a constant hip acceleration (then handing that pose to the leg PD) discards the
   learned stroke; spending it as next stance's knee/ankle **extension** keeps the
   currency and is automatically mass-adaptive (a heavier body just needs the same
   extension, the ground reaction scales).
3. **The hip-height setpoint has no phase authority.** `regulate_height` commands the
   resolver's internal hip height, which the leg PD only chases through angular joints;
   in stance it fights contact *indirectly*, and in flight there is nothing to push on.
   The virtual leg splits the phases: extend in stance (ground reaction), retract in
   flight (clearance), flex on landing (absorb energy). Each phase is a different
   angular-constraint target, all through the existing leg-PD channel.
4. **Measurement is at the hip, and the §7 budget makes that exact.** Apex is read at
   `state.hip_pos.y`; with P3 heavy the CoM is at the hip, so the hip-based apex is the
   CoM apex. (Arms and a leaning stick still offset it a little, so the whole-body CoM
   of §1 remains the exact reference, but the divergence the old budget worried about is
   gone by construction.)

## 5. Loop 3 — measured world-frame attitude, **leg-delivered**, not replayed

- **Current:** `tick_attitude` is the resolver's own inverted-pendulum PD replayed as a
  target (`pin.theta`), continuous, pelvis-referenced (`P1_P2_P3` pivot). Physics
  delivery was angular-only: `tick_loop3_angular` queued a
  `CharacterAngularConstraintEvent` on the `P1_P2_P3` joint with `free_mask: 0b111`.
  But the angle it commanded was the resolver's own replayed model angle, not a
  measurement — and the joint is the wrong actuator.
- **Why the actuator is wrong (the correction to this section):** the body above `P3` is a
  straight, stiff stick (`PH–P0–P1–P2–P3` collinear), and the only external forces on it
  are gravity, foot collision and ground friction. The stick's world verticality is
  therefore decided at the **hip joints `P2–P3–P40` / `P2–P3–P41`**, not at the waist
  `P1–P2–P3`. Raibert's loop 3 has no concept of `P1` at all. Two additional facts make
  the waist the wrong lever: (a) the stick is rigid, so bending the waist *folds* the
  torso rather than tilting it as a unit; (b) the two legs apply opposite torques to the
  stick, so a leg-driven correction swings the legs forward/backward instead of folding
  the upper body.
- **Change (implemented):**
  - **`P1–P2–P3` is posture only.** `tick_loop3_angular` now commands `theta = 0.0` — hold
    `P1`/`P2`/`P3` collinear so the waist never folds. No model replay, no lean command.
  - **Loop 3 rides the leg PD.** The measured world-frame attitude (hip→shoulder, EMA
    filtered, §1) feeds [`attitude_lean_pd`](../examples/game_lib/src/lower_body/resolver.rs:736),
    which returns a *lean of the world-upright reference axis* (radians, not torque):
    `lean = bias + Kp·(bias − θ_meas) − Kd·ω_meas`, clamped to `±attitude_max_angle`.
  - **Delivery point:** [`calculate_leg_pd`](../examples/game_lib/src/character/leg_pd.rs:108)
    already aims the thigh at a world-upright axis when `upright_spine_reference` is on
    (the angular-only walking mode). That axis is now rotated by
    `LimbConfig::upright_reference_lean`, so a forward-leaning stick gets a backward leg
    reference and the stance legs push it upright — delivered through
    `P2–P3–P40`/`P2–P3–P41`, exactly where the physics says it belongs.
  - **Gating:** `attitude_enabled && !external_pin && grounded && measured.n > 0`.
    Grounded-only means **flight is ballistic** — no contact, no leg authority, so the
    reference is held straight and the stick keeps its angular momentum. The external pin,
    when active, already owns the frame pose (mid-air assist).
  - **New knobs:** `HipPinParams::attitude_lean_kp` (0.5) / `attitude_lean_kd` (0.03),
    env `ATT_LEAN_KP` / `ATT_LEAN_KD`. `attitude_compliance`/`attitude_zeta` now only
    govern the waist posture hold and the pinned path.
  - **What `tick_attitude` still does:** the resolver's internal model (whole-stick lean
    about `P3`, no `P1`) is kept — it is the *model*, not the delivery. It still drives the
    `P2`-centred rigid-frame path used by the external pin. Its output is no longer
    replayed into the walking path.
- **Verification:** acceptance is the relaxed bar — no weird behaviour, no physics
  explosion, hip advancing in +x, and a readable walk (swing foot lifting clear of the
  ground and travelling from behind the hip to ahead of it).

## 6. Gait clock — contact-gated authority (additive, not a replacement)

- **Current:** the resolver's per-leg `Stance/Swing` clock is a pure config-timer — `Stance`
  holds the foot at `plant_pos` for `stance_duration_ts`, `Swing` flies for
  `swing_duration_ts` ([`step_leg`](../examples/game_lib/src/lower_body/resolver.rs:1021)).
  It is a kinematic pose generator: it *assumes* the foot is planted whenever the timer
  says Stance, and never checks whether contact is real. Loops are not gated by real
  contacts. Delivery is already angular (leg PD targets + `P1_P2_P3` joint).
- **Change (additive — keep the resolver's `Stance/Swing` as the pose state machine):** add
  a foot-contact event layer that **gates loop authority** to real ground truth and
  **corrects drift** when the physics disagrees with the timer:
  - **Loop 2 thrust only while the foot is genuinely in contact** (there must be a ground
    reaction to push against) — a timer that says Stance while the foot is airborne makes
    loop 2 extend a leg against nothing.
  - **Loop 1 foot placement only during actual swing**; **loop 3 only during actual
    contact** (ungated continuous loops interfere).
    > **Corrected by Phase E (§5).** This line originally said loop 3 should run "mainly
    > during actual flight". Phase E established the opposite: with the waist demoted to a
    > posture hold, loop 3's only actuator is the leg PD's upright reference, and the legs
    > have authority **only through ground contact**. In flight there is nothing to push
    > against, so the default is `lean = 0` (ballistic) and the loop is gated on
    > `grounded` — see
    > [`drive_hip_pin`](../examples/game_lib/src/lower_body/hip_pin.rs:935). Phase F should
    > keep that polarity: refine *which* contact (e.g. require both feet planted, or a
    > minimum normal impulse) rather than moving loop 3 into flight.
  - **Correct the timer:** a leg whose foot is still sliding gets no thrust authority; a
    leg that left the ground early exits Stance on lift-off, not on `stance_duration_ts`;
    a long flight (from loop 2's vertical control) extends the swing until the foot really
    touches. This is the fix for the stance slide — the resolver currently *asks* for a
    pose the physics doesn't deliver, and nothing measures the difference.
  - Gating stays angular-only — each phase changes *which angular-constraint target* is
    active (stance thrust = knee/ankle extension, flight = retract + attitude, swing =
    foot placement), never a particle pin. `hip_pin.rs` (or a contact system) +
    `resolver.rs`.

### 6.1 What ground truth already exists (so Phase F does not rebuild it)

**Contact events already reach the controller.** `GroundSupport` is refreshed every
FixedUpdate tick from the previous tick's `CollisionEventBatch`
([`update_ground_support`](../examples/game_lib/src/lower_body/hip_pin.rs:189)), and
that batch is real GPU narrow-phase output containing, per colliding pair:

- `entity_a` / `entity_b` — so a contact can be attributed to a **specific foot**
  collider (`PRLL` left / `RLL` right), not just to "the character";
- `collision_point_a` / `collision_point_b` — the physical contact point;
- `collision_normal_a` / `collision_normal_b` — the contact normal;
- `velocity_a` / `velocity_b`, `inv_mass_a` / `inv_mass_b` — a physics snapshot that
  allows a normal-impulse estimate.

Source: [`VelloCollisionEvent`](../src/collision/mod.rs:324) and
[`CollisionEventBatch`](../src/collision/mod.rs:176). Reference consumer:
[`make_collision_constraints`](../src/integrations/physics/systems.rs:155). Ordering is
already correct in every app that runs the pin:
`update_ground_support.before(drive_hip_pin)` and
`measure_body.after(update_ground_support).before(drive_hip_pin)`.

**So Phase F is not "add contact detection" — it is "raise the resolution from a global
bool to per-foot, per-tick ground truth".** What `GroundSupport` currently loses:

| lost | why it hurts Phase F |
| --- | --- |
| **which** collider touched | It counts *any* soft-body vs non-soft-body pair with `\|normal.y\| > 0.5`. A foot, a shin, a hand or the head pressing on the floor all count. Loop 2 could be granted thrust authority from a knee contact. |
| **contact age / persistence** | `grounded = contacts > 0` with no hysteresis, so it chatters at lift-off and touchdown. A single-tick flicker is enough to arm or disarm the gait authority. |
| **penetration / impulse magnitude** | Contact without load still counts as grounded, so "touching but weightless" (about to lift off) is indistinguishable from "loaded and pushing". Loop 2 wants the latter. "Still sliding" (the §6 slide problem) genuinely needs this. |
| **foot-specific anchor position** | `collision_point_a/b` is discarded, yet it is the natural anchor for judging whether a stance foot has drifted, and it is what makes a contact-gated clock possible without a particle pin. |

Known caveats to respect in Phase F:

- **The flag is one tick stale.** The batch is written by `run_gpu_collision` at the end
  of FixedUpdate and read at the start of the next tick as the *previous* tick's truth.
  That is fine for gating a clock, but any per-tick contact age counter must be advanced
  with that offset in mind.
- **`CollisionEventBatch::default()` is empty, and `GroundSupport` treats a missing batch
  as `grounded = true`.** Any new per-foot layer must keep that soft fallback, or the
  headless pure-kinematic harnesses and the Raibert sandbox will silently report
  "airborne" and gate every loop off.
- `\|normal.y\| > 0.5` is ~60° from horizontal. It admits steeply-angled wall contacts
  that are not support; a per-foot version should require the normal to point *up*
  (sign, not just magnitude) and probably tighten the cone.

Suggested shape (Phase F decision, not yet implemented): replace the `grounded: bool` /
`contacts: usize` pair with a per-foot struct keyed by the `PRLL`/`RLL` collider entity —
`{ touching: bool, ticks_in_contact: u32, normal: Vec2, point: Vec2, normal_speed: f32 }`
— and keep `grounded` as a derived convenience field so the Phase-E loop-3 gate and the
existing `support_share` gravity logic keep working unchanged.

## 7. Body design — P3-heavy hip, light upper body, modest waist (asset)

**Design rule: make P3 (the hip) the heavy particle and keep the rest light.** The
resolver runs at `hip_pos`; the capture-point and neutral-point theories are written in
terms of hip = CoM. With the mass concentrated at P3, the measured CoM sits essentially
at the hip, so loops 1–2 can measure the hip directly and the CoM≈hip divergence the
user worried about disappears by construction. The upper body is a *rigid stick* the
spine PD holds upright — it carries almost no load-bearing mass, so it should be light.

- **Current:** P3 = 0.05 (weight 20, already heavy); P0/P1/P2 = 1.0 (weight 1);
  knees = 0.2 (weight 5); arms/feet = 1.0.
- **Change (mostly a *tuning* refinement, not a rebuild):**
  - **P3 (hip) stays heavy** — the single dominant mass. This is what makes a
    knee/ankle extension (loop 2) move the CoM, and makes the resolver's hip-based
    balance measurement exact (the capture point then needs no CoM correction).
  - **P2 (waist) a bit heavier than P1/P0** — gives the `P1_P2_P3` joint (pivot at
    P2) enough inertia contrast to rotate the light upper stick about the heavy hip
    when airborne (loop 3 attitude authority).
  - **P0/P1 (head-bottom / chest) light** — the spine PD's vertical-hold constraint
    is more than enough to keep a light stick upright; low inertia means its own
    motion disturbs balance less.
  - **Arms low but nonzero** (pivot on P1) — arm mass must stay well below the
    spine's hold authority so a light P1 is stable; nonzero so the arm PD has
    something to act on.
  - Spine angular constraints stiff + critically damped (zeta ≈ 1); rigid bones
    near-zero compliance. `v10.character.json`.

---

## Not changing (already aligned)

- Actuation is already angular-only (no external pin in normal walking; pin is
  air-control assist + telemetry). Ground is the only external force — matches the doc.

---

## 0. Concepts reference — capture point, Raibert neutral point, hip vs CoM

### Raibert neutral point (the heuristic)

Raibert's original hopper (1986) placed the foot at a "neutral point" ahead of the
hip proportional to horizontal speed, plus a velocity correction:

```
x_foot = x_hip + (v · Ts)/2 + kv · (v − v_desired)
```

- `(v · Ts)/2` — the **forward offset**: where the foot lands if the body just coasts
  at speed `v` for the stance duration. This is the "neutral" point that holds speed.
- `kv · (v − v_desired)` — **feedback correction**: faster than desired → step further
  ahead; slower → step closer/behind. Regulates speed toward `v_desired`.

Measured at the **hip**. That works only because Raibert's robot hip ≈ CoM by
construction (point mass on a massless leg). It is a **tuned heuristic** — the forward
offset was hand-derived from observing how far the hopper needed to step to hold speed.
It has **no notion of falling**: it regulates speed, and balance is an *emergent* side
effect ("steps land in roughly the right place"). If the target lands where the leg
can't reach, it commands it anyway and the character collapses silently.

### Capture point (the LIPM exact solution)

Derived from the **Linear Inverted Pendulum Model**: CoM at constant height `z` on a
massless leg. The dynamics are linear with natural frequency `ω₀ = √(g/z)`, and the
exact solution to "where must the foot go so the CoM comes to rest directly above it" is:

```
x_capture = x_CoM + v_CoM / ω₀
```

Formalized by Kajita (2001, "extrapolated CoM") and Pratt et al. (2006, "capture
point"). Placing the foot at `x_capture` brings the CoM to zero velocity directly
above the contact in exactly one step — the **boundary between capturability and
falling**, computed, not tuned.

The two formulas look near-identical (`x + v·√(h/g)` vs `x + v/ω₀`) — because Raibert's
neutral point was an approximation of exactly this quantity. The upgrade is not the
formula; it is everything that comes with the derivation:

| | Raibert neutral point | Capture point (LIPM) |
|---|---|---|
| Origin | tuned heuristic from observation | exact solution of the template dynamics |
| Reference point | hip (assumes hip ≈ CoM) | true whole-body CoM (measured) |
| Pendulum length | fixed assumed h | measured `z_CoM` — adapts to crouch, lean, thrust |
| Meaning of result | "step roughly here to hold speed" | "step exactly here to stop the fall" |
| Failure awareness | none — steps saturate silently | reachability of `x_capture` IS the balance verdict |
| Speed regulation | explicit `kv` on velocity error | implicit (desired CoM velocity enters via the target) |

### Hip vs CoM — and how they relate

- **Hip** = the joint where the legs attach to the body — a physical particle (`P3` in
  this codebase). A single point you can read directly.
- **CoM** = the mass-weighted average of ALL particle positions:
  `CoM = Σ mᵢ·pᵢ / Σ mᵢ` — a property of the whole body, including torso lean, arms,
  head, and any carried props.

Why it matters: Raibert could use the hip because his hip ≈ CoM by construction. On our
character, **torso lean shifts the CoM relative to the hip exactly when balance
matters** — measure at the hip and loop 1 steps for a mass distribution that no longer
exists. The CoM always works, costs a few lines, and stays correct if the body changes.

The three loops decompose 2D template dynamics around the CoM:

- **Loops 1–2 = translation of the CoM** (x, z). The capture point uses `CoM_x`,
  `v_CoM`, `z_CoM`.
- **Loop 3 = rotation about the CoM** (pitch) — but attitude uses the *measured torso angle*
  `theta` (hip→shoulder vector), a different object than the CoM.
- The CoM is momentum bookkeeping (Koenig decomposition), **not a joint** — actuation
  happens at the hip/knee/ankle; effects are referenced to the CoM.

### Does CoM–hip x-divergence still threaten the capture point?

**No for the formula, yes for two secondary channels.** The capture point is written in
terms of the whole-body CoM, so it is exact for *any* CoM–hip offset — the foot is
placed relative to the CoM wherever it is. That is the entire upgrade over the hip-based
Raibert neutral point (which genuinely broke under divergence). But divergence still
matters through two secondary effects that the balance law alone does not cover:

1. **Reachability margin.** The leg is anchored at the hip while the target is
   CoM-relative, so the required leg extension is `x_foot − x_hip =
   (x_CoM − x_hip) + (v_CoM − v_des)/ω₀`. A large CoM–hip offset adds directly into the
   reachable envelope and shrinks the margin before the reachability clamp fires
   (clamping = "in trouble", §2). It is a margin reduction, not a correctness failure.
2. **Attitude torque load.** Ground reaction at the foot has a moment arm to the CoM
   when the CoM is displaced from the hip, so supporting the CoM requires *sustained
   torque*, not just a vertical reaction. That is rotation — the capture point
   (translation law) does not cover it; loop 3 does, and a large offset can saturate it
   (the limit case is the angular-momentum tripwire, design §5.7).

Net: the P3-heavy budget (§7, CoM ≈ hip) and the whole-body CoM measurement (§1) are
**complementary, not redundant** — the measurement makes the balance law exact; the
budget keeps the two secondary couplings (reach margin, attitude load) small.

### Why the capture point looks simpler — and where the desired velocity goes

**The raw capture point is a balance law, not a speed law.** It answers one question:
"where must the foot go so the CoM comes to rest above it?" That is *stopping the fall*.
It deliberately has **no `v_desired`** because it isn't trying to move anywhere — speed
control is a separate job layered on top.

**It looks simpler because LIPM is linear.** The inverted pendulum at constant height
`z` has one natural frequency `ω₀ = √(g/z)`, so the exact answer is a one-line closed
form `x + v/ω₀`. Raibert's formula was doing **two jobs with two tuned gains** (the
forward-coast offset `v·Ts/2` and the speed correction `kv·(v−v_des)`); the capture
point does **one job exactly**.

**Where `v_desired` enters (implicit speed regulation):** to cruise at `v_des` instead
of stopping, you shift the placement target by the desired velocity:

```
x_foot = x_CoM + (v_CoM − v_des) / ω₀
```

Same shape as Raibert (`hip + (v·Ts)/2 + kv·(v−v_des)`), but the derivation shows the
forward-coast constant is not the stance time — it is the **pendulum time constant
`1/ω₀ = √(z/g)`**, and the same constant serves as the velocity gain. Two heuristics
collapse into one measured quantity (`z_CoM`). That is why "desired CoM velocity enters
via the target" (design §5.3) — it is a target shift, not a separate gain term.
