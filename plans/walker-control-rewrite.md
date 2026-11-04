# Walker control rewrite — observe / control / actuate (2026-11-03)

The loop as stated by the user, organized into **three switchable modules**:

> - **observe** — information gathering: builds **fiction**, the data object
>   representing the current physics status (CoM, velocities, hip state,
>   lean, foot contacts). Fiction is an *input* to the controller, not a
>   parallel story.
> - **control** — the algorithm: from (fiction, command/desired target,
>   gait-schedule state) produce next-frame targets. All the math — gait
>   schedule, DCM placement, vertical/attitude state machines — lives here.
>   **Swappable**: a boost-start controller drops in without touching the
>   other modules.
> - **actuate** — IK + applying results back to physics, through angular +
>   distance constraints only.
>
> Every system runs in `FixedUpdate`, fixed order, **before**
> `update_constraint_world` (the tick where the frame's physics is simulated).

## 0. Module graph

```mermaid
flowchart LR
    subgraph OBS [observe]
        O1[read particles + contact batch] --> O2[Fiction: hip pos and vel, com and v_com, lean, feet]
    end
    subgraph CTL [control - swappable]
        C1[step_lower_body in walker_math] --> C2[gait FSM + DCM + vertical + attitude]
        C2 --> C2b[next-tick hip prediction from fiction]
        C2b --> C3[ControlOut: { pose (hip+feet+facing+lean), hip_vel, mode, debug }]
    end
    subgraph ACT [actuate - IK + emission only]
        A1[solve_leg_ik from ControlOut] --> A2[angular targets: legs + spine]
        A2 --> A3[distance: bone locks + hip-foot strut]
    end
    O2 --> C1
    C3 --> A1
    A3 --> PHYS[make_collision_constraints then update_constraint_world]
    PHYS -. reality, next tick.-> O1
```

Tick order inside `FixedUpdate`:
`(sample_input_tick → observe → control → actuate).chain()` followed by the
existing physics chain `make_collision_constraints → update_constraint_world →
… → run_gpu_collision`, unchanged. Contact/collision data arrives 1 tick late —
the same clock and lag the physics chain already runs on; nothing in the loop
is frame-paced.

## 1. Interfaces (data-only boundaries)

```rust
/// The fiction: one per-tick snapshot of physics status, built by observe.
/// Pure input data. Raw physics frame (Vello y-down) — the observer reports
/// reality as-is; sign/origin mapping happens in the control stamp.
pub struct Fiction {
    pub dt: f32,
    pub hip: Vec2,             // measured hip (P3) position
    pub v_hip: Vec2,           // measured hip velocity, filtered (D2)
    pub lean: f32,             // measured attitude: hip→waist axis vs vertical
    pub lean_rate: f32,        // EMA of d(lean)/dt (filter state lives in observe)
    pub com: Vec2,             // mass-weighted whole-body CoM position
    pub v_com: Vec2,           // measured CoM velocity, filtered
    pub feet: [FootState; 2],  // measured foot positions + contacts (1-tick lag)
    pub grounded: bool,        // debounced any-support truth (observe owns hysteresis)
}

pub struct FootState { pub pos: Vec2, pub vel: Vec2, pub touching: bool }

// Consumed by control (audited against `walker_math::step_lower_body`):
//   CaptureMeasure{com_x, v_com, z_com}   ← com + floor config (`z = floor_y − com.y`)
//   RealityVertical{grounded, h_meas, vy} ← grounded, hip, v_hip (+ floor config)
//   foot_contact[i]{touching, foot_x}     ← feet[i].touching, feet[i].pos.x
//   attitude (theta_meas, omega_meas)     ← lean, lean_rate
//   hip fiction stamp                     ← hip, v_hip
//   measured_apex                         ← latched max of com height by control (schedule state)
// Not needed: raw waist/shoulder positions (lean already derived), foot
// velocities in the math (kept for telemetry + slip gates), contact
// normals/points (thrust gate retired with the pin architecture).

/// Fiction → the fields of ResolverState that describe the BODY are stamped
/// from it (hip_pos, hip_vel, measured CoM/v_com, attitude.theta, contacts).
/// This stamp is the module boundary; it happens in exactly one call.
pub trait Control {
    /// All persistent state here is schedule/reference ONLY (see Law 2):
    /// v_ref ramp, gait FSM, planted-foot plans, swing progress, boot phase.
    /// `facing` is a spawn-time constant, passed because it is one of the five
    /// pose elements control produces.
    fn step(&mut self, cmd: &Command, fiction: &Fiction, cfg: &ControlConfig,
            dt: f32, facing: f32) -> ControlOut;
}

pub struct Command { pub dir: Vec2, pub speed: f32, pub jump: bool }

/// Pose-space output. Actuate knows nothing about gait math; control knows
/// nothing about constraint APIs. **The pose is a field, not a copy** (see the
/// note below): control builds `actuate::Pose` and wraps it with the fields
/// actuate must never see.
/// PREDICTION IS CONTROL'S JOB: `pose.hip` is the one-tick-ahead hip
/// (stance: hip + v_hip*dt; flight: ballistic) computed from fiction, so the
/// leg IK targets lead reality by one tick.
pub struct ControlOut {
    pub pose: Pose,          // the actuate contract: hip + 2 feet(+stance) + facing + upper_angle
    pub hip_vel: Vec2,       // telemetry (`cmd_vx`, `tP3vx`) + prediction bookkeeping
    pub mode: LocoMode,      // diagnostics; LocoState::mode is the authority
    pub debug: Option<DebugInfo>, // CSV passthrough; None outside the gait controller
}

/// One struct for one concept (decided after auditing both versions):
/// - `actuate::Pose` owns the five pose elements,
/// - `control::ControlOut` wraps it and nothing duplicates it,
/// - `FootPose { pos, stance }` is the single foot-target type for both modules.
/// The earlier draft had `ControlOut.upper` (a torso *position* target) — it is
/// gone: loop 3 commands the stick's *orientation* as an angle through the leg
/// hip joints, so a torso point was write-only. The only torso length in the
/// system is `LowerBodyConfig::torso_len`, loop 3's pendulum parameter.

/// The actuate API STRUCTURALLY EXCLUDES position pinning.
pub trait Actuate {
    fn apply(&self, out: &ControlOut, fiction: &Fiction, world: &mut VelloConstraintWorld);
    // may call ONLY: queue_character_angular_constraints,
    //                queue_character_distance_constraints (optional strut)
    // queue_character_one_time_external_position_constraint does not appear
    // in this crate.
}
```

Why the boundaries are shaped like this:

- **Fiction is input, one snapshot per tick.** The old failure mode — fiction
  velocity open-loop while fiction position was reality-synced — cannot be
  expressed: both arrive from observe, or neither does.
- **Control's persistent state is schedule-only.** Body-state fields live in
  fiction and get overwritten at the boundary; control may integrate only
  what cannot be measured (the v_ref ramp, foot placement plans, FSM timers).
- **Actuate's surface lists no pin API.** The 42 %-ext-pin airborne mode and
  the boot P3 position-kick are not "discouraged" — they don't type-check.

## 2. What the fiction-as-input law deletes at the root

| Old mechanism | What it did | Why illegal now |
|---|---|---|
| `state.hip_pos += v·dt` integrator | fiction position self-advanced | body position is fiction = measured; stamped |
| §6 touchdown velocity solve (`v += β(v_sup − v)`) | correct fiction **velocity** at touchdown | mutates fiction mid-tick; placement (deadbeat DCM) remains the only velocity-space corrector — the double-corrector audit's finding made structural |
| `FEEDBACK` / `HIP_V_MIX` / `V_STABILIZE` / `PLAN_MEAS_V` | partial reality→fiction blends | the boundary stamp is the only, complete rule |
| `ext_pin` / `air_control` airborne position-pin mode | hip ownership by external constraint when airborne | not representable in actuate's API |
| boot-lunge P3 position-constraint kick | fake acceleration | boost-start must *actually* accelerate the body (see §5) |
| leg-PD `anchor_chain` / `upright_spine_reference` | second hip owner | one owner: angular targets from actuate |

## 3. Module contents (port vs new)

| Module | Port from existing code | New |
|---|---|---|
| observe | [`measure.rs`](../examples/game_lib/src/lower_body/measure.rs) CoM/velocity/torso-pitch math + EMA; [`ground.rs`](../examples/game_lib/src/lower_body/hip_pin/ground.rs) foot-contact read (simplified to the 1-tick batch flags); [`walker_math::CaptureMeasure`](../walker_math/src/placement.rs) construction | the `Fiction` bundle + the LPF on velocities (D2) |
| control | [`walker_math`](../walker_math/src/lib.rs) **as-is** behind the trait; delete the fiction-mutating paths from its production use (§2) rather than gate them with flags | schedule-only state discipline; boost-start variant (§5) |
| actuate | [`leg_pd.rs`](../examples/game_lib/src/character/leg_pd.rs) angular-target emission (position-target mode, `use_position_constraint`); `tick_loop3_angular` spine angular pattern; [`ik`](../walker_math/src/ik.rs) (`solve_leg_ik`, `select_anterior_knee`); Phase-2 hip↔stance-foot strut emission from `drive.rs` | nothing beyond composition: pure `ik_targets(fiction, out) -> Targets` + thin Bevy emission |

## 3b. Telemetry: the observer writes the reality export

The CSV is mostly *reality status per frame* — and reality is now observe's
product. So the export is generated from the module outputs directly
([`report_body`](../examples/collision_detection/examples/headless_arm_pd/report.rs)
currently digs into controller internals — `pin.last_external_pin`, mirrored
reality caches — and that whole mechanism is deleted):

| CSV column family | Source (new) | Source (old, deleted) |
|---|---|---|
| bare particle channels (`P0x` … `PLLLvy`, 52 cols) | unchanged — `capture_telemetry` reads the live particles | same |
| reality: `cmd_vx`, `v_filt`, `body_theta/omega`, `meas_theta` | `ControlOut.hip_vel.x`, `Fiction.v_com.x`, `Observer.raw` (pre-LPF, so the export shows the true signal the filter smooths) | `pin.state.hip_vel`, `pin.state.v_meas_filt`, `pin.state.attitude`, `pin.last_meas_theta` |
| target: `tP3*`, `tPLLL*`, `tPRLL*` | `ControlOut.pose.hip(+hip_vel)`, `ControlOut.pose.{left,right}_foot.pos` (+ finite difference) | `leg.hip_target/hip_vel`, `leg.target/target_vel` |
| target: `tP40*`, `tP41*` | `actuate::ik_knee_target` — the knee the actuation actually aims at (was frozen at spawn) | `leg.knee_target` (never updated) |
| control internals: `gait_l/r`, `att_target` | `ControlOut.pose.*_foot.stance`, `ControlOut.pose.upper_angle` | resolver gait phase, `pin.last_att_target` |
| absent features: `vphase`, `att_torque`, `hip_accel_x`, `att_enabled`, `ext_pin`, `ns_l/r` | **zeros / `false`** — no vertical phase, no torque command, no pin, no on-off forks (Laws 1 + 4) | env-driven latches |
| `att_rest_cos`/`att_rest_sin` | **1.0 / 0.0** — no rest basis is emitted; the rest is the fixed upright pose | emitted rest angle |
| appended: `mode`, `grounded` | `LocoState.mode`, `Fiction.grounded` | — (new signals with no column of their own) |

Implemented in [`walker_control/src/telemetry.rs`](../walker_control/src/telemetry.rs)
as a pure `telemetry_row(...) -> Vec<(String, String)>` in the exact
`TelemetryRow::set` order, with [`SCALAR_COLUMNS`](../walker_control/src/telemetry.rs)
as the frozen list. Appending the two columns (rather than replacing a mode
flag) keeps every pre-existing column at its committed index, so the fixture
header at [`walk20_mix07_s90.csv`](../tools/physics-qa/fixtures/walk20_mix07_s90.csv)
still lines up for the first 110 columns.

Design note: observe writes both the filtered velocities (into `Fiction`) and
the raw samples (into `TelemetryTracker`), so the row shows the true signal
the filter smooths. Columns for features the new architecture does not have
(`vphase`, `att_torque`, `hip_accel_x`, `att_enabled`, `ext_pin`, `ns_l/r`)
are **zero-filled, not removed** — the user's call, and it is the safer one:
the frozen 110-column header in
[`channels.rs`](../examples/game_lib/src/headless/channels.rs) stays
byte-identical, so `judge.rs`'s `require_rectangular`/rows-hash checks and
every committed fixture and analyzer keep working unchanged. A zero is
readable as "this feature does not exist"; a missing column would silently
shift every index after it.

## 4. Laws (each exists because we measured its violation)

1. **Single authority per DOF.** Only XPBD constraints (angular/distance/
   collision) move the body. Enforced by actuate's API shape.
2. **Fiction is measured input; controller state is schedule + reference.**
   Body-state fields of the resolver are stamped at the boundary each tick;
   nothing else writes them; control integrates nothing that observe can
   measure.
3. **Everything tick-clocked.** One `FixedUpdate` chain, no frame-paced inputs.
   This is the candidate fix for the visible-vs-hidden divergence (old loop
   fed frame-paced input/GPU batch into a fixed-tick chaotic gait; ±1 tick
   changed outcomes).
4. **Config values, not env forks.** Behavior knobs are typed config fields;
   a flag earns existence once and is deleted when proven inert.

## 5. Boost-start (first control-module swap)

Same observe/actuate, different `impl Control`: ramp `v_ref` to `v_cmd` at
`boot_accel` (config), command lean into the run via the attitude PD, and keep
the gait schedule either locked-low (feet roll) or stepping at the low end of
its envelope. The exit condition reads **fiction**: `v_hip.x ≥ threshold` or
timeout. Because fiction is measured, the boot cannot fake velocity — success
means the body actually accelerated, which was exactly the thing
`BOOT_LUNGE` papered over. The feasibility claim (boot accel inside the stride
map: x_max ≈ 54 px, a_max ≈ 685 px/s² at S90) is now testable per-tick with
real numbers.

## 6. Decisions to confirm

- **D1 — determinism strategy.** Build the single-clock loop first; gate on
  hidden ×2 byte-identical, visible ×2 byte-identical, visible ≡ hidden (old
  chain diverged at tick 253). Only if it still diverges: per-tick readback
  block, or CPU narrow-phase for the ~4 character↔ground pairs.
- **D2 — fiction velocity filter.** Measured hip vx is noisy (std 52, p2p 300,
  42 % above 6 Hz from foot collision impulses). Propose: fiction velocities =
  first-order LPF, one config time constant (start ≈ 20 ms); positions raw.
  Filter lives in observe — one knob, one place.
- **D3 — schedule-only state audit.** Walk `ResolverState` field-by-field and
  classify each as fiction-stamp vs schedule-reference vs plan; anything in
  the first class is stamped, second/third kept. (This decides the fate of
  fields like `v_desired_scalar` reference vs `hip_vel`.)
- **D4 — home.** New crate `walker_control` (workspace member; deps:
  `walker_math`, `bevy`, `bevy_vello`, `vello_physics`). Old `lower_body/`
  keeps compiling until the new bridge passes §7, then one delete-pass.

## 7. Validation protocol (tooling unchanged)

Headless `visual_arm_pd` gains a v2 path (or thin second example) using
`walker_control`; same CSV schema, same `run_walk.sh` env discipline,
`long_walk20_s90` first. Gates:

1. determinism per D1;
2. tracking — realized hip vx within ±10 px/s of the fiction target over 2–8 s
   (the flat-90 that TD_SOLVE=0 proved reachable);
3. gait quality — swing arc passes, slip ≤ 20 px/stance (old: 46–55),
   air% < 10 while walking flat;
4. boot — measured (fiction) speed reaches 90 px/s inside the boot window with
   no hip-height spike (the "jumped too high" artifact must not appear);
5. review with `walk_bands.py` / `plot_p3.py` / `pose_viewer.py`.

## 8. Build order

1. `walker_control` skeleton: `Fiction`/`ControlOut`/traits + module seams,
   unit-tested against hand-built fictions (no App).
2. Wiring: the chain before the physics chain; tick-clocked input sampler.
3. Headless bring-up: control module vs the kinematic walker_math sandbox
   should track within constraint lag; then closed-loop.
4. Run §7 gates; tune `LowerBodyConfig` values only.
5. Boost-start variant as a pure control-module addition + A/B.
6. Delete pass on `lower_body/hip_pin` + systems/plugin/measure + consumers.
