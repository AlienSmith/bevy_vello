//! Hip-pin bridge: drive the character's rigid `P1/P2/P3` frame from the
//! Raibert lower-body resolver.
//!
//! Two realization modes are available ([`PinMode`]):
//!
//! * [`PinMode::Rigid`] (legacy, default) — an **absolute** pose projection with
//!   a PD command and a `+g·dt²` feed-forward. The frame cannot be pushed off
//!   the plan; this is the kinematic behaviour the delta mode replaces.
//! * [`PinMode::Delta`] — a **state-relative velocity servo**. The target is the
//!   free-flight prediction `p* = p + v·dt + g·dt²` plus `k_v·(v_ref − v)·dt`.
//!   There is no absolute pose term and no gravity feed-forward, so collisions,
//!   contacts and gravity move the frame and the velocity error decays over
//!   `~1/k_v` ticks. Requires `SUBSTEPS=1` while the command is a frame-level
//!   position constraint.
//!
//! Under the rigid mode the resolver's hip output — **position and velocity** —
//! is the model; the pin realizes it on the three pinned particles with a
//! **PD command + a stiff external position constraint**:
//!
//! * the resolver is stepped with the live fixed `dt` and the game input map
//!   ([`PlayerAction`]),
//! * its hip is mapped into the character's **Vello (y-down) world frame**,
//!   anchored to the live `P3` captured once when the pin first arms,
//! * the `P1`/`P2`/`P3` *pose* targets are derived from the captured rest
//!   offsets by a **pure rigid transform** (translation + the `theta` attitude),
//! * each particle's *command* target is that pose advanced by a PD
//!   acceleration: `a = Kp·(pose − x) + Kd·(v_hip − v)`, integrated to a
//!   position offset `target = pose + a·dt²`. The velocity term uses the
//!   resolver's hip velocity as feed-forward, so the pin tracks a moving hip
//!   without lag ([`pd_target`]),
//! * the constraint's stiffness `1/compliance` is **softened in speed space**
//!   — `compliance_eff = compliance·(1 + s·(soft_scale−1))`,
//!   `s = min(|v|/v_ref, 1)` — so a fast frame yields instead of fighting the
//!   motion, mirroring the limb PD's ω-scheduled compliance
//!   ([`softened_compliance`]),
//! * exactly **three** [`CharacterExternalPositionConstraintEvent`]s are
//!   emitted (`P1`, `P2`, `P3`); **nothing angular** is emitted, so the
//!   `P1_P2_P3` angular joint is left at rest and cannot fight the pin.
//!
//! The spine drive ([`crate::character::systems::tick_spine_drive`]) must be
//! switched off while this runs — the code is **retained but disabled**, not
//! removed, via [`SpineDriveEnabled`](crate::character::SpineDriveEnabled).

use bevy::{prelude::*, time::Fixed};
use bevy_vello::integrations::physics::{
    CharacterExternalPositionConstraintEvent, VelloCharacterPhysicsRoot, VelloConstraintWorld,
    VelloParticle,
};
use leafwing_input_manager::prelude::ActionState;
use vello_physics::ExternalPositionConstraintConfig;

use crate::character::{LeftLegController, RightLegController, SpineController};
use crate::input_management::actions::PlayerAction;
use crate::lower_body::resolver::{step_lower_body, LowerBodyConfig, ResolverState};

/// Runtime knobs for the hip pin. Kept as a Resource (mirroring
/// [`crate::lower_body::LowerBodyParams`]) so the tuning UI can edit it live.
#[derive(Resource)]
pub struct HipPinParams {
    /// The Raibert resolver config (thigh/calf/hip-height/speeds/gravity...).
    pub config: LowerBodyConfig,
    /// PD **position** gain of the command, in `1/s²` (`a += kp·(pose − x)`).
    /// `0` disables the P term; the pin then tracks `pose` through `compliance`
    /// alone (the original position-only behaviour). Start small and raise
    /// until the frame snaps without ringing (ω ≈ √kp).
    pub kp: f32,
    /// PD **velocity-matching** gain of the command, in `1/s`
    /// (`a += kd·(v_hip − v)`). Uses the resolver's hip velocity as the
    /// feed-forward target, so the pin tracks a moving hip without lag. `0`
    /// disables the velocity term. A good start is `kd ≈ 2√kp`.
    pub kd: f32,
    /// Speed (px/s) at which the compliance softening saturates: `s` reaches `1`
    /// at `|v| = v_ref`.
    pub v_ref: f32,
    /// Softening factor: at `s = 1` the emitted compliance is multiplied by this
    /// (larger ⇒ softer-on-fast). `1.0` disables softening (constant
    /// compliance).
    pub soft_scale: f32,
    /// How the pin builds each particle's constraint target.
    pub mode: PinMode,
    /// Velocity-servo gain used by [`PinMode::Delta`], in `0..=1`: the fraction
    /// of the reference-vs-realized velocity error removed per tick. `1.0`
    /// matches the reference velocity outright; lower values yield to external
    /// impulses. Unused by [`PinMode::Rigid`].
    pub k_v: f32,
    /// External position constraint compliance **at rest** (lower = stiffer).
    /// The stock `ExternalPositionConstraint` is position-only, so this is the
    /// base stiffness of the pin; it is scaled up (softer) with speed by
    /// [`Self::soft_scale`].
    pub compliance: f32,
    /// External position constraint damping (0..1). Legacy raw velocity damper;
    /// prefer [`Self::kd`] once tuning.
    pub damping: f32,
    /// When true (default) the pin also writes the resolver's **foot** poses into
    /// the character's [`RightLegController`]/[`LeftLegController`] targets, so
    /// the **whole** lower body (hip frame + knees + feet) is sandbox-driven. The
    /// legs are then realized each tick by
    /// [`update_character_movement`](crate::character::systems::update_character_movement),
    /// which must be ordered **after** this system. Set false to leave the legs
    /// under external control (the pin then only owns `P1/P2/P3`).
    pub hip_drives_legs: bool,
    /// Vertical offset applied to the whole pinned body and both leg targets,
    /// in px. Positive = **down** (Vello y-down for the pin, Bevy −y for the
    /// legs). This lowers the character relative to the *fixed* floor without
    /// invalidating the seed: the floor is spawned from the seed anchor, so a
    /// nonzero drop makes the foot particles sit on the floor line while the
    /// leg collider's stroke round-end margin is ignored.
    pub hip_drop: f32,
}

/// How the hip pin realizes the resolver on `P1/P2/P3`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PinMode {
    /// Legacy kinematic pin: every particle is projected toward the absolute
    /// resolver pose (`pose + a·dt²`, gravity cancelled by a `+g·dt²`
    /// feed-forward). The frame cannot be pushed off the plan.
    #[default]
    Rigid,
    /// State-relative velocity servo: the target is the free-flight prediction
    /// plus a fraction of the reference-vs-realized velocity error,
    /// `p* + k_v·(v_ref − v)·dt`. No absolute pose term, so collisions, contacts
    /// and gravity move the frame and the error decays over a few ticks.
    Delta,
}

impl Default for HipPinParams {
    fn default() -> Self {
        Self {
            config: LowerBodyConfig::default(),
            // Neutral defaults: `kp = kd = 0` reproduces the position-only pin,
            // so the PD is opt-in at tuning time.
            kp: 0.0,
            kd: 0.0,
            v_ref: 300.0,
            soft_scale: 1.0, // 1 ⇒ constant compliance (softening off)
            mode: PinMode::default(),
            // Gentle: a third of the velocity error per tick, so a collision
            // perturbation decays over ~3 ticks instead of being erased.
            k_v: 0.3,
            // Snaps the (now-light) particles to the commanded pose.
            compliance: 1e-7,
            damping: 0.01,
            // Drive the legs from the sandbox by default (the integration).
            hip_drives_legs: true,
            hip_drop: 0.0,
        }
    }
}

/// One leg's pose in the bevy y-up frame (the resolver's own frame).
#[derive(Debug, Clone, Copy)]
struct LegPoseSeed {
    hip: Vec2,
    knee: Vec2,
    foot: Vec2,
}

/// Angle-space sandbox↔leg bridge, captured once on the pin's first
/// leg-driving tick.
///
/// The earlier bridge translated the *resolver pose* into the character frame.
/// That is rigid between resolver poses, but it is not congruent with the live
/// spawn pose: the resolver stands with a bent knee (~61° at `hip_height=90`,
/// `thigh=calf=55`), while the V10 blueprint spawns nearly straight. The leg PD
/// was therefore handed a ~61° step at t=0 and the whole walk was spent
/// converging, which is the persistent foot divergence.
///
/// Instead, capture each leg's live pose at seed and transfer only the
/// resolver's **per-segment rotation deltas** (thigh and calf) onto that pose.
/// At seed every delta is zero, so the emitted targets equal the live pose
/// exactly; afterwards the resolver's swing is added without changing the
/// character's rest geometry or bone lengths.
#[derive(Debug, Clone, Copy)]
struct LegBridgeSeed {
    /// `live_hip_bevy - resolver_hip` (bevy y-up). Both legs share P3, so one
    /// offset carries the hip translation.
    hip_offset: Vec2,
    /// Live leg poses at seed: `[left, right]`.
    live: [LegPoseSeed; 2],
    /// Resolver leg poses at seed: `[left, right]`.
    resolver: [LegPoseSeed; 2],
}

/// Signed angle from `a` to `b`, in `(-π, π]`.
#[inline]
fn signed_angle(a: Vec2, b: Vec2) -> f32 {
    a.perp_dot(b).atan2(a.dot(b))
}

/// Rotate `v` by `angle` radians (bevy y-up).
#[inline]
fn rotate_vec(v: Vec2, angle: f32) -> Vec2 {
    let (s, c) = angle.sin_cos();
    Vec2::new(c * v.x - s * v.y, s * v.x + c * v.y)
}

/// Rotate the seed segment `live0` by the resolver segment's rotation between
/// `res0` (at seed) and `res1` (now). The length of `live0` is preserved, so the
/// target stays on the character's own bone length.
#[inline]
fn transfer_segment(live0: Vec2, res0: Vec2, res1: Vec2) -> Vec2 {
    if res0.length_squared() < 1e-6 || res1.length_squared() < 1e-6 {
        return live0;
    }
    rotate_vec(live0, signed_angle(res0, res1))
}

/// Per-character hip pin. Attach to the character root entity. All fields are
/// seeded lazily on the first tick the physics root reports ready.
#[derive(Component)]
pub struct HipPin {
    /// Persistent Raibert resolver state (owns `hip_pos`/`hip_vel`).
    pub state: ResolverState,
    /// `P1`/`P2`/`P3` rest offsets **relative to `P3`**, in Vello (y-down)
    /// world space. `rest_offsets[2]` (P3 relative to itself) is always zero.
    pub rest_offsets: [Vec2; 3],
    /// Resolver hip at seed time (local y-up). Only the *delta* from this is
    /// mapped into the world, so the absolute resolver frame is irrelevant.
    pub seed_hip: Vec2,
    /// Live `P3` world (Vello y-down) position at seed time — the anchor the
    /// rigid frame is pinned to.
    pub anchor_p3: Vec2,
    /// Set once [`Self::rest_offsets`]/[`Self::seed_hip`]/[`Self::anchor_p3`]
    /// have been captured from the assembled particles.
    pub ready: bool,
    /// Master enable. When false the pin emits nothing (spine drive may run).
    pub enabled: bool,
    /// Rigid-frame attitude in radians. Step 1 uses `0.0` (upright). Reserved
    /// for the later PD-attitude phase.
    pub theta: f32,
    /// Whether a jump action was held at the previous step. Turns the held
    /// level into an edge at the fixed `dt` (a render-frame `just_pressed` is
    /// dropped whenever a render frame runs zero fixed steps).
    jump_was_held: bool,
    /// Seeds for the sandbox↔leg bridge, captured once on the first tick the
    /// pin drives the legs. `None` until then (and after a reset). See
    /// `LegBridgeSeed`.
    leg_seed: Option<LegBridgeSeed>,
    /// Resolver reference for `P3` in Vello world (y-down), refreshed each tick.
    pub last_ref_p3: Vec2,
    /// The constraint target actually emitted for `P3` this tick.
    pub last_cmd_p3: Vec2,
    /// Realized vertical sag of `P3` against the resolver reference:
    /// `P3.y − last_ref_p3.y` (Vello y-down, positive = body below the plan).
    /// Near zero in [`PinMode::Rigid`]; this is the number to watch when the
    /// pin stops cancelling gravity.
    pub last_sag_y: f32,
}

impl HipPin {
    /// Vertical hip→foot distance of the live leg captured at the leg-bridge
    /// seed (Bevy y-up). This is the character's actual stance height, which is
    /// what the floor must match — not the resolver's `hip_height`, because the
    /// pose-aware bridge preserves the live leg's straight/bent configuration.
    pub fn stance_height(&self) -> Option<f32> {
        self.leg_seed
            .map(|s| s.live[0].hip.y - s.live[0].foot.y)
    }

    pub fn new(config: &LowerBodyConfig) -> Self {
        Self {
            state: ResolverState::new(config),
            rest_offsets: [Vec2::ZERO; 3],
            seed_hip: Vec2::ZERO,
            anchor_p3: Vec2::ZERO,
            ready: false,
            enabled: true,
            theta: 0.0,
            jump_was_held: false,
            leg_seed: None,
            last_ref_p3: Vec2::ZERO,
            last_cmd_p3: Vec2::ZERO,
            last_sag_y: 0.0,
        }
    }

    /// Drop the sandbox↔leg seed so the foot targets are re-captured from the
    /// live pose on the next leg-driving tick. Call when toggling
    /// [`HipPinParams::hip_drives_legs`] (so the feet snap to their current
    /// position rather than jump by the accumulated sandbox delta) or after
    /// teleporting the character.
    pub fn reseat_leg_bridge(&mut self) {
        self.leg_seed = None;
    }
}

/// Pure rigid-frame derivation: place `P1/P2/P3` by translating the anchor by
/// `world_delta` and rotating each rest offset by `theta`. No per-tick
/// re-anchoring, so the three targets are a fixed rigid body. Exposed for
/// unit testing (bone lengths must be preserved exactly).
pub fn rigid_frame_targets(
    anchor_p3: Vec2,
    rest_offsets: [Vec2; 3],
    world_delta: Vec2,
    theta: f32,
) -> [Vec2; 3] {
    let (sin, cos) = theta.sin_cos();
    rest_offsets.map(|o| {
        let rotated = Vec2::new(o.x * cos - o.y * sin, o.x * sin + o.y * cos);
        anchor_p3 + world_delta + rotated
    })
}

/// PD command target for one pinned particle: the pose target advanced by the
/// commanded acceleration `a = Kp·(pose − x) + Kd·(v_hip − v)`, integrated to a
/// position offset (`target = pose + a·dt²`). `v_hip` is the resolver's hip
/// velocity (the rigid frame translates as one, so all three particles carry
/// it). Pure; exposed for unit testing.
///
/// This is the [`PinMode::Rigid`] command: it is anchored on the **absolute**
/// pose, so the frame cannot be pushed off the resolver's plan. Use
/// [`delta_target`] when the frame should participate in physics.
pub fn pd_target(
    pose_t: Vec2,
    pos: Vec2,
    vel: Vec2,
    v_hip: Vec2,
    dt: f32,
    kp: f32,
    kd: f32,
) -> Vec2 {
    let a = kp * (pose_t - pos) + kd * (v_hip - vel);
    pose_t + a * (dt * dt)
}

/// Free-flight prediction the XPBD step will make this tick (Vello y-down):
/// `p* = p + v·dt + g·dt²`. Pure; exposed for unit testing.
pub fn predict_pos(pos: Vec2, vel: Vec2, gravity: Vec2, dt: f32) -> Vec2 {
    pos + vel * dt + gravity * (dt * dt)
}

/// State-relative velocity-servo target for [`PinMode::Delta`].
///
/// `target = p* + k_v·(v_ref − v)·dt` with `p* = p + v·dt + g·dt²`.
///
/// The target is built **from the particle's own state**, so the only thing the
/// constraint adds on top of free flight is the velocity-error correction. If a
/// collision changes `v`, next tick's prediction starts from the new velocity
/// and the error decays over `~1/k_v` ticks. There is no absolute pose term and
/// no accumulated error, so no wind-up.
///
/// Applying the resolver's raw delta instead (`p* + v_ref·dt`) would add
/// `v_ref` to the realized velocity every tick and diverge; the `− v` term is
/// what makes it a servo. `k_v` is clamped to `[0, 1]`.
pub fn delta_target(
    pos: Vec2,
    vel: Vec2,
    v_ref: Vec2,
    gravity: Vec2,
    dt: f32,
    k_v: f32,
) -> Vec2 {
    let kv = k_v.clamp(0.0, 1.0);
    predict_pos(pos, vel, gravity, dt) + (v_ref - vel) * (kv * dt)
}

/// Speed-softened compliance: `base·(1 + s·(soft_scale−1))` with
/// `s = min(speed/v_ref, 1)`. `soft_scale = 1` (or `v_ref ≤ 0`) returns `base`
/// unchanged. Pure; exposed for unit testing.
pub fn softened_compliance(base: f32, soft_scale: f32, v_ref: f32, speed: f32) -> f32 {
    let s = if v_ref > 0.0 {
        (speed / v_ref).min(1.0)
    } else {
        0.0
    };
    base * (1.0 + s * (soft_scale - 1.0))
}

/// Seed the pin from the assembled character: capture the `P3` world anchor and
/// the `P1/P2` rest offsets. No-op once `ready` or if the character is not yet
/// assembled.
fn try_seed(
    pin: &mut HipPin,
    spine: &SpineController,
    root: &VelloCharacterPhysicsRoot,
    p_q: &Query<&VelloParticle>,
) {
    if pin.ready || root.initial_frame_coordinates.is_none() {
        return;
    }
    // spine.particles == [PH, P0, P1, P2, P3]; we drive the last three.
    let p1 = match p_q.get(spine.particles[2]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    let p2 = match p_q.get(spine.particles[3]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    let p3 = match p_q.get(spine.particles[4]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    pin.rest_offsets = [p1 - p3, p2 - p3, Vec2::ZERO];
    pin.anchor_p3 = p3;
    pin.seed_hip = pin.state.hip_pos;
    pin.ready = true;
}

/// Step the Raibert resolver and emit the three stiff external position
/// constraints that pin `P1/P2/P3` to the commanded rigid frame.
///
/// Order this **before** the physics constraint world is stepped (the tuning
/// app schedules it `.before(update_constraint_world)`), mirroring how the
/// spine drive used to queue its events.
pub fn drive_hip_pin(
    time: Res<Time<Fixed>>,
    params: Res<HipPinParams>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<(&SpineController, &VelloCharacterPhysicsRoot)>,
    mut right_leg_q: Query<&mut RightLegController>,
    mut left_leg_q: Query<&mut LeftLegController>,
    p_q: Query<&VelloParticle>,
    action_q: Query<&ActionState<PlayerAction>>,
    mut world: ResMut<VelloConstraintWorld>,
    mut log_tick: Local<u64>,
) {
    let dt = time.delta_secs();

    for (entity, mut pin) in pin_q.iter_mut() {
        if !pin.enabled {
            continue;
        }
        let Ok((spine, root)) = spine_q.get(entity) else {
            continue;
        };
        try_seed(&mut pin, spine, root, &p_q);
        if !pin.ready {
            continue;
        }

        // --- Game input → resolver input (Vello y-down action map). ---
        let mut input_dir = Vec2::ZERO;
        if let Ok(action) = action_q.get(entity) {
            if action.pressed(&PlayerAction::MoveRight) {
                input_dir.x += 1.0;
            }
            if action.pressed(&PlayerAction::MoveLeft) {
                input_dir.x -= 1.0;
            }
            // Resolver convention: `input_dir.y > 0.5` is a buffered jump. It
            // must be *edge-triggered*: the resolver re-enters `Stand` while the
            // key may still be held, so a held-level arm re-fires the hop on
            // every landing (the double-hop). Derive the rising edge from the
            // held level here at the fixed `dt` rather than via the action's
            // render-frame `just_pressed`, which a high frame rate drops.
            let jump_held = action.pressed(&PlayerAction::MoveUp);
            if jump_held && !pin.jump_was_held {
                input_dir.y += 1.0;
            }
            pin.jump_was_held = jump_held;
            if action.pressed(&PlayerAction::MoveDown) {
                input_dir.y -= 1.0;
            }
        }

        // --- Step the resolver (deterministic, fixed-cadence). ---
        let (pose, _debug) = step_lower_body(input_dir, &params.config, &mut pin.state, dt);

        // --- Map resolver hip (y-up) → Vello world (y-down) translation. ---
        // Only the delta from the seed hip is used, so the resolver's absolute
        // frame never leaks into the world. The resolver's hip velocity is the
        // PD feed-forward target (all three frame particles translate as one).
        let world_delta = Vec2::new(pose.hip.x - pin.seed_hip.x, -(pose.hip.y - pin.seed_hip.y));
        let v_hip = Vec2::new(pin.state.hip_vel.x, -pin.state.hip_vel.y);

        // --- Rigid pose (attitude theta; 0 = upright). ---
        let mut pose_targets =
            rigid_frame_targets(pin.anchor_p3, pin.rest_offsets, world_delta, pin.theta);
        if params.hip_drop != 0.0 {
            // Vello y-down: +drop lowers the pinned frame relative to the floor.
            for t in pose_targets.iter_mut() {
                t.y += params.hip_drop;
            }
        }

        // --- Gravity ownership on the pinned frame. ---
        // The XPBD step applies world gravity to P1/P2/P3 *before* this
        // constraint solves (and `inv_mass` is now 1.0, so that term is no
        // longer negligible). The resolver already integrates the same gravity
        // into `pose`, so the pinned particle would receive `g` twice. Feed the
        // same `g·dt²` forward into the target to hand gravity ownership to the
        // sandbox; the constraint then cancels the physics gravity exactly and
        // no `-g` bias is left on the frame. This is inert at zero gravity.
        let g_vello = world.gravity();
        let g_dt2 = g_vello * (dt * dt);

        // --- Per-particle command + speed-softened compliance. ---
        // Exactly 3 position constraints, nothing angular.
        for (i, pose_t) in [2usize, 3, 4].into_iter().zip(pose_targets.iter()) {
            let (pos, vel) = match p_q.get(spine.particles[i]) {
                Ok(p) => (p.particle.pos, p.particle.velocity),
                Err(_) => continue,
            };
            let target = match params.mode {
                // Legacy kinematic pin: absolute pose + cancelled gravity.
                PinMode::Rigid => {
                    pd_target(*pose_t, pos, vel, v_hip, dt, params.kp, params.kd) + g_dt2
                }
                // State-relative servo: free-flight prediction plus a fraction of
                // the velocity error. Gravity is already in `predict_pos`, so no
                // `g_dt2` here; the resolver pose enters only through `v_hip`.
                PinMode::Delta => delta_target(pos, vel, v_hip, g_vello, dt, params.k_v),
            };
            if i == 4 {
                pin.last_ref_p3 = *pose_t;
                pin.last_cmd_p3 = target;
                pin.last_sag_y = pos.y - pose_t.y;
            }
            let compliance = softened_compliance(
                params.compliance,
                params.soft_scale,
                params.v_ref,
                vel.length(),
            );

            world.queue_character_one_time_external_position_constraint(
                &CharacterExternalPositionConstraintEvent {
                    character_entity: entity,
                    joint_entity: spine.particles[i],
                    config: ExternalPositionConstraintConfig {
                        target,
                        compliance,
                        damping: params.damping,
                    },
                },
            );
        }

        // --- Pin telemetry (opt-in via `PIN_LOG=1`). ---
        // `sag_y` is the vertical gap between the realized `P3` and the resolver
        // reference; it is ~0 for the rigid pin and grows if the frame falls
        // because the legs/contacts cannot support it.
        let pin_log = std::env::var("PIN_LOG")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);
        if pin_log {
            *log_tick += 1;
            if *log_tick % 90 == 0 {
                let p3 = p_q
                    .get(spine.particles[4])
                    .map(|p| p.particle.pos)
                    .unwrap_or(pin.last_cmd_p3);
                info!(
                    "pin mode={:?} kv={:.2} P3 actual=({:.1},{:.1}) ref=({:.1},{:.1}) cmd=({:.1},{:.1}) sag_y={:.1}",
                    params.mode,
                    params.k_v,
                    p3.x,
                    p3.y,
                    pin.last_ref_p3.x,
                    pin.last_ref_p3.y,
                    pin.last_cmd_p3.x,
                    pin.last_cmd_p3.y,
                    pin.last_sag_y,
                );
            }
        }

        // --- Sandbox → legs: write the resolver foot poses into the leg
        // controllers' IK targets, so the whole lower body (hip frame + knees +
        // feet) follows the sandbox. `update_character_movement` (ordered after
        // this system) reads these targets and runs the leg IK. The seed is
        // captured once — both the resolver feet and the bevy-yup targets — and
        // only the per-tick *delta* is applied, so the resolver's absolute frame
        // never has to match the character's spawn y. A character without legs
        // has no controller and is simply skipped.
        if params.hip_drives_legs {
            let (Ok(mut right_leg), Ok(mut left_leg)) =
                (right_leg_q.get_mut(entity), left_leg_q.get_mut(entity))
            else {
                continue;
            };
            // P2/P3 are owned by the pin for as long as the legs are driven.
            // The leg PD uses this to drop them from the angular mass matrix.
            right_leg.config.anchor_chain = true;
            left_leg.config.anchor_chain = true;
            // Capture the LIVE leg pose (vello y-down → bevy y-up). The old
            // bridge only captured the hip offset and then translated the
            // resolver triangle, which is rigid but not congruent with the
            // character's spawn pose: the resolver stands with a ~61° knee bend
            // while the V10 blueprint spawns nearly straight. That handed the
            // leg PD a ~61° step at t=0 and the walk ran entirely inside the
            // catch-up transient. Capture the whole pose instead, so the
            // resolver's *rotation deltas* can be layered on top of it.
            let vello_to_bevy = |p: &VelloParticle| Vec2::new(p.particle.pos.x, -p.particle.pos.y);
            let live_pose = |particles: [Entity; 4],
                             hip_fb: Vec2,
                             knee_fb: Vec2,
                             foot_fb: Vec2|
             -> LegPoseSeed {
                let get = |e: Entity, fallback: Vec2| {
                    p_q.get(e)
                        .map(&vello_to_bevy)
                        .unwrap_or(fallback)
                };
                LegPoseSeed {
                    hip: get(particles[1], hip_fb),
                    knee: get(particles[2], knee_fb),
                    foot: get(particles[3], foot_fb),
                }
            };
            let live_left = live_pose(
                left_leg.particles,
                left_leg.hip_target,
                left_leg.knee_target,
                left_leg.target,
            );
            let live_right = live_pose(
                right_leg.particles,
                right_leg.hip_target,
                right_leg.knee_target,
                right_leg.target,
            );
            let seed = *pin.leg_seed.get_or_insert(LegBridgeSeed {
                hip_offset: live_left.hip - pose.hip,
                live: [live_left, live_right],
                resolver: [
                    LegPoseSeed {
                        hip: pose.hip,
                        knee: pose.left_knee,
                        foot: pose.left_foot,
                    },
                    LegPoseSeed {
                        hip: pose.hip,
                        knee: pose.right_knee,
                        foot: pose.right_foot,
                    },
                ],
            });

            // Pose-aware target: keep the character's own thigh/calf lengths and
            // add only the resolver's per-segment rotation since the seed. At
            // seed both deltas are zero, so the targets equal the live pose and
            // the PD starts with no error. The hip still carries the resolver's
            // world translation.
            // Bevy y-up: subtract the same drop so the leg targets follow the
            // lowered pinned frame (the floor stays at the seed anchor).
            let hip_t = pose.hip + seed.hip_offset - Vec2::Y * params.hip_drop;
            let knee_l = hip_t
                + transfer_segment(
                    seed.live[0].knee - seed.live[0].hip,
                    seed.resolver[0].knee - seed.resolver[0].hip,
                    pose.left_knee - pose.hip,
                );
            let foot_l = knee_l
                + transfer_segment(
                    seed.live[0].foot - seed.live[0].knee,
                    seed.resolver[0].foot - seed.resolver[0].knee,
                    pose.left_foot - pose.left_knee,
                );
            let knee_r = hip_t
                + transfer_segment(
                    seed.live[1].knee - seed.live[1].hip,
                    seed.resolver[1].knee - seed.resolver[1].hip,
                    pose.right_knee - pose.hip,
                );
            let foot_r = knee_r
                + transfer_segment(
                    seed.live[1].foot - seed.live[1].knee,
                    seed.resolver[1].foot - seed.resolver[1].knee,
                    pose.right_foot - pose.right_knee,
                );

            left_leg.hip_target = hip_t;
            right_leg.hip_target = hip_t;
            left_leg.knee_target = knee_l;
            right_leg.knee_target = knee_r;
            left_leg.target = foot_l;
            right_leg.target = foot_r;
            // Feed the resolver's per-frame joint velocities (bevy y-up, px/s)
            // into the leg PD so the angular constraint's velocity projection
            // can drive each joint's rate toward the sandbox reference
            // (`c_dot_desired` in `calculate_leg_pd`). Same y-up mapping as the
            // foot-position deltas above: the resolver frame is bevy y-up, and
            // only the pin's Vello-y-down world mapping flips y.
            left_leg.hip_vel = pose.hip_vel;
            left_leg.knee_vel = pose.left_knee_vel;
            left_leg.target_vel = pose.left_foot_vel;
            right_leg.hip_vel = pose.hip_vel;
            right_leg.knee_vel = pose.right_knee_vel;
            right_leg.target_vel = pose.right_foot_vel;
        } else {
            // No pin → the frame is free again; let the leg PD use the full
            // three-particle mass matrix.
            if let Ok(mut right_leg) = right_leg_q.get_mut(entity) {
                right_leg.config.anchor_chain = false;
            }
            if let Ok(mut left_leg) = left_leg_q.get_mut(entity) {
                left_leg.config.anchor_chain = false;
            }
        }
    }
}

/// Reset the pin to its resting symmetric stance and re-arm the seed (press `R`).
pub fn reset_hip_pin(
    keys: Res<ButtonInput<KeyCode>>,
    params: Res<HipPinParams>,
    mut pin_q: Query<&mut HipPin>,
) {
    if !keys.just_pressed(KeyCode::KeyR) {
        return;
    }
    for mut pin in pin_q.iter_mut() {
        pin.state = ResolverState::new(&params.config);
        pin.ready = false;
        pin.leg_seed = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rigid_frame_preserves_bone_lengths() {
        // y-down Vello offsets: P1 above-right of P3, P2 near-above of P3.
        let rest = [Vec2::new(12.0, -60.0), Vec2::new(-3.0, -30.0), Vec2::ZERO];
        let anchor = Vec2::new(347.0, 346.0);
        let delta = Vec2::new(55.0, -18.0);

        for theta in [0.0_f32, 0.4, -1.1] {
            let t = rigid_frame_targets(anchor, rest, delta, theta);
            // P1–P3 and P2–P3 lengths must match the rest offsets exactly.
            assert!((t[0] - t[2]).length() - rest[0].length() < 1e-3);
            assert!((t[1] - t[2]).length() - rest[1].length() < 1e-3);
            // P3 is the anchor translated by the world delta.
            assert!((t[2] - (anchor + delta)).length() < 1e-3);
        }
    }

    #[test]
    fn rigid_frame_theta_zero_is_pure_translation() {
        let rest = [Vec2::new(12.0, -60.0), Vec2::new(-3.0, -30.0), Vec2::ZERO];
        let anchor = Vec2::new(10.0, 20.0);
        let delta = Vec2::new(4.0, 5.0);
        let t = rigid_frame_targets(anchor, rest, delta, 0.0);
        for k in 0..3 {
            assert!((t[k] - (anchor + delta + rest[k])).length() < 1e-4);
        }
    }

    /// End-to-end gate for the *mapping* (resolver → rigid P1/P2/P3 frame):
    /// run the real resolver for a walking episode across several fixed `dt`s
    /// (including an extreme 0.2 s step) and assert the derived frame (a) stays
    /// finite, (b) preserves both bone lengths exactly, and (c) puts P3 at the
    /// anchor translated by the resolver's hip delta. Deterministic and
    /// CI-runnable; the visual tracking gate (measured particles → targets under
    /// the XPBD solver) stays a user-run check in the tuning app, per plan §8.
    #[test]
    fn rigid_frame_tracks_resolver_across_dts() {
        let cfg = LowerBodyConfig::default();
        let rest = [Vec2::new(12.0, -60.0), Vec2::new(-3.0, -30.0), Vec2::ZERO];
        let anchor = Vec2::new(347.0, 346.0);
        let l1 = rest[0].length();
        let l2 = rest[1].length();

        for dt in [1.0_f32 / 90.0, 1.0 / 15.0, 0.2] {
            let mut state = ResolverState::new(&cfg);
            let seed = state.hip_pos;
            for step in 0..240 {
                // Hold "move right" — the resolver walks (bounded). No jump: the
                // vertical loop diverges at a coarse dt, which would blow f32
                // precision and mask the mapping under test.
                let input = Vec2::new(1.0, 0.0);
                let (pose, _debug) = step_lower_body(input, &cfg, &mut state, dt);
                let delta = Vec2::new(pose.hip.x - seed.x, -(pose.hip.y - seed.y));
                let t = rigid_frame_targets(anchor, rest, delta, 0.0);

                assert!(
                    t.iter().all(|p| p.x.is_finite() && p.y.is_finite()),
                    "non-finite frame at step {step}, dt {dt}"
                );
                let d1 = (t[0] - t[2]).length();
                let d2 = (t[1] - t[2]).length();
                assert!(
                    (d1 - l1).abs() < 1e-3,
                    "P1-P3 drift, dt {dt} step {step}: got {d1} want {l1} t={t:?}"
                );
                assert!(
                    (d2 - l2).abs() < 1e-3,
                    "P2-P3 drift, dt {dt} step {step}: got {d2} want {l2} t={t:?}"
                );
                assert!(
                    (t[2] - (anchor + delta)).length() < 1e-4,
                    "P3 not anchor+delta, dt {dt}"
                );
            }
        }
    }

    #[test]
    fn pd_target_is_pose_when_gains_are_zero() {
        let pose = Vec2::new(10.0, -5.0);
        let t = pd_target(
            pose,
            Vec2::new(0.0, 0.0),
            Vec2::ZERO,
            Vec2::ZERO,
            1.0 / 90.0,
            0.0,
            0.0,
        );
        assert!((t - pose).length() < 1e-6);
    }

    #[test]
    fn pd_target_leads_toward_pose_and_commanded_velocity() {
        let dt = 1.0 / 90.0;
        let pose = Vec2::new(100.0, 0.0);
        let pos = Vec2::new(90.0, 0.0); // pose ahead of pos in +x
                                        // P term alone pushes the target past `pose` (in +x).
        let tp = pd_target(pose, pos, Vec2::ZERO, Vec2::ZERO, dt, 100.0, 0.0);
        assert!(tp.x > pose.x, "P term should lead ahead of pose: {tp:?}");
        // D term: particle at rest, hip moving +x ⇒ target pushed +x.
        let td = pd_target(pose, pos, Vec2::ZERO, Vec2::new(200.0, 0.0), dt, 0.0, 10.0);
        assert!(
            td.x > pose.x,
            "D term should lead in the hip direction: {td:?}"
        );
        // D term with the particle already matching the hip ⇒ no shift.
        let tm = pd_target(
            pose,
            pos,
            Vec2::new(200.0, 0.0),
            Vec2::new(200.0, 0.0),
            dt,
            0.0,
            10.0,
        );
        assert!(
            (tm - pose).length() < 1e-6,
            "matched velocity ⇒ no shift: {tm:?}"
        );
    }

    #[test]
    fn rigid_hip_offset_preserves_bone_lengths_and_angles() {
        // A single shared translation (the seed `hip_offset`) must slide the
        // whole resolver leg triangle rigidly: every emitted hip/knee/foot
        // target is `pose + off`, so thigh/calf lengths and the joint angle
        // transfer *exactly*. This is the fix for the foot divergence (the old
        // per-joint delta bridge rebuilt the segments from independent deltas
        // and corrupted the rest angles the leg PD derives from hip/knee).
        let off = Vec2::new(120.0, -40.0); // arbitrary character-frame offset
        let hip = Vec2::new(30.0, 90.0);
        let knee = Vec2::new(45.0, 35.0);
        let foot = Vec2::new(60.0, 5.0);

        let hip_t = hip + off;
        let knee_t = knee + off;
        let foot_t = foot + off;

        let thigh = (knee - hip).length();
        let calf = (foot - knee).length();
        let thigh_t = (knee_t - hip_t).length();
        let calf_t = (foot_t - knee_t).length();

        assert!((thigh_t - thigh).abs() < 1e-5, "thigh length must be preserved");
        assert!((calf_t - calf).abs() < 1e-5, "calf length must be preserved");

        // Joint angle (hip→knee→foot) is translation-invariant.
        let angle = (knee - hip).angle_between(foot - knee);
        let angle_t = (knee_t - hip_t).angle_between(foot_t - knee_t);
        assert!((angle_t - angle).abs() < 1e-5, "joint angle must be preserved");
    }

    #[test]
    fn compliance_softens_monotonically_with_speed_and_clamps() {
        let base = 1e-7;
        let slow = softened_compliance(base, 10.0, 300.0, 0.0);
        let mid = softened_compliance(base, 10.0, 300.0, 150.0);
        let fast = softened_compliance(base, 10.0, 300.0, 300.0);
        let satur = softened_compliance(base, 10.0, 300.0, 9999.0);
        assert!((slow - base).abs() < 1e-12, "at rest ⇒ base");
        assert!(
            mid > slow && fast > mid,
            "softer as speed grows: {slow} {mid} {fast}"
        );
        assert!(
            (fast - base * 10.0).abs() < 1e-12,
            "saturated ⇒ base·soft_scale"
        );
        assert!((satur - fast).abs() < 1e-12, "clamped at v_ref");
        // soft_scale = 1 and v_ref = 0 are both no-ops.
        assert!((softened_compliance(base, 1.0, 300.0, 9999.0) - base).abs() < 1e-12);
        assert!((softened_compliance(base, 10.0, 0.0, 9999.0) - base).abs() < 1e-12);
    }

    #[test]
    fn delta_target_reduces_to_prediction_when_velocity_matches() {
        let dt = 1.0 / 90.0;
        let g = Vec2::new(0.0, 980.0);
        let p = Vec2::new(10.0, 20.0);
        let v = Vec2::new(50.0, -5.0);
        let t = delta_target(p, v, v, g, dt, 0.4);
        let pred = predict_pos(p, v, g, dt);
        assert!(
            (t - pred).length() < 1e-5,
            "matched velocity ⇒ prediction only: {t:?} vs {pred:?}"
        );
    }

    #[test]
    fn delta_target_converges_to_reference_velocity() {
        // Horizontal, gravity-free: the servo must drive `v → v_ref` and never
        // accumulate. Models the stiff constraint as `pos_{n+1} = target` and
        // `v_{n+1} = (target − pos)/dt`.
        let dt = 1.0 / 90.0;
        let v_ref = Vec2::new(120.0, 0.0);
        let mut pos = Vec2::ZERO;
        let mut vel = Vec2::ZERO;
        for _ in 0..240 {
            let target = delta_target(pos, vel, v_ref, Vec2::ZERO, dt, 0.3);
            vel = (target - pos) / dt;
            pos = target;
        }
        assert!(
            (vel - v_ref).length() < 1e-3,
            "servo did not converge: {vel:?}"
        );
    }

    #[test]
    fn raw_reference_delta_accumulates_velocity() {
        // Regression guard for the runaway: applying the resolver's raw delta
        // on top of the prediction adds `v_ref` to the realized velocity every
        // tick (`v_{n+1} = v_n + v_ref`).
        let dt = 1.0 / 90.0;
        let v_ref = Vec2::new(120.0, 0.0);
        let mut pos = Vec2::ZERO;
        let mut vel = Vec2::ZERO;
        for _ in 0..10 {
            let raw = predict_pos(pos, vel, Vec2::ZERO, dt) + v_ref * dt;
            vel = (raw - pos) / dt;
            pos = raw;
        }
        assert!(
            (vel.x - 10.0 * v_ref.x).abs() < 1e-3,
            "expected 10×v_ref after 10 ticks, got {vel:?}"
        );

        // The error form over the same 10 ticks stays bounded by `v_ref`.
        let mut pos = Vec2::ZERO;
        let mut vel = Vec2::ZERO;
        for _ in 0..10 {
            let target = delta_target(pos, vel, v_ref, Vec2::ZERO, dt, 0.3);
            vel = (target - pos) / dt;
            pos = target;
        }
        assert!(vel.x <= v_ref.x + 1e-3, "servo overshot: {vel:?}");
    }

    #[test]
    fn delta_target_clamps_kv() {
        let dt = 1.0 / 90.0;
        let p = Vec2::ZERO;
        let v = Vec2::new(10.0, 0.0);
        let v_ref = Vec2::new(110.0, 0.0);
        let pred = predict_pos(p, v, Vec2::ZERO, dt);
        let full = delta_target(p, v, v_ref, Vec2::ZERO, dt, 2.0);
        let one = delta_target(p, v, v_ref, Vec2::ZERO, dt, 1.0);
        let none = delta_target(p, v, v_ref, Vec2::ZERO, dt, -1.0);
        assert!((full - one).length() < 1e-6, "k_v clamps at 1");
        assert!((none - pred).length() < 1e-6, "k_v clamps at 0");
    }
}
