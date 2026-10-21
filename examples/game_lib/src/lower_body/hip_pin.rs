//! Hip pin: step the Raibert resolver and advance `P1/P2/P3` by the hip's
//! per-tick displacement with three external position constraints.
//!
//! The target is the live particle position plus the resolver hip delta, a
//! centroid velocity servo toward the resolver hip velocity, and the
//! `support_share` fraction of gravity. There is no absolute pose term.
//!
//! The loop is closed: at the top of each step the solved `P3` (synced into
//! [`VelloParticle`] by `update_connection_particles` at the end of the
//! previous fixed step) is written back into the resolver state, so the next
//! step starts from where physics actually left the body. While the `P3` pin
//! is stiff this is a near-identity correction; it becomes the real feedback
//! path once the hard pin is later removed. `FEEDBACK=0` disables it for A/B
//! against the feed-forward baseline.
//!
//! The spine drive must stay disabled while the pin runs
//! ([`crate::character::SpineDriveEnabled`]).

use bevy::{prelude::*, time::Fixed};
use bevy_vello::collision::CollisionEventBatch;
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, CharacterExternalPositionConstraintEvent,
    VelloCharacterPhysicsRoot, VelloConstraintWorld, VelloParticle,
};
use bevy_vello::VelloCollider;
use leafwing_input_manager::prelude::ActionState;
use vello_physics::{AngularConstraintConfig, ExternalPositionConstraintConfig};

use crate::character::{LeftLegController, RightLegController, SpineController};
use crate::input_management::actions::PlayerAction;
use crate::lower_body::measure::{
    update_measurements, BodyMeasureFilter, BodyMeasurements, ParticleSample,
};
use crate::lower_body::resolver::{
    attitude_lean_pd, step_lower_body, CaptureMeasure, GaitPhase, LowerBodyConfig, ResolverState,
    VerticalPhase,
};

/// Runtime knobs for the hip pin, editable from the tuning UI.
#[derive(Resource)]
pub struct HipPinParams {
    /// Raibert resolver config.
    pub config: LowerBodyConfig,
    /// Speed at which compliance softening saturates, px/s.
    pub v_ref: f32,
    /// Compliance multiplier at full speed; `1.0` disables softening.
    pub soft_scale: f32,
    /// Fraction of gravity cancelled while grounded (`0..=1`).
    pub support_share: f32,
    /// External position constraint compliance at rest (lower = stiffer).
    pub compliance: f32,
    /// External position constraint damping.
    pub damping: f32,
    /// Also write the resolver's foot poses into the leg controllers.
    pub hip_drives_legs: bool,
    /// Vertical offset of the whole pinned body in px, positive = down.
    pub hip_drop: f32,
    /// Close the XPBD↔Raibert loop: write the solved `P3` back into the
    /// resolver state each step (`FEEDBACK=0` to disable).
    pub feedback: bool,
    /// Emit the `P1/P2/P3` external position constraints. Off for normal floor
    /// walking: the skeleton is then driven by angular constraints only.
    pub external_pin: bool,
    /// Allow the external pin as a short mid-air direction-change assist.
    pub air_control: bool,
    /// Leg angular-constraint compliance (lower = stiffer).
    pub leg_compliance: f32,
    /// Leg angular-constraint damping ratio (`1.0` = critical).
    pub leg_zeta: f32,
    /// Leg rest-angle chase fraction per tick (`1.0` = full target).
    pub leg_alpha: f32,
    /// Leg rest-angle chase rate cap, rad/s.
    pub leg_max_angle_rate: f32,
    /// Enable loop-3 attitude control.
    pub attitude_enabled: bool,
    /// Compliance of the loop-3 actuator when the external pin is active. In
    /// the default (angular-only) mode loop 3 rides the leg PD instead, so this
    /// is only used for the mid-air pin assist and for the `P1_P2_P3`
    /// posture hold.
    pub attitude_compliance: f32,
    /// Damping ratio of the angular-only loop-3 joint (`1.0` = critical).
    pub attitude_zeta: f32,
    /// Additional desired torso lean, radians (`+` = forward).
    pub attitude_target_bias: f32,
    /// Hard cap on the commanded torso lean, radians.
    pub attitude_max_angle: f32,
    /// Loop-3 proportional gain, forwarded to
    /// [`attitude_lean_pd`](crate::lower_body::resolver::attitude_lean_pd).
    ///
    /// Loop 3 is delivered by tilting the leg PD's world-upright reference axis,
    /// so this gain is in "radians of leg-reference lean per radian of measured
    /// torso error". `0.5` = half correction (gentle). Raise toward 1.0+ to
    /// stiffen the torso, but watch for foot chatter — the legs are the only
    /// actuator and the correction competes with the swing/foot-placement
    /// targets.
    pub attitude_lean_kp: f32,
    /// Loop-3 derivative gain (radians of leg-reference lean per rad/s of
    /// measured torso rate). Damps the reference tilt; `0.0` disables.
    pub attitude_lean_kd: f32,
}

impl HipPinParams {
    /// Read the shared harness/tuning environment: `TARGET_SPEED`, `LEGS`,
    /// `COMPLIANCE`, `DAMPING`, `HIP_DROP`, `SUPPORT_SHARE`, `FEEDBACK`,
    /// `EXTERNAL_PIN`, `AIR_CONTROL`, `LEG_COMPLIANCE`, `LEG_ZETA`,
    /// `LEG_ALPHA`, `LEG_MAX_ANG_RATE`, `ATTITUDE`, `ATT_COMPLIANCE`,
    /// `ATT_ZETA`, `ATT_TARGET_DEG`, `ATT_MAX_DEG`, `ATT_LEAN_KP`,
    /// `ATT_LEAN_KD`, and the gait timing / swing-arc shape: `STANCE`,
    /// `SWING`, `STEP_HEIGHT`.
    pub fn from_env() -> Self {
        let mut p = Self::default();
        p.config.target_speed = env_f32("TARGET_SPEED", p.config.target_speed);
        // Gait shape. `SWING` shorter = the foot must travel the step in less
        // time (a faster swing); `STEP_HEIGHT` is the swing arc apex above the
        // plant line, i.e. how high the foot lifts to clear the floor.
        p.config.stance_duration_ts = env_f32("STANCE", p.config.stance_duration_ts).max(1e-3);
        p.config.swing_duration_ts = env_f32("SWING", p.config.swing_duration_ts).max(1e-3);
        p.config.step_height = env_f32("STEP_HEIGHT", p.config.step_height).max(0.0);
        p.compliance = env_f32("COMPLIANCE", p.compliance);
        p.damping = env_f32("DAMPING", p.damping);
        p.hip_drop = env_f32("HIP_DROP", p.hip_drop);
        p.support_share = env_f32("SUPPORT_SHARE", p.support_share).clamp(0.0, 1.0);
        p.hip_drives_legs = !matches!(
            std::env::var("LEGS").ok().as_deref().map(str::trim),
            Some("off") | Some("0") | Some("false")
        );
        p.feedback = env_bool("FEEDBACK", true);
        p.external_pin = env_bool("EXTERNAL_PIN", p.external_pin);
        p.air_control = env_bool("AIR_CONTROL", p.air_control);
        p.leg_compliance = env_f32("LEG_COMPLIANCE", p.leg_compliance).max(1e-12);
        p.leg_zeta = env_f32("LEG_ZETA", p.leg_zeta).max(0.0);
        p.leg_alpha = env_f32("LEG_ALPHA", p.leg_alpha).clamp(0.0, 1.0);
        p.leg_max_angle_rate = env_f32("LEG_MAX_ANG_RATE", p.leg_max_angle_rate).max(0.0);
        p.attitude_enabled = env_bool("ATTITUDE", p.attitude_enabled);
        p.attitude_compliance = env_f32("ATT_COMPLIANCE", p.attitude_compliance).max(1e-12);
        p.attitude_zeta = env_f32("ATT_ZETA", p.attitude_zeta).max(0.0);
        p.attitude_target_bias =
            env_f32("ATT_TARGET_DEG", p.attitude_target_bias.to_degrees()).to_radians();
        p.attitude_max_angle = env_f32("ATT_MAX_DEG", p.attitude_max_angle.to_degrees())
            .to_radians()
            .max(0.0);
        p.attitude_lean_kp = env_f32("ATT_LEAN_KP", p.attitude_lean_kp).max(0.0);
        p.attitude_lean_kd = env_f32("ATT_LEAN_KD", p.attitude_lean_kd).max(0.0);
        p
    }
}

/// Parse a float env var; malformed or missing ⇒ `default`.
fn env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|s| s.trim().parse::<f32>().ok())
        .unwrap_or(default)
}

/// Parse a boolean env var (`1`/`true`/`yes`/`on` ⇒ true); missing ⇒ `default`.
fn env_bool(key: &str, default: bool) -> bool {
    match std::env::var(key).ok().as_deref().map(str::trim) {
        Some("1") | Some("true") | Some("yes") | Some("on") => true,
        Some("0") | Some("false") | Some("no") | Some("off") => false,
        _ => default,
    }
}

/// Ground-contact state gating the pin's vertical authority.
///
/// The pin may cancel gravity and command y only while supported; in flight
/// gravity is ballistic. Register [`update_ground_support`] before
/// [`drive_hip_pin`].
#[derive(Resource, Debug, Clone)]
pub struct GroundSupport {
    /// True when a character collider touches non-character geometry.
    pub grounded: bool,
    /// Number of near-vertical mixed contacts seen this frame.
    pub contacts: usize,
}

impl Default for GroundSupport {
    fn default() -> Self {
        // No collision pipeline ⇒ keep the kinematic behaviour.
        Self {
            grounded: true,
            contacts: 0,
        }
    }
}

/// Refresh [`GroundSupport`] from the previous tick's [`CollisionEventBatch`].
pub fn update_ground_support(
    batch: Option<Res<CollisionEventBatch>>,
    colliders: Query<&VelloCollider>,
    mut support: ResMut<GroundSupport>,
) {
    let Some(batch) = batch else {
        support.grounded = true;
        support.contacts = 0;
        return;
    };
    let mut contacts = 0usize;
    for entry in &batch.entries {
        let a_soft = colliders
            .get(entry.event.entity_a)
            .map(|c| c.is_soft_body())
            .unwrap_or(false);
        let b_soft = colliders
            .get(entry.event.entity_b)
            .map(|c| c.is_soft_body())
            .unwrap_or(false);
        if a_soft == b_soft {
            continue;
        }
        let normal = if a_soft {
            entry.event.collision_normal_a
        } else {
            entry.event.collision_normal_b
        };
        if normal.y.abs() > 0.5 {
            contacts += 1;
        }
    }
    support.contacts = contacts;
    support.grounded = contacts > 0;
}

impl Default for HipPinParams {
    fn default() -> Self {
        Self {
            config: LowerBodyConfig::default(),
            v_ref: 300.0,
            soft_scale: 1.0,
            support_share: 0.5,
            compliance: 1e-7,
            damping: 0.01,
            hip_drives_legs: true,
            hip_drop: 0.0,
            feedback: true,
            // Normal walking is angular-only; the external pin is reserved for
            // explicit mid-air direction changes.
            external_pin: false,
            air_control: true,
            // The pin OWNS the leg angular compliance and rewrites it every
            // frame (see `drive_hip_pin`), so the asset's leg `Angular`
            // compliance is dead at runtime — THIS is the value that matters.
            //
            // `1e-6` is the edge of stability, not a soft choice: the measured
            // sweep (`tools/physics-qa/sweep_leg_compliance.py`,
            // `physics_tuning_progress.md` §8) shows 5e-7 and below drive the
            // body BACKWARDS at up to -193 px/s with the pinned frame tearing
            // (`rig max_rel_dev` 0.05 -> 0.38). Stiffening the loop-3 attitude
            // (`ATT_COMPLIANCE`) or damping the rest-angle chase (`LEG_ALPHA`,
            // `LEG_MAX_ANG_RATE`) does not tame it. So the stiffness knob is
            // not the fix for the non-stepping gait until the per-frame
            // rest-angle jump (alpha 1.0, rate 1e6) is solved properly.
            // `LEG_COMPLIANCE` still overrides for experiments.
            leg_compliance: 1e-6,
            leg_zeta: 1.0,
            leg_alpha: 1.0,
            leg_max_angle_rate: 1e6,
            attitude_enabled: true,
            attitude_compliance: 1e-7,
            attitude_zeta: 1.0,
            attitude_target_bias: 0.0,
            attitude_max_angle: 0.25, // ~14°, inside the 15° joint wall
            // Loop 3 now rides the leg PD's world-upright reference axis, so
            // these are *leg-reference* gains, not torques. `0.5` corrects half
            // the measured lean in one tick; `0.03` damps the reference tilt.
            attitude_lean_kp: 0.5,
            attitude_lean_kd: 0.03,
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

/// Sandbox↔leg bridge captured once on the first leg-driving tick.
///
/// Only the resolver's per-segment rotation deltas are transferred onto the
/// live spawn pose, so the character's own bone lengths and rest geometry are
/// preserved and the leg PD sees no step at t=0.
#[derive(Debug, Clone, Copy)]
struct LegBridgeSeed {
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

/// Rotate the seed segment `live0` by the resolver segment's rotation from
/// `res0` to `res1`, preserving `live0`'s length.
#[inline]
fn transfer_segment(live0: Vec2, res0: Vec2, res1: Vec2) -> Vec2 {
    if res0.length_squared() < 1e-6 || res1.length_squared() < 1e-6 {
        return live0;
    }
    rotate_vec(live0, signed_angle(res0, res1))
}

/// Per-character hip pin, attached to the character root.
#[derive(Component)]
pub struct HipPin {
    /// Persistent Raibert resolver state.
    pub state: ResolverState,
    /// `P1/P2/P3` rest offsets relative to `P3`, in Vello (y-down) world space.
    pub rest_offsets: [Vec2; 3],
    /// Resolver hip at seed time (local y-up).
    pub seed_hip: Vec2,
    /// Live `P3` world (Vello y-down) position at seed time.
    pub anchor_p3: Vec2,
    /// Previous `world_delta`; the pin commands only its per-tick change.
    prev_world_delta: Vec2,
    /// Set once the rest offsets and anchor have been captured.
    pub ready: bool,
    /// Master enable; when false the pin emits nothing.
    pub enabled: bool,
    /// Rigid-frame attitude in radians.
    pub theta: f32,
    /// Whether a jump was held last step, for edge detection at fixed `dt`.
    jump_was_held: bool,
    /// Sandbox↔leg bridge seed, captured lazily.
    leg_seed: Option<LegBridgeSeed>,
    /// Resolver reference for `P3` in Vello world (y-down).
    pub last_ref_p3: Vec2,
    /// Realized vertical sag of `P3` against the reference, positive = below.
    pub last_sag_y: f32,
    /// Loop-3 control input `u` from the last resolver tick, rad/s².
    pub last_attitude_torque: f32,
    /// Measured hip horizontal acceleration from the last resolver tick, px/s².
    pub last_hip_accel_x: f32,
    /// Last measured physical torso attitude (`P2→P1` vs world vertical), rad.
    pub last_meas_theta: f32,
    /// Last loop-3 angular rest basis written to `P1_P2_P3` (`cos`, `sin`).
    pub last_att_rest_cos: f32,
    pub last_att_rest_sin: f32,
    /// Last commanded loop-3 attitude target, rad (`+` = forward lean).
    pub last_att_target: f32,
    /// Whether the external `P1/P2/P3` pin was active this tick.
    pub last_external_pin: bool,
    /// Whole-body CoM measurement (plan §1). Vello y-down frame: `com`,
    /// EMA-filtered `v_com`, `z_com` (height above the floor), measured torso
    /// `theta`/`omega`. Populated by [`measure_body`] each fixed tick.
    pub measured: BodyMeasurements,
    /// EMA + finite-difference filter state for the measured body.
    pub measure_filter: BodyMeasureFilter,
}

impl HipPin {
    /// Live hip→foot distance captured at the leg-bridge seed (bevy y-up).
    pub fn stance_height(&self) -> Option<f32> {
        self.leg_seed.map(|s| s.live[0].hip.y - s.live[0].foot.y)
    }

    pub fn new(config: &LowerBodyConfig) -> Self {
        Self {
            state: ResolverState::new(config),
            rest_offsets: [Vec2::ZERO; 3],
            seed_hip: Vec2::ZERO,
            anchor_p3: Vec2::ZERO,
            prev_world_delta: Vec2::ZERO,
            ready: false,
            enabled: true,
            theta: 0.0,
            jump_was_held: false,
            leg_seed: None,
            last_ref_p3: Vec2::ZERO,
            last_sag_y: 0.0,
            last_attitude_torque: 0.0,
            last_hip_accel_x: 0.0,
            last_meas_theta: 0.0,
            last_att_rest_cos: 1.0,
            last_att_rest_sin: 0.0,
            last_att_target: 0.0,
            last_external_pin: false,
            measured: BodyMeasurements::default(),
            measure_filter: BodyMeasureFilter::default(),
        }
    }

    /// Drop the leg seed so the next leg-driving tick re-captures it from the
    /// live pose (toggle of `hip_drives_legs`, teleport, reset).
    pub fn reseat_leg_bridge(&mut self) {
        self.leg_seed = None;
    }
}

/// Signed physical torso attitude: the `P2→P1` axis against world vertical.
///
/// Vello is y-down, so upright is `(0, -1)` and `+` means a forward lean
/// toward `+x`. Returns `0` when the torso is vertical.
#[inline]
fn measured_attitude(p1: Vec2, p2: Vec2) -> f32 {
    let axis = p1 - p2;
    axis.x.atan2(-axis.y)
}

/// Clamp a commanded lean, falling back to upright if the input is not finite.
/// A raw `f32::clamp` panics on NaN (`min > max, or either was NaN`).
#[inline]
fn clamp_attitude(raw: f32, max_angle: f32) -> f32 {
    if raw.is_finite() {
        raw.clamp(-max_angle.abs(), max_angle.abs())
    } else {
        0.0
    }
}

/// Mirror the solver's `P1_P2_P3` angle measurement
/// (`v1 = P1→P2`, `v2 = P2→P3`, `cos = u1·u2`,
/// `sin = u1.y*u2.x - u1.x*u2.y`), returning only the rest basis.
#[inline]
fn angle_basis(pos0: Vec2, pos1: Vec2, pos2: Vec2) -> Option<(f32, f32)> {
    if !pos0.is_finite() || !pos1.is_finite() || !pos2.is_finite() {
        return None;
    }
    let v1 = pos1 - pos0;
    let v2 = pos2 - pos1;
    let l1 = v1.length();
    let l2 = v2.length();
    if l1 < 1e-6 || l2 < 1e-6 {
        return None;
    }
    let u1 = v1 / l1;
    let u2 = v2 / l2;
    Some((u1.dot(u2), u1.y * u2.x - u1.x * u2.y))
}

/// Rest basis that holds the `P2→P1` upper-body axis at `theta` while `P2`
/// and `P3` remain where the hard pin holds them.
///
/// This is loop 3 about P2: the `P1_P2_P3` angular joint rotates **P1 only**
/// (when `free_mask` excludes P2/P3), so `P3` stays hard-pinned and the upper
/// body is corrected relative to the `P2→P3` lower-body axis.
#[inline]
fn attitude_rest_basis(p1: Vec2, p2: Vec2, p3: Vec2, theta: f32) -> Option<(f32, f32)> {
    if !theta.is_finite() {
        return None;
    }
    let upper_len = (p1 - p2).length().max(1e-3);
    let upper_dir = Vec2::new(theta.sin(), -theta.cos());
    let p1_target = p2 + upper_dir * upper_len;
    angle_basis(p1_target, p2, p3)
}

/// Rigid `[P1, P2, P3]` targets rotated by `theta` about `P2`.
///
/// `rest_offsets` is the seeded `[P1-P3, P2-P3, 0]`, so the P2-relative arms
/// are `P1-P2 = rest[0] - rest[1]` and `P3-P2 = -rest[1]`. Rotating both by the
/// same angle keeps every frame distance unchanged (no triangle over-constraint).
#[inline]
fn p2_rigid_frame_targets(p2_target: Vec2, rest_offsets: [Vec2; 3], theta: f32) -> [Vec2; 3] {
    let (sin, cos) = theta.sin_cos();
    let rotate = |v: Vec2| Vec2::new(v.x * cos - v.y * sin, v.x * sin + v.y * cos);
    let r_p1 = rest_offsets[0] - rest_offsets[1];
    let r_p3 = -rest_offsets[1];
    [
        p2_target + rotate(r_p1),
        p2_target,
        p2_target + rotate(r_p3),
    ]
}

/// Advance the pinned frame by the Raibert hip delta plus a centroid
/// velocity servo.
///
/// `d = hip_delta + (v_hip − v_centroid)·dt + (1 − support_share)·g·dt²`
/// is applied to every particle, so the frame stays rigid and no absolute
/// pose term is used. `support_share` is the gravity fraction the pin cancels;
/// on a released vertical axis the particle keeps its free-flight y
/// (`v·dt + g·dt²`), which leaves gravity ballistic.
fn delta_frame_targets(
    states: [(Vec2, Vec2); 3],
    hip_delta: Vec2,
    v_hip: Vec2,
    gravity: Vec2,
    dt: f32,
    vertical: bool,
    support_share: f32,
) -> [Vec2; 3] {
    let v_c = (states[0].1 + states[1].1 + states[2].1) / 3.0;
    let g_eff = gravity * (1.0 - support_share.clamp(0.0, 1.0));
    let d = hip_delta + (v_hip - v_c) * dt + g_eff * (dt * dt);
    states.map(|(pos, vel)| {
        if vertical {
            pos + d
        } else {
            pos + Vec2::new(d.x, vel.y * dt + gravity.y * (dt * dt))
        }
    })
}

/// Speed-softened compliance: `base·(1 + s·(soft_scale−1))`, `s = |v|/v_ref`
/// clamped to 1. `soft_scale = 1` or `v_ref ≤ 0` returns `base`.
fn softened_compliance(base: f32, soft_scale: f32, v_ref: f32, speed: f32) -> f32 {
    let s = if v_ref > 0.0 {
        (speed / v_ref).min(1.0)
    } else {
        0.0
    };
    base * (1.0 + s * (soft_scale - 1.0))
}

/// Whether the external `P1/P2/P3` position pin should run this tick.
///
/// Normal floor walking is angular-only. The pin is allowed only when the
/// explicit debug switch is on, or as a short mid-air direction-change assist
/// (airborne + horizontal input).
#[inline]
fn use_external_pin(external_pin: bool, air_control: bool, grounded: bool, input_x: f32) -> bool {
    external_pin || (air_control && !grounded && input_x.abs() > 0.0)
}

/// Capture the `P3` world anchor and the `P1/P2` rest offsets once the
/// character is assembled.
fn try_seed(
    pin: &mut HipPin,
    spine: &SpineController,
    root: &VelloCharacterPhysicsRoot,
    hip_drop: f32,
    p_q: &Query<&VelloParticle>,
) {
    if pin.ready || root.initial_frame_coordinates.is_none() {
        return;
    }
    // spine.particles == [PH, P0, P1, P2, P3]; the pin drives the last three.
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
    // Bias the first delta so the configured drop is applied exactly once.
    pin.prev_world_delta = Vec2::new(0.0, -hip_drop);
    pin.ready = true;
}

/// Measurement time constant for the EMA-filtered `v_CoM` and `omega` (plan §3:
/// "EMA-filtered (2–4 frame time constant)"). At 90 Hz, 0.03 s ≈ 3 frames.
pub const MEASURE_TAU: f32 = 0.03;

/// Gather every particle owned by the character and update the whole-body CoM
/// measurement (plan §1 / changes §1).
///
/// Scans all [`VelloParticle`]s whose `root_entity` is the character root (the
/// `HipPin` entity), mass-weights position and velocity over **all** parts
/// (including arms), EMA-filters `v_CoM` and `omega`, and stores the measured
/// torso `theta` + `z_CoM` on the pin. Runs before [`drive_hip_pin`] so the
/// resolver's loops can read the measurement this same fixed step.
pub fn measure_body(
    time: Res<Time<Fixed>>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<&SpineController>,
    p_q: Query<&VelloParticle>,
) {
    let dt = time.delta_secs();
    for (entity, mut pin) in pin_q.iter_mut() {
        if !pin.enabled {
            continue;
        }
        let Ok(spine) = spine_q.get(entity) else {
            continue;
        };
        // Gather all particles of this character (root_entity == entity).
        let mut samples: Vec<ParticleSample> = Vec::with_capacity(16);
        for p in p_q.iter() {
            if p.root_entity == entity {
                samples.push(ParticleSample {
                    pos: p.particle.pos,
                    vel: p.particle.velocity,
                    inv_mass: p.particle.inv_mass,
                });
            }
        }
        if samples.is_empty() {
            continue;
        }
        // Torso axis: v10 spine = [PH, P0, P1, P2, P3]; hip = P3, shoulder = P1.
        let (hip, shoulder) = match (p_q.get(spine.particles[4]), p_q.get(spine.particles[2])) {
            (Ok(hip), Ok(shoulder)) => (hip.particle.pos, shoulder.particle.pos),
            _ => continue,
        };
        // Floor line: the resolver's flat floor, Vello y = 0 (the character is
        // assembled with the floor at world y≈0; see the tuning app / harness).
        let floor_y = 0.0;
        pin.measured = update_measurements(
            &mut pin.measure_filter,
            &samples,
            (hip, shoulder),
            floor_y,
            dt,
            MEASURE_TAU,
        );
    }
}

/// Step the resolver and queue the three stiff external position constraints
/// that pin `P1/P2/P3` to the commanded frame.
///
/// Run before `update_character_movement` (the legs read the targets) and
/// before the constraint world is stepped.
pub fn drive_hip_pin(
    time: Res<Time<Fixed>>,
    params: Res<HipPinParams>,
    ground_support: Option<Res<GroundSupport>>,
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
        try_seed(&mut pin, spine, root, params.hip_drop, &p_q);
        if !pin.ready {
            continue;
        }

        // Feedback: close the XPBD↔Raibert loop *scoped to the horizontal
        // channel*. `update_connection_particles` synced the solved physics
        // into `VelloParticle` at the end of the previous fixed step, so `P3`
        // (spine.particles[4], the hip anchor) holds the realized pose.
        //
        // Only the realized horizontal position is written back into the
        // resolver: foot placement must start from where physics actually left
        // the body. The vertical channel is left to the resolver's own
        // `tick_vertical` authority — its `regulate_height` spring already
        // closes that loop against the measured state, and re-injecting the
        // measured y (with its steady `g_eff·dt²` sag) would accumulate into
        // drift. The commanded hip *velocity* is likewise preserved, because
        // `delta_frame_targets` uses it as the servo reference
        // (`(v_hip − v_c)·dt`); overwriting it with the measured velocity would
        // cancel that command-vs-realized correction. `FEEDBACK=0` disables it
        // for A/B against the feed-forward baseline.
        if params.feedback {
            if let Ok(p3) = p_q.get(spine.particles[4]) {
                // Vello y-down → resolver y-up (flip y). Horizontal only.
                let measured_x = p3.particle.pos.x;
                let anchor_x = pin.anchor_p3.x;
                let world_dx = measured_x - anchor_x;
                pin.state.hip_pos.x = pin.seed_hip.x + world_dx;
            }
        }

        // Loop 1 balance feedback (plan §2): feed the measured whole-body CoM
        // into the resolver so `plan_target_x` uses the LIPM capture point
        // `x_foot = x_CoM + (v_CoM − v_des)/ω₀` instead of the Raibert neutral
        // point. `pin.measured` is in the Vello y-down frame: horizontal x is
        // unaffected by the flip, and `z_com` is already positive-up. The CoM x
        // is mapped into the resolver's seed-relative frame the same way the P3
        // feedback above maps the hip. When no valid measurement exists
        // (`n == 0`, e.g. the pure kinematic sandbox path) the resolver falls
        // back to its Raibert neutral point.
        if pin.measured.n > 0 {
            if let Ok(p3) = p_q.get(spine.particles[4]) {
                let com_offset_x = pin.measured.com.x - p3.particle.pos.x;
                pin.state.measured = Some(CaptureMeasure {
                    com_x: pin.state.hip_pos.x + com_offset_x,
                    v_com: pin.measured.v_com.x,
                    z_com: pin.measured.z_com,
                });
                // Loop 2 (plan §4): close the `Δl` integrator on the measured
                // whole-body CoM apex instead of the resolver's internal
                // ballistic model. `z_com` is the CoM height above the floor;
                // the resolver's `h_apex_prev` is an apex RISE above the
                // standing height, so feed `z_com − standing_hip_height`.
                pin.state.measured_apex = Some(pin.measured.z_com - params.config.hip_height);
            }
        } else {
            pin.state.measured = None;
            pin.state.measured_apex = None;
        }

        // Game input → resolver input. The jump is edge-triggered here because
        // a held key would re-arm the hop on every landing.
        let mut input_dir = Vec2::ZERO;
        if let Ok(action) = action_q.get(entity) {
            if action.pressed(&PlayerAction::MoveRight) {
                input_dir.x += 1.0;
            }
            if action.pressed(&PlayerAction::MoveLeft) {
                input_dir.x -= 1.0;
            }
            let jump_held = action.pressed(&PlayerAction::MoveUp);
            if jump_held && !pin.jump_was_held {
                input_dir.y += 1.0;
            }
            pin.jump_was_held = jump_held;
            if action.pressed(&PlayerAction::MoveDown) {
                input_dir.y -= 1.0;
            }
        }

        // External pin policy: normal floor walking is angular-only. The pin is
        // allowed only behind the explicit debug switch, or as a mid-air
        // direction-change assist (airborne + horizontal input).
        let grounded = ground_support
            .as_deref()
            .map(|s| s.grounded)
            .unwrap_or(true);
        let ext_pin = use_external_pin(
            params.external_pin,
            params.air_control,
            grounded,
            input_dir.x,
        );
        pin.last_external_pin = ext_pin;

        let (pose, debug) = step_lower_body(input_dir, &params.config, &mut pin.state, dt);
        // Loop-3 telemetry latch: the torque/accel are per-tick `DebugInfo`
        // values (not persistent state), so keep the last tick's for the
        // headless harness's whole-body row.
        pin.last_attitude_torque = debug.attitude_torque;
        pin.last_hip_accel_x = debug.hip_accel_x;

        // Loop 3 → physics: the resolver's attitude is the desired torso lean.
        // Sign is consistent with no flip: resolver positive θ = forward lean in
        // its y-up travel frame (+x), `world_delta` maps resolver +x → Vello +x,
        // and an up-on-screen axis `(0, −L)` rotated by positive θ becomes
        // `(L·sinθ, −L·cosθ)` — also a lean toward +x.
        pin.theta = pin.state.attitude.theta;

        // Live P1/P2/P3 states, in the same order as `idx`.
        let idx = [2usize, 3, 4];
        let mut states = [(Vec2::ZERO, Vec2::ZERO); 3];
        let mut all_ready = true;
        for (k, i) in idx.iter().enumerate() {
            match p_q.get(spine.particles[*i]) {
                Ok(p) => states[k] = (p.particle.pos, p.particle.velocity),
                Err(_) => {
                    all_ready = false;
                    break;
                }
            }
        }
        if !all_ready {
            continue;
        }

        // Loop 3: rotate the whole [P1,P2,P3] rigid frame about P2. The
        // angular-joint path is *not* used here: with P2/P3 hard-pinned and the
        // rigid P1-P3 frame distance, changing only the P1_P2_P3 angle would
        // over-constrain the triangle (the solver can flip P1 through the
        // collinear singularity). Rotating all three by the same R(theta) about
        // P2 preserves every frame distance and makes the hip follow the torso.
        let p1_live = states[0].0;
        let p2_live = states[1].0;
        let p3_live = states[2].0;
        pin.last_meas_theta = measured_attitude(p1_live, p2_live);
        let theta = if params.attitude_enabled {
            clamp_attitude(
                pin.theta + params.attitude_target_bias,
                params.attitude_max_angle,
            )
        } else {
            0.0
        };
        pin.last_att_target = theta;
        if let Some((rest_cos, rest_sin)) = attitude_rest_basis(p1_live, p2_live, p3_live, theta) {
            pin.last_att_rest_cos = rest_cos;
            pin.last_att_rest_sin = rest_sin;
        }

        // Resolver hip (y-up) → Vello world (y-down). Only the delta from the
        // seed is used, so the resolver's absolute frame never leaks.
        let world_delta = Vec2::new(pose.hip.x - pin.seed_hip.x, -(pose.hip.y - pin.seed_hip.y));
        let hip_delta = world_delta - pin.prev_world_delta;
        pin.prev_world_delta = world_delta;
        let v_hip = Vec2::new(pin.state.hip_vel.x, -pin.state.hip_vel.y);

        let g_vello = world.gravity();

        // Vertical authority: supported, or a ballistic resolver phase (where
        // tracking the reference preserves the jump).
        let vertical_tracking = matches!(
            pin.state.vs.phase,
            VerticalPhase::Launch | VerticalPhase::Flight
        );
        let vertical_allowed = grounded || vertical_tracking;
        let support_share = if grounded { params.support_share } else { 0.0 };

        let targets = delta_frame_targets(
            states,
            hip_delta,
            v_hip,
            g_vello,
            dt,
            vertical_allowed,
            support_share,
        );

        // P2 carries the delta/velocity/gravity servo. P1/P3 are the rest
        // offsets from P2, rotated by theta; this rotates the whole rigid frame
        // about P2 and preserves all three frame distances.
        let p2_target = targets[1];
        let pose_targets = if params.attitude_enabled {
            p2_rigid_frame_targets(p2_target, pin.rest_offsets, theta)
        } else {
            // Legacy translation-only path: no attitude authority.
            targets
        };

        // All three frame particles are commanded to the rigid, P2-centered
        // pose. P3 remains hard-pinned, but to the attitude-adjusted target.
        for (k, i) in idx.iter().enumerate() {
            let (pos, vel) = states[k];
            let target = pose_targets[k];
            if *i == 4 {
                pin.last_ref_p3 = target;
                pin.last_sag_y = pos.y - target.y;
            }
            if !ext_pin {
                continue;
            }
            // While loop 3 is active the whole frame is a P2-centred attitude
            // pin, so the attitude compliance is the relevant stiffness.
            let base_compliance = if params.attitude_enabled {
                params.attitude_compliance
            } else {
                params.compliance
            };
            let compliance = softened_compliance(
                base_compliance,
                params.soft_scale,
                params.v_ref,
                vel.length(),
            );

            // world.queue_character_one_time_external_position_constraint(
            //     &CharacterExternalPositionConstraintEvent {
            //         character_entity: entity,
            //         joint_entity: spine.particles[*i],
            //         config: ExternalPositionConstraintConfig {
            //             target,
            //             compliance,
            //             damping: params.damping,
            //         },
            //     },
            // );
        }

        // Opt-in telemetry (`PIN_LOG=1`).
        let pin_log = std::env::var("PIN_LOG")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);
        if pin_log {
            *log_tick += 1;
            if *log_tick % 90 == 0 {
                let p3 = p_q
                    .get(spine.particles[4])
                    .map(|p| p.particle.pos)
                    .unwrap_or(pin.last_ref_p3);
                info!(
                    "pin vertical={} grounded={} share={:.2} P3 actual=({:.1},{:.1}) ref=({:.1},{:.1}) sag_y={:.1}",
                    vertical_allowed,
                    grounded,
                    support_share,
                    p3.x,
                    p3.y,
                    pin.last_ref_p3.x,
                    pin.last_ref_p3.y,
                    pin.last_sag_y,
                );
            }
        }

        // Sandbox → legs: write the resolver poses into the leg controllers.
        // `update_character_movement` (ordered after) reads them for the IK.
        if params.hip_drives_legs {
            let (Ok(mut right_leg), Ok(mut left_leg)) =
                (right_leg_q.get_mut(entity), left_leg_q.get_mut(entity))
            else {
                continue;
            };
            // Only the external pin owns P2/P3. In angular-only mode the leg
            // PD uses the full mass matrix and references the desired upright
            // spine, so it supports the torso instead of following it.
            right_leg.config.anchor_chain = ext_pin;
            left_leg.config.anchor_chain = ext_pin;
            right_leg.config.upright_spine_reference = !ext_pin;
            left_leg.config.upright_spine_reference = !ext_pin;

            // Loop 3 (plan §5, corrected delivery): the body above P3 is a
            // straight stiff stick, and the only external forces on it are
            // gravity, foot contact and friction — so its world verticality is
            // decided by the hip joints `P2–P3–P40/P41`, NOT by the waist
            // `P1–P2–P3`. Deliver the measured-attitude PD by tilting the
            // world-upright reference axis the leg PD aims the thigh at.
            //
            // Gating:
            //   * `attitude_enabled` — master switch;
            //   * `!ext_pin` — with the external pin active the pin itself owns
            //     the frame attitude (mid-air assist), so the leg axis must
            //     stay world-vertical;
            //   * `grounded` — in Flight there is no contact and therefore no
            //     leg authority; the stick must stay ballistic, so the
            //     reference is held straight instead of trimming;
            //   * `measured.n > 0` — no valid whole-body measurement yet
            //     (startup / detached parts) means no error signal to act on.
            let upright_lean =
                if params.attitude_enabled && !ext_pin && grounded && pin.measured.n > 0 {
                    attitude_lean_pd(
                        pin.measured.theta,
                        pin.measured.omega,
                        params.attitude_target_bias,
                        params.attitude_lean_kp,
                        params.attitude_lean_kd,
                        params.attitude_max_angle,
                    )
                } else {
                    0.0
                };
            right_leg.config.upright_reference_lean = upright_lean;
            left_leg.config.upright_reference_lean = upright_lean;

            right_leg.config.angular_compliance = params.leg_compliance.max(1e-12);
            left_leg.config.angular_compliance = params.leg_compliance.max(1e-12);
            right_leg.config.zeta = params.leg_zeta.max(0.0);
            left_leg.config.zeta = params.leg_zeta.max(0.0);
            right_leg.config.rest_chase_alpha = params.leg_alpha.clamp(0.0, 1.0);
            left_leg.config.rest_chase_alpha = params.leg_alpha.clamp(0.0, 1.0);
            right_leg.config.max_angle_rate = params.leg_max_angle_rate.max(0.0);
            left_leg.config.max_angle_rate = params.leg_max_angle_rate.max(0.0);

            let vello_to_bevy = |p: &VelloParticle| Vec2::new(p.particle.pos.x, -p.particle.pos.y);
            let live_pose = |particles: [Entity; 4],
                             hip_fb: Vec2,
                             knee_fb: Vec2,
                             foot_fb: Vec2|
             -> LegPoseSeed {
                let get =
                    |e: Entity, fallback: Vec2| p_q.get(e).map(&vello_to_bevy).unwrap_or(fallback);
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

            // Hip target: x follows the live P3 (so impulses move the body),
            // but y follows the resolver's vertical command relative to the
            // seed. Without the external pin, using live y here makes the foot
            // targets sink with the body and the leg PD then drives the feet
            // through the floor.
            let actual_p3_bevy = Vec2::new(states[2].0.x, -states[2].0.y);
            let seed_hip_y = 0.5 * (seed.live[0].hip.y + seed.live[1].hip.y);
            let desired_hip_y = seed_hip_y + (pose.hip.y - pin.seed_hip.y);
            let hip_t = Vec2::new(actual_p3_bevy.x, desired_hip_y) - Vec2::Y * params.hip_drop;
            let mut knee_l = hip_t
                + transfer_segment(
                    seed.live[0].knee - seed.live[0].hip,
                    seed.resolver[0].knee - seed.resolver[0].hip,
                    pose.left_knee - pose.hip,
                );
            let mut foot_l = knee_l
                + transfer_segment(
                    seed.live[0].foot - seed.live[0].knee,
                    seed.resolver[0].foot - seed.resolver[0].knee,
                    pose.left_foot - pose.left_knee,
                );
            let mut knee_r = hip_t
                + transfer_segment(
                    seed.live[1].knee - seed.live[1].hip,
                    seed.resolver[1].knee - seed.resolver[1].hip,
                    pose.right_knee - pose.hip,
                );
            let mut foot_r = knee_r
                + transfer_segment(
                    seed.live[1].foot - seed.live[1].knee,
                    seed.resolver[1].foot - seed.resolver[1].knee,
                    pose.right_foot - pose.right_knee,
                );
            // Safety clamp: never command a foot below its seed floor line.
            // (Bevy y-up, so the floor is the seed foot y.)
            foot_l.y = foot_l.y.max(seed.live[0].foot.y);
            foot_r.y = foot_r.y.max(seed.live[1].foot.y);

            // Loop 2 delivery (plan §4): the resolver's virtual-leg thrust
            // command biases the leg-PD targets. In Bevy y-up, `stance_extend`
            // pushes the stance foot DOWN (extend the leg → ground reaction
            // lifts the CoM), `flight_retract` tucks the knee UP (swing
            // clearance), and `land_absorb` bends the knee DOWN (controlled
            // flexion to absorb the touchdown). Each is applied only to the leg
            // in the matching gait phase, so a swing leg is never extended into
            // the ground and a stance leg is never retracted.
            let thrust = debug.thrust;
            if debug.left_phase == GaitPhase::Swing {
                knee_l.y = knee_l.y + thrust.flight_retract; // tuck knee up
            } else {
                // stance (or flight-sync stance): extend/absorb as commanded
                foot_l.y = foot_l.y - thrust.stance_extend.max(thrust.land_absorb);
            }
            if debug.right_phase == GaitPhase::Swing {
                knee_r.y = knee_r.y + thrust.flight_retract;
            } else {
                foot_r.y = foot_r.y - thrust.stance_extend.max(thrust.land_absorb);
            }

            left_leg.hip_target = hip_t;
            right_leg.hip_target = hip_t;
            left_leg.knee_target = knee_l;
            right_leg.knee_target = knee_r;
            left_leg.target = foot_l;
            right_leg.target = foot_r;
            // Joint-rate feed-forward for the leg PD (bevy y-up, px/s).
            left_leg.hip_vel = pose.hip_vel;
            left_leg.knee_vel = pose.left_knee_vel;
            left_leg.target_vel = pose.left_foot_vel;
            right_leg.hip_vel = pose.hip_vel;
            right_leg.knee_vel = pose.right_knee_vel;
            right_leg.target_vel = pose.right_foot_vel;
        } else {
            // No pin ⇒ the frame is free; use the full mass matrix.
            if let Ok(mut right_leg) = right_leg_q.get_mut(entity) {
                right_leg.config.anchor_chain = false;
                right_leg.config.upright_spine_reference = false;
            }
            if let Ok(mut left_leg) = left_leg_q.get_mut(entity) {
                left_leg.config.anchor_chain = false;
                left_leg.config.upright_spine_reference = false;
            }
        }
    }
}

/// Posture hold for the `P1_P2_P3` waist joint: keep the spine **straight**.
///
/// This used to be the loop-3 actuator (the resolver's model `θ` was commanded
/// here). It is not: the body above `P3` is a straight, stiff stick whose
/// verticality is set by the hip joints `P2–P3–P40/P41`, and the measured
/// attitude is now delivered through the leg PD's upright reference axis (see
/// [`drive_hip_pin`]). What is left for this joint is posture — hold `P1`, `P2`
/// and `P3` collinear (`theta = 0.0`) so the waist never folds while the legs
/// do the balancing.
///
/// Runs after [`drive_hip_pin`] (which updates `pin.measured` and
/// `last_external_pin`) and before the constraint world is stepped.
pub fn tick_loop3_angular(
    params: Res<HipPinParams>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<&SpineController>,
    p_q: Query<&VelloParticle>,
    mut world: ResMut<VelloConstraintWorld>,
) {
    for (entity, mut pin) in pin_q.iter_mut() {
        // The external pin, when active, already owns the frame pose.
        if !pin.enabled || pin.last_external_pin || !params.attitude_enabled {
            continue;
        }
        let Ok(spine) = spine_q.get(entity) else {
            continue;
        };
        let (Ok(p1), Ok(p2), Ok(p3)) = (
            p_q.get(spine.particles[2]),
            p_q.get(spine.particles[3]),
            p_q.get(spine.particles[4]),
        ) else {
            continue;
        };
        let p1 = p1.particle.pos;
        let p2 = p2.particle.pos;
        let p3 = p3.particle.pos;
        pin.last_meas_theta = measured_attitude(p1, p2);
        // Straight stick: no model replay, no extra lean. The measured-attitude
        // correction is delivered by the legs, not here.
        let theta = 0.0;
        pin.last_att_target = theta;
        let Some((rest_cos, rest_sin)) = attitude_rest_basis(p1, p2, p3, theta) else {
            continue;
        };
        pin.last_att_rest_cos = rest_cos;
        pin.last_att_rest_sin = rest_sin;
        world.queue_character_angular_constraints(&CharacterAngularConstraintEvent {
            character_entity: entity,
            joint_entity: spine.angulars[2], // P1_P2_P3, pivot at P2
            config: AngularConstraintConfig {
                rest_cos,
                rest_sin,
                compliance: params.attitude_compliance.max(1e-12),
                zeta: params.attitude_zeta.max(0.0),
                angular_velocity: None,
                sigma: 1.0,
                // No external pin: all three frame particles are free to
                // receive the corrective torque.
                free_mask: 0b111,
                max_angle: None,
            },
        });
    }
}

/// Reset the pin to its resting stance and re-arm the seed (`R`).
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
        pin.prev_world_delta = Vec2::ZERO;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEN: f32 = 70.0;

    #[test]
    fn measured_attitude_is_zero_when_upright() {
        let p2 = Vec2::ZERO;
        let p1 = Vec2::new(0.0, -LEN);
        assert!(measured_attitude(p1, p2).abs() < 1e-6);
    }

    #[test]
    fn measured_attitude_is_positive_for_forward_lean() {
        let p2 = Vec2::ZERO;
        let angle = 0.3_f32;
        let p1 = Vec2::new(LEN * angle.sin(), -LEN * angle.cos());
        assert!((measured_attitude(p1, p2) - angle).abs() < 1e-5);
    }

    #[test]
    fn rest_basis_upright_is_collinear() {
        let p1 = Vec2::new(0.0, -LEN);
        let p2 = Vec2::ZERO;
        let p3 = Vec2::new(0.0, LEN);
        let (cos, sin) = attitude_rest_basis(p1, p2, p3, 0.0).expect("non-degenerate");
        assert!((cos - 1.0).abs() < 1e-6);
        assert!(sin.abs() < 1e-6);
    }

    #[test]
    fn rest_basis_encodes_the_requested_upper_lean() {
        let p1 = Vec2::new(0.0, -LEN);
        let p2 = Vec2::ZERO;
        let p3 = Vec2::new(0.0, LEN);
        let theta = 0.3_f32;
        let (cos, sin) = attitude_rest_basis(p1, p2, p3, theta).expect("non-degenerate");
        assert!((cos - theta.cos()).abs() < 1e-5);
        assert!((sin - theta.sin()).abs() < 1e-5);
        assert!((sin.atan2(cos) - theta).abs() < 1e-5);
    }

    #[test]
    fn rest_basis_compensates_for_lower_body_tilt() {
        let p1 = Vec2::new(0.0, -LEN);
        let p2 = Vec2::ZERO;
        let lower = 0.2_f32;
        let p3 = Vec2::new(LEN * lower.sin(), LEN * lower.cos());
        let (cos, sin) = attitude_rest_basis(p1, p2, p3, 0.0).expect("non-degenerate");
        // With the lower body tilted forward, holding the upper body vertical
        // requires a joint rest angle equal to that lower-body tilt.
        assert!((sin.atan2(cos) - lower).abs() < 1e-5);
    }

    #[test]
    fn rest_basis_rejects_degenerate_geometry() {
        let p = Vec2::new(0.0, -LEN);
        assert!(attitude_rest_basis(p, p, p, 0.0).is_none());
    }

    #[test]
    fn external_pin_policy_is_walking_off_and_air_assist_on() {
        // Debug switch always wins.
        assert!(use_external_pin(true, true, true, 0.0));
        // Grounded + walking: no external pin.
        assert!(!use_external_pin(false, true, true, 1.0));
        // Airborne + horizontal input: assist.
        assert!(use_external_pin(false, true, false, 1.0));
        // Airborne without input: no assist.
        assert!(!use_external_pin(false, true, false, 0.0));
        // Air assist disabled.
        assert!(!use_external_pin(false, false, false, 1.0));
    }

    #[test]
    fn p2_rotation_preserves_all_frame_distances() {
        // Rest geometry: P1 above P2, P3 below, both at distance LEN.
        let p1 = Vec2::new(0.0, -LEN);
        let p2 = Vec2::ZERO;
        let p3 = Vec2::new(0.0, LEN);
        let rest_offsets = [p1 - p3, p2 - p3, Vec2::ZERO];
        let p2_target = Vec2::new(25.0, 10.0);

        let [t1, t2, t3] = p2_rigid_frame_targets(p2_target, rest_offsets, 0.3);
        assert!((t2 - p2_target).length() < 1e-6);
        // Rotating the whole frame about P2 must preserve every side length.
        assert!(((t1 - t2).length() - LEN).abs() < 1e-5);
        assert!(((t3 - t2).length() - LEN).abs() < 1e-5);
        assert!(((t1 - t3).length() - 2.0 * LEN).abs() < 1e-5);
    }
}
