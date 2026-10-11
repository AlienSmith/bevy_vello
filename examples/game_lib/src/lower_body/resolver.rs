//! Pure Raibert lower-body pose resolver (kinematic, no XPBD).
//!
//! This module is the game-layer mathematical core of the lower-body sandbox
//! defined in `plans/raibert_sandbox.md`. It implements the forward step
//! planner and the backward velocity clamp from `plans/Raibert.md` term-for-term,
//! then resolves each leg (hip → knee → foot) with law-of-cosines 2-bone IK.
//!
//! Everything here is a deterministic pure function of `(input, config, state,
//! dt)` so it can be unit-tested without a running `App`, mirroring the arm IK
//! test style in `character/ik.rs`.
//!
//! # Coordinate convention
//! Bevy world coordinates: **x right, y up**. The flat floor is the horizontal
//! line `y = floor_y`. The hip travels along +x when the player pushes right.

use bevy::math::Vec2;

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

/// Runtime-tuneable knobs for the lower-body resolver.
///
/// All fields are editable through the sandbox egui window. The leg-length
/// defaults are seeded from the v10 blueprint (`thigh`/`calf` ≈ 55 px after the
/// character root's `0.5` world scale in the tuning example), but the math is
/// fully parametric and does not depend on that blueprint.
#[derive(Debug, Clone)]
pub struct LowerBodyConfig {
    /// Thigh (hip→knee) length, px.
    pub thigh_len: f32,
    /// Calf (knee→foot) length, px.
    pub calf_len: f32,
    /// Standing hip height above the floor, px.
    pub hip_height: f32,
    /// Crouched hip height above the floor, px (drives `X_max` down).
    pub crouch_height: f32,
    /// Target cruise speed magnitude, px/s (`v_desired`).
    pub target_speed: f32,
    /// Horizontal acceleration toward `v_desired`, px/s².
    pub accel: f32,
    /// Stance duration `Ts` (seconds): how long a foot stays pinned.
    pub stance_duration_ts: f32,
    /// Swing duration (seconds): how long a foot is in the air.
    pub swing_duration_ts: f32,
    /// Raibert proportional velocity-feedback gain (units: **seconds**). See
    /// the derivation in `plans/Raibert.md`.
    pub kv: f32,
    /// Backward-solve blend gain (0..1]. `1.0` = deadbeat: at touchdown the hip
    /// velocity fully adopts the support velocity inverted from the actual
    /// plant offset. Lower values damp the correction, which is what the XPBD
    /// integration will need once feet lag their kinematic targets.
    pub beta: f32,
    /// Fraction of the geometric leg reach `X_max` that is actually usable
    /// (0..1). Lower = more conservative steps.
    pub safety_margin: f32,
    /// Fraction of `X_max` (0..1) reserved for prediction error. The planner
    /// keeps `½·a·T_swing² ≤ prediction_margin·X_max`, which bounds the hip
    /// acceleration at `a_max` (see [`GaitLimits`]).
    pub prediction_margin: f32,
    /// Swing arc apex height above the plant/floor line, px.
    pub step_height: f32,
    /// World y of the flat floor line.
    pub floor_y: f32,
    /// Knee bend direction: +1 = knee in front of the hip→foot line, -1 = behind.
    pub bend_sign: f32,
    /// `true` while crouching (squash height).
    pub crouch: bool,
}

impl Default for LowerBodyConfig {
    fn default() -> Self {
        Self {
            thigh_len: 55.0,
            calf_len: 55.0,
            hip_height: 90.0,
            crouch_height: 55.0,
            target_speed: 120.0,
            accel: 800.0,
            stance_duration_ts: 0.28,
            swing_duration_ts: 0.22,
            kv: 0.12,
            beta: 1.0,
            safety_margin: 0.85,
            prediction_margin: 0.5,
            step_height: 28.0,
            floor_y: 0.0,
            bend_sign: 1.0,
            crouch: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Gait phase
// ---------------------------------------------------------------------------

/// Discrete phase of a single leg.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GaitPhase {
    /// Foot is pinned to the floor.
    Stance,
    /// Foot is travelling through the air toward its next plant.
    Swing,
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Per-leg persistent state.
///
/// `foot_pos` is the current (output) foot position; `plant_pos` is the
/// anchored world position the foot holds while in stance (so a moving hip
/// leaves the stance foot pinned, stretching the leg through IK).
#[derive(Debug, Clone, Copy)]
pub struct LegState {
    pub phase: GaitPhase,
    /// Seconds remaining in the current phase.
    pub phase_timer: f32,
    /// Current foot world position (output cache).
    pub foot_pos: Vec2,
    /// Anchored foot world position used while in stance.
    pub plant_pos: Vec2,
    /// Swing lift-off position.
    pub swing_from: Vec2,
    /// Swing plant target.
    pub swing_to: Vec2,
}

/// The resolver's full persistent state.
#[derive(Debug, Clone)]
pub struct ResolverState {
    pub hip_pos: Vec2,
    pub hip_vel: Vec2,
    /// Current (possibly interpolated) hip height `H`.
    pub hip_height: f32,
    pub left: LegState,
    pub right: LegState,
}

impl ResolverState {
    /// Build a resting, symmetric stance: hip centred, both feet planted either
    /// side of the hip at a reachable offset.
    pub fn new(config: &LowerBodyConfig) -> Self {
        let hip = Vec2::new(0.0, config.floor_y + config.hip_height);
        let spread = 30.0_f32
            .min(config.thigh_len + config.calf_len - 5.0)
            .max(5.0);
        let left_plant = Vec2::new(hip.x - spread, config.floor_y);
        let right_plant = Vec2::new(hip.x + spread, config.floor_y);

        let left = LegState {
            phase: GaitPhase::Stance,
            phase_timer: config.stance_duration_ts,
            foot_pos: left_plant,
            plant_pos: left_plant,
            swing_from: left_plant,
            swing_to: left_plant,
        };
        // Right leg starts half a stance out of phase with the left, so the two
        // legs alternate once the gait begins; both stay planted while standing
        // still (no idle stance→swing hop).
        let right = LegState {
            phase: GaitPhase::Stance,
            phase_timer: config.stance_duration_ts * 0.5,
            foot_pos: right_plant,
            plant_pos: right_plant,
            swing_from: right_plant,
            swing_to: right_plant,
        };

        Self {
            hip_pos: hip,
            hip_vel: Vec2::ZERO,
            hip_height: config.hip_height,
            left,
            right,
        }
    }
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

/// The resolved particle positions: 1 hip + 2 knees + 2 feet.
#[derive(Debug, Clone, Copy)]
pub struct LowerBodyPose {
    pub hip: Vec2,
    pub left_knee: Vec2,
    pub left_foot: Vec2,
    pub right_knee: Vec2,
    pub right_foot: Vec2,
}

/// Telemetry for the UI / debugging; the forward/backward math intermediates.
#[derive(Debug, Clone, Copy)]
pub struct DebugInfo {
    /// Geometric max reach at the current hip height (`X_max`).
    pub x_max: f32,
    /// Velocity clamp ceiling actually applied this tick (`v_ceil`).
    pub v_max: f32,
    /// Desired horizontal speed (`v_desired`), signed, after the plan cap.
    pub v_desired: f32,
    /// Clamped horizontal velocity actually applied this tick.
    pub v_clamped: f32,
    pub left_phase: GaitPhase,
    pub right_phase: GaitPhase,
    /// Forward step target x for each leg (world x), the last time each swung.
    pub left_target_x: f32,
    pub right_target_x: f32,
    /// Max feasible hip acceleration from the prediction-error budget.
    pub a_max: f32,
    /// Absolute speed ceiling from the feasibility envelope.
    pub v_abs: f32,
    /// Backward-solved support velocity at the latest touchdown this tick
    /// (0 when no touchdown event fired).
    pub v_support: f32,
    /// Signed capture-point offset from the hip while braking to a stop
    /// (0 when not stopping).
    pub capture_x: f32,
}

// ---------------------------------------------------------------------------
// Math primitives
// ---------------------------------------------------------------------------

/// Move `current` toward `target` by at most `max_delta`, without overshooting.
#[inline]
pub fn approach(current: f32, target: f32, max_delta: f32) -> f32 {
    let d = target - current;
    if d.abs() <= max_delta {
        target
    } else {
        current + d.signum() * max_delta
    }
}

/// Rotate a vector by `angle` radians (CCW in a y-up frame).
#[inline]
pub fn rotate(v: Vec2, angle: f32) -> Vec2 {
    let (sin, cos) = angle.sin_cos();
    Vec2::new(v.x * cos - v.y * sin, v.x * sin + v.y * cos)
}

/// The backward velocity clamp. Isolates the maximum horizontal speed the hip
/// may have at the current height without outrunning its own feet.
///
/// `X_max` is the (safety-margined) geometric reach at height `H`:
/// `sqrt((L1 + L2)^2 - H^2) * margin`.
#[inline]
pub fn compute_x_max(thigh: f32, calf: f32, hip_height: f32, margin: f32) -> f32 {
    let full = thigh + calf;
    let inner = full * full - hip_height * hip_height;
    if inner <= 0.0 {
        0.0
    } else {
        inner.sqrt() * margin
    }
}

/// Corrected cruise ceiling:
/// `v_max = (X_max + kv * v_desired_mag) / (Ts/2 + kv + T_swing)`.
///
/// The `T_swing` term accounts for the hip's travel while the foot is airborne:
/// a plant aimed `x_target` ahead of the *touchdown* hip sits
/// `x_target + v·T_swing` ahead of the *lift-off* hip. The old denominator
/// omitted it, so at speed the hip planted beyond its own reach and the IK
/// ray-clamp lifted the stance foot off the floor (the "floating foot" bug).
///
/// `v_desired_mag` is the magnitude of the desired cruise speed (non-negative);
/// the result is a symmetric positive speed ceiling for both directions.
#[inline]
pub fn compute_v_max(
    x_max: f32,
    kv: f32,
    stance_ts: f32,
    swing_ts: f32,
    v_desired_mag: f32,
) -> f32 {
    let denom = stance_ts * 0.5 + kv + swing_ts;
    if denom <= f32::EPSILON {
        return 0.0;
    }
    (x_max + kv * v_desired_mag) / denom
}

/// The generalized backward solve (foot → hip). Given the signed offset
/// `delta_x` from the hip **at touchdown** to the support foot, returns the hip
/// velocity whose forward plan would land exactly on that offset:
///
/// `v_support = (delta_x + kv * v_desired) / (Ts/2 + kv)`.
///
/// [`compute_v_max`] (old form) is this evaluated at `delta_x = X_max`.
/// Composing backward∘forward is the identity: a plant error `e` maps to a
/// velocity correction `e / (Ts/2 + kv)` that the next stride annihilates
/// (deadbeat, one-stride convergence). `v_desired` is *signed*.
#[inline]
pub fn compute_support_velocity(delta_x: f32, kv: f32, stance_ts: f32, v_desired: f32) -> f32 {
    let denom = stance_ts * 0.5 + kv;
    if denom <= f32::EPSILON {
        return 0.0;
    }
    (delta_x + kv * v_desired) / denom
}

/// Precomputed feasibility envelope for the gait at a given hip height.
///
/// All four limits are closed-form in the config, so the sandbox can clamp the
/// planner *before* it produces unreachable targets instead of reacting to IK
/// clamping after the fact. Derivations live in `plans/raibert_redesign.md`.
#[derive(Debug, Clone, Copy)]
pub struct GaitLimits {
    /// Safety-margined geometric reach: `sqrt((L1+L2)² − H²) · margin`.
    pub x_max: f32,
    /// Max hip acceleration whose swing-phase prediction error `½·a·T_swing²`
    /// still fits inside `prediction_margin · x_max`.
    pub a_max: f32,
    /// Absolute speed ceiling: at this speed the prediction error budget alone
    /// saturates the reach, and the stop decel floor `v²/(2·X_max)` equals
    /// `a_max` exactly.
    pub v_abs: f32,
    /// Cruise ceiling from the corrected backward solve (includes the
    /// `T_swing` flight term so plants stay in reach at steady state).
    pub v_cruise_max: f32,
    /// Cap on the *desired* speed fed to the planner: the self-consistent
    /// cruise ceiling `X_max / (T_swing + Ts/2)` where the kv feedback term
    /// vanishes. Requesting more would leave a persistent kv bias
    /// `kv·(v − v_d)` that pushes late-stance feet out of reach.
    pub v_plan_cap: f32,
}

impl GaitLimits {
    /// Evaluate the envelope for `config` at hip height `hip_height`, with
    /// desired cruise speed magnitude `v_desired_mag`.
    pub fn from_config(config: &LowerBodyConfig, hip_height: f32, v_desired_mag: f32) -> Self {
        let x_max = compute_x_max(
            config.thigh_len,
            config.calf_len,
            hip_height,
            config.safety_margin,
        );
        let t_swing = config.swing_duration_ts.max(f32::EPSILON);
        let m_e = config.prediction_margin.clamp(0.05, 1.0);
        // ½·a·T_swing² ≤ m_e·X_max  ⇒  a_max = 2·m_e·X_max / T_swing²
        let a_max = 2.0 * m_e * x_max / (t_swing * t_swing);
        // v_abs: speed at which the stop floor v²/(2·X_max) equals a_max.
        let v_abs = 2.0 * x_max * m_e.sqrt() / t_swing;
        let v_plan_cap = x_max / (t_swing + config.stance_duration_ts * 0.5);
        let v_d_eff = v_desired_mag.min(v_plan_cap);
        let v_cruise_max = compute_v_max(
            x_max,
            config.kv,
            config.stance_duration_ts,
            config.swing_duration_ts,
            v_d_eff,
        );
        Self {
            x_max,
            a_max,
            v_abs,
            v_cruise_max,
            v_plan_cap,
        }
    }
}

/// The forward step planner: horizontal distance from the (moving) hip to the
/// next foot plant.
///
/// `x_target = (v * Ts) / 2 + kv * (v - v_desired)`.
///
/// `v` is the clamped horizontal hip velocity, `v_desired` the *signed* desired
/// cruise velocity (positive = moving +x).
#[inline]
pub fn compute_forward_target(v: f32, v_desired: f32, kv: f32, stance_ts: f32) -> f32 {
    (v * stance_ts) * 0.5 + kv * (v - v_desired)
}

/// Law-of-cosines 2-bone IK: returns `(knee, foot)` for the leg reaching from
/// `hip` toward `foot_target` given `thigh_len` and `calf_len`.
///
/// `bend_sign` selects which side of the hip→foot line the knee sits on (+1 or
/// -1).
///
/// The returned `foot` is the **effective** foot position: when `foot_target`
/// is out of reach it is clamped to the nearest reachable point along the
/// hip→target ray, so `|knee - hip| == thigh_len` and `|foot - knee| ==
/// calf_len` always hold exactly. Callers must render/use this effective foot,
/// otherwise the leg bones visually stretch. If the hip and target coincide, a
/// horizontal default direction is used.
pub fn solve_leg_ik(
    hip: Vec2,
    foot_target: Vec2,
    thigh_len: f32,
    calf_len: f32,
    bend_sign: f32,
) -> (Vec2, Vec2) {
    let to = foot_target - hip;
    let dist = to.length();
    let dmax = thigh_len + calf_len;
    let dmin = (thigh_len - calf_len).abs();

    // Clamp the reach into [dmin + eps, dmax - eps] so the acos argument is
    // always well-conditioned.
    let eps = 1e-3;
    let lo = (dmin + eps).max(eps);
    let hi = (dmax - eps).max(lo + eps);
    let d = dist.clamp(lo, hi);

    let dir = if dist > f32::EPSILON {
        to / dist
    } else {
        Vec2::X
    };

    let cos_a = ((thigh_len * thigh_len + d * d - calf_len * calf_len) / (2.0 * thigh_len * d))
        .clamp(-1.0, 1.0);
    let a = cos_a.acos();

    let thigh_dir = rotate(dir, a * bend_sign);
    let knee = hip + thigh_dir * thigh_len;
    // The effective foot is the clamped point along the hip→target ray; when
    // the target is reachable this is the target itself.
    let foot = hip + dir * d;
    (knee, foot)
}

// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------

/// Minimum `|v_desired|` (move intent, px/s) required to initiate a step. With
/// no intent the gait freezes: both feet stay planted and the stop-settle servo
/// in [`step_lower_body`] glides the hip to the support centre. Gating on
/// *intent* rather than actual velocity also prevents the settle motion itself
/// from triggering phantom steps that would walk the character off with no
/// input held.
const SWING_TRIGGER_SPEED: f32 = 1.0;

/// Per-tick planning context shared by both legs.
#[derive(Debug, Clone, Copy)]
struct StepPlan {
    /// Safety-margined reach; swing targets are clamped to `hip ± x_max` so a
    /// plant can never be aimed outside the workspace (which the IK would
    /// ray-clamp into a foot floating off the floor).
    x_max: f32,
    /// True while braking to a stop: no move intent but the hip is still
    /// travelling faster than the swing trigger.
    stopping: bool,
    /// Capture-point offset from the *current* hip while `stopping`:
    /// `v·|v| / (2·a_eff)`. Aimed at the point where the hip will come to rest
    /// under the effective deceleration, so the hip brakes *onto* its foot.
    capture_offset: f32,
}

/// World-x swing target for a foot, given `t_flight` seconds remaining in the
/// air. Used both at lift-off (`t_flight = T_swing`) and by the per-frame
/// re-planning during swing.
///
/// While stopping, targets the capture point instead of the forward plan.
/// The result is always clamped to `hip ± x_max`.
fn plan_target_x(
    hip_x: f32,
    v: f32,
    v_desired: f32,
    t_flight: f32,
    config: &LowerBodyConfig,
    plan: &StepPlan,
) -> f32 {
    let raw = if plan.stopping {
        hip_x + plan.capture_offset
    } else {
        let x_target = compute_forward_target(v, v_desired, config.kv, config.stance_duration_ts);
        // Raibert's `x_target` is the offset from the hip **at touchdown**, not
        // at lift-off: the hip keeps travelling while the foot is airborne, so
        // predict that motion. Re-planning every frame with the shrinking
        // `t_flight` keeps the world-space target stationary at constant
        // velocity (hip advance `v·dt` cancels the prediction shrink) and makes
        // the target track acceleration/braking transients mid-swing.
        hip_x + v * t_flight + x_target
    };
    raw.clamp(hip_x - plan.x_max, hip_x + plan.x_max)
}

/// Advance a single leg through the stance → swing → plant state machine.
///
/// Updates `leg` in place. `v_desired` is the *signed* desired cruise velocity
/// (from the current input, not inferred from `v`), so at standstill it is zero
/// and the foot plants straight under the hip rather than behind it.
///
/// `other_swinging` is true while the opposite leg is airborne. A stance leg may
/// only lift off when the opposite foot is planted, which enforces a strict
/// anti-phase alternating gait: at most one foot is airborne at any instant, so
/// the two legs can never swing into each other.
///
/// Returns `Some(delta_x)` on the tick the foot touches down, where `delta_x`
/// is the signed offset from the hip to the new support foot — the event that
/// drives the backward solve in [`step_lower_body`].
#[allow(clippy::too_many_arguments)]
fn step_leg(
    leg: &mut LegState,
    hip_x: f32,
    v: f32,
    v_desired: f32,
    other_swinging: bool,
    config: &LowerBodyConfig,
    plan: &StepPlan,
    dt: f32,
) -> Option<f32> {
    match leg.phase {
        GaitPhase::Stance => {
            // The stance foot is pinned in world space; the hip moves away and
            // the leg stretches through the IK solve.
            leg.foot_pos = leg.plant_pos;
            leg.phase_timer -= dt;
            // Step while there is move *intent*, or while braking to a stop
            // (the capture-point plan needs a foot to land on). With neither,
            // the gait freezes and the hip settles over the planted feet (see
            // the stop-settle servo in `step_lower_body`). Steps also require
            // the opposite foot to be planted (anti-phase swap).
            let wants_step = v_desired.abs() > SWING_TRIGGER_SPEED
                || (plan.stopping && v.abs() > SWING_TRIGGER_SPEED);
            if leg.phase_timer <= 0.0 && wants_step && !other_swinging {
                let mut swing_to = leg.plant_pos;
                swing_to.x =
                    plan_target_x(hip_x, v, v_desired, config.swing_duration_ts, config, plan);
                swing_to.y = config.floor_y;
                leg.phase = GaitPhase::Swing;
                leg.phase_timer = config.swing_duration_ts;
                leg.swing_from = leg.plant_pos;
                leg.swing_to = swing_to;
            }
            None
        }
        GaitPhase::Swing => {
            leg.phase_timer -= dt;
            // Continuous re-planning: refresh the target from the current hip
            // position, velocity and remaining flight time. Open-loop targets
            // frozen at lift-off are what made sudden stops land both feet on
            // one side of the hip.
            leg.swing_to.x =
                plan_target_x(hip_x, v, v_desired, leg.phase_timer.max(0.0), config, plan);
            // Progress 1→0 over the swing duration (elapsed fraction).
            let total = config.swing_duration_ts.max(f32::EPSILON);
            let t = (1.0 - (leg.phase_timer / total).clamp(0.0, 1.0)).clamp(0.0, 1.0);

            let x = leg.swing_from.x + (leg.swing_to.x - leg.swing_from.x) * t;
            let y = leg.swing_from.y + (leg.swing_to.y - leg.swing_from.y) * t;
            let lift = (t * (1.0 - t) * 4.0) * config.step_height;
            leg.foot_pos = Vec2::new(x, y + lift);

            if leg.phase_timer <= 0.0 {
                leg.foot_pos = leg.swing_to;
                leg.plant_pos = leg.swing_to;
                leg.phase = GaitPhase::Stance;
                leg.phase_timer = config.stance_duration_ts;
                // Touchdown event: signed hip→foot offset for the backward solve.
                return Some(leg.plant_pos.x - hip_x);
            }
            None
        }
    }
}

/// Run one full resolver tick.
///
/// # Pipeline (mirrors `plans/raibert_redesign.md`)
/// 1. Read target hip height (normal vs crouch).
/// 2. Precomputed feasibility envelope ([`GaitLimits`]) at the current `H`.
/// 3. Velocity shaping: accel clamped to `a_max`, stop decel floor
///    `v²/(2·X_max)`, ceiling `min(v_abs, v_cruise_max)`.
/// 4. Stop-settle servo once braking has bled the speed off.
/// 5. Forward math with per-frame re-planning + capture-point stop; targets
///    clamped to `hip ± X_max`.
/// 6. Backward solve at each touchdown event, blended with gain `beta`.
/// 7. Flat floor ⇒ plant y is `floor_y`; parabolic lift during swing.
/// 8. 2-bone IK for each leg → knees/feet.
pub fn step_lower_body(
    input_dir: Vec2,
    config: &LowerBodyConfig,
    state: &mut ResolverState,
    dt: f32,
) -> (LowerBodyPose, DebugInfo) {
    let dir = input_dir.normalize_or_zero();

    // 1. --- Hip height ---
    let h_target = if config.crouch {
        config.crouch_height
    } else {
        config.hip_height
    };
    state.hip_height = approach(state.hip_height, h_target, config.hip_height * dt * 3.0);
    let h = state.hip_height;

    // 2. --- Feasibility envelope at the current height ---
    let limits = GaitLimits::from_config(config, h, config.target_speed);
    let x_max = limits.x_max;
    // Cap the desired speed at the self-consistent cruise ceiling so the kv
    // feedback term can never develop a persistent bias larger than the
    // workspace (which would push late-stance feet out of reach).
    let v_desired_scalar = dir.x * config.target_speed.min(limits.v_plan_cap);

    // 3. --- Velocity shaping ---
    // While braking to a stop, raise the deceleration to at least the stop
    // floor `v²/(2·X_max)`: this guarantees the capture point stays within
    // reach, so the hip can always brake onto its next foot instead of
    // overshooting both plants.
    let v_prev = state.hip_vel.x;
    let stopping = v_desired_scalar == 0.0 && v_prev.abs() > SWING_TRIGGER_SPEED;
    let a_cmd = config.accel.min(limits.a_max).max(f32::EPSILON);
    let a_eff = if stopping {
        let a_stop_floor = v_prev * v_prev / (2.0 * x_max.max(f32::EPSILON));
        a_cmd.max(a_stop_floor)
    } else {
        a_cmd
    };
    // Ceiling: while braking, allow up to the absolute envelope (the hip may be
    // faster than cruise from external pushes); under power, cap at the
    // corrected cruise ceiling so plants stay in reach. Releasing input at
    // speed therefore never snaps the velocity.
    let v_ceil = if v_desired_scalar == 0.0 {
        limits.v_abs
    } else {
        limits.v_abs.min(limits.v_cruise_max)
    };
    let mut v = approach(v_prev, v_desired_scalar, a_eff * dt).clamp(-v_ceil, v_ceil);

    // 4. --- Stop settle: the feet own the hip ---
    // Once braking has bled the speed off and both feet are planted, glide the
    // hip to the support centre (midpoint of the planted feet) so the
    // character comes to rest balanced between them. The settle velocity is
    // capped by the kinematic braking limit `sqrt(2·a·d)` so the hip
    // decelerates smoothly into the midpoint without overshooting.
    if v_desired_scalar == 0.0
        && !stopping
        && state.left.phase == GaitPhase::Stance
        && state.right.phase == GaitPhase::Stance
    {
        let mid_x = (state.left.plant_pos.x + state.right.plant_pos.x) * 0.5;
        let to_mid = mid_x - state.hip_pos.x;
        if to_mid.abs() > 0.5 {
            let settle_time = config.stance_duration_ts.max(f32::EPSILON);
            let brake_limit = (2.0 * a_cmd * to_mid.abs()).sqrt();
            let v_settle = (to_mid / settle_time).clamp(-brake_limit, brake_limit);
            v = approach(v, v_settle, a_cmd * dt);
        } else {
            v = approach(v, 0.0, a_cmd * dt);
        }
    }

    state.hip_vel.x = v;
    // Kinematic sandbox: no vertical hip dynamics.
    state.hip_vel.y = 0.0;

    // Integrate hip position. Height is pinned to `floor_y + h`.
    state.hip_pos.x += v * dt;
    state.hip_pos.y = config.floor_y + h;

    // 4c. --- Stance constraint: the hip may never outrun its pinned feet ---
    // A stance foot is pinned in world space. If the hip travels beyond the
    // (margined) horizontal reach of a planted foot — which the anti-phase
    // gate can provoke during hard transients, when the rear foot's lift-off
    // is delayed until the swinging foot plants — the IK ray-clamp would lift
    // that foot off the floor. Clamp the hip into the intersection of the
    // stance feet's windows `[plant ± X_max]`; the position stays continuous
    // and the constraint releases as soon as the trailing foot re-plants.
    {
        let mut lo = f32::NEG_INFINITY;
        let mut hi = f32::INFINITY;
        for leg in [&state.left, &state.right] {
            if leg.phase == GaitPhase::Stance {
                lo = lo.max(leg.plant_pos.x - x_max);
                hi = hi.min(leg.plant_pos.x + x_max);
            }
        }
        if lo <= hi {
            state.hip_pos.x = state.hip_pos.x.clamp(lo, hi);
        }
    }

    // 5. --- Capture point + per-leg foot placement ---
    // While stopping, aim the swing foot at the point where the hip will come
    // to rest under `a_eff` (Pratt capture point), so the hip brakes onto its
    // foot rather than past it.
    let capture_offset = if stopping {
        v * v.abs() / (2.0 * a_eff)
    } else {
        0.0
    };
    let plan = StepPlan {
        x_max,
        stopping,
        capture_offset,
    };
    let left_td = step_leg(
        &mut state.left,
        state.hip_pos.x,
        v,
        v_desired_scalar,
        state.right.phase == GaitPhase::Swing,
        config,
        &plan,
        dt,
    );
    let right_td = step_leg(
        &mut state.right,
        state.hip_pos.x,
        v,
        v_desired_scalar,
        state.left.phase == GaitPhase::Swing,
        config,
        &plan,
        dt,
    );

    // 6. --- Backward solve at touchdown (foot → hip) ---
    // Invert the *actual* plant offset to get the hip velocity whose next
    // forward plan lands exactly on it, then blend with gain `beta` (1 =
    // deadbeat). Skipped while stopping: capture plants encode "brake to rest
    // here", which the cruise inversion would misread as a velocity command.
    let mut v_support = 0.0;
    if v_desired_scalar != 0.0 {
        if let Some(delta) = left_td.or(right_td) {
            v_support = compute_support_velocity(
                delta,
                config.kv,
                config.stance_duration_ts,
                v_desired_scalar,
            );
            let beta = config.beta.clamp(0.0, 1.0);
            v = (v + beta * (v_support - v)).clamp(-v_ceil, v_ceil);
            state.hip_vel.x = v;
        }
    }

    // 7-8. --- IK ---

    // IK returns the *effective* foot (clamped to the reachable annulus), so the
    // rendered bones always satisfy thigh/calf length even when the hip briefly
    // outruns a pinned stance foot.
    let (left_knee, left_foot) = solve_leg_ik(
        state.hip_pos,
        state.left.foot_pos,
        config.thigh_len,
        config.calf_len,
        config.bend_sign,
    );
    let (right_knee, right_foot) = solve_leg_ik(
        state.hip_pos,
        state.right.foot_pos,
        config.thigh_len,
        config.calf_len,
        config.bend_sign,
    );

    let pose = LowerBodyPose {
        hip: state.hip_pos,
        left_knee,
        right_knee,
        left_foot,
        right_foot,
    };

    let debug = DebugInfo {
        x_max,
        v_max: v_ceil,
        v_desired: v_desired_scalar,
        v_clamped: v,
        left_phase: state.left.phase,
        right_phase: state.right.phase,
        left_target_x: state.left.swing_to.x,
        right_target_x: state.right.swing_to.x,
        a_max: limits.a_max,
        v_abs: limits.v_abs,
        v_support,
        capture_x: capture_offset,
    };

    (pose, debug)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> LowerBodyConfig {
        LowerBodyConfig::default()
    }

    #[test]
    fn test_compute_x_max_pythagorean() {
        // (L1 + L2) = 110, H = 90 → sqrt(110² - 90²) = sqrt(4000) ≈ 66.33
        let cfg = default_config();
        let raw = compute_x_max(cfg.thigh_len, cfg.calf_len, 90.0, 1.0);
        let expected = (110.0f32 * 110.0 - 90.0 * 90.0).sqrt();
        assert!(
            (raw - expected).abs() < 1e-3,
            "raw={raw} expected={expected}"
        );
        // margin scales X_max linearly.
        let margined = compute_x_max(cfg.thigh_len, cfg.calf_len, 90.0, 0.5);
        assert!((margined - expected * 0.5).abs() < 1e-3);
        // Impossibly high hip → clamped to zero reach.
        assert_eq!(compute_x_max(50.0, 50.0, 200.0, 1.0), 0.0);
    }

    /// Non-stopping plan context for the direct `step_leg` unit tests.
    fn cruise_plan(cfg: &LowerBodyConfig) -> StepPlan {
        StepPlan {
            x_max: compute_x_max(
                cfg.thigh_len,
                cfg.calf_len,
                cfg.hip_height,
                cfg.safety_margin,
            ),
            stopping: false,
            capture_offset: 0.0,
        }
    }

    #[test]
    fn test_compute_v_max_formula() {
        // v_max = (X_max + kv·v_desired) / (Ts/2 + kv + T_swing)
        let v_max = compute_v_max(66.33, 0.12, 0.28, 0.22, 120.0);
        let expected = (66.33 + 0.12 * 120.0) / (0.28 * 0.5 + 0.12 + 0.22);
        assert!((v_max - expected).abs() < 1e-3);
        // Guard against degenerate denominator.
        assert_eq!(compute_v_max(50.0, 0.0, 0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn test_gait_limits_envelope() {
        let cfg = default_config();
        let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
        // Hand-computed defaults: X_max = √(110²−90²)·0.85 ≈ 53.76
        let x_max = (110.0f32 * 110.0 - 90.0 * 90.0).sqrt() * 0.85;
        assert!(
            (limits.x_max - x_max).abs() < 1e-2,
            "x_max={}",
            limits.x_max
        );
        // a_max = 2·m_e·X_max / T_swing²
        let a_max = 2.0 * cfg.prediction_margin * x_max / (cfg.swing_duration_ts.powi(2));
        assert!(
            (limits.a_max - a_max).abs() < 1e-1,
            "a_max={}",
            limits.a_max
        );
        // v_abs = 2·X_max·√m_e / T_swing
        let v_abs = 2.0 * x_max * cfg.prediction_margin.sqrt() / cfg.swing_duration_ts;
        assert!(
            (limits.v_abs - v_abs).abs() < 1e-1,
            "v_abs={}",
            limits.v_abs
        );
        // Consistency: at v_abs the stop decel floor v²/(2·X_max) equals a_max.
        let a_stop_at_v_abs = v_abs * v_abs / (2.0 * x_max);
        assert!(
            (a_stop_at_v_abs - a_max).abs() / a_max < 1e-3,
            "a_stop(v_abs)={a_stop_at_v_abs} != a_max={a_max}"
        );
        // Degenerate height (hip above full reach) collapses the envelope.
        let degen = GaitLimits::from_config(&cfg, 200.0, cfg.target_speed);
        assert_eq!(degen.x_max, 0.0);
        assert_eq!(degen.a_max, 0.0);
        assert_eq!(degen.v_abs, 0.0);
    }

    #[test]
    fn test_backward_forward_identity() {
        // backward∘forward = identity: the support velocity inverted from a
        // forward-planned offset recovers the original velocity exactly.
        let cfg = default_config();
        for v in [-200.0, -50.0, 0.0, 37.5, 120.0, 300.0] {
            for v_d in [-120.0, 0.0, 120.0] {
                let delta = compute_forward_target(v, v_d, cfg.kv, cfg.stance_duration_ts);
                let v_back = compute_support_velocity(delta, cfg.kv, cfg.stance_duration_ts, v_d);
                assert!(
                    (v_back - v).abs() < 1e-2,
                    "v={v} v_d={v_d} → v_back={v_back}"
                );
            }
        }
        // Degenerate denominator guard.
        assert_eq!(compute_support_velocity(10.0, 0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn test_support_velocity_corrects_plant_error() {
        // A plant error `e` at touchdown maps to the velocity correction
        // `e / (Ts/2 + kv)`, annihilated in one stride (deadbeat).
        let cfg = default_config();
        let v = cfg.target_speed;
        let e = 7.5;
        let delta = v * cfg.stance_duration_ts * 0.5 + e;
        let v_support = compute_support_velocity(delta, cfg.kv, cfg.stance_duration_ts, v);
        let expected = v + e / (cfg.stance_duration_ts * 0.5 + cfg.kv);
        assert!(
            (v_support - expected).abs() < 1e-3,
            "v_support={v_support} expected={expected}"
        );
    }

    #[test]
    fn test_compute_forward_target_neutral_point() {
        // At v == v_desired, x_target = v·Ts/2 (the neutral stance midpoint).
        let cfg = default_config();
        let v = 120.0;
        let x = compute_forward_target(v, v, cfg.kv, cfg.stance_duration_ts);
        assert!((x - v * cfg.stance_duration_ts * 0.5).abs() < 1e-3);
        // At v == 0, x_target = -kv·v_desired (foot steps behind the hip).
        let stop = compute_forward_target(0.0, 120.0, cfg.kv, cfg.stance_duration_ts);
        assert!((stop - (-cfg.kv * 120.0)).abs() < 1e-3);
    }

    #[test]
    fn test_landing_compensates_hip_motion_during_swing() {
        // Raibert neutral point: the foot must land `v·Ts/2` ahead of the hip
        // *at touchdown*, so the hip passes over the foot at mid-stance. This
        // requires predicting the hip's travel during the swing phase.
        let cfg = default_config();
        let v = cfg.target_speed; // cruise: kv term vanishes
        let mut leg = LegState {
            phase: GaitPhase::Stance,
            phase_timer: 0.0,
            foot_pos: Vec2::ZERO,
            plant_pos: Vec2::ZERO,
            swing_from: Vec2::ZERO,
            swing_to: Vec2::ZERO,
        };

        // Lift off with the hip at x = 0.
        let plan = cruise_plan(&cfg);
        let _ = step_leg(&mut leg, 0.0, v, v, false, &cfg, &plan, 0.001);
        assert_eq!(leg.phase, GaitPhase::Swing);

        // Fly through the swing while the hip keeps moving.
        let dt = 0.01;
        let mut hip_x = 0.0;
        let mut touchdown = None;
        for _ in 0..((cfg.swing_duration_ts / dt).ceil() as usize + 2) {
            hip_x += v * dt;
            touchdown = step_leg(&mut leg, hip_x, v, v, false, &cfg, &plan, dt);
            if leg.phase == GaitPhase::Stance {
                break;
            }
        }
        assert_eq!(leg.phase, GaitPhase::Stance, "swing never completed");

        let offset = leg.plant_pos.x - hip_x;
        assert!(
            offset > 0.0,
            "foot landed behind the hip: offset={offset:.1}px"
        );
        let expected = v * cfg.stance_duration_ts * 0.5;
        assert!(
            (offset - expected).abs() < v * dt * 2.0,
            "offset={offset:.1}px expected≈{expected:.1}px"
        );
        // The touchdown event reports the same signed hip→foot offset.
        assert!(
            touchdown.is_some_and(|d| (d - offset).abs() < 1e-3),
            "touchdown event missing or wrong: {touchdown:?} vs {offset}"
        );
    }

    #[test]
    fn test_gait_alternates_phases() {
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        // Both legs begin planted (no idle hop).
        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);

        // Right leg is half a stance out of phase: after half a stance of
        // forward motion it swings while the left is still in stance.
        let dt = cfg.stance_duration_ts * 0.5;
        let plan = cruise_plan(&cfg);
        let _ = step_leg(
            &mut state.left,
            state.hip_pos.x,
            40.0,
            120.0,
            state.right.phase == GaitPhase::Swing,
            &cfg,
            &plan,
            dt,
        );
        let _ = step_leg(
            &mut state.right,
            state.hip_pos.x,
            40.0,
            120.0,
            state.left.phase == GaitPhase::Swing,
            &cfg,
            &plan,
            dt,
        );
        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Swing);
    }

    #[test]
    fn test_no_step_when_stationary() {
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        // At standstill (v = 0, v_desired = 0) the stance never opens a swing.
        let dt = cfg.stance_duration_ts + cfg.swing_duration_ts + 0.1;
        let plan = cruise_plan(&cfg);
        let _ = step_leg(
            &mut state.left,
            state.hip_pos.x,
            0.0,
            0.0,
            state.right.phase == GaitPhase::Swing,
            &cfg,
            &plan,
            dt,
        );
        let _ = step_leg(
            &mut state.right,
            state.hip_pos.x,
            0.0,
            0.0,
            state.left.phase == GaitPhase::Swing,
            &cfg,
            &plan,
            dt,
        );
        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);
    }

    #[test]
    fn test_stance_foot_holds_position() {
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let planted = state.left.plant_pos;

        // A stance leg keeps its foot pinned even as the hip moves. Step in
        // small increments so the phase never times out into a swing.
        let dt = cfg.stance_duration_ts * 0.25;
        let plan = cruise_plan(&cfg);
        let mut hip_x = state.hip_pos.x;
        for _ in 0..3 {
            hip_x += 40.0 * dt;
            let _ = step_leg(
                &mut state.left,
                hip_x,
                40.0,
                120.0,
                state.right.phase == GaitPhase::Swing,
                &cfg,
                &plan,
                dt,
            );
            assert!(
                (state.left.foot_pos - planted).length() < 1e-3,
                "stance foot drifted: {:?} vs {:?}",
                state.left.foot_pos,
                planted
            );
        }
    }

    #[test]
    fn test_solve_leg_ik_length_preservation() {
        let cfg = default_config();
        let hip = Vec2::new(0.0, cfg.hip_height);

        // In-reach, exactly-at-max-reach, out-of-reach, too-close and coincident
        // targets: bone lengths must hold for the *effective* foot in all cases.
        let targets = [
            Vec2::new(30.0, cfg.floor_y),          // in reach
            Vec2::new(80.0, cfg.floor_y),          // dist ≈ 114 > L1+L2 = 110
            Vec2::new(110.0, cfg.hip_height),      // exactly at max reach
            Vec2::new(200.0, cfg.floor_y + 200.0), // far out of reach
            Vec2::new(0.0, cfg.floor_y),           // straight down (dist = H)
            Vec2::new(0.0, cfg.hip_height),        // coincident with hip
        ];

        for bend in [1.0, -1.0] {
            for target in targets {
                let (knee, foot) = solve_leg_ik(hip, target, cfg.thigh_len, cfg.calf_len, bend);
                let thigh = (knee - hip).length();
                let calf = (foot - knee).length();
                assert!(
                    (thigh - cfg.thigh_len).abs() < 1e-2,
                    "thigh={thigh} bend={bend} target={target:?}"
                );
                assert!(
                    (calf - cfg.calf_len).abs() < 1e-2,
                    "calf={calf} bend={bend} target={target:?}"
                );
                // The effective foot lies on the hip→target ray, never farther
                // than the target itself.
                if (target - hip).length_squared() > 1e-6 {
                    let ray = (target - hip).normalize();
                    let offset = foot - hip;
                    assert!(
                        offset.dot(ray) >= offset.length() - 1e-2,
                        "foot off the hip→target ray: bend={bend} target={target:?}"
                    );
                    assert!(
                        offset.length() <= (target - hip).length() + 1e-3,
                        "foot beyond target: bend={bend} target={target:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_pose_bones_preserve_length_while_walking() {
        // End-to-end invariant: whatever the gait does (accelerate hard, let the
        // hip outrun a pinned stance foot), the *rendered* pose must always have
        // thigh/calf bones of the configured length.
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let input = Vec2::X;
        // Run several full strides at full throttle.
        let frames = ((cfg.stance_duration_ts + cfg.swing_duration_ts) * 60.0 * 4.0) as usize;
        for frame in 0..frames {
            let (pose, _debug) = step_lower_body(input, &cfg, &mut state, dt);
            for (knee, foot, side) in [
                (pose.left_knee, pose.left_foot, "left"),
                (pose.right_knee, pose.right_foot, "right"),
            ] {
                let thigh = (knee - pose.hip).length();
                let calf = (foot - knee).length();
                assert!(
                    (thigh - cfg.thigh_len).abs() < 1e-2,
                    "frame {frame} {side} thigh={thigh}"
                );
                assert!(
                    (calf - cfg.calf_len).abs() < 1e-2,
                    "frame {frame} {side} calf={calf}"
                );
            }
        }
    }

    #[test]
    fn test_hip_stays_between_feet_at_cruise() {
        // The reported symptom: while walking, both feet trailed behind the hip.
        // With the swing-flight prediction, at steady cruise the feet must
        // straddle the hip (one ahead, one behind) for most of the stride.
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let input = Vec2::X;

        // Spin up to cruise speed and past the initial transient (3 full
        // stride cycles).
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let warmup = (stride * 3.0 / dt) as usize;
        for _ in 0..warmup {
            step_lower_body(input, &cfg, &mut state, dt);
        }

        // Measure one more full stride: count frames where the hip is strictly
        // between the two feet.
        let mut straddled = 0usize;
        let total = (stride / dt) as usize;
        for _ in 0..total {
            let (pose, _debug) = step_lower_body(input, &cfg, &mut state, dt);
            let lo = pose.left_foot.x.min(pose.right_foot.x);
            let hi = pose.left_foot.x.max(pose.right_foot.x);
            if pose.hip.x > lo && pose.hip.x < hi {
                straddled += 1;
            }
        }
        let frac = straddled as f32 / total as f32;
        assert!(
            frac >= 0.5,
            "hip only straddled by the feet on {:.0}% of the stride at cruise",
            frac * 100.0
        );
    }

    #[test]
    fn test_sudden_stop_hip_settles_between_feet() {
        // Symptom: on a sudden stop both feet ended up ahead of the hip and
        // the gait froze there. With the stop-settle servo the hip must glide
        // to the support centre and come to rest between the planted feet.
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;

        // Cruise for 3 strides.
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 3.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        let hip_at_stop = state.hip_pos.x;

        // Sudden stop: release input and let it settle.
        for _ in 0..(3.0 / dt) as usize {
            step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
        }

        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);
        let mid = (state.left.plant_pos.x + state.right.plant_pos.x) * 0.5;
        assert!(
            (state.hip_pos.x - mid).abs() < 2.0,
            "hip did not settle between the feet: hip={:.1} mid={mid:.1}",
            state.hip_pos.x
        );
        assert!(
            state.hip_vel.x.abs() < 5.0,
            "hip still sliding after settle: v={}",
            state.hip_vel.x
        );
        // The settle must have actually moved the hip toward the feet (they
        // were planted ahead of it at cruise-stop).
        assert!(
            (state.hip_pos.x - hip_at_stop).abs() > 1.0,
            "hip never moved during settle"
        );
    }

    #[test]
    fn test_no_phantom_steps_after_stop() {
        // With no move intent and the hip fully settled, the gait must freeze:
        // the plants stay put, so the settle servo can never chain into a
        // phantom walk-off. (Braking steps *while* the hip is still moving are
        // legal — they are the capture-point stop — so settle fully first.)
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 3.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        // Brake + settle completely.
        for _ in 0..(3.0 / dt) as usize {
            step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
        }
        assert!(state.hip_vel.x.abs() < SWING_TRIGGER_SPEED);
        let (l_plant, r_plant) = (state.left.plant_pos, state.right.plant_pos);
        for _ in 0..(2.0 / dt) as usize {
            step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
        }
        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);
        assert!(
            (state.left.plant_pos - l_plant).length() < 1e-3,
            "left foot re-planted with no input"
        );
        assert!(
            (state.right.plant_pos - r_plant).length() < 1e-3,
            "right foot re-planted with no input"
        );
    }

    #[test]
    fn test_no_floating_feet_at_max_speed() {
        // Symptom: at high speed the stance foot floated in mid-air because
        // the plant target exceeded the leg reach and the IK ray-clamped it
        // off the floor. With the corrected ceiling + workspace clamp, every
        // stance foot must stay on the floor and within reach at all times.
        let mut cfg = default_config();
        cfg.target_speed = 600.0; // far above the feasible envelope
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let reach = cfg.thigh_len + cfg.calf_len;
        let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        let frames = (stride * 6.0 / dt) as usize;
        let mut v_peak = 0.0f32;
        for frame in 0..frames {
            let (pose, debug) = step_lower_body(Vec2::X, &cfg, &mut state, dt);
            v_peak = v_peak.max(debug.v_clamped.abs());
            for (foot, phase, side) in [
                (pose.left_foot, debug.left_phase, "left"),
                (pose.right_foot, debug.right_phase, "right"),
            ] {
                if phase == GaitPhase::Stance {
                    assert!(
                        (foot.y - cfg.floor_y).abs() < 1e-2,
                        "frame {frame} {side} stance foot off the floor: y={}",
                        foot.y
                    );
                    let d = (foot - pose.hip).length();
                    assert!(
                        d <= reach + 1e-2,
                        "frame {frame} {side} stance foot out of reach: d={d}"
                    );
                }
            }
        }
        // The velocity must have been clamped to the feasible cruise ceiling.
        assert!(
            v_peak <= limits.v_cruise_max + 1e-2,
            "v_peak={v_peak} exceeded cruise ceiling {}",
            limits.v_cruise_max
        );
        assert!(
            v_peak > 0.5 * limits.v_cruise_max,
            "never reached a meaningful speed: v_peak={v_peak}"
        );
    }

    #[test]
    fn test_sudden_stop_from_high_speed() {
        // Symptom: on a sudden stop from speed both feet ended up on one side
        // of the hip. With the capture-point stop + decel floor the hip must
        // brake onto its feet and come to rest between them, and the capture
        // point must never leave the workspace.
        let mut cfg = default_config();
        cfg.target_speed = 350.0;
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;
        for _ in 0..((stride * 4.0) / dt) as usize {
            step_lower_body(Vec2::X, &cfg, &mut state, dt);
        }
        assert!(
            state.hip_vel.x > 100.0,
            "did not reach speed before the stop: v={}",
            state.hip_vel.x
        );

        // Sudden stop: release input, verify the capture point stays in reach
        // while braking, then let it settle.
        let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
        for _ in 0..(4.0 / dt) as usize {
            let (_pose, debug) = step_lower_body(Vec2::ZERO, &cfg, &mut state, dt);
            assert!(
                debug.capture_x.abs() <= limits.x_max + 1e-2,
                "capture point out of reach: {}",
                debug.capture_x
            );
        }

        assert_eq!(state.left.phase, GaitPhase::Stance);
        assert_eq!(state.right.phase, GaitPhase::Stance);
        let mid = (state.left.plant_pos.x + state.right.plant_pos.x) * 0.5;
        assert!(
            (state.hip_pos.x - mid).abs() < 2.0,
            "hip did not settle between the feet: hip={:.1} mid={mid:.1}",
            state.hip_pos.x
        );
        assert!(
            state.hip_vel.x.abs() < 5.0,
            "hip still sliding after settle: v={}",
            state.hip_vel.x
        );
    }

    #[test]
    fn test_velocity_continuity() {
        // The hip velocity must change continuously: bounded by the effective
        // acceleration (command accel clamped to a_max, or the stop decel
        // floor) on every non-touchdown frame. Touchdown frames apply the
        // backward-solve blend and are exempt.
        let cfg = default_config();
        let mut state = ResolverState::new(&cfg);
        let dt = 1.0 / 60.0;
        let limits = GaitLimits::from_config(&cfg, cfg.hip_height, cfg.target_speed);
        let a_bound = cfg.accel.min(limits.a_max);
        let stride = cfg.stance_duration_ts + cfg.swing_duration_ts;

        let mut v_prev = 0.0;
        let frames = (stride * 8.0 / dt) as usize;
        for frame in 0..frames {
            // Cruise, then sudden stop halfway through.
            let input = if frame < frames / 2 {
                Vec2::X
            } else {
                Vec2::ZERO
            };
            let (_pose, debug) = step_lower_body(input, &cfg, &mut state, dt);
            if debug.v_support == 0.0 {
                // Stop decel floor: v²/(2·X_max), evaluated at the frame's
                // starting speed.
                let a_stop = v_prev * v_prev / (2.0 * limits.x_max);
                let dv_max = a_bound.max(a_stop) * dt * 1.05 + 1e-3;
                assert!(
                    (debug.v_clamped - v_prev).abs() <= dv_max,
                    "frame {frame}: velocity jumped {} → {} (dv_max={dv_max})",
                    v_prev,
                    debug.v_clamped
                );
            }
            v_prev = debug.v_clamped;
        }
    }
}
