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
    /// Fraction of the geometric leg reach `X_max` that is actually usable
    /// (0..1). Lower = more conservative steps.
    pub safety_margin: f32,
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
            safety_margin: 0.85,
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
    /// Velocity clamp ceiling (`v_max`).
    pub v_max: f32,
    /// Desired horizontal speed (`v_desired`), signed.
    pub v_desired: f32,
    /// Clamped horizontal velocity actually applied this tick.
    pub v_clamped: f32,
    pub left_phase: GaitPhase,
    pub right_phase: GaitPhase,
    /// Forward step target x for each leg (world x), the last time each swung.
    pub left_target_x: f32,
    pub right_target_x: f32,
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

/// `v_max = (X_max + kv * v_desired_mag) / (Ts/2 + kv)`.
///
/// `v_desired_mag` is the magnitude of the desired cruise speed (non-negative);
/// the result is a symmetric positive speed ceiling for both directions.
#[inline]
pub fn compute_v_max(x_max: f32, kv: f32, stance_ts: f32, v_desired_mag: f32) -> f32 {
    let denom = stance_ts * 0.5 + kv;
    if denom <= f32::EPSILON {
        return 0.0;
    }
    (x_max + kv * v_desired_mag) / denom
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

/// Minimum `|hip velocity|` required to initiate a step (px/s). Below this the
/// character is treated as stationary and both feet stay planted.
const SWING_TRIGGER_SPEED: f32 = 1.0;

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
#[allow(clippy::too_many_arguments)]
fn step_leg(
    leg: &mut LegState,
    hip_x: f32,
    v: f32,
    v_desired: f32,
    other_swinging: bool,
    config: &LowerBodyConfig,
    dt: f32,
) {
    match leg.phase {
        GaitPhase::Stance => {
            // The stance foot is pinned in world space; the hip moves away and
            // the leg stretches through the IK solve.
            leg.foot_pos = leg.plant_pos;
            leg.phase_timer -= dt;
            // Only step while actually moving; at rest the foot stays planted,
            // and only while the opposite foot is planted (anti-phase swap).
            if leg.phase_timer <= 0.0 && v.abs() > SWING_TRIGGER_SPEED && !other_swinging {
                let x_target =
                    compute_forward_target(v, v_desired, config.kv, config.stance_duration_ts);
                // Raibert's `x_target` is the offset from the hip **at touchdown**,
                // not at lift-off: the hip keeps travelling `v * swing_duration`
                // while the foot is airborne, so predict that motion and aim the
                // plant ahead of where the hip *will be* when the foot lands.
                // Without this term the hip outruns every step and both feet
                // trail behind it.
                let mut swing_to = leg.plant_pos;
                swing_to.x = hip_x + v * config.swing_duration_ts + x_target;
                swing_to.y = config.floor_y;
                leg.phase = GaitPhase::Swing;
                leg.phase_timer = config.swing_duration_ts;
                leg.swing_from = leg.plant_pos;
                leg.swing_to = swing_to;
            }
        }
        GaitPhase::Swing => {
            leg.phase_timer -= dt;
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
            }
        }
    }
}

/// Run one full resolver tick.
///
/// # Pipeline (mirrors `plans/Raibert.md`)
/// 1. Read target hip height (normal vs crouch).
/// 2. `X_max` from the current `H`.
/// 3. Backward math → `v_max`.
/// 4. Clamp hip velocity to `v_max`.
/// 5. Forward math → `x_target` for each leg in swing.
/// 6. Flat floor ⇒ plant y is `floor_y`; parabolic lift during swing.
/// 7. 2-bone IK for each leg → knees/feet.
pub fn step_lower_body(
    input_dir: Vec2,
    config: &LowerBodyConfig,
    state: &mut ResolverState,
    dt: f32,
) -> (LowerBodyPose, DebugInfo) {
    let dir = input_dir.normalize_or_zero();
    let v_desired_scalar = dir.x * config.target_speed;

    // 1. --- Hip height ---
    let h_target = if config.crouch {
        config.crouch_height
    } else {
        config.hip_height
    };
    state.hip_height = approach(state.hip_height, h_target, config.hip_height * dt * 3.0);
    let h = state.hip_height;

    // 2. --- X_max at current height ---
    let x_max = compute_x_max(config.thigh_len, config.calf_len, h, config.safety_margin);

    // 3. --- Backward: v_max ---
    let v_max = compute_v_max(
        x_max,
        config.kv,
        config.stance_duration_ts,
        config.target_speed,
    );

    // 4. --- Clamp velocity ---
    let accel_step = config.accel * dt;
    let v_unclamped = approach(state.hip_vel.x, v_desired_scalar, accel_step);
    let v = v_unclamped.clamp(-v_max, v_max);
    state.hip_vel.x = v;
    // Kinematic sandbox: no vertical hip dynamics.
    state.hip_vel.y = 0.0;

    // Integrate hip position. Height is pinned to `floor_y + h`.
    state.hip_pos.x += v * dt;
    state.hip_pos.y = config.floor_y + h;

    // 5-7. --- Per-leg foot placement + IK ---
    step_leg(
        &mut state.left,
        state.hip_pos.x,
        v,
        v_desired_scalar,
        state.right.phase == GaitPhase::Swing,
        config,
        dt,
    );
    step_leg(
        &mut state.right,
        state.hip_pos.x,
        v,
        v_desired_scalar,
        state.left.phase == GaitPhase::Swing,
        config,
        dt,
    );

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
        v_max,
        v_desired: v_desired_scalar,
        v_clamped: v,
        left_phase: state.left.phase,
        right_phase: state.right.phase,
        left_target_x: state.left.swing_to.x,
        right_target_x: state.right.swing_to.x,
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

    #[test]
    fn test_compute_v_max_formula() {
        // v_max = (X_max + kv·v_desired) / (Ts/2 + kv)
        let v_max = compute_v_max(66.33, 0.12, 0.28, 120.0);
        let expected = (66.33 + 0.12 * 120.0) / (0.28 * 0.5 + 0.12);
        assert!((v_max - expected).abs() < 1e-3);
        // Guard against degenerate denominator.
        assert_eq!(compute_v_max(50.0, 0.0, 0.0, 0.0), 0.0);
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
        step_leg(&mut leg, 0.0, v, v, false, &cfg, 0.001);
        assert_eq!(leg.phase, GaitPhase::Swing);

        // Fly through the swing while the hip keeps moving.
        let dt = 0.01;
        let mut hip_x = 0.0;
        for _ in 0..((cfg.swing_duration_ts / dt).ceil() as usize + 2) {
            hip_x += v * dt;
            step_leg(&mut leg, hip_x, v, v, false, &cfg, dt);
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
        step_leg(
            &mut state.left,
            state.hip_pos.x,
            40.0,
            120.0,
            state.right.phase == GaitPhase::Swing,
            &cfg,
            dt,
        );
        step_leg(
            &mut state.right,
            state.hip_pos.x,
            40.0,
            120.0,
            state.left.phase == GaitPhase::Swing,
            &cfg,
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
        step_leg(
            &mut state.left,
            state.hip_pos.x,
            0.0,
            0.0,
            state.right.phase == GaitPhase::Swing,
            &cfg,
            dt,
        );
        step_leg(
            &mut state.right,
            state.hip_pos.x,
            0.0,
            0.0,
            state.left.phase == GaitPhase::Swing,
            &cfg,
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
        let mut hip_x = state.hip_pos.x;
        for _ in 0..3 {
            hip_x += 40.0 * dt;
            step_leg(
                &mut state.left,
                hip_x,
                40.0,
                120.0,
                state.right.phase == GaitPhase::Swing,
                &cfg,
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
}
