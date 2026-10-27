use bevy::prelude::*;

use crate::lower_body::measure::{
    BodyMeasureFilter, BodyMeasurements,
};
use crate::lower_body::resolver::{
    LowerBodyConfig,
    ResolverState,
};


#[derive(Debug, Clone, Copy)]
pub(crate) struct LegPoseSeed {
    pub(crate) hip: Vec2,
    pub(crate) knee: Vec2,
    pub(crate) foot: Vec2,
}

/// Sandbox↔leg bridge captured once on the first leg-driving tick.
///
/// Only the resolver's per-segment rotation deltas are transferred onto the
/// live spawn pose, so the character's own bone lengths and rest geometry are
/// preserved and the leg PD sees no step at t=0.
#[derive(Debug, Clone, Copy)]
pub(crate) struct LegBridgeSeed {
    /// Live leg poses at seed: `[left, right]`.
    pub(crate) live: [LegPoseSeed; 2],
    /// Resolver leg poses at seed: `[left, right]`.
    pub(crate) resolver: [LegPoseSeed; 2],
}

/// Signed angle from `a` to `b`, in `(-π, π]`.
#[inline]
pub(crate) fn signed_angle(a: Vec2, b: Vec2) -> f32 {
    a.perp_dot(b).atan2(a.dot(b))
}

/// Rotate `v` by `angle` radians (bevy y-up).
#[inline]
pub(crate) fn rotate_vec(v: Vec2, angle: f32) -> Vec2 {
    let (s, c) = angle.sin_cos();
    Vec2::new(c * v.x - s * v.y, s * v.x + c * v.y)
}

/// Rotate the seed segment `live0` by the resolver segment's rotation from
/// `res0` to `res1`, preserving `live0`'s length.
#[inline]
pub(crate) fn transfer_segment(live0: Vec2, res0: Vec2, res1: Vec2) -> Vec2 {
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
    pub(crate) prev_world_delta: Vec2,
    /// Set once the rest offsets and anchor have been captured.
    pub ready: bool,
    /// Master enable; when false the pin emits nothing.
    pub enabled: bool,
    /// Rigid-frame attitude in radians.
    pub theta: f32,
    /// Whether a jump was held last step, for edge detection at fixed `dt`.
    pub(crate) jump_was_held: bool,
    /// Sandbox↔leg bridge seed, captured lazily.
    pub(crate) leg_seed: Option<LegBridgeSeed>,
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
    /// Realized standing hip height above the floor, px. Captured from the live
    /// leg-bridge seed once the rig is ready (the character may stand taller or
    /// shorter than `config.hip_height` — e.g. 106 px vs the 90 px model default).
    /// The resolver's ground-referenced geometry (stand height, swing plant, reach
    /// envelope `X_max`, LIPM) is computed against this per-pin value so the virtual
    /// floor coincides with the real collision floor.
    pub standing_height: f32,
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
            standing_height: config.hip_height,
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
pub(crate) fn measured_attitude(p1: Vec2, p2: Vec2) -> f32 {
    let axis = p1 - p2;
    axis.x.atan2(-axis.y)
}

/// Clamp a commanded lean, falling back to upright if the input is not finite.
/// A raw `f32::clamp` panics on NaN (`min > max, or either was NaN`).
#[inline]
pub(crate) fn clamp_attitude(raw: f32, max_angle: f32) -> f32 {
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
pub(crate) fn angle_basis(pos0: Vec2, pos1: Vec2, pos2: Vec2) -> Option<(f32, f32)> {
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
pub(crate) fn attitude_rest_basis(p1: Vec2, p2: Vec2, p3: Vec2, theta: f32) -> Option<(f32, f32)> {
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
pub(crate) fn p2_rigid_frame_targets(p2_target: Vec2, rest_offsets: [Vec2; 3], theta: f32) -> [Vec2; 3] {
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
pub(crate) fn delta_frame_targets(
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
pub(crate) fn softened_compliance(base: f32, soft_scale: f32, v_ref: f32, speed: f32) -> f32 {
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
pub(crate) fn use_external_pin(external_pin: bool, air_control: bool, grounded: bool, input_x: f32) -> bool {
    external_pin || (air_control && !grounded && input_x.abs() > 0.0)
}

