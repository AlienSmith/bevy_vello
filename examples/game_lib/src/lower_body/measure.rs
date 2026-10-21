//! Whole-body CoM measurement layer (plan §1, changes §1).
//!
//! Computes, every fixed tick, the mass-weighted whole-body centre of mass over
//! **all** of the character's particles (including arms), an EMA-filtered CoM
//! velocity, the CoM height above the floor, and the world-frame torso pitch
//! plus filtered angular velocity. With the §7 **P3-heavy** mass budget the
//! measured CoM lands essentially at the hip, so the resolver's hip reference
//! and this exact quantity coincide — loops 1 & 2 can keep the hip as the
//! balance origin *and* use the whole-body CoM as the exact quantity.
//!
//! The math here is pure and deliberately Vello **y-down** (the physics world
//! frame), matching [`VelloParticle`]. Callers that feed the resolver (y-up)
//! flip `y` at the boundary. Torso pitch follows the loop-3 convention: upright
//! is `(0, -1)` ⇒ 0 rad, and `+` means a forward lean toward `+x`.

use bevy::math::Vec2;

/// One particle sample in the Vello (y-down) world frame.
#[derive(Debug, Clone, Copy)]
pub struct ParticleSample {
    pub pos: Vec2,
    pub vel: Vec2,
    /// Inverse mass (XPBD convention: higher = lighter). `m_i = 1/inv_mass`.
    pub inv_mass: f32,
}

/// Mass-weighted whole-body measurement (Vello y-down frame).
#[derive(Debug, Clone, Copy)]
pub struct BodyMeasurements {
    /// Mass-weighted CoM position `Σ mᵢ·pᵢ / Σ mᵢ`.
    pub com: Vec2,
    /// EMA-filtered CoM velocity `Σ mᵢ·vᵢ / Σ mᵢ` (time constant [`MEASURE_TAU`]).
    pub v_com: Vec2,
    /// CoM height above the floor line, px (positive = up; floor at `floor_y`).
    pub z_com: f32,
    /// Torso pitch vs world vertical, rad (`+` = forward lean toward `+x`).
    pub theta: f32,
    /// EMA-filtered torso angular velocity, rad/s.
    pub omega: f32,
    /// Total mass `Σ mᵢ` of the finite samples.
    pub total_mass: f32,
    /// Number of finite samples that contributed to the mass sum.
    pub n: u32,
}

impl Default for BodyMeasurements {
    fn default() -> Self {
        Self {
            com: Vec2::ZERO,
            v_com: Vec2::ZERO,
            z_com: 0.0,
            theta: 0.0,
            omega: 0.0,
            total_mass: 0.0,
            n: 0,
        }
    }
}

/// EMA + finite-difference filter state for the smoothed velocity and attitude.
///
/// `v_com` is an EMA of the mass-weighted CoM velocity; `omega` is an EMA of the
/// finite-differenced torso pitch. `primed` is false until the first sample, so
/// the first tick snaps to the raw value instead of blending from zero.
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyMeasureFilter {
    pub v_com: Vec2,
    pub theta_prev: f32,
    pub omega: f32,
    pub primed: bool,
}

/// `alpha = dt / (tau + dt)` for the EMA; `tau` is the time constant in seconds.
/// Guarded: `dt ≤ 0` or `tau ≤ 0` ⇒ `1.0` (snap to the sample).
#[inline]
pub fn ema_alpha(dt: f32, tau: f32) -> f32 {
    if dt <= 0.0 || tau <= 0.0 {
        1.0
    } else {
        dt / (tau + dt)
    }
}

/// Mass-weighted centre of mass over the samples. Non-finite positions and
/// zero-inverse-mass (infinite-mass / "ignored") particles are skipped. Returns
/// `(com, total_mass, n)`, or `None` if nothing contributed.
pub fn center_of_mass(samples: &[ParticleSample]) -> Option<(Vec2, f32, u32)> {
    let mut total = 0.0f32;
    let mut com = Vec2::ZERO;
    let mut n = 0u32;
    for s in samples {
        if !s.pos.is_finite() || !s.inv_mass.is_finite() || s.inv_mass <= 0.0 {
            continue;
        }
        let m = 1.0 / s.inv_mass;
        total += m;
        com += s.pos * m;
        n += 1;
    }
    if total <= 0.0 {
        return None;
    }
    Some((com / total, total, n))
}

/// Mass-weighted CoM velocity over the samples. Returns `None` if nothing
/// contributed a finite velocity.
pub fn com_velocity(samples: &[ParticleSample]) -> Option<Vec2> {
    let mut total = 0.0f32;
    let mut v = Vec2::ZERO;
    for s in samples {
        if !s.vel.is_finite() || !s.inv_mass.is_finite() || s.inv_mass <= 0.0 {
            continue;
        }
        let m = 1.0 / s.inv_mass;
        total += m;
        v += s.vel * m;
    }
    if total <= 0.0 {
        return None;
    }
    Some(v / total)
}

/// Torso pitch from the hip→shoulder axis against world vertical, Vello y-down.
/// Upright is `(0, -1)` ⇒ 0 rad; `+` = forward lean toward `+x`. Returns `0`
/// for degenerate (zero-length) axes.
#[inline]
pub fn torso_pitch(hip: Vec2, shoulder: Vec2) -> f32 {
    let axis = shoulder - hip;
    if axis.length_squared() < 1e-6 {
        0.0
    } else {
        axis.x.atan2(-axis.y)
    }
}

/// One full measurement update: gather the mass-weighted CoM + velocity, advance
/// the EMA filter, measure the torso pitch, and produce the measurement.
///
/// `torso_axis = (hip, shoulder)` in the Vello frame (P3, P1 for v10).
/// `floor_y` is the floor line in the Vello frame (y = 0 for the flat walk).
pub fn update_measurements(
    filter: &mut BodyMeasureFilter,
    samples: &[ParticleSample],
    torso_axis: (Vec2, Vec2),
    floor_y: f32,
    dt: f32,
    tau: f32,
) -> BodyMeasurements {
    let mut m = BodyMeasurements::default();
    if let Some((com_pos, total, n)) = center_of_mass(samples) {
        m.com = com_pos;
        m.total_mass = total;
        m.n = n;
        // Vello y-down: "up" is −y, so the height above the floor is
        // `floor_y − com_pos.y` (positive when the CoM is above the line).
        m.z_com = (floor_y - com_pos.y).max(0.0);
    }
    if let Some(v) = com_velocity(samples) {
        let alpha = ema_alpha(dt, tau);
        if filter.primed {
            filter.v_com = filter.v_com + (v - filter.v_com) * alpha;
        } else {
            filter.v_com = v;
        }
        m.v_com = filter.v_com;
    }
    let theta = torso_pitch(torso_axis.0, torso_axis.1);
    if filter.primed && dt > 0.0 {
        let raw_omega = (theta - filter.theta_prev) / dt;
        let alpha = ema_alpha(dt, tau);
        filter.omega = filter.omega + (raw_omega - filter.omega) * alpha;
    } else {
        filter.omega = 0.0;
    }
    filter.theta_prev = theta;
    filter.primed = true;
    m.theta = theta;
    m.omega = filter.omega;
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(pos: Vec2, vel: Vec2, inv_mass: f32) -> ParticleSample {
        ParticleSample { pos, vel, inv_mass }
    }

    #[test]
    fn center_of_mass_weights_by_mass() {
        // Two particles: one heavy at (10, 0), one light at (0, 0).
        let samples = [
            sample(Vec2::new(10.0, 0.0), Vec2::ZERO, 1.0), // m = 1
            sample(Vec2::new(0.0, 0.0), Vec2::ZERO, 0.2),  // m = 5
        ];
        let (com, total, n) = center_of_mass(&samples).unwrap();
        // total = 6, x = (10*1 + 0*5)/6 = 10/6 ≈ 1.6667
        assert!((com.x - 10.0 / 6.0).abs() < 1e-5);
        assert!((total - 6.0).abs() < 1e-5);
        assert_eq!(n, 2);
    }

    #[test]
    fn center_of_mass_skips_infinite_mass_and_non_finite() {
        let samples = [
            sample(Vec2::new(1.0, 1.0), Vec2::ZERO, 0.0), // inv_mass 0 ⇒ skipped
            sample(Vec2::new(f32::NAN, 1.0), Vec2::ZERO, 1.0), // non-finite pos ⇒ skipped
            sample(Vec2::new(4.0, 0.0), Vec2::ZERO, 1.0),
        ];
        let (com, total, n) = center_of_mass(&samples).unwrap();
        assert!((com.x - 4.0).abs() < 1e-5);
        assert!((total - 1.0).abs() < 1e-5);
        assert_eq!(n, 1);
    }

    #[test]
    fn center_of_mass_none_when_empty() {
        assert!(center_of_mass(&[]).is_none());
    }

    #[test]
    fn com_velocity_weights_by_mass() {
        let samples = [
            sample(Vec2::ZERO, Vec2::new(0.0, 0.0), 1.0),   // m = 1
            sample(Vec2::ZERO, Vec2::new(0.0, 100.0), 0.2), // m = 5
        ];
        let v = com_velocity(&samples).unwrap();
        // (0*1 + 100*5)/6 ≈ 83.333
        assert!((v.y - 500.0 / 6.0).abs() < 1e-5);
    }

    #[test]
    fn ema_alpha_formula_and_guards() {
        // dt = 0.01, tau = 0.03 → 0.25
        assert!((ema_alpha(0.01, 0.03) - 0.25).abs() < 1e-6);
        // Guards snap to the sample.
        assert_eq!(ema_alpha(0.0, 0.03), 1.0);
        assert_eq!(ema_alpha(0.01, 0.0), 1.0);
        assert_eq!(ema_alpha(-1.0, 0.03), 1.0);
    }

    #[test]
    fn torso_pitch_sign_conventions() {
        // Upright (shoulder directly above hip in y-down): axis = (0, -1).
        assert!((torso_pitch(Vec2::new(0.0, 0.0), Vec2::new(0.0, -10.0))).abs() < 1e-6);
        // Forward lean toward +x: axis = (5, -10) → atan2(5, 10) > 0.
        let forward = torso_pitch(Vec2::new(0.0, 0.0), Vec2::new(5.0, -10.0));
        assert!(forward > 0.0);
        // Backward lean toward -x: axis = (-5, -10) → atan2(-5, 10) < 0.
        let backward = torso_pitch(Vec2::new(0.0, 0.0), Vec2::new(-5.0, -10.0));
        assert!(backward < 0.0);
        // Degenerate axis ⇒ 0.
        assert_eq!(torso_pitch(Vec2::new(1.0, 1.0), Vec2::new(1.0, 1.0)), 0.0);
    }

    #[test]
    fn update_measurements_snaps_then_emafilters() {
        let mut filter = BodyMeasureFilter::default();
        let samples = [sample(Vec2::new(0.0, -90.0), Vec2::new(0.0, 0.0), 1.0)];
        // First tick: snap — v_com equals the sample, omega zero (not primed).
        let m1 = update_measurements(
            &mut filter,
            &samples,
            (Vec2::new(0.0, 0.0), Vec2::new(0.0, -10.0)),
            0.0,
            0.01,
            0.03,
        );
        assert!((m1.v_com.length()).abs() < 1e-5);
        assert!((m1.z_com - 90.0).abs() < 1e-5); // CoM at y=-90, floor y=0 ⇒ h=90
        assert!((m1.theta).abs() < 1e-5);
        assert!((m1.omega).abs() < 1e-5);

        // Second tick: particle moves; EMA blends, omega is the finite-differenced,
        // EMA-filtered rate.
        let moved = [sample(Vec2::new(10.0, -90.0), Vec2::new(100.0, 0.0), 1.0)];
        let m2 = update_measurements(
            &mut filter,
            &moved,
            (Vec2::new(10.0, 0.0), Vec2::new(10.0, -10.0)),
            0.0,
            0.01,
            0.03,
        );
        // EMA toward 100 with alpha 0.25: v_com = 0 + (100-0)*0.25 = 25.
        assert!((m2.v_com.x - 25.0).abs() < 1e-3);
        // theta stays 0 (torso vertical), so omega stays ~0.
        assert!(m2.theta.abs() < 1e-5);
        assert!(m2.omega.abs() < 1e-5);
    }

    #[test]
    fn update_measurements_filters_torso_omega() {
        let mut filter = BodyMeasureFilter::default();
        let samples = [sample(Vec2::ZERO, Vec2::ZERO, 1.0)];
        // Upright first.
        update_measurements(
            &mut filter,
            &samples,
            (Vec2::new(0.0, 0.0), Vec2::new(0.0, -10.0)),
            0.0,
            0.01,
            0.03,
        );
        // Lean +5° then +10° at 100 Hz: raw omega ~= 5 rad/s, EMA-smoothed.
        update_measurements(
            &mut filter,
            &samples,
            (Vec2::new(0.0, 0.0), Vec2::new(1.0, -10.0)),
            0.0,
            0.01,
            0.03,
        );
        let m = update_measurements(
            &mut filter,
            &samples,
            (Vec2::new(0.0, 0.0), Vec2::new(2.0, -10.0)),
            0.0,
            0.01,
            0.03,
        );
        // theta positive (forward lean), omega > 0 (rate of increase).
        assert!(m.theta > 0.0);
        assert!(m.omega > 0.0);
        // omega is EMA-filtered: strictly below the raw rate (alpha 0.25 < 1).
        let raw = m.theta / 0.01;
        assert!(m.omega < raw);
    }
}
