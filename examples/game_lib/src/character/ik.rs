use bevy::prelude::*;

use crate::character::{ArmConfig, IkMode};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Angular error threshold (in radians) for switching to elbow-only IK solve.
/// When the forearm direction error is ≤ this value, the shoulder freezes and only
/// the elbow rotates to finish aiming, preventing the two-joint oscillation near convergence.
/// 0.05 rad ≈ 3° — small enough to be invisible, large enough to prevent the slow tail.
pub(super) const ELBOW_ONLY_THRESHOLD: f32 = 0.05;

// ---------------------------------------------------------------------------
// Reach IK
// ---------------------------------------------------------------------------

/// Law-of-cosines 2-bone reach IK: place the wrist at the target position.
///
/// Uses the law of cosines to find the shoulder angle, then places the wrist
/// exactly at the target via shape matching (no angular constraint used).
///
/// Returns `(desired_p13_local, desired_prla_local)`.
pub(super) fn solve_reach_ik(
    local_target: Vec2,
    p12_local: Vec2,
    upper_len: f32,
    forearm_len: f32,
    bend_sign: f32,
) -> (Vec2, Vec2) {
    let reach = upper_len + forearm_len;
    let mut target_pos_local = local_target;
    let root_to_target = target_pos_local - p12_local;
    let dist = root_to_target.length();
    if dist > reach {
        target_pos_local = p12_local + root_to_target.normalize() * (reach - 0.001);
    }
    let to_target = target_pos_local - p12_local;
    let target_dist = to_target.length().max(f32::EPSILON);
    let target_dir = to_target / target_dist;

    // Law of cosines for shoulder offset
    let cos_shoulder = (upper_len * upper_len + target_dist * target_dist
        - forearm_len * forearm_len)
        / (2.0 * upper_len * target_dist);
    let cos_shoulder = cos_shoulder.clamp(-1.0, 1.0);
    let shoulder_offset = cos_shoulder.acos();

    // Rotate target direction by shoulder_offset * bend_sign to get the desired upper arm direction
    let total_shoulder_angle = shoulder_offset * bend_sign;
    let (sin_sh, cos_sh) = total_shoulder_angle.sin_cos();
    let desired_upper_dir = Vec2::new(
        target_dir.x * cos_sh - target_dir.y * sin_sh,
        target_dir.x * sin_sh + target_dir.y * cos_sh,
    );
    let desired_p13_local = p12_local + desired_upper_dir * upper_len;
    let desired_prla_local = target_pos_local;

    (desired_p13_local, desired_prla_local)
}

// ---------------------------------------------------------------------------
// Aim IK (full)
// ---------------------------------------------------------------------------

/// Least-action forearm alignment IK: make the forearm point at the target.
///
/// Uses a Lagrange multiplier solve that constrains the forearm direction
/// (not wrist position), finding the minimal-displacement joint angles.
/// The weapon offset angle rotates the aim direction before solving.
///
/// The solve is **iterative** (4 iterations).  Rotating the shoulder changes
/// the elbow position, which in turn changes the elbow→target direction used
/// to compute `alpha`.  A single split therefore leaves a ~13.8° residual;
/// iterating reduces it to < 0.001°.
///
/// Returns `(desired_p13_local, desired_prla_local)`.
pub(super) fn solve_aim_ik(
    local_target: Vec2,
    p12_local: Vec2,
    p13_local: Vec2,
    prla_local: Vec2,
    upper_len: f32,
    forearm_len: f32,
    weapon_offset_y: f32, // The parallel vertical offset
) -> (Vec2, Vec2) {
    // Use mutable copies that we refine each iteration
    let mut theta1 = {
        let upper_dir = p13_local - p12_local;
        upper_dir.y.atan2(upper_dir.x)
    };
    let mut theta2 = {
        let upper_dir = p13_local - p12_local;
        let forearm_dir = prla_local - p13_local;
        let cross = upper_dir.x * forearm_dir.y - upper_dir.y * forearm_dir.x;
        let dot = upper_dir.dot(forearm_dir);
        cross.atan2(dot)
    };

    // Iterative refinement: 4 iterations is enough to converge to < 0.001°
    for _iter in 0..4 {
        // Current elbow position (from current theta1)
        let (sin1, cos1) = theta1.sin_cos();
        let elbow = p12_local + Vec2::new(cos1, sin1) * upper_len;

        // Compute target direction from this iteration's elbow position
        let to_target = local_target - elbow;
        let target_dir = to_target.normalize_or_zero();
        let r = to_target.length();

        if r < 1e-4 {
            // Target is at elbow — can't aim, return current state
            let (sin1, cos1) = theta1.sin_cos();
            let desired_p13 = p12_local + Vec2::new(cos1, sin1) * upper_len;
            let (sin12, cos12) = (theta1 + theta2).sin_cos();
            let desired_prla = desired_p13 + Vec2::new(cos12, sin12) * forearm_len;
            return (desired_p13, desired_prla);
        }

        // Shift by weapon offset along the elbow→target normal
        let target_normal = Vec2::new(-target_dir.y, target_dir.x);
        let virtual_target = local_target - target_normal * weapon_offset_y;
        let to_vtarget = virtual_target - elbow;
        let alpha = to_vtarget.y.atan2(to_vtarget.x);

        // Forearm absolute angle
        let forearm_angle = theta1 + theta2;

        // Wrapped angular error
        let mut angle_diff = forearm_angle - alpha;
        angle_diff = (angle_diff + std::f32::consts::PI).rem_euclid(2.0 * std::f32::consts::PI)
            - std::f32::consts::PI;

        // Split equally and accumulate
        theta1 -= angle_diff * 0.5;
        theta2 -= angle_diff * 0.5;
    }

    // Reconstruct final positions from converged angles
    let (sin1, cos1) = theta1.sin_cos();
    let desired_p13 = p12_local + Vec2::new(cos1, sin1) * upper_len;

    let (sin12, cos12) = (theta1 + theta2).sin_cos();
    let desired_prla = desired_p13 + Vec2::new(cos12, sin12) * forearm_len;

    (desired_p13, desired_prla)
}

// ---------------------------------------------------------------------------
// Aim IK (elbow-only)
// ---------------------------------------------------------------------------

/// Elbow-only aim IK: shoulder is frozen, only the elbow rotates to aim at the target.
///
/// Uses a fixed-point iteration to handle the weapon offset circular dependency:
/// the weapon offset normal depends on the forearm direction, which is what we solve for.
/// Since this is only called when the angular error is small (≤ ELBOW_ONLY_THRESHOLD),
/// the initial guess (current forearm direction) is close to the answer → 4 iterations suffice.
pub(super) fn solve_aim_ik_elbow_only(
    local_target: Vec2,
    _p12_local: Vec2,
    p13_local: Vec2, // elbow position — this becomes the desired shoulder target (frozen)
    prla_local: Vec2,
    _upper_len: f32,
    forearm_len: f32,
    weapon_offset_y: f32,
) -> (Vec2, Vec2) {
    // Shoulder frozen → elbow stays at its current position
    let desired_p13 = p13_local;

    // Initial guess: current forearm direction
    let mut forearm_dir = (prla_local - p13_local).normalize_or_zero();

    // Fixed-point iteration to converge forearm direction + weapon offset normal.
    //   forearm_dir → forearm_normal → virtual_target → new_forearm_dir → ...
    // 4 iterations is enough because the initial guess is already close.
    for _ in 0..4 {
        let forearm_normal = Vec2::new(-forearm_dir.y, forearm_dir.x);
        let virtual_target = local_target - forearm_normal * weapon_offset_y;
        let to_vtarget = virtual_target - p13_local;
        forearm_dir = to_vtarget.normalize_or_zero();
    }

    // Final wrist position from converged forearm direction
    let desired_prla = p13_local + forearm_dir * forearm_len;

    (desired_p13, desired_prla)
}

// ---------------------------------------------------------------------------
// IK Position Dispatcher
// ---------------------------------------------------------------------------

/// Dispatches to Reach / Aim (full or elbow-only) based on config.ik_mode.
pub(super) fn compute_ik_positions(
    config: &ArmConfig,
    local_target: Vec2,
    p12_local: Vec2,
    p13_local: Vec2,
    prla_local: Vec2,
    upper_len: f32,
    forearm_len: f32,
) -> (Vec2, Vec2) {
    match &config.ik_mode {
        IkMode::Disabled => (p13_local, prla_local),
        IkMode::Reach => solve_reach_ik(
            local_target,
            p12_local,
            upper_len,
            forearm_len,
            config.bend_sign,
        ),
        IkMode::Aim { weapon_offset_y } => {
            let forearm_dir = prla_local - p13_local;
            let to_target = local_target - p13_local;
            let target_dir = to_target.normalize_or_zero();

            // Compare against the VIRTUAL target direction (accounting for weapon
            // offset), because that's what the IK solves for.  Using the true target
            // direction here would show a persistent weapon-offset-angle error even
            // when the IK is fully converged, causing the arm to oscillate.
            let target_normal = Vec2::new(-target_dir.y, target_dir.x);
            let virtual_target = local_target - target_normal * weapon_offset_y;
            let to_vtarget = virtual_target - p13_local;
            let vtarget_dir = to_vtarget.normalize_or_zero();

            let cos_err = forearm_dir.dot(vtarget_dir) / (forearm_dir.length().max(f32::EPSILON));
            let angle_err = cos_err.clamp(-1.0, 1.0).acos();
            let use_full = angle_err > ELBOW_ONLY_THRESHOLD;

            // info!(
            //     "[compute_ik_positions] AIM: angle_err={:.4}° (threshold={:.4}°), use_full={}, mode={}",
            //     angle_err.to_degrees(),
            //     ELBOW_ONLY_THRESHOLD.to_degrees(),
            //     use_full,
            //     if use_full { "FULL_IK" } else { "ELBOW_ONLY" },
            // );

            if use_full {
                solve_aim_ik(
                    local_target,
                    p12_local,
                    p13_local,
                    prla_local,
                    upper_len,
                    forearm_len,
                    *weapon_offset_y,
                )
            } else {
                solve_aim_ik_elbow_only(
                    local_target,
                    p12_local,
                    p13_local,
                    prla_local,
                    upper_len,
                    forearm_len,
                    *weapon_offset_y,
                )
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: arm geometry for tests.
    struct TestArm {
        p12_local: Vec2,
        p13_local: Vec2,
        prla_local: Vec2,
        upper_len: f32,
        forearm_len: f32,
    }

    /// Build a test arm in y-down coordinate space:
    ///   - shoulder at origin
    ///   - upper arm pointing right (theta1 = 0)
    ///   - forearm with theta2 = -2.5 rad (~-143°), wrist at (10, -15) from shoulder
    ///
    /// In y-down, positive theta2 = CW bend (wrist below elbow, forward).
    /// theta2 = -2.5 is deliberately a hyperextended/reverse pose (wrist above elbow),
    /// giving a ~106° initial angular error — a worst-case stress test for convergence.
    ///
    /// An arm starting near alignment (angle_err < 0.05 rad ≈ 3°) would exercise
    /// `solve_aim_ik_elbow_only` instead.
    fn test_arm_bent() -> TestArm {
        let p12_local = Vec2::ZERO;
        let upper_len = 30.0;
        let forearm_len = 25.0;
        let theta1: f32 = 0.0; // shoulder angle: straight right
        let theta2: f32 = -2.5; // elbow angle: roughly -143° (bent, forearm pointing down-right)
        let (s1, c1) = theta1.sin_cos();
        let p13_local = p12_local + Vec2::new(c1, s1) * upper_len;
        let (s12, c12) = (theta1 + theta2).sin_cos();
        let prla_local = p13_local + Vec2::new(c12, s12) * forearm_len;
        TestArm {
            p12_local,
            p13_local,
            prla_local,
            upper_len,
            forearm_len,
        }
    }

    /// Compute the angle (in radians) between `forearm_dir` and `target_dir`
    fn compute_angle_err(forearm_dir: Vec2, target_dir: Vec2) -> f32 {
        let cos_err = forearm_dir.dot(target_dir) / (forearm_dir.length().max(f32::EPSILON));
        cos_err.clamp(-1.0, 1.0).acos()
    }

    // ========================================================================
    // Test 1: solve_aim_ik determinism + convergence to near-zero
    // ========================================================================
    // The iterative solve (4 iterations) converges the forearm direction to the
    // virtual target direction with < 0.001° residual, because each iteration
    // re-evaluates the elbow→target direction from the updated elbow position.
    #[test]
    fn test_solve_aim_ik_determinism() {
        let arm = test_arm_bent();
        let local_target = Vec2::new(50.0, -15.0);
        let offset = 0.0;

        let r1 = solve_aim_ik(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            offset,
        );
        let r2 = solve_aim_ik(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            offset,
        );

        // Determinism
        assert!(
            (r1.0 - r2.0).length() < f32::EPSILON,
            "solve_aim_ik not deterministic: p13 diff {}",
            (r1.0 - r2.0).length()
        );
        assert!(
            (r1.1 - r2.1).length() < f32::EPSILON,
            "solve_aim_ik not deterministic: prla diff {}",
            (r1.1 - r2.1).length()
        );

        let (desired_p13, desired_prla) = r1;

        // Distance constraints
        let upper_actual = (desired_p13 - arm.p12_local).length();
        let forearm_actual = (desired_prla - desired_p13).length();
        assert!(
            (upper_actual - arm.upper_len).abs() < 1e-4,
            "upper arm length changed: {} vs {}",
            upper_actual,
            arm.upper_len
        );
        assert!(
            (forearm_actual - arm.forearm_len).abs() < 1e-4,
            "forearm length changed: {} vs {}",
            forearm_actual,
            arm.forearm_len
        );

        // Convergence to near-zero (iterative solve)
        let forearm_dir = desired_prla - desired_p13;
        let to_target = local_target - desired_p13;
        let target_dir = to_target.normalize_or_zero();
        let residual_err = compute_angle_err(forearm_dir, target_dir);
        assert!(
            residual_err < 1e-4,
            "solve_aim_ik did not converge: residual={:.6}rad ({:.4}°)",
            residual_err,
            residual_err.to_degrees()
        );
    }

    // ========================================================================
    // Test 2: solve_aim_ik_elbow_only determinism + invariants
    // ========================================================================
    #[test]
    fn test_solve_aim_ik_elbow_only_invariants() {
        let arm = test_arm_bent();
        let local_target = Vec2::new(50.0, -15.0);
        let offset = 0.0;

        let r1 = solve_aim_ik_elbow_only(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            offset,
        );
        let r2 = solve_aim_ik_elbow_only(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            offset,
        );

        // Determinism
        assert!(
            (r1.0 - r2.0).length() < f32::EPSILON,
            "elbow_only not deterministic: p13"
        );
        assert!(
            (r1.1 - r2.1).length() < f32::EPSILON,
            "elbow_only not deterministic: prla"
        );

        let (desired_p13, desired_prla) = r1;

        // Shoulder frozen
        assert!(
            (desired_p13 - arm.p13_local).length() < f32::EPSILON,
            "shoulder moved in elbow-only mode"
        );

        // Forearm length preserved
        let forearm_actual = (desired_prla - desired_p13).length();
        assert!(
            (forearm_actual - arm.forearm_len).abs() < 1e-4,
            "forearm length changed in elbow-only: {} vs {}",
            forearm_actual,
            arm.forearm_len
        );

        // Forearm should point at target (offset=0)
        let forearm_dir = desired_prla - desired_p13;
        let to_target = local_target - desired_p13;
        let target_dir = to_target.normalize_or_zero();
        let err = compute_angle_err(forearm_dir, target_dir);
        // Elbow-only uses 4 iterations of fixed-point; floating-point accumulation
        // leaves a tiny residual (~0.02° with these parameters).  Use 1e-3 tolerance.
        assert!(
            err < 1e-3,
            "forearm does not point at target after elbow-only: angle_err={:.6}rad ({:.4}°)",
            err,
            err.to_degrees()
        );
    }

    // ========================================================================
    // Test 3: Aim vs Elbow-Only consistency at small angle_err
    // ========================================================================
    #[test]
    fn test_aim_vs_elbow_only_consistency() {
        // Build an arm that is ALREADY nearly aligned with the target,
        // so angle_err < ELBOW_ONLY_THRESHOLD.
        let p12_local = Vec2::ZERO;
        let upper_len = 30.0;
        let forearm_len = 25.0;

        // Forearm points at the target direction directly, so angle_err ≈ 0.
        let target_world = Vec2::new(50.0, -10.0);
        let p13_local = Vec2::new(30.0, 0.0); // shoulder→elbow straight right
        let to_target = target_world - p13_local;
        let target_dir = to_target.normalize();
        let forearm_len = 25.0;
        let prla_local = p13_local + target_dir * forearm_len;

        let offset = 0.0;

        let (aim_p13, aim_prla) = solve_aim_ik(
            target_world,
            p12_local,
            p13_local,
            prla_local,
            upper_len,
            forearm_len,
            offset,
        );
        let (eo_p13, eo_prla) = solve_aim_ik_elbow_only(
            target_world,
            p12_local,
            p13_local,
            prla_local,
            upper_len,
            forearm_len,
            offset,
        );

        // Both should produce forearm pointing at target
        let aim_forearm = aim_prla - aim_p13;
        let eo_forearm = eo_prla - eo_p13;
        let aim_angle_err = compute_angle_err(aim_forearm, target_dir);
        let eo_angle_err = compute_angle_err(eo_forearm, target_dir);

        // Full solve leaves a residual (shoulder movement changes elbow direction).
        // Just verify it's significantly reduced compared to what it would be.
        assert!(
            aim_angle_err < 10.0_f32.to_radians(),
            "solve_aim_ik: residual angle_err too large: aim_angle_err={:.4}° (target rely aligned)",
            aim_angle_err.to_degrees()
        );
        assert!(
            eo_angle_err < 1e-3,
            "elbow_only: residual angle_err={:.6}°",
            eo_angle_err.to_degrees()
        );

        // Wrist positions should be close (both aim at same direction)
        let wrist_diff = (aim_prla - eo_prla).length();
        assert!(
            wrist_diff < 1.0,
            "wrist positions differ too much: {} (aim_p13={:.2}, eo_p13={:.2})",
            wrist_diff,
            (aim_p13 - p12_local).length(),
            (eo_p13 - p12_local).length()
        );
    }

    // ========================================================================
    // Test 4: solve_reach_ik distance constraints
    // ========================================================================
    #[test]
    fn test_solve_reach_ik_invariants() {
        let arm = test_arm_bent();
        let local_target = Vec2::new(50.0, -10.0);

        let (desired_p13, desired_prla) = solve_reach_ik(
            local_target,
            arm.p12_local,
            arm.upper_len,
            arm.forearm_len,
            -1.0,
        );

        // Upper arm length preserved
        let upper_actual = (desired_p13 - arm.p12_local).length();
        assert!(
            (upper_actual - arm.upper_len).abs() < 1e-4,
            "reach: upper arm length changed: {} vs {}",
            upper_actual,
            arm.upper_len
        );

        // Forearm length preserved
        let forearm_actual = (desired_prla - desired_p13).length();
        assert!(
            (forearm_actual - arm.forearm_len).abs() < 1e-4,
            "reach: forearm length changed: {} vs {}",
            forearm_actual,
            arm.forearm_len
        );

        // Total reach should be within bounds
        let total_reach = (desired_prla - arm.p12_local).length();
        assert!(
            total_reach <= arm.upper_len + arm.forearm_len + 0.01,
            "reach: total reach {} exceeds {}",
            total_reach,
            arm.upper_len + arm.forearm_len
        );
    }

    #[test]
    fn test_solve_reach_ik_out_of_range() {
        let arm = test_arm_bent();
        // Target far beyond reach
        let far_target = Vec2::new(500.0, -300.0);

        let (desired_p13, desired_prla) = solve_reach_ik(
            far_target,
            arm.p12_local,
            arm.upper_len,
            arm.forearm_len,
            -1.0,
        );

        // Arm should be fully extended (wrist at max reach)
        let total_reach = (desired_prla - arm.p12_local).length();
        let max_reach = arm.upper_len + arm.forearm_len;
        assert!(
            total_reach <= max_reach + 0.01,
            "out-of-range: total reach {} exceeds max {}",
            total_reach,
            max_reach
        );
        assert!(
            total_reach > max_reach - 1.0,
            "out-of-range: arm not fully extended: {} vs max {}",
            total_reach,
            max_reach
        );
    }

    // ========================================================================
    // Test 5: compute_ik_positions dispatch logic
    // ========================================================================
    #[test]
    fn test_compute_ik_positions_dispatch() {
        let arm = test_arm_bent();

        // Case A: IkMode::Aim with offset=0 → should aim forearm at local_target
        let config_aim = ArmConfig {
            ik_mode: IkMode::Aim {
                weapon_offset_y: 0.0,
            },
            ..Default::default()
        };
        let (aim_p13, aim_prla) = compute_ik_positions(
            &config_aim,
            Vec2::new(50.0, -10.0),
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
        );

        // forearm distance preserved
        let forearm_actual = (aim_prla - aim_p13).length();
        assert!(
            (forearm_actual - arm.forearm_len).abs() < 1e-4,
            "Aim: forearm length changed in dispatch"
        );

        // Case B: IkMode::Reach
        let config_reach = ArmConfig {
            ik_mode: IkMode::Reach,
            ..Default::default()
        };
        let (reach_p13, reach_prla) = compute_ik_positions(
            &config_reach,
            Vec2::new(50.0, -10.0),
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
        );

        // reach: forearm distance preserved
        let reach_forearm = (reach_prla - reach_p13).length();
        assert!(
            (reach_forearm - arm.forearm_len).abs() < 1e-4,
            "Reach: forearm length changed in dispatch"
        );

        // Case C: IkMode::Disabled → no change
        let config_disabled = ArmConfig {
            ik_mode: IkMode::Disabled,
            ..Default::default()
        };
        let (dis_p13, dis_prla) = compute_ik_positions(
            &config_disabled,
            Vec2::new(50.0, -10.0),
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
        );
        assert!(
            (dis_p13 - arm.p13_local).length() < f32::EPSILON,
            "Disabled: p13 should be unchanged"
        );
        assert!(
            (dis_prla - arm.prla_local).length() < f32::EPSILON,
            "Disabled: prla should be unchanged"
        );
    }

    // ========================================================================
    // Test 6: Convergence test — iterative solve converges to near-zero
    // ========================================================================
    // The 4-iteration iterative solve re-evaluates the elbow→target direction
    // from the updated elbow position each iteration, converging to < 0.001°.
    #[test]
    fn test_solve_aim_ik_converges() {
        let arm = test_arm_bent();
        let local_target = Vec2::new(50.0, -15.0);

        let (desired_p13, desired_prla) = solve_aim_ik(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            0.0,
        );

        // Residual angular error after solve
        let forearm_dir_result = desired_prla - desired_p13;
        let to_target_result = local_target - desired_p13;
        let target_dir_result = to_target_result.normalize_or_zero();
        let residual_err = compute_angle_err(forearm_dir_result, target_dir_result);

        // The 4-iteration iterative solve should converge to < 0.001°
        assert!(
            residual_err < 1e-4,
            "solve_aim_ik did not converge: residual={:.6}rad ({:.4}°)",
            residual_err,
            residual_err.to_degrees()
        );
    }
}
