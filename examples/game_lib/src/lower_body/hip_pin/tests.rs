use super::ground::*;
use super::state::*;
use super::*;
use bevy::math::Vec2;

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
fn foot_names_are_recognised() {
    assert!(is_left_foot("PLLL"));
    assert!(is_left_foot("LLL"));
    assert!(is_right_foot("PRLL"));
    assert!(is_right_foot("RLL"));
    // A shin/knee/arm must never be credited as a foot.
    assert!(!is_left_foot("LLLA"));
    assert!(!is_right_foot("RULA"));
    assert!(!is_left_foot("S0"));
    assert!(!is_right_foot("P3"));
}

#[test]
fn contact_needs_two_ticks_to_arm_but_one_to_release() {
    // Tick 1: first touch must NOT arm loop 2 yet.
    let f = debounce_contact(FootContact::default(), true, FootContact::default());
    assert!(f.touching);
    assert!(!f.grounded, "a single touching tick must not arm thrust");
    assert_eq!(f.on_ticks, 1);

    // Tick 2: now it arms.
    let f = debounce_contact(f, true, FootContact::default());
    assert!(f.grounded);
    assert_eq!(f.on_ticks, 2);

    // Release is immediate (CONTACT_OFF_TICKS == 1).
    let f = debounce_contact(f, false, FootContact::default());
    assert!(!f.touching);
    assert!(!f.grounded, "losing contact must disarm thrust at once");
    assert_eq!(f.off_ticks, 1);
}

#[test]
fn contact_flicker_does_not_arm_thrust() {
    // Alternating touch/no-touch (the chatter case at touchdown) must never
    // accumulate `on_ticks`, so `grounded` stays false throughout.
    let mut f = FootContact::default();
    for tick in 0..6 {
        let touching = tick % 2 == 0;
        f = debounce_contact(f, touching, FootContact::default());
        assert!(!f.grounded, "flicker armed thrust on tick {tick}");
    }
}

#[test]
fn contact_normal_and_point_are_retained() {
    let raw = FootContact {
        normal: Vec2::new(0.0, -1.0),
        point: Vec2::new(12.5, 280.0),
        normal_speed: 34.0,
        ..Default::default()
    };
    let f = debounce_contact(FootContact::default(), true, raw);
    assert_eq!(f.normal, raw.normal);
    assert_eq!(f.point, raw.point);
    // The closing speed must survive the debounce: loop 2 needs it to tell
    // "loaded and pushing" from "touching but weightless".
    assert!((f.normal_speed - 34.0).abs() < 1e-6);
}

#[test]
fn default_support_preserves_kinematic_fallback() {
    // No collision pipeline ⇒ no per-foot truth, so the resolver must fall
    // back to its pre-§6 timer rather than reporting "airborne".
    let s = GroundSupport::default();
    assert!(s.grounded);
    assert!(!s.feet_valid);
    assert!(!s.feet[0].grounded);
    assert!(!s.feet[1].grounded);
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
