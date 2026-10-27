use bevy::math::Vec2;
use super::vertical::rotate;
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

/// The facing sign, clamped to ±1 (0 or NaN input means "right" = +1).
#[inline]
pub fn facing_sign(facing: f32) -> f32 {
    if facing < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// Pick the anatomical knee from the 2-bone IK's two mirror solutions. The IK
/// has exactly two valid knees for any reachable target, mirror images across
/// the hip→foot chord. Facing right, the human bend is the one whose knee is
/// more ANTERIOR along the facing axis — `knee.x · facing` maximal. Return that
/// one, mirroring only when `knee` came out the other solution.
///
/// Why *selection* and not enforcement: the strict "knee ahead of the ankle"
/// test is geometrically impossible at full-extension heel-strike (a foot far
/// ahead of the hip leaves no solution with `knee.x > foot.x`), but one of the
/// two mirrors is ALWAYS at least as anterior as the other, so the rule holds
/// at every chord angle and every reach. The comparison uses only the x axis,
/// which is identical in the vello y-down and Bevy y-up frames — immune to the
/// y-handedness that made the chord-relative `bend_sign` (and any cross-product
/// winding test) ambiguous between the two.
#[inline]
pub fn select_anterior_knee(hip: Vec2, foot: Vec2, knee: Vec2, facing: f32) -> Vec2 {
    let f = facing_sign(facing);
    let mirrored = mirror_knee_across_chord(hip, foot, knee);
    if mirrored.x * f > knee.x * f {
        mirrored
    } else {
        knee
    }
}

/// Mirror `knee` across the hip→foot chord if its winding around the chord
/// contradicts `want` (+1: `cross(chord, knee−hip) ≥ 0` in the vello frame,
/// i.e. clockwise on the y-down screen; −1: counter-clockwise). Reflection
/// preserves `|knee − hip|` and `|knee − foot|` exactly.
#[inline]
pub fn enforce_knee_winding(hip: Vec2, foot: Vec2, knee: Vec2, want: f32) -> Vec2 {
    let chord = foot - hip;
    if chord.length_squared() < 1e-6 {
        return knee; // degenerate chord: nothing to mirror about
    }
    let to_knee = knee - hip;
    let cross = chord.x * to_knee.y - chord.y * to_knee.x;
    if cross * want >= 0.0 {
        return knee;
    }
    mirror_knee_across_chord(hip, foot, knee)
}

/// Reflect `knee` across the line through `hip` and `foot`. Bone lengths are
/// preserved exactly; returns `knee` unchanged for a degenerate chord.
#[inline]
fn mirror_knee_across_chord(hip: Vec2, foot: Vec2, knee: Vec2) -> Vec2 {
    let chord = foot - hip;
    let c2 = chord.length_squared();
    if c2 < 1e-6 {
        return knee;
    }
    let to_knee = knee - hip;
    // knee' = hip + tangent − normal (tangent = chord projection).
    let proj = chord * (to_knee.dot(chord) / c2);
    let normal = to_knee - proj;
    hip + proj - normal
}
