//! The jump stroke as **keyframed data** — the authored expert, expressed on the
//! general [`walker_math::Clip`] track rather than as hardcoded phase ramps.
//!
//! # Why a clip
//! The residual trainer wants the expert to be an *animation reference* it can
//! correct: `target = reference(phase) + bounded δ`. Writing the stroke as keys
//! makes that explicit and makes the same machinery reusable for any other
//! action (a wave, a kick) — only the [`JumpKey`] payload and the track differ.
//!
//! # The 5-param pose
//! Each key is the same polar 5-dim format the residual acts in
//! (`plans/jump-ppo-residual-polar-action.md` §1):
//!
//! ```text
//! [ r_L, s_L, r_R, s_R, s_θ ]
//!   r_i  chord / leg_len        (strut length; 1.0 = full rest extension)
//!   s_i  sin φ_i, φ off straight-down (hip→foot direction)
//!   s_θ  sin θ,  torso lean off straight-up
//! ```
//!
//! Normalized here (not px) so the clip is rig-scale agnostic; [`JumpCtrl`]
//! multiplies by the live `leg_len` / `hip_height` when it builds the pose.
//!
//! # The stroke (data)
//! `stand → crouch → extend → land-absorb → stand`, eased. The numbers reproduce
//! the previously hardcoded stroke: rest chord ≈ 110.8 px ⇒ `r ≈ 1.0`; crouch
//! depth 78 px ⇒ `r ≈ 0.70`; the small `s` offsets reproduce the rest x-offsets
//! (`−33`/`+39.5` over a 106 px depth ⇒ `s ≈ ∓0.30`, sign per vello y-down).
//! `s_θ = 0` throughout: the hop is a pure vertical pencil.

use glam::Vec2;
use walker_math::{Clip, Easing, Key, Lerp};

use crate::jump::JumpPhase;

/// Normalized chord length of the standing rest pose. `r = 1.0` means the full
/// `leg_len`, so the reconstructed chord is exactly `leg_len` (110 px) — within
/// ~1 px of the v10 rest chords (`|(−33,106)| ≈ 111`, `|(39.5,103)| ≈ 110.3`),
/// which is the accepted rig-normalization (same tiny mismatch the strut
/// absorbs everywhere else).
pub const REST_R: f32 = 1.0;
/// Standing rest `s` per leg: `sin φ = x / chord` of the v10 rest offsets
/// (`chord = |(−33,106)|`, `|(39.5,103)|`). This is the *direction* of the
/// hip→foot vector, exact regardless of the 1 px chord shortening.
pub const REST_S_L: f32 = -33.0 / 111.02;
pub const REST_S_R: f32 = 39.5 / 110.31;
/// Crouch chord: depth 78 px over `leg_len 110` ⇒ `r ≈ 0.709`.
pub const CROUCH_R: f32 = 78.0 / 110.0;

/// One key of the jump clip: the polar 5-dim pose plus the stance flags.
///
/// Stance is carried in the key (not derived) so a future clip can lift a foot
/// mid-move; the current stroke sets it `true` everywhere except Flight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JumpKey {
    /// `[r_L, s_L, r_R, s_R, s_θ]` — normalized.
    pub pose: [f32; 5],
    /// Ground-stiff (`true`) or airborne-soft (`false`) struts.
    pub stance: bool,
}

impl JumpKey {
    pub fn new(pose: [f32; 5], stance: bool) -> Self {
        Self { pose, stance }
    }

    /// The **flight** stance (soft) is not interpolable — it is a discrete mode.
    /// A blend of `stance` picks the *starting* key's flag (see [`Lerp`]).
    pub fn pose(&self) -> [f32; 5] {
        self.pose
    }
}

/// Interpolating a `JumpKey` lerps the pose and takes the *starting* key's
/// stance flag (stance is discrete — you are on the ground or you are not; a
/// 50 % stance is meaningless and would half-soften the strut).
impl Lerp for JumpKey {
    fn lerp(self, other: Self, t: f32) -> Self {
        Self {
            pose: self.pose.lerp(other.pose, t),
            stance: self.stance,
        }
    }
}

/// Build the authored jump stroke as a [`Clip<JumpKey>`] over **local stroke
/// time** (seconds since the stroke began). Times match the previous phase
/// durations: Load ends at 0.22, Launch at 0.34, Flight at 0.79.
///
/// The clip is *monotonic in shape*; the Flight→Land transition is still
/// **event-triggered** by contact in [`JumpCtrl`] (a clip cannot see the world).
/// This track is the ground-phase reference the residual corrects.
pub fn jump_clip() -> Clip<JumpKey> {
    let rest = JumpKey::new([REST_R, REST_S_L, REST_R, REST_S_R, 0.0], true);
    // Crouch: same x-offsets (kept per the module docs — sweeping inward drags
    // planted feet), shorter chord ⇒ hip descends.
    let crouch = JumpKey::new([CROUCH_R, REST_S_L, CROUCH_R, REST_S_R, 0.0], true);
    // Flight: extended shape, struts soft.
    let flight = JumpKey::new([REST_R, REST_S_L, REST_R, REST_S_R, 0.0], false);
    Clip::new(vec![
        Key::with_easing(0.0, rest, Easing::Smoothstep), // stand
        Key::with_easing(0.22, crouch, Easing::Smoothstep), // load → crouch
        Key::with_easing(0.34, rest, Easing::CubicBezier(0.3, 0.0, 0.2, 1.0)), // launch (snappy)
        Key::with_easing(0.79, flight, Easing::Linear),  // flight hold
        Key::with_easing(1.04, rest, Easing::Smoothstep), // land absorb → rest
    ])
}

/// Map a stroke-local time to the [`JumpPhase`] the expert reports this tick
/// (telemetry + stance authority). Uses the same boundaries as [`jump_clip`]:
/// ground phases before/after the flight window; contact can end Flight early
/// in [`JumpCtrl`], which overrides this.
pub fn phase_at(t: f32) -> JumpPhase {
    if t < 0.22 {
        JumpPhase::Load
    } else if t < 0.34 {
        JumpPhase::Launch
    } else if t < 0.79 {
        JumpPhase::Flight
    } else {
        JumpPhase::Land
    }
}

/// Convenience: the rest pose as a raw 5-array (used by stand-fallbacks).
pub fn rest_pose() -> [f32; 5] {
    [REST_R, REST_S_L, REST_R, REST_S_R, 0.0]
}

/// Rebuild world hip→foot offsets from a normalized key, for a rig of the given
/// `leg_len`. Mirrors [`JumpResidual`](crate::jump_residual) FK:
/// `offset = (leg_len·r·s, leg_len·r·√(1−s²))` in vello y-down (foot below hip).
pub fn key_to_offsets(pose: [f32; 5], leg_len: f32) -> (Vec2, Vec2) {
    let foot = |r: f32, s: f32| {
        let chord = (r * leg_len).max(0.0);
        let s = s.clamp(-1.0, 1.0);
        Vec2::new(chord * s, chord * (1.0 - s * s).max(0.0).sqrt())
    };
    (foot(pose[0], pose[1]), foot(pose[2], pose[3]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clip_is_sorted_and_has_the_expected_span() {
        let c = jump_clip();
        assert_eq!(c.start(), 0.0);
        assert!((c.end() - 1.04).abs() < 1e-6);
        assert!(c.keys().windows(2).all(|w| w[0].t <= w[1].t));
    }

    #[test]
    fn rest_and_crouch_endpoints_match_the_constants() {
        let c = jump_clip();
        let stand = c.sample(0.0);
        assert!((stand.pose[0] - REST_R).abs() < 1e-6);
        assert!(stand.stance);
        let crouch = c.sample(0.22);
        assert!((crouch.pose[0] - CROUCH_R).abs() < 1e-5);
        assert!(crouch.stance);
    }

    #[test]
    fn flight_key_is_soft_and_extended() {
        // The soft stance lives on the Flight key (t = 0.79). With start-key
        // interpolation semantics, sampling *at* that key reports it; the
        // controller authors stance from the phase, so this only checks the
        // data. The extended shape holds across the whole [launch, flight] span.
        let c = jump_clip();
        assert!(!c.sample(0.79).stance, "flight key is soft");
        let f = c.sample(0.6);
        assert!((f.pose[0] - REST_R).abs() < 1e-6, "extended pencil shape");
    }

    #[test]
    fn crouch_shortens_the_chord_below_rest() {
        let c = jump_clip();
        // Mid-load: chord strictly between crouch and rest.
        let mid = c.sample(0.11);
        assert!(mid.pose[0] < REST_R && mid.pose[0] > CROUCH_R);
    }

    #[test]
    fn key_to_offsets_reproduces_the_rest_geometry() {
        let (l, r) = key_to_offsets(rest_pose(), 110.0);
        // v10 rest: (−33,106) and (39.5,103), reproduced within the ~1 px chord
        // normalization (r = 1 ⇒ chord = leg_len = 110).
        assert!((l.x - (-33.0)).abs() < 2.0, "left x ~= -33, got {}", l.x);
        assert!((l.y - 106.0).abs() < 2.0, "left depth ~= 106, got {}", l.y);
        assert!((r.x - 39.5).abs() < 2.0, "right x ~= 39.5, got {}", r.x);
        assert!((r.y - 103.0).abs() < 2.0, "right depth ~= 103, got {}", r.y);
    }

    #[test]
    fn phase_at_matches_the_clip_boundaries() {
        assert_eq!(phase_at(0.0), JumpPhase::Load);
        assert_eq!(phase_at(0.21), JumpPhase::Load);
        assert_eq!(phase_at(0.22), JumpPhase::Launch);
        assert_eq!(phase_at(0.34), JumpPhase::Flight);
        assert_eq!(phase_at(0.79), JumpPhase::Land);
    }

    #[test]
    fn stance_is_discrete_across_a_blend() {
        let a = JumpKey::new([1.0, 0.0, 1.0, 0.0, 0.0], true);
        let b = JumpKey::new([1.0, 0.0, 1.0, 0.0, 0.0], false);
        let mid = a.lerp(b, 0.5);
        assert!(mid.stance, "starting key's stance wins");
    }
}
