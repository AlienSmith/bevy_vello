//! General **keyframe-track interpolation** — the reusable animation core.
//!
//! This is the pose-agnostic half of "author a motion as data". A [`Clip<T>`] is
//! a sorted list of [`Key<T>`]s; [`Clip::sample`] interpolates between the two
//! keys surrounding a time, applying the *starting* key's [`Easing`] to the
//! segment. It is deliberately independent of poses, legs, or sockets — the
//! same machinery drives a jump stroke, a walk cycle, a wave, or anything else
//! that is a `T: Lerp`.
//!
//! The model mirrors Lottie's (which this repo already consumes via `velato`):
//! per-segment easing, `Step` hold keys, and `CubicBezier(x1,y1,x2,y2)` handles
//! (the format's `i`/`o` control points). We reimplement it here rather than
//! depend on the parser because our keys are not SVG paths — they are typed
//! poses — and the interpolation is the only piece we need.
//!
//! ```text
//! t0 ──── key A ──[easing]──► key B ──[easing]──► key C ──── t_end
//! ```
//!
//! Sampling clamps to the track's `[t_first, t_last]` range, so a caller may
//! pass a local phase time without pre-clamping.

use glam::Vec2;

/// Values a [`Clip`] can interpolate. Implemented for the scalar and vector
/// primitives this crate uses; any pose type elsewhere implements it too.
pub trait Lerp: Copy {
    /// Linear blend: `self` at `t = 0`, `other` at `t = 1`.
    fn lerp(self, other: Self, t: f32) -> Self;
}

impl Lerp for f32 {
    #[inline]
    fn lerp(self, other: Self, t: f32) -> Self {
        self + (other - self) * t
    }
}

impl Lerp for Vec2 {
    #[inline]
    fn lerp(self, other: Self, t: f32) -> Self {
        self + (other - self) * t
    }
}

impl Lerp for [f32; 2] {
    #[inline]
    fn lerp(self, other: Self, t: f32) -> Self {
        [self[0].lerp(other[0], t), self[1].lerp(other[1], t)]
    }
}

impl Lerp for [f32; 5] {
    #[inline]
    fn lerp(self, other: Self, t: f32) -> Self {
        let mut out = [0.0f32; 5];
        for i in 0..5 {
            out[i] = self[i].lerp(other[i], t);
        }
        out
    }
}

/// The `smoothstep` easing `3t² − 2t³` (a.k.a. Lottie's default "easy ease").
#[inline]
pub fn smoothstep(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Per-segment timing curve, applied to the normalized `t ∈ [0, 1]` of a
/// segment. Stored on the segment's **starting** key (Lottie's convention).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Easing {
    /// Straight linear blend.
    Linear,
    /// Hold the starting value for the whole segment, then jump at its end.
    Step,
    /// `3t² − 2t³` — the "easy ease".
    Smoothstep,
    /// CSS/Lottie cubic-bezier with control points `(x1, y1)` and `(x2, y2)`,
    /// both `x`-components clamped to `[0, 1]` for a well-defined `x → t` map.
    CubicBezier(f32, f32, f32, f32),
}

impl Default for Easing {
    fn default() -> Self {
        Easing::Smoothstep
    }
}

impl Easing {
    /// Map normalized time `t ∈ [0, 1]` through the curve. Output is *not*
    /// clamped to `[0, 1]` in general (an overshooting bezier may exceed it);
    /// the chosen curves here stay monotone, but callers must not assume it.
    pub fn apply(self, t: f32) -> f32 {
        let t = t.clamp(0.0, 1.0);
        match self {
            Easing::Linear => t,
            Easing::Step => 0.0,
            Easing::Smoothstep => smoothstep(t),
            Easing::CubicBezier(x1, y1, x2, y2) => cubic_bezier(t, x1, y1, x2, y2),
        }
    }
}

/// Evaluate a cubic-bezier easing at `x = t`, solving for the curve parameter
/// `s` such that the curve's `x(s) = t`, then returning `y(s)`.
///
/// This is the standard Lottie/CSS algorithm: the curve is
/// `B(s) = 3(1−s)²s·P1 + 3(1−s)s²·P2 + s³` with `P0 = (0,0)`, `P3 = (1,1)`.
pub fn cubic_bezier(t: f32, x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    // Degenerate: a linear handle.
    if x1 == y1 && x2 == y2 {
        return t;
    }
    let cx = |s: f32| {
        let u = 1.0 - s;
        3.0 * u * u * s * x1 + 3.0 * u * s * s * x2 + s * s * s
    };
    let cy = |s: f32| {
        let u = 1.0 - s;
        3.0 * u * u * s * y1 + 3.0 * u * s * s * y2 + s * s * s
    };
    // Newton-Raphson, then a bisection fallback for the rare flat region.
    let mut s = t;
    for _ in 0..8 {
        let x = cx(s) - t;
        if x.abs() < 1e-6 {
            return cy(s);
        }
        let u = 1.0 - s;
        let dx = 3.0 * u * u * x1 + 6.0 * u * s * (x2 - x1) + 3.0 * s * s * (1.0 - x2);
        if dx.abs() < 1e-6 {
            break;
        }
        s -= x / dx;
        s = s.clamp(0.0, 1.0);
    }
    let (mut lo, mut hi) = (0.0f32, 1.0f32);
    let mut s = t.clamp(0.0, 1.0);
    for _ in 0..32 {
        let x = cx(s);
        if (x - t).abs() < 1e-6 {
            break;
        }
        if x < t {
            lo = s;
        } else {
            hi = s;
        }
        s = 0.5 * (lo + hi);
    }
    cy(s)
}

/// One keyframe. `easing` shapes the segment that **starts** at this key; it is
/// ignored on the final key (nothing follows it).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Key<T> {
    /// Absolute time within the track, seconds. Order is restored by
    /// [`Clip::new`].
    pub t: f32,
    /// The value held exactly at `t`.
    pub value: T,
    /// Easing of the segment from this key to the next.
    pub easing: Easing,
}

impl<T> Key<T> {
    pub fn new(t: f32, value: T) -> Self {
        Self {
            t,
            value,
            easing: Easing::Smoothstep,
        }
    }
    pub fn with_easing(t: f32, value: T, easing: Easing) -> Self {
        Self { t, value, easing }
    }
}

/// A sorted keyframe track over `T: Lerp`.
#[derive(Debug, Clone, PartialEq)]
pub struct Clip<T> {
    keys: Vec<Key<T>>,
}

impl<T: Lerp> Clip<T> {
    /// Build a track. Keys are sorted by time (a stable sort, so equal times
    /// keep insertion order). Panics on an empty track — a clip must have at
    /// least one key to sample.
    pub fn new(mut keys: Vec<Key<T>>) -> Self {
        assert!(!keys.is_empty(), "a Clip needs at least one key");
        keys.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
        Self { keys }
    }

    pub fn keys(&self) -> &[Key<T>] {
        &self.keys
    }

    /// Time of the first key.
    pub fn start(&self) -> f32 {
        self.keys.first().unwrap().t
    }

    /// Time of the last key.
    pub fn end(&self) -> f32 {
        self.keys.last().unwrap().t
    }

    /// Span from the first to the last key, seconds.
    pub fn duration(&self) -> f32 {
        (self.end() - self.start()).max(0.0)
    }

    /// Sample at absolute time `t`. Clamps outside `[start, end]`: before the
    /// first key returns the first value, after the last returns the last.
    pub fn sample(&self, t: f32) -> T {
        let first = &self.keys[0];
        if t <= first.t {
            return first.value;
        }
        let last = self.keys.last().unwrap();
        if t >= last.t {
            return last.value;
        }
        // Find the segment `i` with `keys[i].t <= t < keys[i+1].t`.
        let mut lo = 0usize;
        let mut hi = self.keys.len() - 1;
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if self.keys[mid].t <= t {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let a = &self.keys[lo];
        let b = &self.keys[lo + 1];
        let span = b.t - a.t;
        if span <= 0.0 {
            return b.value;
        }
        let u = (t - a.t) / span;
        let k = a.easing.apply(u);
        a.value.lerp(b.value, k)
    }

    /// Sample by normalized position in `[0, 1]` across the whole track.
    pub fn sample01(&self, u: f32) -> T {
        self.sample(self.start() + u.clamp(0.0, 1.0) * self.duration())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lerps_between_two_keys_and_clamps_outside() {
        let c = Clip::new(vec![
            Key::with_easing(0.0, 0.0f32, Easing::Linear),
            Key::with_easing(1.0, 10.0f32, Easing::Linear),
        ]);
        assert_eq!(c.sample(-1.0), 0.0);
        assert_eq!(c.sample(0.0), 0.0);
        assert!((c.sample(0.5) - 5.0).abs() < 1e-6);
        assert_eq!(c.sample(1.0), 10.0);
        assert_eq!(c.sample(2.0), 10.0);
    }

    #[test]
    fn sorts_keys_by_time() {
        let c = Clip::new(vec![Key::new(1.0, 10.0f32), Key::new(0.0, 0.0f32)]);
        assert_eq!(c.start(), 0.0);
        assert_eq!(c.end(), 1.0);
        assert_eq!(c.duration(), 1.0);
        assert!((c.sample(0.5) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn step_holds_then_jumps() {
        let c = Clip::new(vec![
            Key::with_easing(0.0, 1.0f32, Easing::Step),
            Key::with_easing(1.0, 9.0f32, Easing::Step),
        ]);
        assert_eq!(c.sample(0.99), 1.0);
        assert_eq!(c.sample(1.0), 9.0);
    }

    #[test]
    fn smoothstep_is_slow_at_the_ends() {
        let c = Clip::new(vec![
            Key::with_easing(0.0, 0.0f32, Easing::Smoothstep),
            Key::with_easing(1.0, 1.0f32, Easing::Linear),
        ]);
        // Smoothstep(0.1) = 0.028 < 0.1 ⇒ lower than linear at the start.
        assert!(c.sample(0.1) < 0.1);
        assert!((c.sample(0.5) - 0.5).abs() < 1e-6);
        assert!(c.sample(0.9) > 0.9);
    }

    #[test]
    fn cubic_bezier_is_a_valid_curve() {
        // ease-in-out: (0.42,0)(0.58,1) ⇒ symmetric, maps 0→0, 1→1, mid→0.5.
        let e = Easing::CubicBezier(0.42, 0.0, 0.58, 1.0);
        assert!((e.apply(0.0) - 0.0).abs() < 1e-5);
        assert!((e.apply(1.0) - 1.0).abs() < 1e-5);
        assert!((e.apply(0.5) - 0.5).abs() < 1e-3);
        assert!(e.apply(0.25) < 0.25);
        assert!(e.apply(0.75) > 0.75);
        // Degenerate linear handle.
        let lin = Easing::CubicBezier(0.5, 0.5, 0.5, 0.5);
        assert!((lin.apply(0.3) - 0.3).abs() < 1e-6);
    }

    #[test]
    fn vec2_and_arrays_interpolate_componentwise() {
        let c = Clip::new(vec![
            Key::with_easing(0.0, Vec2::new(0.0, 10.0), Easing::Linear),
            Key::with_easing(2.0, Vec2::new(20.0, 0.0), Easing::Linear),
        ]);
        assert!((c.sample(1.0) - Vec2::new(10.0, 5.0)).length() < 1e-6);

        let a = Clip::new(vec![
            Key::with_easing(0.0, [0.0f32; 5], Easing::Linear),
            Key::with_easing(1.0, [1.0f32; 5], Easing::Linear),
        ]);
        let got = a.sample(0.5);
        assert!(got.iter().all(|v| (v - 0.5).abs() < 1e-6));
    }

    #[test]
    fn multi_segment_track_picks_the_right_segment() {
        let c = Clip::new(vec![
            Key::with_easing(0.0, 0.0f32, Easing::Linear),
            Key::with_easing(1.0, 1.0f32, Easing::Linear),
            Key::with_easing(3.0, 5.0f32, Easing::Linear),
        ]);
        // Second segment runs t∈[1,3] → 1..5.
        assert!((c.sample(2.0) - 3.0).abs() < 1e-6);
        // sample01(0.5) ⇒ t = 0 + 0.5·3 = 1.5 ⇒ second segment, u=0.25 ⇒ 1+4·0.25.
        assert!((c.sample01(0.5) - 2.0).abs() < 1e-6);
    }

    #[test]
    fn duplicate_times_do_not_divide_by_zero() {
        let c = Clip::new(vec![
            Key::with_easing(0.0, 0.0f32, Easing::Linear),
            Key::with_easing(0.0, 7.0f32, Easing::Linear),
            Key::with_easing(1.0, 9.0f32, Easing::Linear),
        ]);
        assert!(c.sample(0.5).is_finite());
        assert_eq!(c.sample(1.0), 9.0);
    }
}
