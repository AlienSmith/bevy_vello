use bevy::math::{Affine2, Mat2, Mat4, Vec2};
use bevy::prelude::*;
use bevy_vello::vello::kurbo::{BezPath, Rect, Shape};

/// A one-shot timer component. When the timer expires, a [`DelayedEventTrigger`]
/// is fired on the same entity, then the entity is despawned.
///
/// Attach payload components to the same entity and observe
/// [`DelayedEventTrigger`] to react when the timer fires.
#[derive(Component)]
pub struct DelayedEvent {
    pub timer: Timer,
}

impl DelayedEvent {
    pub fn new(seconds: f32) -> Self {
        Self {
            timer: Timer::from_seconds(seconds, TimerMode::Once),
        }
    }
}

/// Fired on an entity when its [`DelayedEvent`] timer expires.
#[derive(Event)]
pub struct DelayedEventTrigger;

/// Ticks all [`DelayedEvent`] timers. When a timer finishes, triggers
/// [`DelayedEventTrigger`] on the entity and despawns it.
pub fn tick_delayed_events(
    mut commands: Commands,
    time: Res<Time>,
    mut q: Query<(Entity, &mut DelayedEvent)>,
) {
    for (entity, mut delayed) in q.iter_mut() {
        delayed.timer.tick(time.delta());
        if delayed.timer.just_finished() {
            commands.trigger_targets(DelayedEventTrigger, entity);
            commands.entity(entity).despawn();
        }
    }
}

pub fn crate_capsuele(distance: f32, radius: f32) -> (BezPath, Rect) {
    let half_distance = distance * 0.5;
    let delta_x = (half_distance + radius) as f64;
    let delta_y = radius as f64;
    let rect = Rect::new(-delta_x, -delta_y, delta_x, delta_y);
    let rect_path = rect.to_path(0.1);
    (rect_path, rect)
}

/// Convert a linear per-frame displacement into the angular step that sweeps
/// the same arc about a pivot at distance `radius`. `θ = s / r`.
///
/// Used to derive the per-frame angular speed cap from the position speed cap
/// (`max_ang_step = max_pos_step / r_m`), so both caps describe the *same*
/// physical motion of the spine endpoints sweeping circular arcs about the P2
/// pivot — rather than two independently tuned numbers.
#[inline]
pub fn linear_to_angle(linear: f32, radius: f32) -> f32 {
    // Radius of zero means "no lever arm" — any finite linear step is an
    // arbitrarily large angular step (degenerate: the pivot coincides with
    // the point being moved).
    if radius.abs() <= f32::EPSILON {
        return f32::INFINITY;
    }
    linear / radius
}

/// Inverse of [`linear_to_angle`]: the arc length swept by an angular step at
/// distance `radius`. `s = θ · r`.
///
/// Use to back an authored `max_ang_speed` out to the implied linear cap
/// (e.g. for validation/tests), or to keep the two speed caps consistent.
#[inline]
pub fn angle_to_linear(angle: f32, radius: f32) -> f32 {
    angle * radius
}

pub fn nlerp_cos_sin(start: (f32, f32), end: (f32, f32), t: f32) -> (f32, f32) {
    // 1. Linear interpolation
    let cos_blend = start.0 + (end.0 - start.0) * t;
    let sin_blend = start.1 + (end.1 - start.1) * t;

    // 2. Calculate the shrunken length
    let length = (cos_blend * cos_blend + sin_blend * sin_blend).sqrt();

    // 3. Normalize back to length of 1
    (cos_blend / length, sin_blend / length)
}

/// Converts a 3D `Mat4` (Y-up, X-right) to a 2D [`glam::Affine2`] (Y-down, X-right).
/// Same logic as [`mat4_to_affine`] but returns a glam type instead of kurbo.
pub fn mat4_to_affine2(raw_transform: Mat4) -> Affine2 {
    let t = raw_transform.to_cols_array();

    // | a c e |     | t[0]  -t[4]   t[12] |
    // | b d f |  =  | -t[1]  t[5]  -t[13] |
    // | 0 0 1 |
    Affine2::from_mat2_translation(
        Mat2::from_cols(
            Vec2::new(t[0], -t[1]), // column 0: x-axis (Y flipped)
            Vec2::new(-t[4], t[5]), // column 1: y-axis (Y flipped)
        ),
        Vec2::new(t[12], -t[13]), // translation (Y flipped)
    )
}
