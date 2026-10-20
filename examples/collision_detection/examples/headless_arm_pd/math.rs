//! Small frame / math helpers shared by the harness modules.

use bevy::prelude::Vec2;

/// Engine Vello (y-down) → Bevy (y-up).
#[inline]
pub fn to_bevy(p: Vec2) -> Vec2 {
    Vec2::new(p.x, -p.y)
}

/// Bevy (y-up) → engine Vello (y-down). The inverse of [`to_bevy`]; used to
/// export the leg controllers' bevy y-up world targets in the same y-down frame
/// as the realized particle channels, so `t<name>x/y` can be subtracted from
/// `<name>x/y` directly.
#[inline]
pub fn to_vello(p: Vec2) -> Vec2 {
    Vec2::new(p.x, -p.y)
}

#[inline]
pub fn cross(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Signed angular velocity (rad/s) of a bone `arm` (far end − pivot) whose far
/// end moves with relative velocity `dvel`: `ω = cross(arm, dvel) / |arm|²`.
#[inline]
pub fn bone_ang_vel(arm: Vec2, dvel: Vec2) -> f32 {
    let l2 = arm.length_squared();
    if l2 > f32::EPSILON {
        cross(arm, dvel) / l2
    } else {
        0.0
    }
}

/// Parse `f32` from an env var, falling back to `default`.
pub fn env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// Truthy env flag (`1`/`true`/`yes`).
pub fn env_flag(key: &str) -> bool {
    matches!(
        std::env::var(key).as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}
