//! Spine indicator / spine-drive input handle.
//!
//! A single scene entity that the user drags around with the mouse (it snaps to
//! the cursor) and rotates around its center with the `Q` / `E` keys. It draws a
//! small triangle + three linking circles in a local, origin-centred frame that
//! mirrors the relative positions of the character's quad spine particles P1/P2/P3
//! (a vertical pole in the character blueprint).
//!
//! The *shape* is baked once at spawn; position and rotation live on the entity's
//! [`Transform`]. This makes it a convenient, moving/rotating target that the
//! spine controller pulls the real particles towards (external position constraint
//! with damping), instead of driving them by impulse.
//!
//! ## Spawning
//! The indicator is **not** spawned lazily by the draw system. It is spawned by
//! [`spawn_spine_indicator`], an observer on `OnAdd<SpineController>` — i.e. exactly
//! when a character is assembled. This lets us derive the three circle offsets
//! (`local_points`) directly from the real, already-scaled physics particle
//! positions, so there is no second hard-coded copy of the particle layout to
//! keep in sync.
//!
//! ## Coordinates
//! The particle rest positions in [`VelloParticle::particle_init`] are in Vello
//! (y-down) world space, already scaled by the character root's transform during
//! assembly. We store `local_points` centred on P2 in the same Vello y-down space;
//! [`compute_bevy_targets`] (in `spine_position_constraint`) and the baked scene
//! understand that convention, so the indicator shape and the constraint targets
//! both land exactly on the particles.

use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_vello::{
    integrations::physics::VelloParticle,
    prelude::{kurbo, peniko},
    VelloScene, VelloSceneBundle,
};
use game_lib::SpineController;

/// Rotation speed of the spine indicator, in degrees per second.
///
/// Adjustable at runtime with the `K` (faster) / `L` (slower) keys and clamped
/// to the `0..=720` degree-per-second range. Holding a key keeps applying the
/// step every [`REPEAT_DELAY`] seconds.
#[derive(Resource)]
pub struct RotateSpeed {
    pub angle_per_second: f32,
}

impl Default for RotateSpeed {
    fn default() -> Self {
        Self {
            angle_per_second: 720.0,
        }
    }
}

/// How much [`RotateSpeed`] changes per `K`/`L` key step (degrees per second).
const SPEED_STEP: f32 = 30.0;
/// Minimum allowed rotation speed, in degrees per second.
const MIN_SPEED: f32 = 0.0;
/// Maximum allowed rotation speed, in degrees per second.
const MAX_SPEED: f32 = 720.0;
/// How long a `K`/`L` key must be held before its step starts repeating.
const REPEAT_DELAY: f32 = 0.5;

/// Tracks how long the `K`/`L` keys have been held so the speed step repeats
/// while a key stays down.
#[derive(Default)]
pub struct RepeatHold {
    /// Accumulated hold time for the `K` (faster) key, in seconds.
    k: f32,
    /// Accumulated hold time for the `L` (slower) key, in seconds.
    l: f32,
}

/// Which input drives the indicator's position.
#[derive(Resource, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum IndicatorControl {
    /// Follow the mouse cursor.
    #[default]
    Mouse,
    /// Move with the arrow keys.
    Arrow,
}

/// Whether the spine indicator visualization is shown (toggled with the `P` key).
#[derive(Resource, Clone, Copy, PartialEq, Eq, Debug)]
pub struct IndicatorVisibility(pub bool);

impl Default for IndicatorVisibility {
    fn default() -> Self {
        Self(true)
    }
}

/// Movement speed of the indicator while using arrow-key control, in px/s
/// (adjustable in the UI).
#[derive(Resource)]
pub struct MoveSpeed {
    pub px_per_second: f32,
}

impl Default for MoveSpeed {
    fn default() -> Self {
        Self {
            px_per_second: 400.0,
        }
    }
}

/// Indices of the driven quad spine particles within
/// [`SpineController::particles`] (`[PH, P0, P1, P2, P3]`) → P1/P2/P3 at 2, 3, 4.
const QUAD_INDICES: [usize; 3] = [2, 3, 4];
/// The quad index that is the shape centre (P2 → `particles[3]`).
const CENTRE_INDEX: usize = QUAD_INDICES[1];

/// Marker + input state for the single spine-indicator entity.
#[derive(Component)]
pub struct SpineIndicator {
    /// Current rotation about the shape centre, in radians (bevy Z-rotation).
    pub angle: f32,
    /// The three quad-particle offsets, in local Vello (y-down) coords, centred
    /// on P2. Computed from the real assembled particles at spawn.
    pub local_points: [Vec2; 3],
}

impl Default for SpineIndicator {
    fn default() -> Self {
        Self {
            angle: 0.0,
            local_points: [Vec2::ZERO; 3],
        }
    }
}

/// Convert a Vello (y-down) point to Bevy (y-up).
#[inline]
fn vello_to_bevy(p: Vec2) -> Vec2 {
    Vec2::new(p.x, -p.y)
}

/// Radius of the per-particle circles, in local Vello units.
const CIRCLE_RADIUS: f32 = 12.0;
/// Stroke width of the connecting lines, in local Vello units.
const LINE_WIDTH: f64 = 4.0;

/// Build the fixed indicator shape once (local coords, centred at the origin).
fn build_indicator_scene(points: &[Vec2; 3]) -> VelloScene {
    let mut scene = VelloScene::default();

    // Connecting line: P1→P2→P3→P1 (a closed triangle, degenerate here).
    let mut line_path = kurbo::BezPath::new();
    line_path.push(kurbo::PathEl::MoveTo((points[0].x, points[0].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[1].x, points[1].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[2].x, points[2].y).into()));
    line_path.push(kurbo::PathEl::LineTo((points[0].x, points[0].y).into()));
    scene.stroke(
        &kurbo::Stroke::new(LINE_WIDTH),
        kurbo::Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(0.0, 0.8, 1.0, 0.9), 1.0),
        None,
        &line_path,
    );

    // Three filled circles at the particle positions, one distinct color each.
    let circle_colors = [
        peniko::Color::rgba(1.0, 0.2, 0.2, 0.9), // P1 — red
        peniko::Color::rgba(0.2, 0.8, 0.3, 0.9), // P2 — green (centre)
        peniko::Color::rgba(0.2, 0.4, 1.0, 0.9), // P3 — blue
    ];
    for (point, color) in points.iter().zip(circle_colors.iter()) {
        let circle = kurbo::Circle::new((point.x as f64, point.y as f64), CIRCLE_RADIUS as f64);
        scene.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::IDENTITY,
            *color,
            None,
            &circle,
        );
    }

    scene
}

/// Spawn the spine indicator when a character is assembled (`OnAdd<SpineController>`).
///
/// Reads the real P1/P2/P3 particle rest positions from [`VelloParticle`] so the
/// indicator's geometry (and later the constraint targets) exactly match the
/// assembled character — no hard-coded particle offsets.
pub fn spawn_spine_indicator(
    trigger: Trigger<OnAdd, SpineController>,
    mut commands: Commands,
    spine_q: Query<&SpineController>,
    particle_q: Query<&VelloParticle>,
) {
    let Ok(spine) = spine_q.get(trigger.target()) else {
        return;
    };

    // Centre (P2) rest position, in Vello (y-down) world space.
    let Ok(centre) = particle_q.get(spine.particles[CENTRE_INDEX]) else {
        return;
    };
    let centre_pos = centre.particle_init.pos;

    // Offsets relative to the centre — already in world/scaled space, so they are
    // the true on-screen spacing (no extra scale factor needed at render/constraint).
    let mut local_points = [Vec2::ZERO; 3];
    for (k, i) in QUAD_INDICES.iter().enumerate() {
        if let Ok(p) = particle_q.get(spine.particles[*i]) {
            local_points[k] = p.particle_init.pos - centre_pos;
        }
    }

    let scene = build_indicator_scene(&local_points);
    commands.spawn((
        VelloSceneBundle {
            scene,
            transform: Transform::from_translation(vello_to_bevy(centre_pos).extend(1000.0)),
            ..Default::default()
        },
        SpineIndicator {
            angle: 0.0,
            local_points,
        },
    ));
}

/// Snap the indicator to the mouse and rotate it about its centre with Q/E.
///
/// The indicator entity itself is spawned by [`spawn_spine_indicator`]; this
/// system only moves/rotates the existing entity, and is a no-op if it is absent.
pub fn draw_spine_indicator(
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut rotate_speed: ResMut<RotateSpeed>,
    mut control: ResMut<IndicatorControl>,
    mut visibility: ResMut<IndicatorVisibility>,
    move_speed: Res<MoveSpeed>,
    mut q_indicator: Query<(Entity, &mut SpineIndicator, &mut Transform, &mut Visibility)>,
    mut repeat: Local<RepeatHold>,
) {
    let dt = time.delta_secs();

    // Toggle between mouse control and arrow-key control with the O key.
    if keys.just_pressed(KeyCode::KeyO) {
        *control = match *control {
            IndicatorControl::Mouse => IndicatorControl::Arrow,
            IndicatorControl::Arrow => IndicatorControl::Mouse,
        };
    }
    // Toggle the indicator visualization on/off with the P key.
    if keys.just_pressed(KeyCode::KeyP) {
        visibility.0 = !visibility.0;
    }

    // Returns `true` once per step: immediately on the press, then again every
    // REPEAT_DELAY seconds while the key stays held (key auto-repeat).
    fn step(hold: &mut f32, delta: f32, pressed_now: bool, held: bool) -> bool {
        if pressed_now {
            *hold = 0.0;
            return true;
        }
        if held {
            *hold += delta;
            if *hold >= REPEAT_DELAY {
                // Keep the remainder so repeat cadence stays even.
                *hold -= REPEAT_DELAY;
                return true;
            }
        } else {
            *hold = 0.0;
        }
        false
    }

    // Adjust rotation speed with K (faster) / L (slower), clamped to the
    // 0..=720 degree-per-second range.
    if step(
        &mut repeat.k,
        dt,
        keys.just_pressed(KeyCode::KeyK),
        keys.pressed(KeyCode::KeyK),
    ) {
        rotate_speed.angle_per_second =
            (rotate_speed.angle_per_second + SPEED_STEP).clamp(MIN_SPEED, MAX_SPEED);
    }
    if step(
        &mut repeat.l,
        dt,
        keys.just_pressed(KeyCode::KeyL),
        keys.pressed(KeyCode::KeyL),
    ) {
        rotate_speed.angle_per_second =
            (rotate_speed.angle_per_second - SPEED_STEP).clamp(MIN_SPEED, MAX_SPEED);
    }
    let Ok((_entity, mut indicator, mut transform, mut vis)) = q_indicator.single_mut() else {
        return;
    };
    // Apply the P-key visibility toggle to the rendered entity.
    *vis = if visibility.0 {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    };

    // Rotate about the shape centre with Q/E at the current adjustable speed
    // (the resource is stored in degrees/second, so convert to radians/second).
    let speed_rad_per_s = rotate_speed.angle_per_second.to_radians();
    if keys.pressed(KeyCode::KeyQ) {
        indicator.angle += speed_rad_per_s * dt;
    }
    if keys.pressed(KeyCode::KeyE) {
        indicator.angle -= speed_rad_per_s * dt;
    }

    match *control {
        // Mouse control: snap to the cursor.
        IndicatorControl::Mouse => {
            let Some(mouse) = windows
                .iter()
                .next()
                .and_then(|window| window.cursor_position())
            else {
                return;
            };
            let Ok((camera, camera_transform)) = camera_query.single() else {
                return;
            };
            let Ok(world_pos) = camera.viewport_to_world_2d(camera_transform, mouse) else {
                return;
            };
            transform.translation = world_pos.extend(1000.0);
        }
        // Arrow-key control: move with the arrow keys.
        IndicatorControl::Arrow => {
            let mut delta = Vec2::ZERO;
            if keys.pressed(KeyCode::ArrowUp) {
                delta.y += 1.0;
            }
            if keys.pressed(KeyCode::ArrowDown) {
                delta.y -= 1.0;
            }
            if keys.pressed(KeyCode::ArrowLeft) {
                delta.x -= 1.0;
            }
            if keys.pressed(KeyCode::ArrowRight) {
                delta.x += 1.0;
            }
            if delta != Vec2::ZERO {
                delta = delta.normalize() * move_speed.px_per_second;
                transform.translation.x += delta.x * dt;
                transform.translation.y += delta.y * dt;
            }
        }
    }

    transform.rotation = Quat::from_rotation_z(indicator.angle);
}
