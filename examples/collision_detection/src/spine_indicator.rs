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
/// Q/E rotation speed, in radians per second.
const ROTATE_SPEED: f32 = 1.5;

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

    // Three filled circles at the particle positions.
    for point in points {
        let circle = kurbo::Circle::new((point.x as f64, point.y as f64), CIRCLE_RADIUS as f64);
        scene.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::IDENTITY,
            peniko::Color::rgba(1.0, 0.2, 0.2, 0.7),
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
    mut q_indicator: Query<(Entity, &mut SpineIndicator, &mut Transform)>,
) {
    // Mouse → world position (bevy Y-up world coords).
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

    let Ok((_entity, mut indicator, mut transform)) = q_indicator.single_mut() else {
        return;
    };

    // Rotate about the shape centre with Q/E.
    let dt = time.delta_secs();
    if keys.pressed(KeyCode::KeyQ) {
        indicator.angle += ROTATE_SPEED * dt;
    }
    if keys.pressed(KeyCode::KeyE) {
        indicator.angle -= ROTATE_SPEED * dt;
    }

    // Snap to the cursor and apply rotation.
    transform.translation = world_pos.extend(1000.0);
    transform.rotation = Quat::from_rotation_z(indicator.angle);
}
