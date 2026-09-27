//! Spine indicator / spine-drive input handle.
//!
//! A single scene entity that the user drags around with the mouse (it snaps to
//! the cursor) and rotates around its center with the `Q` / `E` keys. It draws a
//! small triangle + three linking circles in a local, origin-centred frame that
//! mirrors the relative positions of the character's quad particles P0/P1/P2
//! (a vertical pole in the character blueprint).
//!
//! The *shape* is baked once at spawn; position and rotation live on the entity's
//! [`Transform`]. This makes it a convenient, moving/rotating target that the
//! future spine controller can pull the real particles towards (position
//! constraint with damping), instead of driving them by impulse.
//!
//! ## Coordinates
//! The scene shape is authored in local (Vello) units centred at the origin
//! (`local_p0/local_p1/local_p2`). The render pipeline (`prepare_scene_affines`)
//! maps the entity's `GlobalTransform` to a Vello affine, so:
//! - `Translation` → mouse position (via `viewport_to_world_2d`, bevy Y-up; the
//!   renderer flips Y so the shape lands exactly under the cursor).
//! - `Rotation.z` → rotation about the shape centre, driven by Q/E.

use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_vello::{
    prelude::{kurbo, peniko},
    VelloScene, VelloSceneBundle,
};

/// Marker + input state for the single spine-indicator entity.
#[derive(Component)]
pub struct SpineIndicator {
    /// Current rotation about the shape centre, in radians (bevy Z-rotation).
    pub angle: f32,
    /// The three quad-particle positions, in local Vello coords, centred on P1.
    pub local_points: [Vec2; 3],
}

impl Default for SpineIndicator {
    fn default() -> Self {
        Self {
            angle: 0.0,
            // P0/P1/P2 are stacked vertically x≈347 with ~59 spaces between them
            // (SVG: y 143.75, 202.75, 275.75) → half-span 66, centred on P1.
            local_points: [
                Vec2::new(0.0, -66.0), // P0
                Vec2::new(0.0, 0.0),   // P1 (shape centre)
                Vec2::new(0.0, 66.0),  // P2
            ],
        }
    }
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

    // Connecting line: P0→P1→P2→P0 (a closed triangle, degenerate here).
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

/// Update/snap the indicator to the mouse and rotate it with Q/E.
pub fn draw_spine_indicator(
    mut commands: Commands,
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

    if let Ok((_entity, mut indicator, mut transform)) = q_indicator.single_mut() {
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
    } else {
        // Spawn the single indicator entity once, with its shape baked in.
        let scene = build_indicator_scene(&SpineIndicator::default().local_points);
        commands.spawn((
            VelloSceneBundle {
                scene,
                transform: Transform::from_translation(world_pos.extend(1000.0)),
                ..Default::default()
            },
            SpineIndicator::default(),
        ));
    }
}
