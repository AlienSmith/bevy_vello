//! Mouse cannon test ground.
//!
//! Move the cannon with WASD (Bevy y-up world space). An aiming arrow originates
//! at the cannon and always points toward the mouse cursor. Right-click fires a
//! small dynamic circle collider along that aim direction so the character's
//! collision response can be exercised.
//!
//! The mouse wheel is left to the edge-pan camera for zoom — the cannon no longer
//! owns the wheel.
//!
//! An egui "Cannon" window lets you:
//! - set the projectile muzzle speed and inverse mass,
//! - remove all spawned projectiles at once.
//!
//! The projectile uses its own collision group (`PROJECTILE_COLLISION_GROUP`),
//! distinct from the player character's group, so projectiles collide with the
//! character (pairs with equal non-zero groups are filtered out by the broad
//! phase).
//!
//! Coordinates:
//! - Bevy world is y-up. Aim direction is computed from the cannon position and
//!   the mouse world position, both in Bevy y-up space.
//! - Vello scenes are y-down. When drawing the arrow we flip the y-axis; when
//!   spawning the projectile we pass `initial_velocity` in Bevy y-up space,
//!   because the physics integration negates the collider y for its own world.

use bevy::{
    input::{keyboard::KeyCode, mouse::MouseButton},
    prelude::*,
    window::PrimaryWindow,
};
use bevy_egui::{egui, EguiContexts};
use bevy_vello::{
    collision::{CollisionConstraintConfig, SoftBodyInitConfig},
    prelude::{kurbo, peniko},
    VelloScene, VelloSceneBundle,
};
use kurbo::Shape;

use crate::make_collision_shape;

/// Movement speed of the cannon, in px/s.
const MOVE_SPEED: f32 = 600.0;
/// Length of the aiming arrow, in world px (tail = cannon position).
const ARROW_LENGTH: f32 = 90.0;
/// Radius of the fired projectile collider, in world px.
const PROJECTILE_RADIUS: f32 = 14.0;
/// Collision group for projectiles, distinct from the player's group (=1) so
/// the broad phase does not filter out projectile-vs-playable pairs.
pub const PROJECTILE_COLLISION_GROUP: u32 = 3;
/// Default muzzle speed, in px/s.
const DEFAULT_MUZZLE_SPEED: f32 = 1200.0;
/// Default projectile inverse mass.
const DEFAULT_PROJECTILE_INV_MASS: f32 = 1.0;

/// Persistent cannon state, editable from the "Cannon" egui window.
#[derive(Resource)]
pub struct CannonParams {
    pub muzzle_speed: f32,
    pub projectile_inverse_mass: f32,
}

impl Default for CannonParams {
    fn default() -> Self {
        Self {
            muzzle_speed: DEFAULT_MUZZLE_SPEED,
            projectile_inverse_mass: DEFAULT_PROJECTILE_INV_MASS,
        }
    }
}

/// Marker on the single cannon aiming-arrow scene entity. The entity's
/// `Transform` translation is the cannon's position (Bevy y-up world space).
#[derive(Component)]
pub struct CannonIndicator;

/// Marker on fired projectiles so they can be found and removed en masse.
#[derive(Component)]
pub struct Projectile;

/// Spawn the aiming-arrow scene entity (empty initially; filled each frame).
pub fn spawn_cannon_indicator(mut commands: Commands) {
    commands.spawn((
        VelloSceneBundle {
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, 200.0)),
            ..Default::default()
        },
        CannonIndicator,
    ));
}

/// Move the cannon with WASD (Bevy y-up world space).
pub fn move_cannon_with_wasd(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut cannon: Query<&mut Transform, With<CannonIndicator>>,
) {
    let Ok(mut transform) = cannon.single_mut() else {
        return;
    };

    let mut dir = Vec2::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        dir.y += 1.0;
    }
    if keys.pressed(KeyCode::KeyS) {
        dir.y -= 1.0;
    }
    if keys.pressed(KeyCode::KeyA) {
        dir.x -= 1.0;
    }
    if keys.pressed(KeyCode::KeyD) {
        dir.x += 1.0;
    }
    if dir == Vec2::ZERO {
        return;
    }

    let dir = dir.normalize();
    transform.translation.x += dir.x * MOVE_SPEED * time.delta_secs();
    transform.translation.y += dir.y * MOVE_SPEED * time.delta_secs();
}

/// Rebuild the arrow scene in LOCAL coordinates (cannon at origin), aimed toward
/// the mouse cursor. The entity's `Transform` translation (moved by WASD) places
/// the arrow in the world, matching where the projectile spawns.
///
/// The scene is rendered in `CoordinateSpace::WorldSpace`, so the renderer adds
/// the entity's world translation to the scene's local coordinates. We therefore
/// draw the arrow with the cannon at the origin and let the transform position it.
pub fn update_cannon_indicator(
    windows: Query<&Window, With<PrimaryWindow>>,
    camera: Query<(&Camera, &GlobalTransform)>,
    mut cannon: Query<(&mut VelloScene, &Transform), With<CannonIndicator>>,
) {
    let Ok((mut scene_handle, transform)) = cannon.single_mut() else {
        return;
    };
    let Ok((camera, camera_transform)) = camera.single() else {
        return;
    };
    let Some(cursor) = windows.single().ok().and_then(|w| w.cursor_position()) else {
        return;
    };
    let Ok(mouse_world) = camera.viewport_to_world_2d(camera_transform, cursor) else {
        return;
    };

    // Cannon world position in Bevy y-up space (from the entity transform).
    let cannon_pos = Vec2::new(transform.translation.x, transform.translation.y);

    if cannon_pos == Vec2::ZERO {
        return;
    }

    // Aim direction in Bevy y-up space: from cannon position to the mouse.
    let to_mouse = mouse_world - cannon_pos;
    if to_mouse == Vec2::ZERO {
        return;
    }
    let dir_bevy = to_mouse.normalize();
    let dir_vello = Vec2::new(dir_bevy.x, -dir_bevy.y);

    // Local positions in Vello (y-down) coordinates, cannon at origin.
    let origin = Vec2::ZERO;
    let tail = origin;
    let head = origin + dir_vello * ARROW_LENGTH;

    // Arrowhead flap size.
    let flap = 20.0;
    let perp = Vec2::new(-dir_vello.y, dir_vello.x);
    let head_l = head - dir_vello * flap + perp * (flap * 0.6);
    let head_r = head - dir_vello * flap - perp * (flap * 0.6);

    let mut scene = VelloScene::default();

    // Shaft + arrowhead.
    let mut path = kurbo::BezPath::new();
    let pt = |p: Vec2| kurbo::Point::new(p.x as f64, p.y as f64);
    path.push(kurbo::PathEl::MoveTo(pt(tail)));
    path.push(kurbo::PathEl::LineTo(pt(head)));
    path.push(kurbo::PathEl::LineTo(pt(head_l)));
    path.move_to(pt(head));
    path.line_to(pt(head_r));
    scene.stroke(
        &kurbo::Stroke::new(6.0),
        kurbo::Affine::IDENTITY,
        peniko::GlowColor::new(peniko::Color::rgba(0.2, 1.0, 1.0, 0.95), 1.0),
        None,
        &path,
    );

    // Crosshair dot at the cannon muzzle.
    let dot = kurbo::Circle::new((origin.x, origin.y), 6.0);
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::IDENTITY,
        peniko::Color::rgba(1.0, 1.0, 0.2, 0.95),
        None,
        &dot,
    );

    *scene_handle = scene;
}

/// Fire a projectile from the cannon position toward the mouse cursor on
/// right-click. The projectile is a small dynamic circle collider.
pub fn fire_cannon(
    mut commands: Commands,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera: Query<(&Camera, &GlobalTransform)>,
    button: Res<ButtonInput<MouseButton>>,
    cannon: Query<&Transform, With<CannonIndicator>>,
    params: Res<CannonParams>,
) {
    if !button.just_pressed(MouseButton::Right) {
        return;
    }
    let Ok(cannon_transform) = cannon.single() else {
        return;
    };
    let Ok((camera, camera_transform)) = camera.single() else {
        return;
    };
    let Some(cursor) = windows.single().ok().and_then(|w| w.cursor_position()) else {
        return;
    };
    let Ok(mouse_world) = camera.viewport_to_world_2d(camera_transform, cursor) else {
        return;
    };

    // Aim direction in Bevy y-up space: from cannon position to the mouse.
    let cannon_pos = Vec2::new(
        cannon_transform.translation.x,
        cannon_transform.translation.y,
    );
    let to_mouse = mouse_world - cannon_pos;
    if to_mouse == Vec2::ZERO {
        return;
    }
    let dir_bevy = to_mouse.normalize();

    let make_circle = || {
        let circle = kurbo::Circle::new((0.0, 0.0), PROJECTILE_RADIUS as f64);
        let aabb = circle.bounding_box();
        (circle.to_path(0.1), aabb)
    };

    let entity = make_collision_shape(
        &mut commands,
        Vec4::new(cannon_pos.x, cannon_pos.y, 0.0, 1.0),
        make_circle,
        peniko::Brush::SolidGlow(peniko::GlowColor {
            color: peniko::Color::rgba(1.0, 0.5, 0.0, 0.95),
            glow: 1.0,
        }),
        // Bevy y-up velocity; physics negates y internally.
        dir_bevy * params.muzzle_speed,
        params.projectile_inverse_mass,
        true,
        Some(SoftBodyInitConfig::default()),
        Some(CollisionConstraintConfig::default()),
        PROJECTILE_COLLISION_GROUP,
    );
    // Track the projectile for the "remove all" button.
    commands.entity(entity).insert(Projectile);
}

/// egui "Cannon" window: live edit muzzle speed & projectile inverse mass, and
/// remove all spawned projectiles.
pub fn cannon_ui(
    mut contexts: EguiContexts,
    mut params: ResMut<CannonParams>,
    projectiles: Query<Entity, With<Projectile>>,
    mut commands: Commands,
) {
    let count = projectiles.iter().count();
    egui::Window::new("Cannon").show(contexts.ctx_mut(), |ui| {
        ui.label("WASD moves the cannon. Right-click fires.");
        ui.separator();

        ui.heading("Projectile");
        ui.add(
            egui::Slider::new(&mut params.muzzle_speed, 0.0..=4000.0).text("muzzle speed (px/s)"),
        );
        ui.add(
            egui::Slider::new(&mut params.projectile_inverse_mass, 0.01..=10.0)
                .text("inverse mass"),
        );
        ui.separator();

        ui.horizontal(|ui| {
            if ui.button("Remove all").clicked() {
                for e in &projectiles {
                    commands.entity(e).despawn();
                }
            }
            ui.label(format!("{count} projectile(s)"));
        });
    });
}
