use bevy::{prelude::*, window::PrimaryWindow};
#[derive(Clone, Component)]
pub struct Turrent {
    /// rotation speed in radians per second
    rotation_speed: f32,
}

impl Default for Turrent {
    fn default() -> Self {
        Self {
            rotation_speed: f32::to_radians(20.0),
        }
    }
}

impl Turrent {
    pub fn new(angle: f32) -> Self {
        Self {
            rotation_speed: f32::to_radians(angle),
        }
    }
}

pub fn control_system(
    time: Res<Time>,
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut query_scene: Query<(&mut Transform, &Turrent, &GlobalTransform)>,
) {
    let (camera, camera_transform) = camera_query.single();
    let mouse_pos = if let Some(mouse_position) = windows
        .iter()
        .next()
        .and_then(|window| window.cursor_position())
    {
        Some(mouse_position)
    } else {
        None
    };
    let (mut transform, turrent, global_transform) = query_scene.single_mut();
    if let Some(cursor_position) = mouse_pos {
        if let Some(world_pos) = camera.viewport_to_world_2d(camera_transform, cursor_position) {
            let dif = (world_pos - global_transform.translation().xy()).normalize();
            let dif_3 = Vec3::new(dif.x, dif.y, 0.0);
            let y = Vec3::new(0.0, 1.0, 0.0);
            let global_to_local = global_transform.compute_matrix().inverse();
            let dif_3_local = global_to_local.transform_vector3(dif_3);
            let cross = y.cross(dif_3_local).z;
            let angle = y.angle_between(dif_3_local);
            let rotate_direction = cross.signum();
            if angle > 0.01 {
                transform.rotate_z(
                    rotate_direction * angle.min(turrent.rotation_speed * time.delta_seconds()),
                );
            }
        }
    }
}
