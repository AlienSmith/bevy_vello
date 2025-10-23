use bevy::{prelude::*, window::PrimaryWindow};
#[derive(Resource, Default)]
pub struct MouseStatus {
    pub world_pos: Vec2,
    pub screen_pos: Vec2,
}

pub fn update_mouse_position(
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut p: ResMut<MouseStatus>,
) {
    let (camera, camera_transform) = camera_query.single();
    if let Some(mouse_position) = windows
        .iter()
        .next()
        .and_then(|window| window.cursor_position())
    {
        if let Some(world_pos) = camera.viewport_to_world_2d(camera_transform, mouse_position) {
            p.screen_pos = mouse_position;
            p.world_pos = world_pos;
        }
    }
}
