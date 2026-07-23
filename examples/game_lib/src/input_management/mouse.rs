use bevy::prelude::*;
use bevy::window::PrimaryWindow;

/// Tracks the mouse cursor position in world space. Updated every frame in
/// [`PreUpdate`] by [`update_mouse_world_position`].
///
/// This is separate from the example's `MouseStatus` resource, which serves
/// the UI/drag interaction systems.
#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct MouseWorldPosition {
    pub pos: Vec2,
}

/// Reads the cursor position from the primary window and converts it to world
/// space using the active 2D camera. Runs in [`PreUpdate`] so the position is
/// fresh before input-reading systems run in [`Update`].
pub fn update_mouse_world_position(
    windows: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut mouse_pos: ResMut<MouseWorldPosition>,
) {
    let Ok((camera, camera_transform)) = camera_query.get_single() else {
        return;
    };
    if let Some(cursor_pos) = windows
        .iter()
        .next()
        .and_then(|window| window.cursor_position())
    {
        if let Ok(world_pos) = camera.viewport_to_world_2d(camera_transform, cursor_pos) {
            mouse_pos.pos = world_pos;
        }
    }
}
