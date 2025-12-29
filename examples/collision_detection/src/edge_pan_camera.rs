use bevy::input::mouse::MouseWheel;
pub use bevy::prelude::*;
pub use bevy::window::PrimaryWindow;
#[derive(Component)]
pub struct EdgePanCamera {
    pub pan_speed: f32,
    pub edge_margin: f32, // pixels from edge that triggers movement
    pub zoom_level: f32,
}

impl Default for EdgePanCamera {
    fn default() -> Self {
        Self {
            pan_speed: 500.0,
            edge_margin: 50.0,
            zoom_level: 1.5, //The application always use a DPI scaling of 1.5 despite os settings,
                             //in other words the logical size and physical size of the view port is always different.
                             //could be a problem with winit or bevy. use zoom to make it 1 unit to 1 px.
        }
    }
}

pub fn update_edge_pan_camera(
    windows: Query<&Window, With<PrimaryWindow>>,
    mut scroll_events: EventReader<MouseWheel>,
    mut cameras: Query<(&mut Transform, &mut EdgePanCamera, &mut Projection)>,
    time: Res<Time>,
) {
    let window = windows.single().unwrap();

    let Ok((mut transform, mut camera, mut projection)) = cameras.single_mut() else {
        return;
    };

    if let Projection::Orthographic(ref mut ortho) = *projection {
        for event in scroll_events.read() {
            // Update your custom zoom_level
            camera.zoom_level = (camera.zoom_level - event.y * 0.1).clamp(0.01, 5.0);
        }

        // 3. This now correctly updates the actual camera projection
        ortho.scale = camera.zoom_level;
    }

    if let Some(cursor_pos) = window.cursor_position() {
        let window_size = Vec2::new(window.width(), window.height());
        let mut movement = Vec2::ZERO;

        // Check each edge (with margin)
        if cursor_pos.x < camera.edge_margin {
            movement.x -= 1.0; // Left edge
        } else if cursor_pos.x > window_size.x - camera.edge_margin {
            movement.x += 1.0; // Right edge
        }

        if cursor_pos.y < camera.edge_margin {
            movement.y += 1.0; // Bottom edge (Bevy has Y-up)
        } else if cursor_pos.y > window_size.y - camera.edge_margin {
            movement.y -= 1.0; // Top edge
        }

        // Normalize diagonal movement
        if movement.length() > 0.0 {
            movement = movement.normalize();
        }

        // Apply movement (scaled by zoom and time)
        let move_speed = camera.pan_speed * camera.zoom_level;
        transform.translation += (movement * move_speed * time.delta_secs()).extend(0.0);
    }
}
