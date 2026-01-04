use bevy_vello::vello::kurbo::{BezPath, Rect, Shape};

pub fn crate_capsuele(distance: f32, radius: f32) -> (BezPath, Rect) {
    let half_distance = distance * 0.5;
    let delta_x = (half_distance + radius) as f64;
    let delta_y = radius as f64;
    let rect = Rect::new(-delta_x, -delta_y, delta_x, delta_y);
    let rect_path = rect.to_path(0.1);
    (rect_path, rect)
}
