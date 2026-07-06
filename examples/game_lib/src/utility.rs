use bevy_vello::vello::kurbo::{BezPath, Rect, Shape};

pub fn crate_capsuele(distance: f32, radius: f32) -> (BezPath, Rect) {
    let half_distance = distance * 0.5;
    let delta_x = (half_distance + radius) as f64;
    let delta_y = radius as f64;
    let rect = Rect::new(-delta_x, -delta_y, delta_x, delta_y);
    let rect_path = rect.to_path(0.1);
    (rect_path, rect)
}

pub fn nlerp_cos_sin(start: (f32, f32), end: (f32, f32), t: f32) -> (f32, f32) {
    // 1. Linear interpolation
    let cos_blend = start.0 + (end.0 - start.0) * t;
    let sin_blend = start.1 + (end.1 - start.1) * t;

    // 2. Calculate the shrunken length
    let length = (cos_blend * cos_blend + sin_blend * sin_blend).sqrt();

    // 3. Normalize back to length of 1
    (cos_blend / length, sin_blend / length)
}
