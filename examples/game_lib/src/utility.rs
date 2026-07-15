use bevy::math::{Affine2, Mat2, Mat4, Vec2};
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

/// Converts a 3D `Mat4` (Y-up, X-right) to a 2D [`glam::Affine2`] (Y-down, X-right).
/// Same logic as [`mat4_to_affine`] but returns a glam type instead of kurbo.
pub fn mat4_to_affine2(raw_transform: Mat4) -> Affine2 {
    let t = raw_transform.to_cols_array();

    // | a c e |     | t[0]  -t[4]   t[12] |
    // | b d f |  =  | -t[1]  t[5]  -t[13] |
    // | 0 0 1 |
    Affine2::from_mat2_translation(
        Mat2::from_cols(
            Vec2::new(t[0], -t[1]), // column 0: x-axis (Y flipped)
            Vec2::new(-t[4], t[5]), // column 1: y-axis (Y flipped)
        ),
        Vec2::new(t[12], -t[13]), // translation (Y flipped)
    )
}
