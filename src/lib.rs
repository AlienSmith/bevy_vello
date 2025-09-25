#![allow(clippy::type_complexity)]
// #![deny(missing_docs)] -- This would be great! But we are far away.
//! An integration to render SVG and Lottie assets in Bevy with Vello.

use crate::prelude::*;
use bevy::prelude::*;

mod plugin;
pub use plugin::VelloPlugin;

pub mod collision;
pub mod debug;
pub mod dock;
pub mod integrations;
pub mod render;
pub mod text;

// Re-exports
pub use velato;
pub use vello;
pub use vello_svg;

pub mod prelude {
    pub use vello::{self, kurbo, peniko, skrifa};

    pub use crate::{
        debug::DebugVisualizations,
        integrations::{VectorFile, VelloAsset, VelloAssetAlignment, VelloReplaySceneAsset},
        render::VelloCanvasMaterial,
        text::{VelloFont, VelloText, VelloTextAlignment},
        CoordinateSpace, VelloAssetBundle, VelloScene, VelloSceneBundle, VelloTextBundle,
    };

    #[cfg(feature = "experimental-dotLottie")]
    pub use crate::integrations::dot_lottie::{DotLottiePlayer, PlayerState, PlayerTransition};
    #[cfg(feature = "lottie")]
    pub use crate::integrations::lottie::{
        LottieExt, PlaybackDirection, PlaybackLoopBehavior, PlaybackOptions, PlaybackPlayMode,
        Playhead, Theme,
    };
}

/// Which coordinate space the transform is relative to.
#[derive(PartialEq, Eq, PartialOrd, Ord, Component, Default, Copy, Clone, Debug, Reflect)]
#[reflect(Component)]
pub enum CoordinateSpace {
    #[default]
    WorldSpace,
    ScreenSpace,
}

#[derive(Bundle, Default)]
pub struct VelloAssetBundle {
    /// Asset data to render
    pub vector: Handle<VelloAsset>,
    /// How the bounding asset is aligned, respective to the transform.
    pub alignment: VelloAssetAlignment,
    /// The coordinate space in which this vector should be rendered.
    pub coordinate_space: CoordinateSpace,
    /// A transform to apply to this vector
    pub transform: Transform,
    /// The global transform managed by Bevy
    pub global_transform: GlobalTransform,
    /// Whether to render debug visualizations
    pub debug_visualizations: DebugVisualizations,
    /// User indication of whether an entity is visible. Propagates down the entity hierarchy.
    pub visibility: Visibility,
    /// Whether or not an entity is visible in the hierarchy.
    pub inherited_visibility: InheritedVisibility,
    /// Algorithmically-computed indication of whether an entity is visible. Should be extracted
    /// for rendering.
    pub view_visibility: ViewVisibility,
}

#[derive(Bundle, Default)]
pub struct VelloSceneBundle {
    /// Scene to render
    pub scene: VelloScene,
    /// The coordinate space in which this scene should be rendered.
    pub coordinate_space: CoordinateSpace,
    /// A transform to apply to this scene
    pub transform: Transform,
    /// The global transform managed by Bevy
    pub global_transform: GlobalTransform,
    /// User indication of whether an entity is visible. Propagates down the entity hierarchy.
    pub visibility: Visibility,
    /// Whether or not an entity is visible in the hierarchy.
    pub inherited_visibility: InheritedVisibility,
    /// Algorithmically-computed indication of whether an entity is visible. Should be extracted
    /// for rendering.
    pub view_visibility: ViewVisibility,
}

#[derive(Bundle, Default)]
pub struct VelloTextBundle {
    /// Font to render
    pub font: Handle<VelloFont>,
    /// Text to render
    pub text: VelloText,
    /// How the bounding text is aligned, respective to the transform.
    pub alignment: VelloTextAlignment,
    /// The coordinate space in which this text should be rendered.
    pub coordinate_space: CoordinateSpace,
    /// A transform to apply to this text
    pub transform: Transform,
    /// The global transform managed by Bevy
    pub global_transform: GlobalTransform,
    /// Whether to render debug visualizations
    pub debug_visualizations: DebugVisualizations,
    /// User indication of whether an entity is visible. Propagates down the entity hierarchy.
    pub visibility: Visibility,
    /// Whether or not an entity is visible in the hierarchy.
    pub inherited_visibility: InheritedVisibility,
    /// Algorithmically-computed indication of whether an entity is visible. Should be extracted
    /// for rendering.
    pub view_visibility: ViewVisibility,
}

/// A simple newtype component wrapper for [`vello::Scene`] for rendering.
#[derive(Component, Default, Clone)]
pub struct VelloScene(vello::Scene);

pub use collision::VelloCollider;

pub use integrations::physics::VelloCollisionResponsePlugin;

impl std::ops::Deref for VelloScene {
    type Target = vello::Scene;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl std::ops::DerefMut for VelloScene {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl VelloScene {
    pub fn new() -> Self {
        Self::default()
    }
}

impl From<vello::Scene> for VelloScene {
    fn from(scene: vello::Scene) -> Self {
        Self(scene)
    }
}

#[derive(Component, Default, Clone)]
pub struct VelloSceneReplayer(vello::SceneReplayer);
impl From<vello::SceneReplayer> for VelloSceneReplayer {
    fn from(replayer: vello::SceneReplayer) -> Self {
        Self(replayer)
    }
}

pub fn add_default_light(mut commands: Commands) {
    let mut light_scene: VelloScene = VelloScene::default();
    let light_radius = 800.0;
    //let light_shape_ratio = 1.0 / 40.0;
    light_scene.push_point_light(
        kurbo::Affine::scale(light_radius * 2.0),
        &[1.0, 1.0, 1.0],
        100.0 / (light_radius as f32),
    );
    commands.spawn((VelloSceneBundle {
        scene: light_scene,
        ..Default::default()
    },));
}

pub fn mat4_to_affine(raw_transform: Mat4) -> kurbo::Affine {
    let transform: [f32; 16] = raw_transform.to_cols_array();

    // | a c e |
    // | b d f |
    // | 0 0 1 |
    let transform: [f64; 6] = [
        transform[0] as f64,   // a
        -transform[1] as f64,  // b
        -transform[4] as f64,  // c
        transform[5] as f64,   // d
        transform[12] as f64,  // e
        -transform[13] as f64, // f
    ];

    kurbo::Affine::new(transform)
}

pub fn affine_to_mat4(affine: kurbo::Affine) -> Mat4 {
    let coeffs = affine.as_coeffs();

    // The Affine coefficients are in the order:
    // [a, b, c, d, e, f] corresponding to:
    // | a c e |
    // | b d f |
    // | 0 0 1 |

    Mat4::from_cols_array(&[
        coeffs[0] as f32,
        -coeffs[1] as f32,
        0.0,
        0.0, // column 0
        -coeffs[2] as f32,
        coeffs[3] as f32,
        0.0,
        0.0, // column 1
        0.0,
        0.0,
        1.0,
        0.0, // column 2
        coeffs[4] as f32,
        -coeffs[5] as f32,
        0.0,
        1.0, // column 3
    ])
}

pub fn affine_to_transform(affine: kurbo::Affine, z: f32) -> Transform {
    let coeffs = affine.as_coeffs();

    // Extract translation
    let translation = Vec3::new(
        coeffs[4] as f32, // e (x translation)
        coeffs[5] as f32, // f (y translation)
        z,                // z translation (2D, so 0)
    );

    // Extract rotation and scale from the 2x2 matrix [a c; b d]
    let a = coeffs[0] as f32;
    let b = coeffs[1] as f32;
    let c = coeffs[2] as f32;
    let d = coeffs[3] as f32;

    // Calculate scale (assuming uniform scaling)
    let scale_x = (a * a + b * b).sqrt();
    let scale_y = (c * c + d * d).sqrt();
    let scale = Vec3::new(scale_x, scale_y, 1.0);

    // Calculate rotation (extract angle from the upper 2x2 matrix)
    let angle = b.atan2(a); // atan2(b, a) gives the rotation angle

    Transform {
        translation,
        rotation: Quat::from_rotation_z(angle),
        scale,
    }
}
