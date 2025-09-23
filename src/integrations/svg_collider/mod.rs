use bevy::prelude::*;
use vello::kurbo::{self, BezPath};

mod asset_loader;

mod plugin;

pub use plugin::SvgColliderPlugin;

use crate::integrations::{asset::AssetWithMeta, VelloAssetManager};

#[derive(Asset, TypePath, Clone)]
pub struct SvgColliderAsset {
    pub shape: BezPath,
    pub aabb: kurbo::Rect,
}

#[derive(Copy, Clone, Default)]
pub struct VelloColliderAssetMetaData;

impl AssetWithMeta for SvgColliderAsset {
    type Meta = VelloColliderAssetMetaData;
}

pub type SvgColliderAssetManager = VelloAssetManager<SvgColliderAsset>;
