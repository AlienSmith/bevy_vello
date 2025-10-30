use bevy::prelude::*;
use vello::{
    kurbo::{self, BezPath},
    peniko::Image,
};

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

#[derive(Asset, TypePath, Clone)]
pub struct VelloImageAsset {
    pub image: Image,
}

#[derive(Copy, Clone, Default)]
pub struct VelloImageAssetMetaData;

impl AssetWithMeta for VelloImageAsset {
    type Meta = VelloImageAssetMetaData;
}

pub type VelloImageAssetManager = VelloAssetManager<VelloImageAsset>;
