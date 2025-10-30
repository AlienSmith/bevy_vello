use bevy::prelude::*;

use crate::integrations::svg_collider::{
    asset_loader::{VelloColliderSvgLoader, VelloImageAssetLoader},
    SvgColliderAsset, SvgColliderAssetManager, VelloImageAsset, VelloImageAssetManager,
};

pub struct SvgColliderPlugin;

impl Plugin for SvgColliderPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset_loader::<VelloColliderSvgLoader>()
            .init_asset_loader::<VelloImageAssetLoader>()
            .insert_resource(SvgColliderAssetManager::default())
            .insert_resource(VelloImageAssetManager::default())
            .init_asset::<SvgColliderAsset>()
            .init_asset::<VelloImageAsset>();
    }
}
