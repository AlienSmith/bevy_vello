use bevy::prelude::*;

use crate::integrations::svg_collider::{
    asset_loader::VelloColliderSvgLoader, SvgColliderAsset, SvgColliderAssetManager,
};

pub struct SvgColliderPlugin;

impl Plugin for SvgColliderPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset_loader::<VelloColliderSvgLoader>()
            .insert_resource(SvgColliderAssetManager::default())
            .init_asset::<SvgColliderAsset>();
    }
}
