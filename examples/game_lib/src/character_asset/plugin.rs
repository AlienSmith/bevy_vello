use bevy::prelude::*;

use crate::character_asset::{
    asset_loader::{BlueprintCharacterAssetLoader, SvgCharacterAssetLoader},
    BlueprintCharacterAsset, BlueprintCharacterAssetManager, SvgCharacterAsset,
    SvgCharacterAssetManager,
};
pub struct CharacterLoaderPlugin;

impl Plugin for CharacterLoaderPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset_loader::<SvgCharacterAssetLoader>()
            .init_asset_loader::<BlueprintCharacterAssetLoader>()
            .insert_resource(SvgCharacterAssetManager::default())
            .insert_resource(BlueprintCharacterAssetManager::default())
            .init_asset::<SvgCharacterAsset>()
            .init_asset::<BlueprintCharacterAsset>();
    }
}
