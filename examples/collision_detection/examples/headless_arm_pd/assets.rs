//! Character asset registration: push the SVG + blueprint into their loaders on
//! startup and mark each parsed asset loaded-with-dependencies once its events
//! arrive (the gate `drive_script` waits on before spawning V10).

use bevy::asset::{AssetEvent, AssetServer};
use bevy::prelude::{EventReader, Res, ResMut};

use game_lib::character_asset::{
    BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
    SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
};

use crate::config::{BP_ASSET, BP_PATH, SVG_ASSET, SVG_PATH};

pub fn register_character_assets(
    mut character_svg: ResMut<SvgCharacterAssetManager>,
    mut character_blueprint: ResMut<BlueprintCharacterAssetManager>,
    asset_server: Res<AssetServer>,
) {
    character_svg.push(
        asset_server.load(SVG_PATH),
        SvgCharacterAssetMetaData::default(),
        SVG_ASSET,
    );
    character_blueprint.push(
        asset_server.load(BP_PATH),
        BlueprintCharacterAssetMetaData::default(),
        BP_ASSET,
    );
}

pub fn mark_assets_loaded(
    mut cs: EventReader<AssetEvent<SvgCharacterAsset>>,
    mut cb: EventReader<AssetEvent<BlueprintCharacterAsset>>,
    mut svg: ResMut<SvgCharacterAssetManager>,
    mut bp: ResMut<BlueprintCharacterAssetManager>,
) {
    for e in cs.read() {
        if let AssetEvent::LoadedWithDependencies { id } = e {
            svg.mark_as_loaded(id);
        }
    }
    for e in cb.read() {
        if let AssetEvent::LoadedWithDependencies { id } = e {
            bp.mark_as_loaded(id);
        }
    }
}
