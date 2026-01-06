mod asset_loader;
pub mod plugin;

use bevy::{platform::collections::HashMap, prelude::*};
use bevy_vello::{
    integrations::{AssetWithMeta, VelloAssetManager},
    vello::kurbo::BezPath,
    vello_svg::{self, usvg},
};
use thiserror::Error;
use vello::kurbo;
use vello_physics::CharacterBlueprint;
#[derive(Asset, TypePath, Clone)]
pub struct BlueprintCharacterAsset {
    pub data: CharacterBlueprint,
}

#[derive(Copy, Clone, Default)]
pub struct BlueprintCharacterAssetMetaData;

impl AssetWithMeta for BlueprintCharacterAsset {
    type Meta = BlueprintCharacterAssetMetaData;
}

pub type BlueprintCharacterAssetManager = VelloAssetManager<BlueprintCharacterAsset>;

#[derive(Debug, Error)]
pub enum BlueprintCharacterLoaderError {
    // Io error
    #[error("Could not load file: {0}")]
    Io(#[from] std::io::Error),
    // string processing error
    #[error("Could not parse utf-8: {0}")]
    FromStrUtf8(#[from] std::str::Utf8Error),
    // serde json error
    #[error("Could not deserialize JSON: {0}")]
    Json(#[from] serde_json::Error),
}

pub fn load_character_blueprint_from_bytes(
    bytes: &[u8],
) -> Result<BlueprintCharacterAsset, BlueprintCharacterLoaderError> {
    // ? will automatically convert serde_json::Error into BlueprintCharacterLoaderError::Json
    let blueprint: CharacterBlueprint = serde_json::from_slice(bytes)?;

    Ok(BlueprintCharacterAsset { data: blueprint })
}

#[derive(Asset, TypePath, Clone)]
pub struct SvgCharacterAsset {
    pub data: HashMap<String, (BezPath, kurbo::Rect)>,
}

#[derive(Copy, Clone, Default)]
pub struct SvgCharacterAssetMetaData;

impl AssetWithMeta for SvgCharacterAsset {
    type Meta = SvgCharacterAssetMetaData;
}

pub type SvgCharacterAssetManager = VelloAssetManager<SvgCharacterAsset>;

#[derive(Debug, Error)]
pub enum SvgCharacterLoaderError {
    #[error("Could not load file: {0}")]
    Io(#[from] std::io::Error),
    #[error("Could not parse utf-8: {0}")]
    FromStrUtf8(#[from] std::str::Utf8Error),
    #[error("Could not parse svg: {0}")]
    Usvg(#[from] vello_svg::usvg::Error),
    #[error("The Svg Can not be loaded as a Character")]
    WrongSvgContent,
    #[error("The Svg Contains Duplicated Content: {0}")]
    DuplicateContent(String),
}

pub fn load_character_svg_from_bytes(
    bytes: &[u8],
) -> Result<SvgCharacterAsset, SvgCharacterLoaderError> {
    let svg_str = std::str::from_utf8(bytes)?;
    //this svg won't contain fonts
    let mut map: HashMap<String, (BezPath, kurbo::Rect)> = HashMap::new();
    let usvg = usvg::Tree::from_str(svg_str, &usvg::Options::default(), &Default::default())?;
    if let Ok(mut result) = vello_svg::extract_shape_in_colliders(&usvg) {
        for (name, path) in result.drain(..) {
            if map.insert(name.clone(), path).is_some() {
                return Err(SvgCharacterLoaderError::DuplicateContent(name));
            }
        }
    } else {
        return Err(SvgCharacterLoaderError::WrongSvgContent);
    }
    Ok(SvgCharacterAsset { data: map })
}
