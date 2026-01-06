use bevy::{
    asset::{io::Reader, AssetLoader, LoadContext},
    prelude::*,
};

use crate::character_asset::{
    load_character_blueprint_from_bytes, load_character_svg_from_bytes, BlueprintCharacterAsset,
    BlueprintCharacterLoaderError, SvgCharacterAsset, SvgCharacterLoaderError,
};

#[derive(Default)]
pub struct BlueprintCharacterAssetLoader;

impl AssetLoader for BlueprintCharacterAssetLoader {
    type Asset = BlueprintCharacterAsset;
    type Settings = ();
    type Error = BlueprintCharacterLoaderError;
    // Bevy 0.15: Remove explicit <'a> lifetimes and Box::pin
    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader
            .read_to_end(&mut bytes)
            .await
            .map_err(|e| BlueprintCharacterLoaderError::Io(e))?;

        // Use the LoadContext to get the path more conveniently
        let path = load_context.path();
        let ext = path
            .extension()
            .and_then(std::ffi::OsStr::to_str)
            .ok_or_else(|| {
                BlueprintCharacterLoaderError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Invalid file extension",
                ))
            })?;

        match ext {
            "json" => {
                let vello_vector = load_character_blueprint_from_bytes(&bytes)?;
                // Using modern tracing formatting for 2025
                info!(
                    path = %path.display(),
                    "finished parsing character svg asset"
                );
                Ok(vello_vector)
            }
            _ => Err(BlueprintCharacterLoaderError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Unsupported extension: {ext}"),
            ))),
        }
    }

    fn extensions(&self) -> &[&str] {
        &[".character.json"]
    }
}

#[derive(Default)]
pub struct SvgCharacterAssetLoader;

impl AssetLoader for SvgCharacterAssetLoader {
    type Asset = SvgCharacterAsset;
    type Settings = ();
    type Error = SvgCharacterLoaderError;

    // Bevy 0.15: Remove explicit <'a> lifetimes and Box::pin
    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader
            .read_to_end(&mut bytes)
            .await
            .map_err(|e| SvgCharacterLoaderError::Io(e))?;

        // Use the LoadContext to get the path more conveniently
        let path = load_context.path();
        let ext = path
            .extension()
            .and_then(std::ffi::OsStr::to_str)
            .ok_or_else(|| {
                SvgCharacterLoaderError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Invalid file extension",
                ))
            })?;

        match ext {
            "svg" => {
                let vello_vector = load_character_svg_from_bytes(&bytes)?;
                // Using modern tracing formatting for 2025
                info!(
                    path = %path.display(),
                    "finished parsing character svg asset"
                );
                Ok(vello_vector)
            }
            _ => Err(SvgCharacterLoaderError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Unsupported extension: {ext}"),
            ))),
        }
    }

    fn extensions(&self) -> &[&str] {
        &[".character.svg"]
    }
}
