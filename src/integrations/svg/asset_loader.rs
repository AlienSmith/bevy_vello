use crate::{
    integrations::{svg::load_svg_from_bytes, VectorLoaderError},
    VelloAsset,
};
use bevy::{
    asset::{io::Reader, AssetLoader, AsyncReadExt, LoadContext},
    prelude::*,
    utils::ConditionalSendFuture,
};

#[derive(Default)]
pub struct VelloSvgLoader;

impl AssetLoader for VelloSvgLoader {
    type Asset = VelloAsset;
    type Settings = ();
    type Error = VectorLoaderError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;

        // Get the standard library Path from the AssetPath
        let asset_path = load_context.asset_path();
        let path = asset_path.path();

        let ext = path
            .extension()
            .and_then(std::ffi::OsStr::to_str)
            .ok_or_else(|| {
                VectorLoaderError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Invalid file extension",
                ))
            })?;

        match ext {
            "svg" => {
                let vello_vector = load_svg_from_bytes(&bytes)?;
                // Using modern tracing formatting for 2025
                info!(
                    path = %path.display(),
                    size = ?(vello_vector.width, vello_vector.height),
                    "finished parsing svg asset"
                );
                Ok(vello_vector)
            }
            _ => Err(VectorLoaderError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Invalid file extension: '{ext}'"),
            ))),
        }
    }

    fn extensions(&self) -> &[&str] {
        &["svg"]
    }
}
