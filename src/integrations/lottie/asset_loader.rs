use crate::{
    integrations::{lottie::load_lottie_from_bytes, VectorLoaderError},
    VelloAsset,
};
use bevy::{
    asset::{io::Reader, AssetLoader, AsyncReadExt, LoadContext},
    prelude::*,
    utils::ConditionalSendFuture,
};

#[derive(Default)]
pub struct VelloLottieLoader;

impl AssetLoader for VelloLottieLoader {
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

        // In 0.15, LoadContext path access is slightly different
        let path = load_context.asset_path().path();
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
            "json" => {
                let vello_vector = load_lottie_from_bytes(&bytes)?;
                // Using direct path display for logging
                info!(
                    path = %path.display(),
                    size = ?(vello_vector.width, vello_vector.height),
                    "finished parsing lottie json asset"
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
        &["json"]
    }
}
