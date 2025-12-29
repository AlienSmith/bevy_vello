use crate::integrations::{
    error::{ColliderLoaderError, ImageLoaderError},
    svg::load_collider_svg_from_bytes,
    svg_collider::{SvgColliderAsset, VelloImageAsset},
};
use bevy::{
    asset::{io::Reader, AssetLoader, AsyncReadExt, LoadContext},
    prelude::*,
};
use vello::kurbo;

#[derive(Default)]
pub struct VelloImageAssetLoader;
impl AssetLoader for VelloImageAssetLoader {
    type Asset = VelloImageAsset;
    type Settings = ();
    type Error = ImageLoaderError;

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
            .map_err(|e| ImageLoaderError::Io(e))?;

        // Use the LoadContext to get the path more conveniently
        let path = load_context.path();
        let ext = path
            .extension()
            .and_then(std::ffi::OsStr::to_str)
            .ok_or_else(|| {
                ImageLoaderError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Invalid file extension",
                ))
            })?;

        match ext {
            "png" => {
                if let Ok(r) = vello::decode_image(&bytes) {
                    Ok(VelloImageAsset { image: r })
                } else {
                    Err(ImageLoaderError::CouldNotLoadImage)
                }
            }
            _ => Err(ImageLoaderError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Unsupported extension: {ext}"),
            ))),
        }
    }

    fn extensions(&self) -> &[&str] {
        &["png"]
    }
}

#[derive(Default)]
pub struct VelloColliderSvgLoader;

impl AssetLoader for VelloColliderSvgLoader {
    type Asset = SvgColliderAsset;
    type Settings = ();
    type Error = ColliderLoaderError;

    // Use native async fn; lifetimes are inferred and Box::pin is removed
    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        // reader.read_to_end is an async method on the Reader trait
        reader
            .read_to_end(&mut bytes)
            .await
            .map_err(|e| ColliderLoaderError::Io(e))?;

        let path = load_context.path();
        let ext = path
            .extension()
            .and_then(std::ffi::OsStr::to_str)
            .ok_or_else(|| {
                ColliderLoaderError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Invalid file extension",
                ))
            })?;

        // Optional: Use log::debug! or bevy::log::debug!
        bevy::log::debug!("parsing {:?}...", path);

        match ext {
            "svg" => {
                let r = load_collider_svg_from_bytes(&bytes)?;
                if let Some((path_data, x, y, z, w)) = r {
                    Ok(SvgColliderAsset {
                        shape: path_data,
                        aabb: kurbo::Rect::new(x as f64, y as f64, z as f64, w as f64),
                    })
                } else {
                    Err(ColliderLoaderError::WrongSvgContent)
                }
            }
            _ => Err(ColliderLoaderError::Io(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Unsupported extension: {ext}"),
            ))),
        }
    }

    fn extensions(&self) -> &[&str] {
        &["collider.svg"]
    }
}
