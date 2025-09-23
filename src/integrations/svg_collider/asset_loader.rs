use crate::integrations::{
    error::ColliderLoaderError, svg::load_collider_svg_from_bytes, svg_collider::SvgColliderAsset,
};
use bevy::{
    asset::{io::Reader, AssetLoader, AsyncReadExt, LoadContext},
    prelude::*,
    utils::ConditionalSendFuture,
};
use vello::kurbo;

#[derive(Default)]
pub struct VelloColliderSvgLoader;

impl AssetLoader for VelloColliderSvgLoader {
    type Asset = SvgColliderAsset;

    type Settings = ();

    type Error = ColliderLoaderError;

    fn load<'a>(
        &'a self,
        reader: &'a mut Reader,
        _settings: &'a Self::Settings,
        load_context: &'a mut LoadContext,
    ) -> impl ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> {
        Box::pin(async move {
            let mut bytes = Vec::new();
            reader.read_to_end(&mut bytes).await?;
            let path = load_context.path().to_owned();
            let ext = path.extension().and_then(std::ffi::OsStr::to_str).ok_or(
                ColliderLoaderError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Invalid file extension",
                )),
            )?;

            debug!("parsing {}...", load_context.path().display());
            match ext {
                "svg" => {
                    let r = load_collider_svg_from_bytes(&bytes)?;
                    if let Some((path, x, y, z, w)) = r {
                        Ok(SvgColliderAsset {
                            shape: path,
                            aabb: kurbo::Rect::new(x as f64, y as f64, z as f64, w as f64),
                        })
                    } else {
                        Err(ColliderLoaderError::WrongSvgContent)
                    }
                }
                ext => Err(ColliderLoaderError::Io(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!("Invalid file extension: '{ext}'"),
                ))),
            }
        })
    }

    fn extensions(&self) -> &[&str] {
        &["collider.svg"]
    }
}
