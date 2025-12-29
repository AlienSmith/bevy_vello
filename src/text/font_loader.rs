use super::font::VelloFont;
use crate::integrations::VectorLoaderError;
use bevy::asset::{io::Reader, AssetLoader, AsyncReadExt, LoadContext};

#[derive(Default)]
pub struct VelloFontLoader;

impl AssetLoader for VelloFontLoader {
    type Asset = VelloFont;

    type Settings = ();

    type Error = VectorLoaderError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let vello_font = VelloFont::new(bytes.to_vec());

        Ok(vello_font)
    }

    fn extensions(&self) -> &[&str] {
        &["ttf"]
    }
}
