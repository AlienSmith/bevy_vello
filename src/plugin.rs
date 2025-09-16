use crate::{
    collision::VelloCollisionPlugin, debug::DebugVisualizationsPlugin,
    integrations::VelloReplaySceneAssetLoader, prelude::VelloReplaySceneAsset,
    render::VelloRenderPlugin, text::VelloFontLoader, VelloAsset, VelloFont,
};
use bevy::prelude::*;

pub struct VelloPlugin;

impl Plugin for VelloPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(VelloRenderPlugin)
            .add_plugins(DebugVisualizationsPlugin)
            .add_plugins(VelloCollisionPlugin)
            .init_asset::<VelloAsset>()
            .init_asset::<VelloFont>()
            .init_asset::<VelloReplaySceneAsset>()
            .init_asset_loader::<VelloFontLoader>()
            .init_asset_loader::<VelloReplaySceneAssetLoader>();
        #[cfg(feature = "svg")]
        app.add_plugins(crate::integrations::svg::SvgIntegrationPlugin);
        #[cfg(feature = "lottie")]
        app.add_plugins(crate::integrations::lottie::LottieIntegrationPlugin);
        #[cfg(feature = "experimental-dotLottie")]
        app.add_plugins(crate::integrations::dot_lottie::DotLottieIntegrationPlugin);
    }
}
