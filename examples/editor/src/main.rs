use bevy::{
    asset::{embedded_asset, AssetMetaCheck},
    prelude::*,
};
use bevy_vello::{
    dock::{
        lottie_loader::LottieLoaderPlugin, stream_factory::push_dock, svg_loader::SvgLoaderPlugin,
    },
    prelude::*,
    VelloPlugin,
};
const SVG_DATA: &[u8] = include_bytes!("./assets/fountain.svg");
const LOTTIE_DATA: &[u8] = include_bytes!("./assets/tiger.json");
fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        ..default()
    }))
    .add_plugins(VelloPlugin)
    .add_plugins(SvgLoaderPlugin)
    .add_plugins(LottieLoaderPlugin)
    .add_systems(Startup, recieve_svg);
    app.run();
}

fn recieve_svg(mut commands: Commands, mut assets: ResMut<Assets<VelloAsset>>) {
    commands.spawn(Camera2dBundle::default());
    push_dock("fountain.svg".to_owned(), SVG_DATA.to_vec());
    push_dock("tiger.json".to_owned(), LOTTIE_DATA.to_vec());
}
