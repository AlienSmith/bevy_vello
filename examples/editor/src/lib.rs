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

mod wasm {
    extern crate wasm_bindgen;
    use wasm_bindgen::prelude::wasm_bindgen;
    #[wasm_bindgen(start)]
    pub fn runner() {
        crate::run();
    }
    #[wasm_bindgen]
    pub fn load_assets_from_bytes(name: String, data: Vec<u8>) {
        bevy_vello::dock::stream_factory::push_dock(name, data);
    }
}

pub fn run() {
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
}
