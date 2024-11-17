use bevy::{
    asset::{embedded_asset, AssetMetaCheck},
    prelude::*,
};
use bevy_vello::{
    dock::{
        lottie_loader::LottieLoaderPlugin, stream_factory::push_dock, svg_loader::SvgLoaderPlugin,
    },
    integrations::svg,
    prelude::*,
    VelloPlugin,
};
const SVG_DATA: &[u8] = include_bytes!("./assets/fountain.svg");
const LOTTIE_DATA: &[u8] = include_bytes!("./assets/tiger.json");
use futures::channel::oneshot;
use std::sync::{Arc, Mutex};
//Reciver is Send but not sync, add mutex make a send type also sync
#[derive(Resource)]
struct DataReceivers {
    svg_r: Arc<Mutex<oneshot::Receiver<u32>>>,
    lottie_r: Arc<Mutex<oneshot::Receiver<u32>>>,
}

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        ..default()
    }))
    .add_plugins(VelloPlugin)
    .add_plugins(SvgLoaderPlugin)
    .add_plugins(LottieLoaderPlugin)
    .add_systems(Startup, recieve_svg)
    .add_systems(Update, recive);
    app.run();
}

fn recieve_svg(mut commands: Commands, mut assets: ResMut<Assets<VelloAsset>>) {
    commands.spawn(Camera2dBundle::default());
    let svg_r = push_dock("fountain.svg".to_owned(), SVG_DATA.to_vec());
    let lottie_r = push_dock("tiger.json".to_owned(), LOTTIE_DATA.to_vec());
    commands.insert_resource(DataReceivers {
        svg_r: Arc::new(Mutex::new(svg_r)),
        lottie_r: Arc::new(Mutex::new(lottie_r)),
    });
}

fn recive(mut commands: Commands, recivers: Option<ResMut<DataReceivers>>) {}
