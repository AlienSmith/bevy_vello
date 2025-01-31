use bevy::{
    asset::{embedded_asset, AssetMetaCheck},
    prelude::*,
};
use bevy_vello::{add_default_light, prelude::*, VelloPlugin};

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        ..default()
    }))
    .add_plugins(VelloPlugin)
    .add_systems(Startup, load_svg)
    .add_systems(Startup, add_default_light);
    embedded_asset!(app, "assets/fountain.svg");
    app.run();
}

fn load_svg(mut commands: Commands, asset_server: ResMut<AssetServer>) {
    commands.spawn(Camera2dBundle::default());

    // Yes, it's this simple.
    commands.spawn(VelloAssetBundle {
        vector: asset_server.load("embedded://svg/assets/fountain.svg"),
        debug_visualizations: DebugVisualizations::Visible,
        transform: Transform::from_scale(Vec3::splat(5.0)),
        ..default()
    });
}
