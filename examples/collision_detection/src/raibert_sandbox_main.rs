//! Thin entry point for the Raibert lower-body kinematic sandbox.
//!
//! Run:
//!   cargo run -p collision_detection --bin raibert_sandbox --release
//!
//! This binary owns rendering + egui only; all the Raibert math lives in
//! `game_lib::lower_body` (pure functions), registered via
//! `game_lib::LowerBodySandboxPlugin`.

mod raibert_sandbox;

use bevy::{asset::AssetMetaCheck, prelude::*, window::PresentMode};
use bevy_egui::EguiPlugin;
use bevy_vello::VelloPlugin;
use game_lib::LowerBodySandboxPlugin;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Vello Raibert Lower-Body Sandbox".into(),
                        present_mode: PresentMode::AutoNoVsync,
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin {
                    meta_check: AssetMetaCheck::Never,
                    ..default()
                }),
        )
        .add_plugins(EguiPlugin {
            enable_multipass_for_primary_context: false,
        })
        .add_plugins(VelloPlugin)
        .add_plugins(LowerBodySandboxPlugin)
        .add_systems(Startup, raibert_sandbox::setup_sandbox)
        .add_systems(
            Update,
            (raibert_sandbox::update_sandbox, raibert_sandbox::tuning_ui),
        )
        .run();

    Ok(())
}
