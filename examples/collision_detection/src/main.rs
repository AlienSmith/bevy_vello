//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

use std::time::Duration;

use bevy::{ecs::entity, prelude::*};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_hanabi::prelude::*;

use bevy::asset::AssetMetaCheck;
use bevy_vello::{
    add_default_light,
    integrations::{HanabiIntegrationPlugin, VelloSceneSubBundle},
    vello::{
        kurbo::{self, Affine, Shape, Stroke},
        peniko::{self, GlowColor},
        scene::StorkeExpand,
    },
    VelloCollider,
};
use bevy_vello::{prelude::*, VelloPlugin};
use particles_lib::Explosion;
use ron::value::Float;
use tankgame_lib::{decor::make_scene_from_vello_replay_scene, prelude::*, update_particle_scene};
#[derive(Clone, Eq, PartialEq, Debug, Hash, Default, States)]
enum GameState {
    #[default]
    Loading,
    Game,
}
#[derive(Default, Resource)]

struct UiState {
    current: ParticleState,
    previous: ParticleState,
    respawn_on_modified: bool,
    clear_on_respawn: bool,
    just_clear: bool,
    just_spawn: bool,
}

#[derive(PartialEq, Clone)]
struct ParticleState {
    life_in_seconds: f32,
    number_of_particles: f32,
    rotation: f32,
    cone_angle: f32,
    speed: f32,
    size: f32,
    is_trace: bool,
}

impl Default for ParticleState {
    fn default() -> Self {
        Self {
            life_in_seconds: 1.0,
            number_of_particles: 50.0,
            rotation: 0.0,
            cone_angle: 30.0,
            speed: 50.0,
            size: 1.0,
            is_trace: false,
        }
    }
}

#[derive(Clone, Default, Component)]
pub struct BloodPaintParticles {
    explosion: Explosion,
    death_timer: Timer,
}

//Notic without "meta_check: AssetMetaCheck::Never" bevy would complain about the HanabiNode.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = App::default();
    app.insert_resource(ClearColor(Color::BLACK))
        .add_plugins(
            DefaultPlugins
                .set(AssetPlugin {
                    meta_check: AssetMetaCheck::Never,
                    ..default()
                })
                .set(bevy::log::LogPlugin {
                    // Uncomment this to override the default log settings:
                    // level: bevy::log::Level::TRACE,
                    // filter: "wgpu=warn,bevy_ecs=info".to_string(),
                    ..default()
                }),
        )
        .init_state::<GameState>()
        .insert_resource(TankGameAssets::default())
        .insert_resource(UiState::default())
        .add_plugins(HanabiIntegrationPlugin)
        .add_plugins(EguiPlugin);
    // Systems that create Egui widgets should be run during the `CoreSet::Update` set,
    // or after the `EguiSet::BeginPass` system (which belongs to the `CoreSet::PreUpdate` set).

    // #[cfg(feature = "examples_world_inspector")]
    // app.add_plugins(WorldInspectorPlugin::default());
    app.add_plugins(VelloPlugin)
        .add_systems(Startup, setup_back_ground)
        .add_systems(Startup, add_default_light)
        .add_systems(Startup, setup_resources)
        .add_systems(
            Update,
            check_assets_loaded.run_if(in_state(GameState::Loading)),
        )
        .add_systems(OnEnter(GameState::Game), setup_entity)
        .add_systems(
            Update,
            (
                ui_example_system,
                //update_blood_particles.after(ui_example_system),
            )
                .run_if(in_state(GameState::Game)),
        )
        .run();

    Ok(())
}

fn ui_example_system(mut ui_state: ResMut<UiState>, mut contexts: EguiContexts) {
    egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
        ui_state.just_clear = false;
        ui_state.just_spawn = false;
        if ui.button("Quit").clicked() {
            std::process::exit(0);
        }
    });
}

//make a white background
fn setup_back_ground(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
    let mut scene: VelloScene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgb(1.0, 1.0, 1.0),
        None,
        &kurbo::Rect::new(-1024.0, -1024.0, 1024.0, 1024.0),
    );

    commands.spawn((VelloSceneBundle {
        scene,
        ..Default::default()
    },));
}

fn setup_entity(mut commands: Commands) {
    let mut scene: VelloScene = VelloScene::default();

    let rect = kurbo::Rect::new(-24.0, -24.0, 24.0, 24.0);
    let rect_path = rect.to_path(0.1);

    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgba(1.0, 0.0, 0.0, 0.7),
        None,
        &rect,
    );

    commands.spawn((
        VelloSceneBundle {
            scene,
            ..Default::default()
        },
        VelloCollider::new(&rect_path, &rect),
    ));

    let mut scene1: VelloScene = VelloScene::default();
    scene1.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgba(0.0, 1.0, 0.0, 0.7),
        None,
        &rect,
    );
    commands.spawn((
        VelloSceneBundle {
            scene: scene1,
            transform: Transform::from_translation(Vec3::new(40.0, 40.0, 0.0)),
            ..Default::default()
        },
        VelloCollider::new(&rect_path, &rect),
    ));
}

fn check_assets_loaded(
    mut ev_asset: EventReader<AssetEvent<VelloReplaySceneAsset>>,
    mut tank_parts: ResMut<TankGameAssets>,
    mut next_state: ResMut<NextState<GameState>>,
) {
    for ev in ev_asset.read() {
        match ev {
            AssetEvent::LoadedWithDependencies { id } => {
                tank_parts.mark_as_loaded(id);
                if tank_parts.all_loaded() {
                    next_state.set(GameState::Game);
                }
            }
            _ => {}
        }
    }
}

fn setup_resources(mut tank_parts: ResMut<TankGameAssets>, asset_server: Res<AssetServer>) {
    tank_parts.push(
        asset_server.load("scenes/blood.scene"),
        TankGameAssetsMetaData::default(),
    );
}

//fn update_blood_instances()
