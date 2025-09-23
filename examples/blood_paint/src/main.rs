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
    integrations::{HanabiIntegrationPlugin, TankGameAssetsMetaData, VelloSceneSubBundle},
    vello::{
        kurbo::{self, Affine, Stroke},
        peniko::{self, GlowColor},
        scene::StorkeExpand,
    },
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
        .insert_resource(AssetManager::default())
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
        .add_systems(
            Update,
            (
                ui_example_system,
                update_blood_particles.after(ui_example_system),
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
        ui.separator();
        ui.checkbox(&mut ui_state.respawn_on_modified, "Spawn On Modified");
        ui.checkbox(&mut ui_state.clear_on_respawn, "Clear On Spawn");
        ui.separator();
        ui.label("BloodParticles");
        ui.add(egui::Slider::new(&mut ui_state.current.life_in_seconds, 0.1..=10.0).text("time"));
        ui.add(
            egui::Slider::new(&mut ui_state.current.number_of_particles, 1.0..=1000.0)
                .text("particles_count"),
        );
        ui.add(egui::Slider::new(&mut ui_state.current.rotation, 0.0..=360.0).text("rotation"));
        ui.add(egui::Slider::new(&mut ui_state.current.cone_angle, 1.0..=360.0).text("cone_angle"));
        ui.add(egui::Slider::new(&mut ui_state.current.speed, 10.0..=2000.0).text("speed"));
        ui.add(egui::Slider::new(&mut ui_state.current.size, 0.1..=10.0).text("size"));
        ui.checkbox(&mut ui_state.current.is_trace, "is_trace");
        if ui.button("Spawn Particle").clicked()
            || (ui_state.respawn_on_modified && ui_state.previous != ui_state.current)
        {
            ui_state.just_spawn = true;
            if ui_state.clear_on_respawn {
                ui_state.just_clear = true;
            }
        }
        ui_state.previous = ui_state.current.clone();
    });
}

fn update_blood_particles(
    mut commands: Commands,
    parts: Res<AssetManager>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
    time: Res<Time>,
    ui_state: Res<UiState>,
    mut q: Query<(Entity, &mut VelloScene, &mut BloodPaintParticles)>,
) {
    let delta_time = time.delta_seconds();
    let delta = time.delta();
    for (entity, mut scene, mut particles) in q.iter_mut() {
        particles.death_timer.tick(delta);
        if ui_state.just_clear || particles.death_timer.finished() {
            commands.entity(entity).despawn();
        } else {
            particles.explosion.update(delta_time);
            let mut transforms: Vec<kurbo::Affine> = vec![];
            particles.explosion.get_transforms(&mut transforms);
            scene.update_instance_data_only(&transforms);
        }
    }
    if ui_state.just_spawn {
        let temp = ui_state.current.clone();
        spawn_blood_particles_at(
            &mut commands,
            &parts,
            &custom_assets,
            temp.life_in_seconds,
            temp.cone_angle,
            temp.rotation,
            temp.speed,
            temp.size,
            temp.number_of_particles as usize,
            temp.is_trace,
            Transform::from_translation(Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            }),
        );
    }
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

fn check_assets_loaded(
    mut ev_asset: EventReader<AssetEvent<VelloReplaySceneAsset>>,
    mut tank_parts: ResMut<AssetManager>,
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

fn setup_resources(mut tank_parts: ResMut<AssetManager>, asset_server: Res<AssetServer>) {
    tank_parts.push(
        asset_server.load("scenes/blood.scene"),
        TankGameAssetsMetaData::default(),
    );
}

pub fn spawn_blood_particles_at(
    commands: &mut Commands,
    parts: &Res<AssetManager>,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    time_in_seconds: f32,
    cone_angle: f32,
    direction: f32,
    speed: f32,
    size: f32,
    count: usize,
    is_trace: bool,
    transform: Transform,
) {
    let mut sb = VelloScene::default();
    let mut explosion = Explosion::new(
        0.0, 0.0, cone_angle, direction, speed, size, count, is_trace,
    );
    explosion.set_decay_rate(0.1);
    let mut transforms: Vec<Affine> = vec![];
    explosion.get_transforms(&mut transforms);
    sb.push_instance_with_transforms(&transforms);
    custom_assets
        .get(&parts.get_index(0 as usize).unwrap())
        .unwrap()
        .player
        .apply_to_scene(&mut sb);
    sb.pop_instance();

    commands.spawn((
        BloodPaintParticles {
            explosion,
            death_timer: Timer::from_seconds(time_in_seconds, TimerMode::Once),
        },
        VelloSceneBundle {
            scene: sb,
            transform,
            ..Default::default()
        },
    ));
}

//fn update_blood_instances()
