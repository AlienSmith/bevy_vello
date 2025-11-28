//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

use std::str::SplitWhitespace;

use bevy::render::render_resource::AsBindGroupShaderType;
use bevy::{prelude::*, scene};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy::asset::AssetMetaCheck;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use bevy_vello::{
    add_default_light,
    integrations::{
        particles::{BurstEmitterConfig, ExplosionEffect},
        *,
    },
    vello::{
        kurbo,
        peniko::{self},
    },
};
use bevy_vello::{prelude::*, VelloPlugin};

#[derive(Resource, Default)]
struct UiState {
    just_spawn: bool,
}

#[derive(Resource)]
struct ClickSpawner {
    pos: Vec2,
    last_spawn_time: f32,
    time_threhold: f32,
}

impl ClickSpawner {
    pub fn new(time_threhold: f32) -> Self {
        Self {
            pos: Vec2 { x: 0.0, y: 0.0 },
            last_spawn_time: 0.0,
            time_threhold,
        }
    }
}

#[derive(Clone, Default, Component)]
pub struct ExplosionFading {
    timer: Timer,
    init_color: Vec3,
    end_color: Vec3,
    particle_scales: f32,
}

//Notic without "meta_check: AssetMetaCheck::Never" bevy would complain about the HanabiNode.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = App::default();
    app.insert_resource(ClearColor(Color::BLACK)).add_plugins(
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
    );

    // #[cfg(feature = "examples_world_inspector")]
    // app.add_plugins(WorldInspectorPlugin::default());
    app.add_plugins(VelloPlugin)
        .add_plugins(particles::VelloPartclePlugin)
        .add_plugins(EguiPlugin)
        .insert_resource(UiState::default())
        .insert_resource(ClickSpawner::new(0.1))
        .add_systems(Startup, (setup_back_ground, add_default_light))
        .add_systems(Update, (ui_example_system, spawn_particles))
        .run();

    Ok(())
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

fn ui_example_system(mut ui_state: ResMut<UiState>, mut contexts: EguiContexts) {
    egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
        ui_state.just_spawn = false;
        if ui.button("Quit").clicked() {
            std::process::exit(0);
        }
        ui.separator();
        if ui.button("Spawn Particle").clicked() {
            ui_state.just_spawn = true;
        }
    });
}

fn spawn_particles(
    mut commands: Commands,
    time: Res<Time>,
    mut spawner: ResMut<ClickSpawner>,
    ui: Res<UiState>,
) {
    let current = time.elapsed_seconds();
    if current - spawner.last_spawn_time > spawner.time_threhold && ui.just_spawn {
        spawner.last_spawn_time = current;

        let mut scene = VelloScene::default();
        scene.push_instance_with_transforms(&[]);
        scene.fill(
            peniko::Fill::NonZero,
            kurbo::Affine::default(),
            peniko::Color::rgba(1.0, 0.0, 0.0, 0.5),
            None,
            &kurbo::Circle::new((0.0, 0.0), 20.0),
        );
        scene.pop_instance();

        commands.spawn((
            VelloSceneBundle {
                scene,
                ..Default::default()
            },
            ExplosionEffect::new(
                particles::GravityParticleConfig {
                    gravity: Vec2::new(0.0, -98.0),
                    drag: 0.0,
                    persistent: false,
                },
                particles::BurstEmitterConfig {
                    count: 100,
                    speed_range: (10.0, 100.0),
                    lifetime_range: (1.0, 1.2),
                    origin: Vec2::new(0.0, 0.0),
                },
                500,
            ),
        ));
    }
}
