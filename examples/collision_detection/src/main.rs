//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

use std::f32::consts::PI;

use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
    window::WindowResolution,
};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_egui::{egui, EguiContexts, EguiPlugin};

use bevy::asset::AssetMetaCheck;
use bevy_vello::{
    add_default_light,
    integrations::{
        svg,
        svg_collider::{SvgColliderAsset, SvgColliderAssetManager, VelloColliderAssetMetaData},
        HanabiIntegrationPlugin, TankGameAssetsMetaData,
    },
    vello::{
        kurbo::{self, BezPath, Shape},
        peniko::{self, GlowColor},
    },
    VelloCollider, VelloCollisionResponsePlugin,
};
use bevy_vello::{prelude::*, VelloPlugin};
use particles_lib::Explosion;
use tankgame_lib::prelude::*;
#[derive(Clone, Eq, PartialEq, Debug, Hash, Default, States)]
enum GameState {
    #[default]
    Loading,
    Game,
}

#[derive(PartialEq, Clone, Default)]
struct EntityConfig {
    pos_x: f32,
    pos_y: f32,
    vec_x: f32,
    vec_y: f32,
}

#[derive(Default, Resource)]

struct UiState {
    current: EntityConfig,
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
                    // Uncomment this to override the default log settings:0
                    // level: bevy::log::Level::TRACE,
                    // filter: "wgpu=warn,bevy_ecs=info".to_string(),
                    ..default()
                }),
        )
        .init_state::<GameState>()
        .insert_resource(AssetManager::default())
        .insert_resource(UiState::default())
        .add_plugins(HanabiIntegrationPlugin)
        .add_plugins(EguiPlugin)
        .add_plugins(FrameTimeDiagnosticsPlugin::default());
    // Systems that create Egui widgets should be run during the `CoreSet::Update` set,
    // or after the `EguiSet::BeginPass` system (which belongs to the `CoreSet::PreUpdate` set).

    // #[cfg(feature = "examples_world_inspector")]
    // app.add_plugins(WorldInspectorPlugin::default());
    app.add_plugins(VelloPlugin)
        .add_plugins(VelloCollisionResponsePlugin)
        .add_systems(Startup, setup_back_ground)
        .add_systems(Startup, add_light)
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
                spawn_colliders_from_ui.after(ui_example_system),
                update_edge_pan_camera,
                //update_blood_particles.after(ui_example_system),
            )
                .run_if(in_state(GameState::Game)),
        )
        .run();

    Ok(())
}

fn ui_example_system(
    mut ui_state: ResMut<UiState>,
    mut contexts: EguiContexts,
    diagnostics: Res<DiagnosticsStore>,
) {
    egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
        ui_state.just_spawn = false;
        // Get the FPS diagnostic path
        let fps_path = FrameTimeDiagnosticsPlugin::FPS;

        // Fetch the FPS value
        if let Some(fps) = diagnostics.get(&fps_path) {
            if let Some(value) = fps.value() {
                ui.label(format!("FPS: {:.1}", value));
            }
            if let Some(avg) = fps.average() {
                ui.label(format!("Avg FPS: {:.1}", avg));
            }
        }
        ui.add(egui::Slider::new(&mut ui_state.current.pos_x, -900.0..=900.0).text("pox_x"));
        ui.add(egui::Slider::new(&mut ui_state.current.pos_y, -500.0..=500.0).text("pox_y"));
        ui.add(egui::Slider::new(&mut ui_state.current.vec_x, -500.0..=500.0).text("vec_x"));
        ui.add(egui::Slider::new(&mut ui_state.current.vec_y, -500.0..=500.0).text("vec_y"));
        if ui.button("Spawn").clicked() {
            ui_state.just_spawn = true;
        }
        if ui.button("Quit").clicked() {
            std::process::exit(0);
        }
    });
}

fn spawn_colliders_from_ui(
    ui_state: Res<UiState>,
    mut commands: Commands,
    svg_colliders: Res<SvgColliderAssetManager>,
    custom_assets: Res<Assets<SvgColliderAsset>>,
) {
    let make_rect = || {
        let rect = kurbo::Rect::new(-20.0, -20.0, 20.0, 20.0);
        let rect_path = rect.to_path(0.1);
        (rect_path, rect)
    };

    let make_collider = || {
        let svg_collider = custom_assets
            .get(&svg_colliders.get_index(0 as usize).unwrap())
            .unwrap();
        (svg_collider.shape.clone(), svg_collider.aabb.clone())
    };

    if ui_state.just_spawn {
        make_collision_shape(
            &mut commands,
            Vec4::new(ui_state.current.pos_x, ui_state.current.pos_y, 0.0, 1.0),
            make_collider,
            GlowColor {
                color: peniko::Color::rgba(0.0, 1.0, 0.0, 0.99),
                glow: 1.0,
            },
            Vec2::new(ui_state.current.vec_x, ui_state.current.vec_y),
            1.0,
        );
    }
}

//make a white background
fn setup_back_ground(mut commands: Commands) {
    commands.spawn((
        Camera2dBundle::default(),
        EdgePanCamera {
            edge_margin: -10.0,
            ..Default::default()
        },
    ));
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

fn setup_entity(
    mut commands: Commands,
    svg_colliders: Res<SvgColliderAssetManager>,
    custom_assets: Res<Assets<SvgColliderAsset>>,
) {
    let make_rect = || {
        let rect = kurbo::Rect::new(-20.0, -20.0, 20.0, 20.0);
        let rect_path = rect.to_path(0.1);
        (rect_path, rect)
    };

    let make_collider = || {
        let svg_collider = custom_assets
            .get(&svg_colliders.get_index(0 as usize).unwrap())
            .unwrap();
        (svg_collider.shape.clone(), svg_collider.aabb.clone())
    };

    // make_collision_shape(
    //     &mut commands,
    //     Vec4::new(200.0, 0.0, 0.0, 0.3),
    //     make_collider,
    //     GlowColor {
    //         color: peniko::Color::rgb(0.0, 1.0, 0.0),
    //         glow: 1.0,
    //     },
    //     Vec2::new(-200.0, 0.0),
    //     1.0,
    // );
    make_static_scene(&mut commands);
}

fn make_static_scene(commands: &mut Commands) {
    let make_long_rect = || {
        let rect: kurbo::Rect = kurbo::Rect::new(-980.0, -20.0, 980.0, 20.0);
        let rect_path = rect.to_path(0.1);
        (rect_path, rect)
    };
    //aabb will only take the effect of position ignoring entity rotation and scale.
    //in other words if your static collider contains rotation or scaling you need to account for that
    let make_short_rect = || {
        let rect = kurbo::Rect::new(-20.0, -560.0, 20.0, 560.0);
        let rect_path = rect.to_path(0.1);
        (rect_path, rect)
    };
    make_collision_shape(
        commands,
        Vec4::new(0.0, 540.0, 0.0, 1.0),
        make_long_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(0.0, 0.0),
        0.0,
    );

    make_collision_shape(
        commands,
        Vec4::new(0.0, -540.0, 0.0, 1.0),
        make_long_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(0.0, 0.0),
        0.0,
    );

    make_collision_shape(
        commands,
        Vec4::new(-960.0, 0.0, 0.0, 1.0),
        make_short_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(-0.0, 0.0),
        0.0,
    );

    make_collision_shape(
        commands,
        Vec4::new(960.0, 0.0, 0.0, 1.0),
        make_short_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(-0.0, 0.0),
        0.0,
    );
}

fn make_collision_shape(
    commands: &mut Commands,
    transform: Vec4,
    f: impl Fn() -> (BezPath, kurbo::Rect),
    color: peniko::GlowColor,
    velocity: Vec2,
    inverse_mass: f32,
) {
    let mut scene: VelloScene = VelloScene::default();
    let (shape, rect) = f();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgba(0.0, 1.0, 0.0, 0.7),
        None,
        &shape,
    );
    commands.spawn((
        VelloSceneBundle {
            scene,
            transform: Transform {
                translation: Vec3::new(transform.x, transform.y, 0.0),
                rotation: Quat::from_rotation_z(transform.z.to_radians()),
                scale: Vec3::new(transform.w, transform.w, 1.0),
            },
            ..Default::default()
        },
        VelloCollider::new(&shape, &rect, velocity, color, inverse_mass),
    ));
}

fn check_assets_loaded(
    mut ev_asset: EventReader<AssetEvent<VelloReplaySceneAsset>>,
    mut sc_asset: EventReader<AssetEvent<SvgColliderAsset>>,
    mut tank_parts: ResMut<AssetManager>,
    mut colliders: ResMut<SvgColliderAssetManager>,
    mut next_state: ResMut<NextState<GameState>>,
) {
    for ev in ev_asset.read() {
        match ev {
            AssetEvent::LoadedWithDependencies { id } => {
                tank_parts.mark_as_loaded(id);
                if tank_parts.all_loaded() && colliders.all_loaded() {
                    next_state.set(GameState::Game);
                }
            }
            _ => {}
        }
    }

    for sc in sc_asset.read() {
        match sc {
            AssetEvent::LoadedWithDependencies { id } => {
                colliders.mark_as_loaded(id);
                if tank_parts.all_loaded() && colliders.all_loaded() {
                    next_state.set(GameState::Game);
                }
            }
            _ => {}
        }
    }
}

fn setup_resources(
    mut tank_parts: ResMut<AssetManager>,
    mut colliders: ResMut<SvgColliderAssetManager>,
    asset_server: Res<AssetServer>,
) {
    tank_parts.push(
        asset_server.load("scenes/blood.scene"),
        TankGameAssetsMetaData::default(),
    );
    colliders.push(
        asset_server.load("colliders/star.collider.svg"),
        VelloColliderAssetMetaData::default(),
    )
}

//fn update_blood_instances()

pub fn add_light(mut commands: Commands) {
    let mut light_scene: VelloScene = VelloScene::default();
    let light_radius = 800.0;
    //let light_shape_ratio = 1.0 / 40.0;
    light_scene.push_point_light(
        kurbo::Affine::scale(light_radius * 2.0),
        &[1.0, 1.0, 1.0],
        200.0 / (light_radius as f32),
    );
    commands.spawn((VelloSceneBundle {
        scene: light_scene,
        ..Default::default()
    },));
}
