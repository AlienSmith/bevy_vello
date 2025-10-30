//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

mod utility;

use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_egui::{egui, EguiContexts, EguiPlugin};

use bevy::asset::AssetMetaCheck;
use bevy_vello::{
    collision::{
        generate_uvs, path_to_ccw_quad_path, VelloCollisionBroadPhase, VelloCollisionWorld,
    },
    integrations::{
        physics::VelloConstraintWorld,
        svg_collider::{
            SvgColliderAsset, SvgColliderAssetManager, VelloColliderAssetMetaData, VelloImageAsset,
            VelloImageAssetManager, VelloImageAssetMetaData,
        },
        HanabiIntegrationPlugin,
    },
    vello::{
        kurbo::{self, BezPath, Shape},
        peniko::{self, GlowColor},
    },
    VelloCollider, VelloCollisionResponsePlugin,
};
use bevy_vello::{prelude::*, VelloPlugin};

use tankgame_lib::prelude::*;

use crate::utility::{
    get_default_parameters, ui_example_system, update_collider_from_mouse, update_mouse,
    update_mouse_position, ColliderType, UiState,
};

#[derive(Clone, Eq, PartialEq, Debug, Hash, Default, States)]
enum GameState {
    #[default]
    Loading,
    Game,
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
    app.insert_resource(utility::MouseStatus::default())
        .insert_resource(utility::ColliderStatus::default())
        .add_plugins(VelloPlugin)
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
                utility::update_mouse_position,
                ui_example_system,
                update_from_ui.after(ui_example_system),
                update_edge_pan_camera,
                (update_mouse, update_collider_from_mouse).chain(), //update_blood_particles.after(ui_example_system),
            )
                .run_if(in_state(GameState::Game)),
        )
        .run();

    Ok(())
}

fn spawn_collider(
    commands: &mut Commands,
    ui_state: &Res<UiState>,
    color: peniko::Brush,
    scale_modifier: f32,
    f: impl Fn() -> (BezPath, kurbo::Rect),
    complexity_modifier: i32,
) {
    for index in 0..10 {
        let reverse_velocity = if index > 4 { -1.0 } else { 1.0 };
        make_collision_shape(
            commands,
            Vec4::new(
                ui_state.current.pos_x + (100.0 * index as f32) - 500.0,
                ui_state.current.pos_y,
                ui_state.current.rotation,
                ui_state.current.scale * scale_modifier,
            ),
            &f,
            color.clone(),
            Vec2::new(
                reverse_velocity * ui_state.current.vec_x,
                ui_state.current.vec_y,
            ),
            1.0,
            complexity_modifier,
            true,
        );
    }
}

fn update_from_ui(
    ui_state: Res<UiState>,
    mut commands: Commands,
    svg_colliders: Res<SvgColliderAssetManager>,
    images: Res<VelloImageAssetManager>,
    image_assets: Res<Assets<VelloImageAsset>>,
    custom_assets: Res<Assets<SvgColliderAsset>>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
    query: Query<(Entity, &VelloCollider)>,
) {
    if ui_state.delete_all_dynamic {
        for (entity, collider) in query.iter() {
            if collider.is_soft_body() {
                commands.entity(entity).despawn();
            }
        }
    }
    if ui_state.just_modified {
        constraint_world.set_gravity(Vec2::new(
            ui_state.c_config.gravity_x,
            ui_state.c_config.gravity_y,
        ));
    }
    if ui_state.just_spawn {
        let (color, scaler, complexity_modifier) =
            get_default_parameters(ui_state.current.collider_type);
        match ui_state.current.collider_type {
            ColliderType::Rect => {
                let make_rect = || {
                    let rect = kurbo::Rect::new(-20.0, -20.0, 20.0, 20.0);
                    let rect_path = rect.to_path(0.1);
                    (rect_path, rect)
                };
                spawn_collider(
                    &mut commands,
                    &ui_state,
                    bevy_vello::prelude::peniko::Brush::SolidGlow(GlowColor { color, glow: 1.0 }),
                    scaler,
                    make_rect,
                    complexity_modifier,
                );
            }
            ColliderType::Circle => {
                let make_circle = || {
                    let rect = kurbo::Rect::new(-20.0, -20.0, 20.0, 20.0);
                    let circle = kurbo::Circle::new((0.0, 0.0), 20.0);
                    let rect_path = circle.to_path(0.1);
                    (rect_path, rect)
                };
                spawn_collider(
                    &mut commands,
                    &ui_state,
                    bevy_vello::prelude::peniko::Brush::SolidGlow(GlowColor { color, glow: 1.0 }),
                    scaler,
                    make_circle,
                    complexity_modifier,
                );
            }
            ColliderType::Ammo => {
                let make_collider = || {
                    let svg_collider = custom_assets
                        .get(
                            &svg_colliders
                                .get_index(
                                    (ui_state.current.collider_type as u32
                                        - ColliderType::Star as u32)
                                        as usize,
                                )
                                .unwrap(),
                        )
                        .unwrap();
                    (svg_collider.shape.clone(), svg_collider.aabb.clone())
                };
                let albedo = image_assets
                    .get(&images.get_index(0 as usize).unwrap())
                    .unwrap()
                    .image
                    .clone()
                    .with_usage(peniko::ImageUsageType::MASKED);
                let normals = image_assets
                    .get(&images.get_index(1 as usize).unwrap())
                    .unwrap()
                    .image
                    .clone()
                    .with_usage(peniko::ImageUsageType::NORMAL);
                let brush = bevy_vello::prelude::peniko::Brush::PBRImage(peniko::PBRImages::new(
                    albedo, normals, 0.9, 0.2,
                ));
                // let image = image_assets
                //     .get(&images.get_index(2 as usize).unwrap())
                //     .unwrap()
                //     .image
                //     .clone()
                //     .with_usage(peniko::ImageUsageType::TRANSPARENT);
                // let brush = bevy_vello::prelude::peniko::Brush::Image(image);

                spawn_collider(
                    &mut commands,
                    &ui_state,
                    brush,
                    scaler,
                    make_collider,
                    complexity_modifier,
                );
            }
            _ => {
                let make_collider = || {
                    let svg_collider = custom_assets
                        .get(
                            &svg_colliders
                                .get_index(
                                    (ui_state.current.collider_type as u32
                                        - ColliderType::Star as u32)
                                        as usize,
                                )
                                .unwrap(),
                        )
                        .unwrap();
                    (svg_collider.shape.clone(), svg_collider.aabb.clone())
                };

                spawn_collider(
                    &mut commands,
                    &ui_state,
                    bevy_vello::prelude::peniko::Brush::SolidGlow(GlowColor { color, glow: 1.0 }),
                    scaler,
                    make_collider,
                    complexity_modifier,
                );
            }
        };
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

fn setup_entity(mut commands: Commands) {
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
    make_collision_shape_from_color(
        commands,
        Vec4::new(0.0, 540.0, 0.0, 1.0),
        make_long_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(0.0, 0.0),
        0.0,
        0,
        false,
    );

    make_collision_shape_from_color(
        commands,
        Vec4::new(0.0, -540.0, 0.0, 1.0),
        make_long_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(0.0, 0.0),
        0.0,
        0,
        false,
    );

    make_collision_shape_from_color(
        commands,
        Vec4::new(-960.0, 0.0, 0.0, 1.0),
        make_short_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(-0.0, 0.0),
        0.0,
        0,
        false,
    );

    make_collision_shape_from_color(
        commands,
        Vec4::new(960.0, 0.0, 0.0, 1.0),
        make_short_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(-0.0, 0.0),
        0.0,
        0,
        false,
    );
}

fn make_collision_shape(
    commands: &mut Commands,
    transform: Vec4,
    f: impl Fn() -> (BezPath, kurbo::Rect),
    color: peniko::Brush,
    velocity: Vec2,
    inverse_mass: f32,
    complexity_modifier: i32,
    is_soft_body: bool,
) {
    let mut scene: VelloScene = VelloScene::default();
    let (s, rect) = f();
    let shape = path_to_ccw_quad_path(&s);
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgba(0.0, 1.0, 0.0, 0.7),
        None,
        &shape,
    );
    let uvs = match &color {
        peniko::Brush::Image(_) | peniko::Brush::PBRImage(_) => Some(generate_uvs(&shape, &rect)),
        _ => None,
    };
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
        VelloCollider::new(
            &shape,
            &rect,
            velocity,
            color,
            inverse_mass,
            complexity_modifier,
            is_soft_body,
            uvs,
        ),
    ));
}

fn make_collision_shape_from_color(
    commands: &mut Commands,
    transform: Vec4,
    f: impl Fn() -> (BezPath, kurbo::Rect),
    color: peniko::GlowColor,
    velocity: Vec2,
    inverse_mass: f32,
    complexity_modifier: i32,
    is_soft_body: bool,
) {
    make_collision_shape(
        commands,
        transform,
        f,
        peniko::Brush::SolidGlow(color),
        velocity,
        inverse_mass,
        complexity_modifier,
        is_soft_body,
    );
}

fn check_assets_loaded(
    mut ev_asset: EventReader<AssetEvent<VelloReplaySceneAsset>>,
    mut sc_asset: EventReader<AssetEvent<SvgColliderAsset>>,
    mut im_asset: EventReader<AssetEvent<VelloImageAsset>>,
    mut tank_parts: ResMut<AssetManager>,
    mut colliders: ResMut<SvgColliderAssetManager>,
    mut images: ResMut<VelloImageAssetManager>,
    mut next_state: ResMut<NextState<GameState>>,
) {
    for ev in ev_asset.read() {
        match ev {
            AssetEvent::LoadedWithDependencies { id } => {
                tank_parts.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    for sc in sc_asset.read() {
        match sc {
            AssetEvent::LoadedWithDependencies { id } => {
                colliders.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    for im in im_asset.read() {
        match im {
            AssetEvent::LoadedWithDependencies { id } => {
                images.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    if tank_parts.all_loaded() && colliders.all_loaded() && images.all_loaded() {
        next_state.set(GameState::Game);
    }
}

fn setup_resources(
    mut _tank_parts: ResMut<AssetManager>,
    mut colliders: ResMut<SvgColliderAssetManager>,
    mut images: ResMut<VelloImageAssetManager>,
    asset_server: Res<AssetServer>,
) {
    colliders.push(
        asset_server.load("colliders/star.collider.svg"),
        VelloColliderAssetMetaData::default(),
    );
    colliders.push(
        asset_server.load("colliders/heart.collider.svg"),
        VelloColliderAssetMetaData::default(),
    );
    colliders.push(
        asset_server.load("colliders/key.collider.svg"),
        VelloColliderAssetMetaData::default(),
    );
    colliders.push(
        asset_server.load("colliders/shield.collider.svg"),
        VelloColliderAssetMetaData::default(),
    );
    colliders.push(
        asset_server.load("colliders/knife.collider.svg"),
        VelloColliderAssetMetaData::default(),
    );
    colliders.push(
        asset_server.load("colliders/ammo.collider.svg"),
        VelloColliderAssetMetaData::default(),
    );
    images.push(
        asset_server.load("image/ammo_albedo.png"),
        VelloImageAssetMetaData::default(),
    );
    images.push(
        asset_server.load("image/ammo_normal.png"),
        VelloImageAssetMetaData::default(),
    );
    images.push(
        asset_server.load("image/test.png"),
        VelloImageAssetMetaData::default(),
    );
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
