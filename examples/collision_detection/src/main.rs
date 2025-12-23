//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

mod connections;
mod edge_pan_camera;
mod utility;
use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, prelude::*, utils::HashMap};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_egui::EguiPlugin;

use bevy::asset::AssetMetaCheck;
use bevy_vello::{
    collision::{
        generate_uvs, path_to_ccw_quad_path, CollisionConstraintConfig, CollisionSystems,
        SoftBodyInitConfig, VelloCollisionEvent,
    },
    integrations::{
        particles::{self, ExplosionEffect},
        physics::{ConnectionHandle, SoftBodyConnections, VelloConstraintWorld},
        svg_collider::{
            SvgColliderAsset, SvgColliderAssetManager, VelloColliderAssetMetaData, VelloImageAsset,
            VelloImageAssetManager, VelloImageAssetMetaData,
        },
    },
    vello::{
        kurbo::{self, BezPath, Shape},
        peniko::{self, GlowColor},
    },
    VelloCollider, VelloCollisionResponsePlugin,
};
use bevy_vello::{prelude::*, VelloPlugin};

use crate::{
    connections::ConnectionStatus,
    utility::{
        get_default_parameters, ui_example_system, update_collider_from_mouse, update_mouse,
        ColliderType, UiState,
    },
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

#[derive(Resource, Default)]
struct CollisionEventTracker {
    data: HashMap<(Entity, Entity), f32>,
    last_perge_time: f32,
    time_threshold: f32,
    distance_threshold_squre: f32,
    filtered_events: Vec<VelloCollisionEvent>,
}

impl CollisionEventTracker {
    pub fn new(time_threshold: f32, distance_threshold: f32) -> Self {
        Self {
            time_threshold,
            distance_threshold_squre: distance_threshold * distance_threshold,
            ..default()
        }
    }

    pub fn insert(&mut self, event: VelloCollisionEvent, time: f32) {
        let a = event.entity_a;
        let b = event.entity_b;
        let pair = if a < b { (a, b) } else { (b, a) };
        let diff = event.collision_point_a - event.collision_point_b;
        let dis_squre = diff.dot(diff);
        if dis_squre < self.distance_threshold_squre {
            return;
        }
        if let Some(last_time) = self.data.get_mut(&pair) {
            let time_diff = time - *last_time;
            (*last_time) = time;
            if time_diff < self.time_threshold {
                return;
            }
        } else {
            self.data.insert(pair, time);
        }
        self.filtered_events.push(event);
    }

    pub fn try_purge(&mut self, time: f32) {
        if time - self.last_perge_time > 10.0 * self.time_threshold + 5.0 {
            self.data
                .retain(|_, &mut last_frame| time - last_frame < 10.0 * self.time_threshold + 5.0);
            self.last_perge_time = time;
        }
    }
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
        .insert_resource(UiState::default())
        .insert_resource(CollisionEventTracker::new(0.5, 1.1))
        .add_plugins(EguiPlugin)
        .add_plugins(FrameTimeDiagnosticsPlugin::default());
    // Systems that create Egui widgets should be run during the `CoreSet::Update` set,
    // or after the `EguiSet::BeginPass` system (which belongs to the `CoreSet::PreUpdate` set).

    // #[cfg(feature = "examples_world_inspector")]
    // app.add_plugins(WorldInspectorPlugin::default());
    app.insert_resource(utility::MouseStatus::default())
        .insert_resource(utility::ColliderStatus::default())
        .insert_resource(connections::ConnectionStatus::default())
        .add_plugins(VelloPlugin)
        .add_plugins(VelloCollisionResponsePlugin)
        .add_plugins(particles::VelloPartclePlugin)
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
                edge_pan_camera::update_edge_pan_camera,
                (update_mouse, update_collider_from_mouse).chain(), //update_blood_particles.after(ui_example_system),
                collision_response,
            )
                .run_if(in_state(GameState::Game)),
        )
        .add_systems(
            FixedUpdate,
            filter_collision_event
                .in_set(CollisionSystems::CollisionResponsePhysics)
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
    soft_body_config: SoftBodyInitConfig,
    collision_config: CollisionConstraintConfig,
) {
    for index in 0..1 {
        let reverse_velocity = if index > 4 { -1.0 } else { 1.0 };
        let temp = make_collision_shape(
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
            soft_body_config.total_inv_mass,
            true,
            Some(soft_body_config),
            Some(collision_config),
            1,
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
    mut connections: ResMut<ConnectionStatus>,
    mut connection_handles: ResMut<SoftBodyConnections>,
    query: Query<(Entity, &VelloCollider)>,
) {
    if ui_state.delete_all_dynamic {
        for (entity, collider) in query.iter() {
            if collider.is_soft_body() {
                commands.entity(entity).despawn();
            }
        }
        for item in connections.reset().drain(..) {
            if let Some(index) = connection_handles.remove(item) {
                constraint_world.remove_connection(index);
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
        let config = ui_state.soft_body_config;
        let (color, scaler, _complexity_modifier) =
            get_default_parameters(ui_state.current.collider_type);
        let collision_config = ui_state.collision_config;
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
                    config,
                    collision_config,
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
                    config,
                    collision_config,
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
                    config,
                    collision_config,
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
                    config,
                    collision_config,
                );
            }
        };
    }
}

//make a white background
fn setup_back_ground(mut commands: Commands) {
    commands.spawn((
        Camera2dBundle::default(),
        edge_pan_camera::EdgePanCamera {
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
    make_static_collision_shape(
        commands,
        Vec4::new(0.0, 540.0, 0.0, 1.0),
        make_long_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(0.0, 0.0),
        0.0,
        false,
    );

    make_static_collision_shape(
        commands,
        Vec4::new(0.0, -540.0, 0.0, 1.0),
        make_long_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(0.0, 0.0),
        0.0,
        false,
    );

    make_static_collision_shape(
        commands,
        Vec4::new(-960.0, 0.0, 0.0, 1.0),
        make_short_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(-0.0, 0.0),
        0.0,
        false,
    );

    make_static_collision_shape(
        commands,
        Vec4::new(960.0, 0.0, 0.0, 1.0),
        make_short_rect,
        GlowColor {
            color: peniko::Color::rgb(1.0, 0.0, 0.0),
            glow: 1.0,
        },
        Vec2::new(-0.0, 0.0),
        0.0,
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
    is_soft_body: bool,
    soft_body_init_config: Option<SoftBodyInitConfig>,
    collision_config: Option<CollisionConstraintConfig>,
    collision_group: u32,
) -> Entity {
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
    let frame_path = rect.to_path(0.1);
    commands
        .spawn((
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
                &frame_path,
                &rect,
                velocity,
                color,
                inverse_mass,
                is_soft_body,
                uvs,
                soft_body_init_config,
                collision_config,
                collision_group,
            ),
        ))
        .id()
}

fn make_static_collision_shape(
    commands: &mut Commands,
    transform: Vec4,
    f: impl Fn() -> (BezPath, kurbo::Rect),
    color: peniko::GlowColor,
    velocity: Vec2,
    inverse_mass: f32,
    _is_soft_body: bool,
) {
    let entity = make_collision_shape(
        commands,
        transform,
        f,
        peniko::Brush::SolidGlow(color),
        velocity,
        inverse_mass,
        false,
        None,
        None,
        0,
    );
    info!("static_entity: {:?}", entity);
}

fn check_assets_loaded(
    mut sc_asset: EventReader<AssetEvent<SvgColliderAsset>>,
    mut im_asset: EventReader<AssetEvent<VelloImageAsset>>,
    mut colliders: ResMut<SvgColliderAssetManager>,
    mut images: ResMut<VelloImageAssetManager>,
    mut next_state: ResMut<NextState<GameState>>,
) {
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

    if colliders.all_loaded() && images.all_loaded() {
        next_state.set(GameState::Game);
    }
}

fn setup_resources(
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

fn filter_collision_event(
    mut reader: EventReader<VelloCollisionEvent>,
    mut c: ResMut<CollisionEventTracker>,
    time: Res<Time>,
) {
    let t = time.elapsed_seconds();
    // notice you might recieved events from previous frame and this frame.
    for item in reader.read() {
        c.insert(item.clone(), t);
    }
    c.try_purge(t);
}

fn collision_response(
    mut commands: Commands,
    mut c: ResMut<CollisionEventTracker>,
    ui_state: Res<UiState>,
) {
    if ui_state.spawn_particles {
        for item in c.filtered_events.drain(..) {
            let pos = 0.5 * (item.collision_point_a + item.collision_point_b);
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
                    transform: Transform::from_translation(pos.extend(100.0)),
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
            info!(
                "spawn particles at {:?}, {:?}",
                item.collision_point_a, item.collision_point_b
            );
        }
    }
}
