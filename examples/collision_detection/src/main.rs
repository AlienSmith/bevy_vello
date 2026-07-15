//! A particle system with a 2D camera.
//!
//! The particle effect instance override its `z_layer_2d` field, which can be
//! tweaked at runtime via the egui inspector to move the 2D rendering layer of
//! particle above or below the reference square.

mod connections;
mod edge_pan_camera;
mod utility;
use bevy::{
    diagnostic::FrameTimeDiagnosticsPlugin,
    math::VectorSpace,
    platform::collections::HashMap,
    prelude::*,
    render::{
        settings::{InstanceFlags, RenderCreation, WgpuSettings},
        RenderDebugFlags, RenderPlugin,
    },
    window::PresentMode,
};
// #[cfg(feature = "examples_world_inspector")]
// use bevy_inspector_egui::quick::WorldInspectorPlugin;

use bevy_egui::EguiPlugin;

use bevy::asset::AssetMetaCheck;
use bevy_vello::{
    collision::{
        generate_uvs, path_to_ccw_quad_path, CollisionConstraintConfig, CollisionSystems,
        SoftBodyInitConfig, VelloCollisionEvent, VelloCollisionTrigger,
        VELLO_COLLISION_COOL_DOWN_TIME,
    },
    integrations::{
        particles::{self, ExplosionEffect},
        physics::{
            ConnectionConstraintInitConfig, ConnectionInitConfig, VelloConstraintWorld, VelloJoint,
            VelloParticle,
        },
        svg_collider::{
            self, SvgColliderAsset, SvgColliderAssetManager, VelloColliderAssetMetaData,
            VelloImageAsset, VelloImageAssetManager, VelloImageAssetMetaData,
        },
    },
    vello::{
        kurbo::{self, BezPath, Shape},
        peniko::{self, GlowColor},
    },
    VelloCollider, VelloCollisionResponsePlugin,
};
use bevy_vello::{prelude::*, VelloPlugin};
use game_lib::{
    character_asset::{
        BlueprintCharacterAsset, BlueprintCharacterAssetManager, BlueprintCharacterAssetMetaData,
        SvgCharacterAsset, SvgCharacterAssetManager, SvgCharacterAssetMetaData,
    },
    weapons::{AttachPistolToCharacterEvent, FireEvent, PistolControl},
    CharacterController, CharacterPartEvent, CharacterRoot, ColliderRoot, ConnectivityRoot,
    GameLabSystems, IkMode, LeftArmController, RightArmController, SpineController,
    VelloCharacterPlugin,
};

use crate::{
    connections::ConnectionStatus,
    utility::{
        bevy_to_vello, get_default_parameters, ui_example_system, update_collider_from_mouse,
        update_mouse, ColliderType, MouseStatus, Preview, StaticSceneComponent, UiState,
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

/// Tracks whether the pistol collider has been registered to the character.
/// Used to guard the UnregisterPart event so we don't try to unregister twice.
#[derive(Resource, Default)]
struct PistolState {
    registered: bool,
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
//cargo run --package collision_detection --bin collision_detection --release
//Notic without "meta_check: AssetMetaCheck::Never" bevy would complain about the HanabiNode.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut app = App::default();
    app.insert_resource(ClearColor(Color::BLACK))
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Vello Study - Unlocked".into(),
                        // PresentMode::AutoNoVsync is usually the best for Linux/NVIDIA
                        // It avoids the 60fps cap while preventing "tearing" where possible.
                        // Use PresentMode::Immediate if you want absolute raw output.
                        present_mode: PresentMode::AutoNoVsync,
                        ..default()
                    }),
                    ..default()
                })
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
            //Enable the following code to have the source code build in the shader binary to work with aftermath, renderdoc etc
            // .set(RenderPlugin {
            //     debug_flags: RenderDebugFlags::all(),
            //     render_creation: RenderCreation::Automatic(WgpuSettings {
            //         // 2. EXPLICITLY UNSET DISCARD_HAL_LABELS
            //         // By default, InstanceFlags::from_build_config() unsets markers in release.
            //         // We must force them back on.
            //         instance_flags: InstanceFlags::DEBUG | InstanceFlags::VALIDATION,
            //         ..Default::default()
            //     }),
            //     ..Default::default()
            // }),
        )
        .init_state::<GameState>()
        .insert_resource(UiState::default())
        .add_plugins(EguiPlugin {
            enable_multipass_for_primary_context: false,
        })
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(VelloCharacterPlugin::default());
    // Systems that create Egui widgets should be run during the `CoreSet::Update` set,
    // or after the `EguiSet::BeginPass` system (which belongs to the `CoreSet::PreUpdate` set).

    // #[cfg(feature = "examples_world_inspector")]
    // app.add_plugins(WorldInspectorPlugin::default());
    app.insert_resource(PistolState::default())
        .insert_resource(utility::MouseStatus::default())
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
        .add_systems(
            OnEnter(GameState::Game),
            (
                setup_entity,
                setup_pistol
                    .after(setup_entity)
                    .before(GameLabSystems::WriteCharacterPartEvent),
            ),
        )
        .add_systems(
            Update,
            (
                player_movement,
                utility::update_mouse_position,
                ui_example_system,
                update_from_ui.after(ui_example_system),
                edge_pan_camera::update_edge_pan_camera,
                (update_mouse, update_collider_from_mouse).chain(), //update_blood_particles.after(ui_example_system),
            )
                .run_if(in_state(GameState::Game)),
        )
        .run();

    Ok(())
}

fn update_preview(
    commands: &mut Commands,
    ui_state: &ResMut<UiState>,
    query: &mut Query<(&mut VelloScene, &mut Transform), With<Preview>>,
    f: impl Fn() -> (BezPath, kurbo::Rect),
) {
    let scaler = ui_state.current.scale * ui_state.current.scale_modifier;
    let transform = Transform {
        translation: Vec3::new(ui_state.current.pos_x, ui_state.current.pos_y, 2000.0),
        rotation: Quat::from_rotation_z(ui_state.current.rotation.to_radians()),
        scale: Vec3::new(scaler, scaler, 1.0),
    };

    if let Ok((_, mut t)) = query.single_mut() {
        *t = transform;
    }
    if ui_state.preview_state_just_modified {
        let scene = if ui_state.preview_state {
            let (p, _) = f();
            let mut scene: VelloScene = VelloScene::default();
            scene.fill(
                peniko::Fill::NonZero,
                kurbo::Affine::default(),
                peniko::Color::rgba(0.0, 0.0, 1.0, 0.5),
                None,
                &p,
            );
            Some(scene)
        } else {
            None
        };
        if let Ok((mut s, _)) = query.single_mut() {
            s.reset();
            if let Some(scene) = scene {
                *s = scene;
            }
        } else {
            if let Some(scene) = scene {
                commands.spawn((
                    VelloSceneBundle {
                        scene,
                        transform,
                        ..Default::default()
                    },
                    Preview,
                ));
            }
        }
    }
}

fn spawn_collider(
    commands: &mut Commands,
    ui_state: &ResMut<UiState>,
    color: peniko::Brush,
    scale_modifier: f32,
    f: impl Fn() -> (BezPath, kurbo::Rect),
    soft_body_config: SoftBodyInitConfig,
    collision_config: CollisionConstraintConfig,
) {
    if !ui_state.just_spawn {
        return;
    }
    let mut entitys = vec![];
    let count = 1;
    for index in 0..count {
        let reverse_velocity = if index > 4 { -1.0 } else { 1.0 };
        let temp = make_collision_shape(
            commands,
            Vec4::new(
                // ui_state.current.pos_x + (100.0 * index as f32) - 500.0,
                ui_state.current.pos_x,
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
            11,
        );
        entitys.push(temp);
    }
}

fn update_from_ui(
    mut ui_state: ResMut<UiState>,
    mut commands: Commands,
    svg_colliders: Res<SvgColliderAssetManager>,
    images: Res<VelloImageAssetManager>,
    image_assets: Res<Assets<VelloImageAsset>>,
    custom_assets: Res<Assets<SvgColliderAsset>>,
    mut constraint_world: ResMut<VelloConstraintWorld>,
    query_c: Query<Entity, (With<VelloCollider>, Without<StaticSceneComponent>)>,
    query_j: Query<Entity, With<VelloJoint>>,
    mut preview_query: Query<(&mut VelloScene, &mut Transform), With<Preview>>,
) {
    if ui_state.delete_all_dynamic {
        query_c.iter().for_each(|e| {
            commands.entity(e).despawn();
        });
        query_j.iter().for_each(|e| {
            commands.entity(e).despawn();
        });
    }
    if ui_state.just_modified {
        constraint_world.set_gravity(Vec2::new(
            ui_state.c_config.gravity_x,
            ui_state.c_config.gravity_y,
        ));
    }
    if ui_state.just_spawn || (ui_state.preview_state_just_modified && ui_state.preview_state) {
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
                ui_state.current.scale_modifier = scaler;
                update_preview(&mut commands, &ui_state, &mut preview_query, make_rect);
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
                ui_state.current.scale_modifier = scaler;
                update_preview(&mut commands, &ui_state, &mut preview_query, make_circle);
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
                let index =
                    (ui_state.current.collider_type as u32 - ColliderType::Star as u32) as usize;
                let make_collider = || {
                    let svg_collider = custom_assets
                        .get(&svg_colliders.get_index(index).unwrap())
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
                ui_state.current.scale_modifier = scaler;
                update_preview(&mut commands, &ui_state, &mut preview_query, make_collider);
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
            ColliderType::PISTOL => {
                let index =
                    (ui_state.current.collider_type as u32 - ColliderType::Star as u32) as usize;
                let make_collider = || {
                    let svg_collider = custom_assets
                        .get(&svg_colliders.get_index(index).unwrap())
                        .unwrap();
                    (svg_collider.shape.clone(), svg_collider.aabb.clone())
                };
                let albedo = image_assets
                    .get(&images.get_index(2 as usize).unwrap())
                    .unwrap()
                    .image
                    .clone()
                    .with_usage(peniko::ImageUsageType::MASKED);
                let normals = image_assets
                    .get(&images.get_index(3 as usize).unwrap())
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
                ui_state.current.scale_modifier = scaler;
                update_preview(&mut commands, &ui_state, &mut preview_query, make_collider);
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
                let index =
                    (ui_state.current.collider_type as u32 - ColliderType::Star as u32) as usize;
                let make_collider = || {
                    let svg_collider = custom_assets
                        .get(&svg_colliders.get_index(index).unwrap())
                        .unwrap();
                    (svg_collider.shape.clone(), svg_collider.aabb.clone())
                };
                ui_state.current.scale_modifier = scaler;
                update_preview(&mut commands, &ui_state, &mut preview_query, make_collider);
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
    } else {
        let make_rect = || {
            let rect = kurbo::Rect::new(-20.0, -20.0, 20.0, 20.0);
            let rect_path = rect.to_path(0.1);
            (rect_path, rect)
        };
        update_preview(&mut commands, &ui_state, &mut preview_query, make_rect);
    }
}

//make a white background
fn setup_back_ground(mut commands: Commands) {
    commands.spawn((
        Camera2d::default(),
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
    let mut scene: VelloScene = VelloScene::default();
    scene.fill(
        peniko::Fill::NonZero,
        kurbo::Affine::default(),
        peniko::Color::rgb(1.0, 0.0, 0.0),
        None,
        &kurbo::Rect::new(-10.0, -10.0, 10.0, 10.0),
    );

    commands.spawn((
        VelloSceneBundle {
            transform: Transform {
                translation: Vec3::new(0.0, 0.0, 100.0),
                scale: Vec3::new(0.5, 0.5, 1.0),
                ..Default::default()
            },
            scene,
            ..Default::default()
        },
        CharacterRoot {
            svg_asset_id: "v6.character.svg".to_owned(),
            blueprint_asset_id: "v6.character.json".to_owned(),
        },
        CharacterController {
            move_vector: Vec2::ZERO,
            point_vector: Vec2::ZERO,
            ..Default::default()
        },
    ));
}

/// Spawn a pistol collider and connect it to the character's PRLA particle
/// via a bilinear joint. Runs after [`setup_entity`] so the character and its
/// particles are fully assembled.
fn setup_pistol(
    mut commands: Commands,
    mut events: EventWriter<AttachPistolToCharacterEvent>,
    character_q: Query<(Entity, &ConnectivityRoot), With<CharacterRoot>>,
    mut pistol_state: ResMut<PistolState>,
) {
    // There is only one character — get its entity and ConnectivityRoot.
    let (character_entity, _root) = character_q
        .single()
        .expect("expected exactly one character");

    let soft_body_init_transform = Transform {
        translation: Vec3::new(325.0, -90.0, 0.0),
        rotation: Quat::from_rotation_z(0.0_f32.to_radians()),
        scale: Vec3::new(0.1, 0.1, 1.0),
    };

    let pistol_entity = commands
        .spawn((
            VelloSceneBundle {
                ..Default::default()
            },
            ColliderRoot {
                svg_asset_id: "pistol.collider.svg".to_string(),
                albedo_asset_id: "pistol_albedo.png".to_string(),
                normal_asset_id: "pistol_normal.png".to_string(),
                metallic: 0.9,
                roughness: 0.2,
                softbody_config: SoftBodyInitConfig::default(),
                collision_config: CollisionConstraintConfig::default(),
                soft_body_init_transform,
                initial_velocity: Vec2::ZERO,
            },
            PistolControl {
                wrist_binding_uv: Vec2::new(0.15, 0.75),
                gun_point_uv: Vec2::new(1.0, 0.125),
                ..Default::default()
            },
        ))
        .observe(on_collision_spawn_particle)
        .id();
    events.write(AttachPistolToCharacterEvent {
        character: character_entity,
        pistol: pistol_entity,
    });
    pistol_state.registered = true;
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
    let soft_body_init_transform = Transform {
        translation: Vec3::new(transform.x, transform.y, 0.0),
        rotation: Quat::from_rotation_z(transform.z.to_radians()),
        scale: Vec3::new(transform.w, transform.w, 1.0),
    };
    let entity = commands
        .spawn((
            VelloSceneBundle {
                scene,
                transform: Transform {
                    translation: Vec3::new(transform.x, transform.y, 0.0),
                    ..Default::default()
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
                soft_body_init_transform,
                VELLO_COLLISION_COOL_DOWN_TIME,
            ),
        ))
        .id();
    if !is_soft_body {
        commands.entity(entity).insert(StaticSceneComponent);
    }
    entity
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
}

fn check_assets_loaded(
    mut sc_asset: EventReader<AssetEvent<SvgColliderAsset>>,
    mut im_asset: EventReader<AssetEvent<VelloImageAsset>>,
    mut cs_asset: EventReader<AssetEvent<SvgCharacterAsset>>,
    mut cb_asset: EventReader<AssetEvent<BlueprintCharacterAsset>>,
    mut colliders: ResMut<SvgColliderAssetManager>,
    mut images: ResMut<VelloImageAssetManager>,
    mut next_state: ResMut<NextState<GameState>>,
    mut character_svg: ResMut<SvgCharacterAssetManager>,
    mut character_blueprint: ResMut<BlueprintCharacterAssetManager>,
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

    for cs in cs_asset.read() {
        match cs {
            AssetEvent::LoadedWithDependencies { id } => {
                character_svg.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    for cb in cb_asset.read() {
        match cb {
            AssetEvent::LoadedWithDependencies { id } => {
                character_blueprint.mark_as_loaded(id);
            }
            _ => {}
        }
    }

    if colliders.all_loaded()
        && images.all_loaded()
        && character_svg.all_loaded()
        && character_blueprint.all_loaded()
    {
        next_state.set(GameState::Game);
    }
}

fn setup_resources(
    mut colliders: ResMut<SvgColliderAssetManager>,
    mut images: ResMut<VelloImageAssetManager>,
    mut character_svg: ResMut<SvgCharacterAssetManager>,
    mut character_blueprint: ResMut<BlueprintCharacterAssetManager>,
    asset_server: Res<AssetServer>,
) {
    let mut collider = |path: &str| {
        let (_, name) = path
            .split_once("/")
            .expect(&format!("wrong asset path {}", path));
        colliders.push(
            asset_server.load(path),
            VelloColliderAssetMetaData::default(),
            name,
        );
    };
    let mut image = |path: &str| {
        let (_, name) = path
            .split_once("/")
            .expect(&format!("wrong asset path {}", path));
        images.push(
            asset_server.load(path),
            VelloImageAssetMetaData::default(),
            name,
        );
    };
    let mut c_svg = |path: &str| {
        let (_, name) = path
            .split_once("/")
            .expect(&format!("wrong asset path {}", path));
        character_svg.push(
            asset_server.load(path),
            SvgCharacterAssetMetaData::default(),
            name,
        );
    };
    let mut c_blueprint = |path: &str| {
        let (_, name) = path
            .split_once("/")
            .expect(&format!("wrong asset path {}", path));
        character_blueprint.push(
            asset_server.load(path),
            BlueprintCharacterAssetMetaData::default(),
            name,
        );
    };
    collider("colliders/star.collider.svg");
    collider("colliders/heart.collider.svg");
    collider("colliders/key.collider.svg");
    collider("colliders/shield.collider.svg");
    collider("colliders/knife.collider.svg");
    collider("colliders/capsule.collider.svg");
    collider("colliders/ammo.collider.svg");
    collider("colliders/pistol.collider.svg");
    image("image/ammo_albedo.png");
    image("image/ammo_normal.png");
    image("image/pistol_albedo.png");
    image("image/pistol_normal.png");
    c_svg("character/v6.character.svg");
    c_blueprint("character/v6.character.json");
}

//fn update_blood_instances()
pub fn add_light(mut commands: Commands) {
    let mut light_scene: VelloScene = VelloScene::default();
    let light_radius = 800.0;
    //let light_shape_ratio = 1.0 / 40.0;
    info!("Add Light");
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

fn on_collision_spawn_particle(trigger: Trigger<VelloCollisionTrigger>, mut commands: Commands) {
    let event = trigger.event();
    let pos = event.collision_point;
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
}

fn player_movement(
    mouse_status: ResMut<MouseStatus>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut spine_q: Query<&mut SpineController>,
    mut pistol_q: Query<(Entity, &mut PistolControl)>,
    mut legacy_q: Query<&mut CharacterController>,
    mut pistol_state: ResMut<PistolState>,
    mut events: EventWriter<CharacterPartEvent>,
    mut fire: EventWriter<FireEvent>,
    character_q: Query<(Entity, &ConnectivityRoot), With<CharacterRoot>>,
) {
    let mut direction = Vec2::ZERO;

    // Check for key presses
    if keyboard_input.pressed(KeyCode::KeyW) {
        direction.y += 50.0;
    }
    if keyboard_input.pressed(KeyCode::KeyS) {
        direction.y -= 50.0;
    }
    if keyboard_input.pressed(KeyCode::KeyA) {
        direction.x -= 50.0;
    }
    if keyboard_input.pressed(KeyCode::KeyD) {
        direction.x += 50.0;
    }

    if let Ok(mut spine) = spine_q.single_mut() {
        spine.move_vector = direction;
    }

    let target = mouse_status.world_pos;
    if let Ok((entity, mut arm)) = pistol_q.single_mut() {
        if keyboard_input.pressed(KeyCode::KeyC) {
            arm.world_aim_trarget = Some(target);
        } else {
            arm.world_aim_trarget = None;
        };
        if keyboard_input.pressed(KeyCode::KeyF) {
            fire.write(FireEvent { weapon: entity });
        }
    }

    // Legacy: keep CharacterController in sync for any old systems still reading it.
    if let Ok(mut item) = legacy_q.single_mut() {
        item.move_vector = direction;
        item.point_vector = target;
    }

    // ── V key: Unregister the pistol (throw/drop) ────────────────────────
    if keyboard_input.just_pressed(KeyCode::KeyV) && pistol_state.registered {
        info!("Player pressed V — unregistering pistol");

        let (character_entity, _) = character_q
            .single()
            .expect("expected exactly one character");

        events.write(CharacterPartEvent::UnregisterPart {
            character: character_entity,
            path_id: "pistol".to_string(),
        });

        pistol_state.registered = false;
    }
}
