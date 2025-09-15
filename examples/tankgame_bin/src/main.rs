mod scene_gen;
use std::default;

use avian2d::prelude::*;
use bevy::asset::{embedded_asset, AssetMetaCheck};
use bevy::math::vec3;
use bevy::prelude::*;
use bevy::reflect::EnumInfo;
use bevy_vello::{
    add_default_light, integrations::HanabiIntegrationPlugin, prelude::*, VelloPlugin,
};

use tankgame_lib::{
    handle_collisions, pop_text_update, spawn_pop_text_at, spawn_static_enemy_at, spawn_stone_at,
    spawn_tree_at, static_alien_control_system, text::DefaultFonts, update_edge_pan_camera,
    update_tree, EdgePanCamera, TankGameAssets, TankGameAssetsType,
};
use tankgame_lib::{
    init_particles_player,
    tank::{self, shell::update_shell},
    update_particle_scene, ParticlesPlayer,
};
use tankgame_lib::{
    make_sprite_sheet_scene_from_vello_replay_scene, spawn_zombie_at, StateAwarePlugin,
    TankGameAssetsMetaData, Zombie, ZombieInputComponent, ZombieInputEvent,
};

#[derive(Default, Deref, DerefMut, Resource)]
struct CursorPosition(Option<Vec2>);

fn update_cursor_position(
    cameras: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
    mut position: ResMut<CursorPosition>,
) {
    let (camera, transform) = cameras.single();
    **position = windows
        .single()
        .cursor_position()
        .and_then(|cursor_position| camera.viewport_to_world_2d(transform, cursor_position));
}

fn test_zombie_input(
    mouse: Res<ButtonInput<MouseButton>>,
    cursor_position: Res<CursorPosition>,
    mut zombies: Query<&mut ZombieInputComponent>,
) {
    //clear all previous events
    let this_frame_event = if mouse.just_pressed(MouseButton::Left) {
        Some(ZombieInputEvent::Attack)
    } else if let Some(pos) = mouse
        .just_pressed(MouseButton::Right)
        .then_some(())
        .and(**cursor_position)
    {
        Some(ZombieInputEvent::MoveTo(pos))
    } else {
        None
    };
    for mut input in zombies.iter_mut() {
        input.event = this_frame_event;
    }
}

pub fn test_spawn_zombie(
    mut commands: Commands,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    spawn_zombie_at(
        &mut commands,
        &parts,
        &custom_assets,
        Vec3::new(200.0, 200.0, -100.0),
        0.4,
    );
}

pub fn test_spawn_deco(
    mut commands: Commands,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    spawn_tree_at(
        &mut commands,
        &parts,
        &custom_assets,
        Vec3 {
            x: 500.0,
            y: 500.0,
            z: 0.0,
        },
    );
    spawn_tree_at(
        &mut commands,
        &parts,
        &custom_assets,
        Vec3 {
            x: 0.0,
            y: 500.0,
            z: 0.0,
        },
    );
    spawn_stone_at(
        &mut commands,
        &parts,
        &custom_assets,
        Vec3 {
            x: 500.0,
            y: 0.0,
            z: 0.0,
        },
    );

    spawn_stone_at(
        &mut commands,
        &parts,
        &custom_assets,
        Vec3 {
            x: 500.0,
            y: -500.0,
            z: 0.0,
        },
    );
}

pub fn test_spawn_enemy(mut commands: Commands, asset_server: Res<AssetServer>) {
    spawn_static_enemy_at(
        &mut commands,
        Transform::from_scale(Vec3::new(0.05, 0.05, 1.0)),
        &asset_server,
    );
}

pub fn test_spawn_pop_text(mut commands: Commands, fonts: Res<DefaultFonts>) {
    spawn_pop_text_at(
        &mut commands,
        &fonts,
        Vec3::new(0.0, 0.0, 1000.0),
        "12345",
        100.0,
    );
}

#[derive(Clone, Eq, PartialEq, Debug, Hash, Default, States)]
enum GameState {
    #[default]
    Loading,
    Game,
}

fn main() {
    let tank_lib_plugin = StateAwarePlugin::new(GameState::Game);
    let mut app = App::new();
    app.add_plugins(tank_lib_plugin)
        .add_plugins(PhysicsDebugPlugin::default())
        .insert_resource(CursorPosition::default())
        .init_state::<GameState>()
        .add_systems(Startup, (setup_resources, setup_on_screen_info))
        .add_systems(
            Update,
            check_assets_loaded.run_if(in_state(GameState::Loading)),
        )
        .add_systems(
            OnEnter(GameState::Game),
            (
                setup_vector_graphics,
                test_spawn_deco,
                add_default_light,
                init_particles_player,
                test_spawn_enemy,
                test_spawn_pop_text,
                test_spawn_zombie,
            ),
        )
        .add_systems(
            Update,
            (test_zombie_input, update_cursor_position).run_if(in_state(GameState::Game)),
        );
    embedded_asset!(app, "../assets/Rubik-Medium.ttf");
    app.run();
    bevy::log::warn!("Initialize");
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

fn setup_resources(
    mut fonts: ResMut<DefaultFonts>,
    mut tank_parts: ResMut<TankGameAssets>,
    asset_server: Res<AssetServer>,
) {
    tank_parts.push(
        asset_server.load("scenes/base.scene"),
        TankGameAssetsMetaData::default(),
    );
    tank_parts.push(
        asset_server.load("scenes/turrent.scene"),
        TankGameAssetsMetaData::default(),
    );
    tank_parts.push(
        asset_server.load("scenes/gun.scene"),
        TankGameAssetsMetaData::default(),
    );
    tank_parts.push(
        asset_server.load("scenes/tree.scene"),
        TankGameAssetsMetaData::default(),
    );
    tank_parts.push(
        asset_server.load("scenes/stone.scene"),
        TankGameAssetsMetaData::default(),
    );
    tank_parts.push(
        asset_server.load("scenes/zombie_idle_5_4_17.scene"),
        TankGameAssetsMetaData::SpriteSheet(17),
    );
    tank_parts.push(
        asset_server.load("scenes/zombie_move_5_4_17.scene"),
        TankGameAssetsMetaData::SpriteSheet(17),
    );
    tank_parts.push(
        asset_server.load("scenes/zombie_attack_3_3_9.scene"),
        TankGameAssetsMetaData::SpriteSheet(9),
    );
    tank_parts.push(
        asset_server.load("scenes/gunflare_2_2_4.scene"),
        TankGameAssetsMetaData::SpriteSheet(4),
    );
    tank_parts.push(
        asset_server.load("scenes/gunfire_2_2_4.scene"),
        TankGameAssetsMetaData::SpriteSheet(4),
    );

    fonts.default_font = asset_server.load("fonts/Rubik-Medium.ttf");
}

pub fn spawn_gun_fire_at(
    commands: &mut Commands,
    parts: &Res<TankGameAssets>,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    transform: Transform,
    start_time: f32,
) {
    let mut b_s = VelloScene::default();
    make_sprite_sheet_scene_from_vello_replay_scene(
        &mut b_s,
        &custom_assets,
        &parts,
        TankGameAssetsType::GUN_FIRE,
        Some(true),
        Some(start_time),
        Some(16.0),
    );
    commands.spawn((VelloSceneBundle {
        scene: b_s,
        transform,
        ..Default::default()
    },));
}

fn setup_on_screen_info(mut commands: Commands, asset_server: ResMut<AssetServer>) {
    commands.spawn(VelloTextBundle {
        font: asset_server.load("embedded://text/assets/Rubik-Medium.ttf"),
        text: VelloText {
            content: "Welcome to the test ground!".to_string(),
            size: 15.0,
            brush: Some(peniko::Brush::Solid(peniko::Color::RED)),
        },
        alignment: bevy_vello::text::VelloTextAlignment::TopLeft,
        transform: Transform::from_xyz(100.0, 85.0, 0.0),
        coordinate_space: CoordinateSpace::ScreenSpace,
        debug_visualizations: DebugVisualizations::Visible,
        ..default()
    });
}

fn setup_vector_graphics(
    mut commands: Commands,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    commands.spawn((Camera2dBundle::default(), EdgePanCamera::default()));
    let mut b_s = VelloScene::default();
    custom_assets
        .get(&parts.get_index(TankGameAssetsType::BASE).unwrap())
        .unwrap()
        .player
        .apply_to_scene(&mut b_s);

    let mut t_s = VelloScene::default();
    custom_assets
        .get(&parts.get_index(TankGameAssetsType::TURRENT).unwrap())
        .unwrap()
        .player
        .apply_to_scene(&mut t_s);

    let mut g_s = VelloScene::default();
    custom_assets
        .get(&parts.get_index(TankGameAssetsType::GUN).unwrap())
        .unwrap()
        .player
        .apply_to_scene(&mut g_s);

    commands
        .spawn((
            VelloSceneBundle {
                scene: b_s,
                transform: Transform::from_translation(Vec3::new(0., 0., 0.)),
                ..Default::default()
            },
            tank::base::Base::new(200.0, 50.0),
        ))
        .with_children(|parent| {
            parent
                .spawn((
                    VelloSceneBundle {
                        scene: t_s,
                        transform: Transform::from_translation(Vec3::new(0., 0., 2.)),
                        ..Default::default()
                    },
                    tank::turrent::Turrent::new(20.0),
                ))
                .with_children(|parent| {
                    parent.spawn((
                        VelloSceneBundle {
                            scene: g_s,
                            transform: Transform::from_translation(Vec3::new(0., 0., -1.)),
                            ..Default::default()
                        },
                        tank::gun::Gun::default(),
                    ));
                });
        });

    // spawn_gun_fire_at(
    //     &mut commands,
    //     &parts,
    //     &custom_assets,
    //     Transform::from_translation(Vec3 {
    //         x: 0.0,
    //         y: 0.0,
    //         z: 65536.0,
    //     }),
    //     0.0,
    // )
}
