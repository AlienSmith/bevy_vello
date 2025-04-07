mod scene_gen;
use avian2d::prelude::*;
use bevy::asset::AssetMetaCheck;
use bevy::math::vec3;
use bevy::prelude::*;
use bevy_vello::{
    add_default_light, integrations::HanabiIntegrationPlugin, prelude::*, VelloPlugin,
};
use seldom_state::prelude::*;
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
use tankgame_lib::{make_sprite_sheet_scene_from_vello_replay_scene, TankGameAssetsMetaData};

#[derive(Clone, Component, Reflect)]
struct Zombie;

#[derive(Clone, Component, Reflect)]
#[component(storage = "SparseSet")]
struct Idle;

#[derive(Clone, Copy, Component, Reflect)]
#[component(storage = "SparseSet")]
struct GoToSelection {
    speed: f32,
    target: Vec2,
}

#[derive(Clone, Component, Reflect)]
#[component(storage = "SparseSet")]
struct Attack {
    timer: Option<Timer>,
}

fn go_to_target(
    mut commands: Commands,
    mut go_to_selections: Query<(Entity, &mut Transform, &GlobalTransform, &GoToSelection)>,
    time: Res<Time>,
) {
    for (entity, mut transform, global_transform, go_to_selection) in &mut go_to_selections {
        let target = go_to_selection.target;
        let delta = target - transform.translation.truncate();
        let movement = delta.normalize_or_zero() * go_to_selection.speed * time.delta_seconds();
        let global_to_local = global_transform.compute_matrix().inverse();
        let dif_3_local = global_to_local.transform_vector3(movement.extend(0.0));
        let x = vec3(1.0, 0.0, 0.0);
        let cross = x.cross(dif_3_local).z;
        let angle = x.angle_between(dif_3_local);
        let rotate_direction = cross.signum();
        if movement.length() > delta.length() {
            transform.translation = target.extend(transform.translation.z);
            // The player has reached the target! Add the `Done` component to the player, causing
            // `done` to trigger. It will be automatically removed later this frame.
            commands.entity(entity).insert(Done::Success);
            info!("Done!")
        } else {
            transform.translation += movement.extend(0.);
        }
        transform.rotate_z(rotate_direction * angle);
    }
}

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

fn attack_click(mouse: Res<ButtonInput<MouseButton>>) -> Result<(), ()> {
    if mouse.just_pressed(MouseButton::Left) {
        Ok(())
    } else {
        Err(())
    }
}

fn click(
    mouse: Res<ButtonInput<MouseButton>>,
    cursor_position: Res<CursorPosition>,
) -> Option<Vec2> {
    mouse
        .just_pressed(MouseButton::Right)
        .then_some(())
        .and(**cursor_position)
}

pub fn test_spawn_zombie(
    mut commands: Commands,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    let mut b_s = VelloScene::default();
    make_sprite_sheet_scene_from_vello_replay_scene(
        &mut b_s,
        &custom_assets,
        &parts,
        TankGameAssetsType::ZOMBIE_IDEL,
        None,
        None,
        None,
    );
    // commands.spawn();
    commands.spawn((
        Zombie,
        Idle,
        StateMachine::default()
            // When the player clicks, go there
            .trans_builder(click, |_: &Idle, pos| {
                Some(GoToSelection {
                    speed: 200.,
                    target: pos,
                })
            })
            // `done` triggers when the `Done` component is added to the entity. When they're done
            // going to the selection, idle.
            .trans::<GoToSelection, _>(done(Some(Done::Success)), Idle)
            .trans::<Idle, _>(attack_click, Attack { timer: None })
            .trans::<Attack, _>(done(Some(Done::Success)), Idle)
            .set_trans_logging(true),
        VelloSceneBundle {
            scene: b_s,
            transform: Transform::from_xyz(200.0, 200.0, -100.0),
            ..Default::default()
        },
    ));
}

fn on_add_attack_to_zombie(
    time: Res<Time>,
    mut query: Query<(&mut VelloScene, &mut Attack), (Added<Attack>, With<Zombie>)>,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    let mut temp: Option<VelloScene> = None;
    for (mut scene, mut attack) in query.iter_mut() {
        if temp.is_none() {
            let start_time = time.elapsed_seconds();
            let mut b_s = VelloScene::default();
            make_sprite_sheet_scene_from_vello_replay_scene(
                &mut b_s,
                &custom_assets,
                &parts,
                TankGameAssetsType::ZOMBIE_ATTACK,
                Some(false),
                Some(start_time),
                Some(9.0),
            );
            temp = Some(b_s);
        }
        *scene = temp.clone().unwrap();
        attack.timer = Some(Timer::from_seconds(1.1, TimerMode::Once));
    }
}

fn update_attack_timer(
    mut commands: Commands,
    time: Res<Time>,
    mut query: Query<(Entity, &mut Attack), With<Zombie>>,
) {
    for (entity, mut attack) in query.iter_mut() {
        if let Some(timer) = &mut attack.timer {
            timer.tick(time.delta());
            if timer.finished() {
                commands.entity(entity).insert(Done::Success);
                info!("Done!")
            }
        }
    }
}

fn on_add_move_to_zombie(
    time: Res<Time>,
    mut query: Query<&mut VelloScene, (Added<GoToSelection>, With<Zombie>)>,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    let mut temp: Option<VelloScene> = None;
    for mut scene in query.iter_mut() {
        if temp.is_none() {
            let start_time = time.elapsed_seconds();
            let mut b_s = VelloScene::default();
            make_sprite_sheet_scene_from_vello_replay_scene(
                &mut b_s,
                &custom_assets,
                &parts,
                TankGameAssetsType::ZOMBIE_MOVE,
                None,
                Some(start_time),
                None,
            );
            temp = Some(b_s);
        }
        *scene = temp.clone().unwrap();
    }
}

fn on_add_idle_to_zombie(
    time: Res<Time>,
    mut query: Query<&mut VelloScene, (Added<Idle>, With<Zombie>)>,
    parts: Res<TankGameAssets>,
    custom_assets: Res<Assets<VelloReplaySceneAsset>>,
) {
    let mut temp: Option<VelloScene> = None;
    for mut scene in query.iter_mut() {
        if temp.is_none() {
            let start_time = time.elapsed_seconds();
            let mut b_s = VelloScene::default();
            make_sprite_sheet_scene_from_vello_replay_scene(
                &mut b_s,
                &custom_assets,
                &parts,
                TankGameAssetsType::ZOMBIE_IDEL,
                None,
                Some(start_time),
                None,
            );
            temp = Some(b_s);
        }
        *scene = temp.clone().unwrap();
    }
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
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }))
        .add_plugins(HanabiIntegrationPlugin)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(PhysicsDebugPlugin::default())
        .add_plugins(VelloPlugin)
        .add_plugins(StateMachinePlugin)
        .insert_resource(ParticlesPlayer::default())
        .insert_resource(TankGameAssets::default())
        .insert_resource(DefaultFonts::default())
        .insert_resource(CursorPosition::default())
        .init_state::<GameState>()
        .add_systems(Startup, setup_resources)
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
            (
                tank::base::control_system,
                tank::turrent::control_system,
                tank::gun::control_system,
                update_particle_scene,
                update_edge_pan_camera,
                static_alien_control_system,
                pop_text_update,
                update_shell,
                update_tree,
                update_cursor_position,
                go_to_target,
                on_add_move_to_zombie,
                on_add_idle_to_zombie,
                on_add_attack_to_zombie,
                update_attack_timer,
            )
                .run_if(in_state(GameState::Game)),
        )
        .add_systems(
            PostUpdate,
            (
                handle_collisions
                    .after(PhysicsSet::StepSimulation)
                    .before(PhysicsSet::Sync), // Important!
            ),
        )
        .run();
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

    fonts.default_font = asset_server.load("fonts/Rubik-Medium.ttf");
}

fn setup_vector_graphics(
    mut commands: Commands,
    parts: ResMut<TankGameAssets>,
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
}
