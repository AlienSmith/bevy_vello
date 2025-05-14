pub mod tank {
    pub mod base;
    pub mod gun;
    pub mod particles;
    pub mod shell;
    pub mod turrent;
}

pub mod enemy {
    pub mod alien;
    pub mod zombie;
}

pub mod camera {
    pub mod edge_pan_camera;
}

pub mod collision;
pub mod decor;
pub mod tank_parts;
pub mod text;

pub mod sprite_sheet;

use avian2d::{prelude::PhysicsSet, PhysicsPlugins};
use bevy::{asset::AssetMetaCheck, prelude::*};
use bevy_vello::{
    integrations::HanabiIntegrationPlugin, prelude::VelloReplaySceneAsset, VelloPlugin, VelloScene,
    VelloSceneBundle,
};
pub use camera::edge_pan_camera::{update_edge_pan_camera, EdgePanCamera};
use collision::remove_single_frame_colliders;
pub use collision::{handle_collisions, ColliderFlags, ColliderResponds, Health};
pub use enemy::alien::{spawn_static_enemy_at, static_alien_control_system};
use enemy::zombie;
use seldom_state::StateMachinePlugin;
use text::DefaultFonts;
pub use text::{pop_text_update, spawn_pop_text_at, PopTextAnim};

pub use tank::particles::{
    init_particles_player, spawn_particle_at, update_particle_scene, ParticleSceneAnim,
    ParticlesPlayer,
};

pub use decor::{spawn_stone_at, spawn_tree_at, update_tree};
pub use sprite_sheet::{make_sprite_sheet_scene_from_vello_replay_scene, spawn_sprite_sheet_at};
pub use tank_parts::TankGameAssets;

pub mod prelude {
    pub use crate::{
        camera::edge_pan_camera::{update_edge_pan_camera, EdgePanCamera},
        collision::{handle_collisions, ColliderFlags, ColliderResponds, Health},
        decor::{spawn_stone_at, spawn_tree_at, update_tree},
        enemy::alien::{spawn_static_enemy_at, static_alien_control_system},
        spawn_one_time_sprite_sheet_at,
        sprite_sheet::{make_sprite_sheet_scene_from_vello_replay_scene, spawn_sprite_sheet_at},
        tank_parts::TankGameAssets,
        text::{pop_text_update, spawn_pop_text_at, PopTextAnim},
        zombie::{spawn_zombie_at, Zombie, ZombieInputComponent, ZombieInputEvent},
        OneTimeEntity, StateAwarePlugin, TankGameAssetsMetaData, TankGameAssetsType,
    };
}

#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(C)]
pub struct TankGameAssetsType(pub u32);

impl TankGameAssetsType {
    pub const BASE: Self = Self(0);
    pub const TURRENT: Self = Self(1);
    pub const GUN: Self = Self(2);
    pub const TREE: Self = Self(3);
    pub const STONE: Self = Self(4);
    pub const ZOMBIE_IDEL: Self = Self(5);
    pub const ZOMBIE_MOVE: Self = Self(6);
    pub const ZOMBIE_ATTACK: Self = Self(7);
    pub const GUN_FLARE: Self = Self(8);
    pub const GUN_FIRE: Self = Self(9);
}

#[derive(Clone, Component)]
pub struct OneTimeEntity {
    timer: Timer,
}

pub fn spawn_one_time_sprite_sheet_at(
    commands: &mut Commands,
    parts: &Res<TankGameAssets>,
    custom_assets: &Res<Assets<VelloReplaySceneAsset>>,
    transform: Transform,
    start_time: f32,
    duration_in_seconds: f32,
    part_type: TankGameAssetsType,
) -> Option<Entity> {
    let mut b_s = VelloScene::default();
    if let TankGameAssetsMetaData::SpriteSheet(frame) =
        parts.get_part_at_index(part_type).unwrap().meta
    {
        let fps = frame as f32 / duration_in_seconds;
        make_sprite_sheet_scene_from_vello_replay_scene(
            &mut b_s,
            &custom_assets,
            &parts,
            part_type,
            Some(false),
            Some(start_time),
            Some(fps),
        );
        let entity = commands
            .spawn((
                VelloSceneBundle {
                    scene: b_s,
                    transform,
                    ..Default::default()
                },
                OneTimeEntity {
                    timer: Timer::from_seconds(duration_in_seconds, TimerMode::Once),
                },
            ))
            .id();
        return Some(entity);
    }
    None
}

pub fn update_one_time_entity(
    time: Res<Time>,
    mut commands: Commands,
    mut q: Query<(Entity, &mut OneTimeEntity)>,
) {
    let delta = time.delta();
    for (entity, mut timer) in q.iter_mut() {
        timer.timer.tick(delta);
        if timer.timer.finished() {
            commands.entity(entity).despawn();
        }
    }
}

#[derive(Clone, Default)]
pub enum TankGameAssetsMetaData {
    #[default]
    PBR,
    //we need the total frame counts
    SpriteSheet(u32),
}

impl From<TankGameAssetsType> for usize {
    fn from(value: TankGameAssetsType) -> Self {
        value.0 as usize
    }
}

pub use zombie::{spawn_zombie_at, Zombie, ZombieInputComponent, ZombieInputEvent};

pub struct StateAwarePlugin<S: States> {
    state: S,
}

impl<S: States> StateAwarePlugin<S> {
    /// Create a new plugin that runs systems only in `state`.
    pub fn new(state: S) -> Self {
        Self { state }
    }
}

impl<S: States> Plugin for StateAwarePlugin<S> {
    fn build(&self, app: &mut App) {
        app.add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }))
        .add_plugins(HanabiIntegrationPlugin)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(VelloPlugin)
        .add_plugins(StateMachinePlugin)
        .insert_resource(ParticlesPlayer::default())
        .insert_resource(TankGameAssets::default())
        .insert_resource(DefaultFonts::default())
        .add_systems(
            Update,
            (
                tank::base::control_system,
                tank::turrent::control_system,
                tank::gun::control_system,
                tank::shell::update_shell,
                update_particle_scene,
                update_edge_pan_camera,
                static_alien_control_system,
                pop_text_update,
                update_tree,
                update_one_time_entity,
                zombie::zombie_go_to_target_update,
                zombie::on_add_move_to_zombie,
                zombie::on_add_idle_to_zombie,
                zombie::on_add_attack_to_zombie,
                zombie::update_zombie_attack,
                // Add more systems here...
            )
                .run_if(in_state(self.state.clone())), // Automatically apply the condition
        )
        .add_systems(
            PostUpdate,
            (
                handle_collisions
                    .after(PhysicsSet::StepSimulation)
                    .before(PhysicsSet::Sync), // Important!
                remove_single_frame_colliders
                    .after(handle_collisions)
                    .before(PhysicsSet::Sync),
            ),
        );
    }
}
