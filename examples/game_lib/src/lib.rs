mod ai;
mod character;
pub mod character_asset;
mod character_factory;
mod collider_factory;
mod damage;
mod death_channel;
mod decorations;
mod headless;
mod health;
mod input_management;
mod lower_body;
mod utility;
pub mod weapons;
use bevy::prelude::*;

use crate::{
    ai::AiPlugin,
    character::plugin::GameCharacterPlugin,
    character_asset::plugin::CharacterLoaderPlugin,
    character_factory::plugin::CharacterFactoryPlugin,
    damage::DamagePlugin,
    death_channel::plugin::DeathChannelPlugin,
    decorations::DecorationPlugin,
    health::HealthPlugin,
    utility::{tick_delayed_events, DelayedEventTrigger},
    weapons::plugin::WeaponPlugin,
};
#[derive(Default)]
pub struct VelloCharacterPlugin;
pub use crate::{
    ai::{build_enemy_ai_tree, AiPhase, AiState},
    character::{
        CharacterController, Connectivity, ConnectivityRoot, IkMode, LeftArmController,
        LeftLegController, LimbConfig, ResetLimbControlConstraintsEvent, RightArmController,
        RightLegController, SpineConfig, SpineController, SpineDriveEnabled, SpineIndicator,
        StringPool, WhichLimb,
    },
    character_factory::{CharacterPartEvent, CharacterRoot},
    damage::{AttackStats, Damageable, PartKind, TotalHealth},
    decorations::{Decoration, DecorationAnchor},
    headless::{
        capture_telemetry, cell_verdict, check_bone_rigidity, check_invariants, diff_rows,
        flush_csv, flush_rows, rows_hash, tail_verdict, Bone, HeadlessTelemetryPlugin,
        InvariantConfig, InvariantReport, ParticleTelemetryConfig, ResolvedParticles,
        RigidityReport, TelemetryBuffer, TelemetryRow, Verdict, V10_PARTICLES,
    },
    health::{Die, Health},
    input_management::{
        actions::{default_input_map, PlayerAction},
        mouse::MouseWorldPosition,
        systems::Player as PlayerMarker,
        InputPlugin,
    },
    lower_body::{
        compute_forward_target, compute_support_velocity, compute_v_max, compute_x_max,
        drive_hip_pin, pd_target, reset_hip_pin, rigid_frame_targets, softened_compliance,
        solve_leg_ik, DebugInfo, GaitLimits, GaitPhase, HipPin, HipPinParams, LegState,
        LowerBodyConfig, LowerBodyController, LowerBodyInput, LowerBodyParams, LowerBodyPose,
        LowerBodySandboxPlugin, ResolverState,
    },
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, SystemSet)]
pub enum GameLabSystems {
    WriteCharacterPartEvent,
    ReadCharacterPartEvent,
    ProcessDeathEvents,
    CheckHealth,
}

impl Plugin for VelloCharacterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InputPlugin)
            .add_plugins(AiPlugin)
            .add_plugins(GameCharacterPlugin)
            .add_plugins(CharacterFactoryPlugin)
            .add_plugins(CharacterLoaderPlugin)
            .add_plugins(ColliderFactoryPlugin)
            .add_plugins(WeaponPlugin)
            .add_plugins(HealthPlugin)
            .add_plugins(DamagePlugin)
            .add_plugins(DecorationPlugin)
            .add_plugins(DeathChannelPlugin)
            .add_event::<DelayedEventTrigger>()
            .configure_sets(
                Update,
                (
                    GameLabSystems::WriteCharacterPartEvent,
                    GameLabSystems::ProcessDeathEvents,
                    GameLabSystems::ReadCharacterPartEvent,
                    GameLabSystems::CheckHealth,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                tick_delayed_events.in_set(GameLabSystems::WriteCharacterPartEvent),
            );
    }
}

pub use crate::{collider_factory::plugin::ColliderFactoryPlugin, collider_factory::ColliderRoot};
