use std::f32::consts::PI;

use bevy::{
    ecs::intern::{Interned, Interner},
    platform::collections::{HashMap, HashSet},
    prelude::*,
};

mod ik;
mod observers;
pub mod plugin;
mod systems;

#[derive(Component, Clone)]
pub struct Connectivity {
    pub name: Interned<str>,
    pub character: Entity,
    pub parts: HashSet<Entity>,
    pub death_propegate: bool,
}

#[derive(Component, Default, Clone)]
pub struct ConnectivityRoot {
    pub parts: HashMap<Interned<str>, Entity>,
}

impl Connectivity {
    pub fn new(character: Entity, death_propegate: bool, name: Interned<str>) -> Self {
        Connectivity {
            name,
            character,
            parts: HashSet::default(),
            death_propegate,
        }
    }
}

#[derive(Resource, Default)]
pub struct StringPool {
    pub pool: Interner<str>,
}

// ---------------------------------------------------------------------------
// IK Mode
// ---------------------------------------------------------------------------

/// IK mode for the arm.
#[derive(Clone, Default)]
pub enum IkMode {
    /// IK is disabled — no events emitted.
    #[default]
    Disabled,
    /// Position-based IK: place wrist at target (law of cosines).
    Reach,
    /// Direction-based IK: aim forearm at target (least-action solver).
    /// the assumption is the aim direction and elbow to arm would be alined but offset vertically against the aim direction
    Aim { weapon_offset_y: f32 },
}

// ---------------------------------------------------------------------------
// Config structs (not Components)
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct SpineConfig {
    /// Compliance for the external position constraints (lower = stiffer).
    pub compliance: f32,
    /// Damping for the external position constraints (0..1, higher = more).
    pub damping: f32,
    /// Interpolation factor per frame for the target centre position.
    pub pos_alpha: f32,
    /// Interpolation factor per frame for the target heading angle.
    pub ang_alpha: f32,
    /// Max centre speed in px/s; `max_pos_step = max_pos_speed * dt` is the
    /// per-frame position cap.
    pub max_pos_speed: f32,
    /// Maps per-frame angular step (turn speed this frame) to the P1_P2_P3
    /// rest-angle lean. Faster turn ⇒ bigger lean.
    pub lean_gain: f32,
}

impl Default for SpineConfig {
    fn default() -> Self {
        Self {
            compliance: 1e-7,
            damping: 0.1,
            pos_alpha: 0.5,
            ang_alpha: 0.5,
            max_pos_speed: 900.0,
            lean_gain: 0.06,
        }
    }
}

#[derive(Clone)]
pub struct ArmConfig {
    /// Fraction of distance to blend toward true target each frame.
    /// 0.3 = move virtual target 30% toward true target from current wrist.
    pub target_blend: f32,
    /// Distance threshold: when |wrist - true_target| < this, stop updating.
    pub convergence_threshold: f32,
    /// Compliance for angular constraints on arm joints.
    /// Must be significantly softer than the default 0.000001 from the character
    /// JSON so the XPBD solver can actually move the arm. 0.1 is a good start.
    pub angular_compliance: f32,
    /// Maximum angular velocity for the constraint rest angle, in radians per second.
    /// The rest angle chases the IK target at this rate, preventing sudden jumps
    /// that cause overshoot and body wobble.
    /// π rad/s = 180°/s — fast enough to be responsive, slow enough to prevent overshoot.
    pub max_angle_rate: f32,
    /// Bend direction for the arm IK.
    /// -1.0 = bend downward (elbow below shoulder-wrist line, default for right arm).
    /// +1.0 = bend upward (elbow above shoulder-wrist line, default for left arm).
    pub bend_sign: f32,
    /// IK mode for the arm.
    pub ik_mode: IkMode,
}

impl Default for ArmConfig {
    fn default() -> Self {
        Self {
            target_blend: 0.3,
            convergence_threshold: 2.0,
            angular_compliance: 5e-8,
            max_angle_rate: std::f32::consts::PI,
            bend_sign: -1.0,
            ik_mode: IkMode::Disabled,
        }
    }
}

// ---------------------------------------------------------------------------
// New controller Components (with pre-cached entity handles)
// ---------------------------------------------------------------------------

/// Spine controller with pre-cached particle entity handles + per-frame input.
/// Spine particles: [PH, P0, P1, P2, P3]
#[derive(Component, Clone)]
pub struct SpineController {
    pub particles: [Entity; 5],
    pub angulars: [Entity; 3],
    pub config: SpineConfig,
    /// Movement direction. Written by player_movement, AI, etc.
    pub move_vector: Vec2,
}

/// The spine "virtual handle": the commanded pose the external position
/// constraints pull P1/P2/P3 toward.
///
/// The virtual pose (`center` + `angle`) is per-frame interpolated + capped
/// toward a *latched* desired goal (`desired_center` + `desired_angle`):
///
/// - `desired_center` / `desired_angle` are the goal. While the player holds
///   input they are derived from the input direction + the current P2 position
///   (`P2 + heading_dir * reach`); when input stops they are **frozen** so the
///   character keeps its last commanded pose instead of resetting/chasing.
/// - `center` / `angle` are the virtual pose: the per-frame interpolation
///   toward the desired goal, clamped by `max_pos_step` / `max_ang_step`. This
///   is exactly what the indicator visual is drawn at AND what the external
///   position constraints pull the spine toward (one shared target).
/// - `local_points` are the three P2-centred quad-particle offsets (P1/P2/P3),
///   in local Vello (y-down) coords, derived once from the assembled particles.
/// - `character` is the root entity of the driven character (for resolving its
///   [`SpineController`] particles/joints directly).
/// - `linear_speed` / `angular_speed` store the per-frame speeds (for scaling
///   the `P1_P2_P3` rest-angle lean), re-derived each frame — no accumulated state.
#[derive(Component, Clone)]
pub struct SpineIndicator {
    /// Virtual heading (world/Vello radians), interpolated toward `desired_angle`.
    pub angle: f32,
    /// Virtual centre (P2 world position, Vello y-down), interpolated toward `desired_center`.
    pub center: Vec2,
    /// Latched desired heading goal (world/Vello radians).
    pub desired_angle: f32,
    /// Latched desired centre goal (P2 world position, Vello y-down).
    pub desired_center: Vec2,
    /// The three quad-particle offsets (P1/P2/P3), P2-centred, in Vello y-down coords.
    pub local_points: [Vec2; 3],
    /// Root entity of the driven character (for resolving its SpineController).
    pub character: Entity,
    /// Per-frame linear speed (px/s).
    pub linear_speed: f32,
    /// Per-frame angular speed (rad/s).
    pub angular_speed: f32,
}

impl Default for SpineIndicator {
    fn default() -> Self {
        Self {
            angle: 0.0,
            center: Vec2::ZERO,
            desired_angle: 0.0,
            desired_center: Vec2::ZERO,
            local_points: [Vec2::ZERO; 3],
            character: Entity::PLACEHOLDER,
            linear_speed: 0.0,
            angular_speed: 0.0,
        }
    }
}

/// Right arm controller with pre-cached entity handles + per-frame input.
/// Particles: [P1, P12, P13, PRLA]
/// Joints: [P1_P12_P13, P12_P13_PRLA]
#[derive(Component, Clone)]
pub struct RightArmController {
    pub particles: [Entity; 4],
    pub joints: [Entity; 2],
    pub config: ArmConfig,
    /// Aim target in world space. Written by weapon system, AI, player input, etc.
    pub target: Vec2,
}

/// Left arm controller with pre-cached entity handles + per-frame input.
/// Particles: [P1, P11, P10, PLLA]
/// Joints: [P1_P11_P10, P11_P10_PLLA]
#[derive(Component, Clone)]
pub struct LeftArmController {
    pub particles: [Entity; 4],
    pub joints: [Entity; 2],
    pub config: ArmConfig,
    /// Aim target in world space. Written by weapon system, AI, player input, etc.
    pub target: Vec2,
}

// ---------------------------------------------------------------------------
// Legacy — kept for migration, will be removed later
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct ArmController {
    /// Fraction of distance to blend toward true target each frame.
    /// 0.3 = move virtual target 30% toward true target from current wrist.
    pub target_blend: f32,
    /// Distance threshold: when |wrist - true_target| < this, stop updating.
    pub convergence_threshold: f32,
    /// Compliance for angular constraints on arm joints.
    /// Must be significantly softer than the default 0.000001 from the character
    /// JSON so the XPBD solver can actually move the arm. 0.1 is a good start.
    pub angular_compliance: f32,
    /// Maximum angular velocity for the constraint rest angle, in radians per second.
    /// The rest angle chases the IK target at this rate, preventing sudden jumps
    /// that cause overshoot and body wobble.
    /// π rad/s = 180°/s — fast enough to be responsive, slow enough to prevent overshoot.
    pub max_angle_rate: f32,
}

impl Default for ArmController {
    fn default() -> Self {
        Self {
            target_blend: 0.3,
            convergence_threshold: 2.0,
            angular_compliance: 5e-8,
            max_angle_rate: std::f32::consts::PI,
        }
    }
}

#[derive(Clone, Copy)]
pub enum WhichArm {
    Left,
    Right,
}

#[derive(Event)]
pub struct ResetArmControlConstraintsEvent {
    pub arm: WhichArm,
    pub character: Entity,
}

#[derive(Component, Clone)]
pub struct CharacterController {
    pub move_vector: Vec2,
    pub point_vector: Vec2,
    pub spine_config: SpineConfig,
    pub arm_controller: ArmController,
}

impl Default for CharacterController {
    fn default() -> Self {
        Self {
            move_vector: Vec2::ZERO,
            point_vector: Vec2::ZERO,
            spine_config: SpineConfig::default(),
            arm_controller: ArmController::default(),
        }
    }
}
