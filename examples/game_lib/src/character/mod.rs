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
    /// Forward/back linear impulse scaler (y-axis in Vello space).
    pub compliance: f32,
    pub damping: f32,
    /// Max angular velocity for the rest-angle blend (rad/s).
    pub steer_angle: f32,
}

impl Default for SpineConfig {
    fn default() -> Self {
        Self {
            compliance: 1e-7,
            damping: 0.1,
            steer_angle: 0.15 * PI,
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
