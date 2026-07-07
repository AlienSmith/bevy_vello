use bevy::{
    ecs::intern::{Interned, Interner},
    platform::collections::{HashMap, HashSet},
    prelude::*,
};
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

#[derive(Clone)]
pub struct SpineController {
    /// How aggressively the spine rotates toward the desired direction.
    /// Higher values snap faster (e.g. 4.0–8.0 for aim-first).
    pub rotation_gain: f32,
    /// Forward velocity scale when aligned.
    pub velocity_scale: f32,
    /// Lerp factor blending current→target particle velocity per frame.
    /// 0.0 = freeze (no control response), 1.0 = instant (original behavior).
    pub velocity_blending: f32,
    /// Hard cap on particle velocity magnitude. Blended result is clamped
    /// to this limit before being applied.
    pub max_speed: f32,
}

impl Default for SpineController {
    fn default() -> Self {
        Self {
            rotation_gain: 24.0,
            velocity_scale: 10.0,
            velocity_blending: 0.5,
            max_speed: 600.0,
        }
    }
}

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
            angular_compliance: 1e-6,
            max_angle_rate: std::f32::consts::PI,
        }
    }
}

#[derive(Component, Clone)]
pub struct CharacterController {
    pub move_vector: Vec2,
    pub point_vector: Vec2,
    pub spine_controller: SpineController,
    pub arm_controller: ArmController,
}

impl Default for CharacterController {
    fn default() -> Self {
        Self {
            move_vector: Vec2::ZERO,
            point_vector: Vec2::ZERO,
            spine_controller: SpineController::default(),
            arm_controller: ArmController::default(),
        }
    }
}
