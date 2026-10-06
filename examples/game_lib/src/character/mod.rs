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
    /// Max centre speed in px/s; `max_pos_step = max_pos_speed * dt` is the
    /// per-frame position cap.
    pub max_pos_speed: f32,
    /// Max angular speed (rad/s) for each spine heading; `max_ang_step =
    /// max_ang_speed * dt` is the per-frame per-heading angular cap. The two
    /// headings independently carry angular velocity now (no more derived
    /// `linear_to_angle` estimate), so this is a first-class, directly tuned cap.
    pub max_ang_speed: f32,
    /// X-PBD compliance for the `P1_P2_P3` angular constraint. Re-rest every
    /// frame against the two-heading commanded pose so the position and angular
    /// constraints agree by construction (no fighting). Soft enough to let the
    /// spine bend against incoming knocks, stiff enough to hold the commanded
    /// bend.
    pub bend_compliance: f32,
}

impl Default for SpineConfig {
    fn default() -> Self {
        Self {
            compliance: 1e-7,
            damping: 0.01,
            max_pos_speed: 900.0,
            max_ang_speed: 10.0,
            bend_compliance: 1e-5,
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
    /// 5e-8 is empirically what lets the IK actually move the arm — far stiffer
    /// than the softbody default. Much softer (e.g. 0.1) and the solver cannot
    /// move the arm at all; stiffer and it overshoots (waggles).
    pub angular_compliance: f32,
    /// XPBD damping time constant (seconds) for the arm's angular constraints.
    /// Opposes joint angular velocity — the derivative (D) term of the PD pair.
    /// 0 = no damping.
    pub angular_damping: f32,
    /// Maximum angular velocity for the constraint rest angle, in radians per second.
    /// The rest angle chases the IK target at this rate, preventing sudden jumps
    /// that cause overshoot and body wobble.
    /// π rad/s = 180°/s — fast enough to be responsive, slow enough to prevent overshoot.
    pub max_angle_rate: f32,
    /// Angular error dead zone (radians) below which no constraint event is
    /// emitted for shoulder and forearm. Prevents the IK↔physics feedback loop.
    pub angular_dead_zone: f32,
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
            angular_damping: 0.005,
            max_angle_rate: std::f32::consts::PI,
            angular_dead_zone: 0.003,
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
}

/// The spine "virtual handle": the commanded pose the external position
/// constraints pull P1/P2/P3 toward.
///
/// The virtual pose is **one centre (P2) + two independent bone headings**
/// (`heading_upper` = P2→P1, `heading_lower` = P2→P3). Each DOF is per-frame
/// interpolated + capped toward a *latched* desired goal:
///
/// - `desired_center` / `desired_heading_upper` / `desired_heading_lower` are
///   the goal. While the player holds input, `desired_center` is derived from
///   `P2 + commanded_dir * command_reach` and `desired_heading_upper` from the
///   input direction heading; `desired_heading_lower` follows the upper heading
///   plus the *live* bend (so the lower bone keeps its current deflection while
///   steering). With no input both references are frozen (no snap-back).
/// - `center` / `heading_upper` / `heading_lower` are the virtual pose: the
///   per-frame interpolation toward the desired goal, each clamped by its own
///   `max_pos_step` / `max_ang_step`. This is exactly what the indicator
///   visual is drawn at AND what the external position constraints pull P1/P2/P3
///   toward (one shared target).
/// - `local_points` are the three P2-centred offsets (P1/P2/P3), in local Vello
///   (y-down) coords, derived once from the assembled particles.
/// - `character` is the root entity of the driven character (for resolving its
///   [`SpineController`] particles/joints directly).
#[derive(Component, Clone)]
pub struct SpineIndicator {
    /// Virtual upper heading (P2→P1, world/Vello radians), interpolated toward
    /// `desired_heading_upper`.
    pub heading_upper: f32,
    /// Virtual lower heading (P2→P3, world/Vello radians), interpolated toward
    /// `desired_heading_lower`.
    pub heading_lower: f32,
    /// Virtual centre (P2 world position, Vello y-down), interpolated toward
    /// `desired_center`.
    pub center: Vec2,
    /// Latched desired upper heading (P2→P1, world/Vello radians).
    pub desired_heading_upper: f32,
    /// Latched desired lower heading (P2→P3, world/Vello radians).
    pub desired_heading_lower: f32,
    /// Latched desired centre goal (P2 world position, Vello y-down).
    pub desired_center: Vec2,
    /// The three quad-particle offsets (P1/P2/P3), P2-centred, in Vello y-down coords.
    pub local_points: [Vec2; 3],
    /// Root entity of the driven character (for resolving its SpineController).
    pub character: Entity,
    /// Drive tuning for the external position constraints. Lives on the
    /// indicator so it can be edited at runtime (e.g. via the tuning UI)
    /// rather than baked in from the blueprint JSON (which needs a restart).
    pub config: SpineConfig,
    /// Unit-length movement direction commanded by the arrow keys (Vello y-down
    /// world coords). While a movement command is *active* the desired centre is
    /// re-anchored each fixed tick to `P2 + commanded_dir * command_reach`, so a
    /// held key keeps the goal a constant distance ahead of the spine (it never
    /// drifts away from or clamps down onto P2).
    pub commanded_dir: Vec2,
    /// Fixed distance ahead of the current P2 at which the desired target is
    /// held while a movement command is active (px in Vello space). The example
    /// derives this from its `MoveSpeed` resource (exposed as the "arrow move
    /// speed" slider) so the gap is user-tunable.
    pub command_reach: f32,
    /// `true` while a movement command is held. When it flips `false`, the last
    /// re-anchored `desired_center` is **frozen** so the virtual pose keeps
    /// decaying asymptotically toward it (no snap-back).
    pub command_active: bool,
}

impl Default for SpineIndicator {
    fn default() -> Self {
        Self {
            heading_upper: 0.0,
            heading_lower: 0.0,
            center: Vec2::ZERO,
            desired_heading_upper: 0.0,
            desired_heading_lower: 0.0,
            desired_center: Vec2::ZERO,
            local_points: [Vec2::ZERO; 3],
            character: Entity::PLACEHOLDER,
            config: SpineConfig::default(),
            commanded_dir: Vec2::X,
            command_reach: 40.0,
            command_active: false,
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
