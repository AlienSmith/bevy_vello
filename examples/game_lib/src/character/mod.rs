use std::f32::consts::PI;

use bevy::{
    ecs::intern::{Interned, Interner},
    platform::collections::{HashMap, HashSet},
    prelude::*,
};

mod ik;
mod leg_pd;
mod observers;
pub mod plugin;
mod systems;

// Re-exported so hosts can order the hip pin (or any leg-target writer) *before*
// the limb driver: `drive_hip_pin.before(update_character_movement)`.
pub use systems::update_character_movement;

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
    /// Responsiveness knob for the pose interpolation: fraction of the remaining
    /// gap to the desired pose that is closed per fixed tick, *before* the rate
    /// clamp (interpolate-then-clamp). This is the "Kp" of the spine's rest-angle
    /// / rest-centre chase. Shared with the limb controller
    /// ([`LimbConfig::rest_chase_alpha`]) so the whole driven rig responds to ONE
    /// knob. 0.5 reproduces the historical hardcoded behaviour; lower = softer /
    /// slower, higher = snappier. Expected in `(0, 1]`.
    pub rest_chase_alpha: f32,
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
            rest_chase_alpha: 0.5,
            bend_compliance: 1e-7,
        }
    }
}

/// Runtime kill-switch for the spine drive ([`systems::tick_spine_drive`]).
///
/// The hip-pin bridge (`lower_body::hip_pin`) owns the `P1/P2/P3` frame while it
/// is enabled; setting `SpineDriveEnabled(false)` makes the drive emit nothing
/// so the two actuators never fight over the same particles. Defaults to `true`
/// (drive on) so every existing scene is unchanged.
#[derive(Resource)]
pub struct SpineDriveEnabled(pub bool);

impl Default for SpineDriveEnabled {
    fn default() -> Self {
        Self(true)
    }
}

#[derive(Clone)]
pub struct LimbConfig {
    /// Fraction of distance to blend toward true target each frame.
    /// 0.3 = move virtual target 30% toward true target from current wrist.
    pub target_blend: f32,
    /// Distance threshold: when |wrist - true_target| < this, stop updating.
    pub convergence_threshold: f32,
    /// Compliance for angular constraints on arm joints, expressed at the
    /// reference bone length [`Self::compliance_ref_length`].
    ///
    /// The `AngularConstraint` solve divides its gradients by bone length
    /// (`∇C ~ 1/l`), so `wSum ~ w/l²` and the same compliance value behaves
    /// `l²`-softer on longer bones. The emitted compliance is therefore
    /// pre-scaled per joint by `(compliance_ref_length / bone_length)²` (see
    /// `calculate_limb_ik`), making this knob a length-independent stiffness:
    /// it means the same thing for every character scale / bone length.
    pub angular_compliance: f32,
    /// Reference bone length (px) at which [`Self::angular_compliance`] is
    /// calibrated. Joints emit `angular_compliance * (ref/l)²` so stiffness
    /// stays consistent across bone lengths. Tuning rule: if the arm feels too
    /// stiff on long bones / too soft on short ones, this constant is wrong —
    /// match it to the character's typical upper-arm length.
    pub compliance_ref_length: f32,
    /// Angular speed (rad/s) at which the ω-compliance schedule saturates to
    /// its softest. Each joint measures the ω of its own rotating bone
    /// (shoulder → upper arm, elbow → forearm) and softens its compliance as
    /// `s = min(|ω| / omega_ref, 1)` grows — the back-EMF analogy: a fast-
    /// spinning motor has less torque authority left, so the spring withholds
    /// correction power during fast swings instead of fighting/pumping them
    /// (which is what manufactures the overshoot ring). 0 or less disables
    /// the schedule (always stiff).
    pub omega_ref: f32,
    /// Maximum softening multiple at `|ω| >= omega_ref`: compliance is scaled
    /// by up to this factor while the joint is swinging fast, and returns to
    /// 1× (full [`Self::angular_compliance`] stiffness) at rest, so the aim
    /// line locks crisply. 1.0 disables the schedule. Start ~8; raise toward
    /// 30+ if fast swings still ring, lower if the arm feels floaty mid-swing.
    pub soft_scale: f32,
    /// Damping ratio `ζ` of the implicit spring-damper baked into the XPBD
    /// angular solver (see `AngularConstraint`). This is the ONLY damping knob:
    /// the solver auto-derives `Kd = 2·ζ·sqrt(Kp·m)` from the compliance
    /// (spring: `Kp = 1.0/compliance`) and the dynamically-varying generalized
    /// mass `m = 1/w_sum` (bone-length/angle dependent).
    ///
    /// - `ζ = 1.0` (default) → critical damping: fastest approach with no
    ///   overshoot — the target snaps and stops.
    /// - `ζ < 1.0` → underdamped: springy, bouncy (ringing on settle).
    /// - `ζ > 1.0` → overdamped: heavy, sluggish.
    /// - `ζ <= 0` → disables damping, reduces exactly to the legacy (undamped)
    ///   XPBD angular solve.
    pub zeta: f32,
    /// Responsiveness knob for the rest-angle chase (the "Kp" of the PD in
    /// target space). Fraction of the remaining angle to the desired rest angle
    /// that is closed per tick, *before* the `max_angle_rate` clamp
    /// (interpolate-then-clamp). MUST match [`SpineConfig::rest_chase_alpha`] —
    /// the same value is threaded to both so the spine and the limbs share ONE
    /// responsiveness knob. 0.5 reproduces the historical hardcoded behaviour.
    pub rest_chase_alpha: f32,
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

impl Default for LimbConfig {
    fn default() -> Self {
        Self {
            target_blend: 0.3,
            convergence_threshold: 2.0,
            angular_compliance: 5e-8,
            compliance_ref_length: 30.0,
            omega_ref: 20.0,
            soft_scale: 20.0,
            zeta: 1.0,
            rest_chase_alpha: 0.5,
            max_angle_rate: 2.0 * std::f32::consts::PI,
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
    pub config: LimbConfig,
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
    pub config: LimbConfig,
    /// Aim target in world space. Written by weapon system, AI, player input, etc.
    pub target: Vec2,
}

/// Right leg controller with pre-cached entity handles + per-frame input.
///
/// A leg is structurally identical to an arm — four particles in IK order
/// `[ref, pivot, mid, end]` and two angular joints — so it reuses
/// [`LimbConfig`] and the shared [`calculate_limb_ik`](super::systems::calculate_limb_ik)
/// driver. The mapping for V10 is:
///   `ref` = P2 (waist), `pivot` = P3 (hip), `mid` = P41 (knee), `end` = PRLL (foot)
/// with joints `[P2_P3_P41, P3_P41_PRLL]`.
///
/// Stage 1 drives legs in [`IkMode::Reach`] only, reusing the arm `LimbConfig`
/// defaults (legs and arms are near-equal bone length on V10).
#[derive(Component, Clone)]
pub struct RightLegController {
    pub particles: [Entity; 4],
    pub joints: [Entity; 2],
    pub config: LimbConfig,
    /// Foot target in world space. Written by the gait/AI layer.
    pub target: Vec2,
    /// Foot target velocity (world, y-up). Written by the resolver so the leg PD
    /// can close on a velocity-error term instead of pure damping.
    pub target_vel: Vec2,
    /// Hip target in world space (y-up). The resolver's hip position.
    pub hip_target: Vec2,
    /// Knee target in world space (y-up). The resolver's knee position.
    pub knee_target: Vec2,
    /// Hip target velocity (world, y-up).
    pub hip_vel: Vec2,
    /// Knee target velocity (world, y-up).
    pub knee_vel: Vec2,
}

/// Left leg controller with pre-cached entity handles + per-frame input.
/// Particles: [P2, P3, P40, PLLL]; Joints: [P2_P3_P40, P3_P40_PLLL].
#[derive(Component, Clone)]
pub struct LeftLegController {
    pub particles: [Entity; 4],
    pub joints: [Entity; 2],
    pub config: LimbConfig,
    /// Foot target in world space. Written by the gait/AI layer.
    pub target: Vec2,
    /// Foot target velocity (world, y-up). Written by the resolver so the leg PD
    /// can close on a velocity-error term instead of pure damping.
    pub target_vel: Vec2,
    /// Hip target in world space (y-up). The resolver's hip position.
    pub hip_target: Vec2,
    /// Knee target in world space (y-up). The resolver's knee position.
    pub knee_target: Vec2,
    /// Hip target velocity (world, y-up).
    pub hip_vel: Vec2,
    /// Knee target velocity (world, y-up).
    pub knee_vel: Vec2,
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
            max_angle_rate: 0.01 * std::f32::consts::PI,
        }
    }
}

#[derive(Clone, Copy)]
pub enum WhichLimb {
    Left,
    Right,
    LeftLeg,
    RightLeg,
}

#[derive(Event)]
pub struct ResetLimbControlConstraintsEvent {
    pub limb: WhichLimb,
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
