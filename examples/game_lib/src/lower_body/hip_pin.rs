//! Hip pin: step the Raibert resolver and advance `P1/P2/P3` by the hip's
//! per-tick displacement with three external position constraints.
//!
//! The target is the live particle position plus the resolver hip delta, a
//! centroid velocity servo toward the resolver hip velocity, and the
//! `support_share` fraction of gravity. There is no absolute pose term.
//!
//! The spine drive must stay disabled while the pin runs
//! ([`crate::character::SpineDriveEnabled`]).

use bevy::{prelude::*, time::Fixed};
use bevy_vello::collision::CollisionEventBatch;
use bevy_vello::integrations::physics::{
    CharacterExternalPositionConstraintEvent, VelloCharacterPhysicsRoot, VelloConstraintWorld,
    VelloParticle,
};
use bevy_vello::VelloCollider;
use leafwing_input_manager::prelude::ActionState;
use vello_physics::ExternalPositionConstraintConfig;

use crate::character::{LeftLegController, RightLegController, SpineController};
use crate::input_management::actions::PlayerAction;
use crate::lower_body::resolver::{step_lower_body, LowerBodyConfig, ResolverState, VerticalPhase};

/// Runtime knobs for the hip pin, editable from the tuning UI.
#[derive(Resource)]
pub struct HipPinParams {
    /// Raibert resolver config.
    pub config: LowerBodyConfig,
    /// Speed at which compliance softening saturates, px/s.
    pub v_ref: f32,
    /// Compliance multiplier at full speed; `1.0` disables softening.
    pub soft_scale: f32,
    /// Fraction of gravity cancelled while grounded (`0..=1`).
    pub support_share: f32,
    /// External position constraint compliance at rest (lower = stiffer).
    pub compliance: f32,
    /// External position constraint damping.
    pub damping: f32,
    /// Also write the resolver's foot poses into the leg controllers.
    pub hip_drives_legs: bool,
    /// Vertical offset of the whole pinned body in px, positive = down.
    pub hip_drop: f32,
}

impl HipPinParams {
    /// Read the shared harness/tuning environment: `TARGET_SPEED`, `LEGS`,
    /// `COMPLIANCE`, `DAMPING`, `HIP_DROP`, `SUPPORT_SHARE`.
    pub fn from_env() -> Self {
        let mut p = Self::default();
        p.config.target_speed = env_f32("TARGET_SPEED", p.config.target_speed);
        p.compliance = env_f32("COMPLIANCE", p.compliance);
        p.damping = env_f32("DAMPING", p.damping);
        p.hip_drop = env_f32("HIP_DROP", p.hip_drop);
        p.support_share = env_f32("SUPPORT_SHARE", p.support_share).clamp(0.0, 1.0);
        p.hip_drives_legs = !matches!(
            std::env::var("LEGS").ok().as_deref().map(str::trim),
            Some("off") | Some("0") | Some("false")
        );
        p
    }
}

/// Parse a float env var; malformed or missing ⇒ `default`.
fn env_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|s| s.trim().parse::<f32>().ok())
        .unwrap_or(default)
}

/// Ground-contact state gating the pin's vertical authority.
///
/// The pin may cancel gravity and command y only while supported; in flight
/// gravity is ballistic. Register [`update_ground_support`] before
/// [`drive_hip_pin`].
#[derive(Resource, Debug, Clone)]
pub struct GroundSupport {
    /// True when a character collider touches non-character geometry.
    pub grounded: bool,
    /// Number of near-vertical mixed contacts seen this frame.
    pub contacts: usize,
}

impl Default for GroundSupport {
    fn default() -> Self {
        // No collision pipeline ⇒ keep the kinematic behaviour.
        Self {
            grounded: true,
            contacts: 0,
        }
    }
}

/// Refresh [`GroundSupport`] from the previous tick's [`CollisionEventBatch`].
pub fn update_ground_support(
    batch: Option<Res<CollisionEventBatch>>,
    colliders: Query<&VelloCollider>,
    mut support: ResMut<GroundSupport>,
) {
    let Some(batch) = batch else {
        support.grounded = true;
        support.contacts = 0;
        return;
    };
    let mut contacts = 0usize;
    for entry in &batch.entries {
        let a_soft = colliders
            .get(entry.event.entity_a)
            .map(|c| c.is_soft_body())
            .unwrap_or(false);
        let b_soft = colliders
            .get(entry.event.entity_b)
            .map(|c| c.is_soft_body())
            .unwrap_or(false);
        if a_soft == b_soft {
            continue;
        }
        let normal = if a_soft {
            entry.event.collision_normal_a
        } else {
            entry.event.collision_normal_b
        };
        if normal.y.abs() > 0.5 {
            contacts += 1;
        }
    }
    support.contacts = contacts;
    support.grounded = contacts > 0;
}

impl Default for HipPinParams {
    fn default() -> Self {
        Self {
            config: LowerBodyConfig::default(),
            v_ref: 300.0,
            soft_scale: 1.0,
            support_share: 0.5,
            compliance: 1e-7,
            damping: 0.01,
            hip_drives_legs: true,
            hip_drop: 0.0,
        }
    }
}

/// One leg's pose in the bevy y-up frame (the resolver's own frame).
#[derive(Debug, Clone, Copy)]
struct LegPoseSeed {
    hip: Vec2,
    knee: Vec2,
    foot: Vec2,
}

/// Sandbox↔leg bridge captured once on the first leg-driving tick.
///
/// Only the resolver's per-segment rotation deltas are transferred onto the
/// live spawn pose, so the character's own bone lengths and rest geometry are
/// preserved and the leg PD sees no step at t=0.
#[derive(Debug, Clone, Copy)]
struct LegBridgeSeed {
    /// Live leg poses at seed: `[left, right]`.
    live: [LegPoseSeed; 2],
    /// Resolver leg poses at seed: `[left, right]`.
    resolver: [LegPoseSeed; 2],
}

/// Signed angle from `a` to `b`, in `(-π, π]`.
#[inline]
fn signed_angle(a: Vec2, b: Vec2) -> f32 {
    a.perp_dot(b).atan2(a.dot(b))
}

/// Rotate `v` by `angle` radians (bevy y-up).
#[inline]
fn rotate_vec(v: Vec2, angle: f32) -> Vec2 {
    let (s, c) = angle.sin_cos();
    Vec2::new(c * v.x - s * v.y, s * v.x + c * v.y)
}

/// Rotate the seed segment `live0` by the resolver segment's rotation from
/// `res0` to `res1`, preserving `live0`'s length.
#[inline]
fn transfer_segment(live0: Vec2, res0: Vec2, res1: Vec2) -> Vec2 {
    if res0.length_squared() < 1e-6 || res1.length_squared() < 1e-6 {
        return live0;
    }
    rotate_vec(live0, signed_angle(res0, res1))
}

/// Per-character hip pin, attached to the character root.
#[derive(Component)]
pub struct HipPin {
    /// Persistent Raibert resolver state.
    pub state: ResolverState,
    /// `P1/P2/P3` rest offsets relative to `P3`, in Vello (y-down) world space.
    pub rest_offsets: [Vec2; 3],
    /// Resolver hip at seed time (local y-up).
    pub seed_hip: Vec2,
    /// Live `P3` world (Vello y-down) position at seed time.
    pub anchor_p3: Vec2,
    /// Previous `world_delta`; the pin commands only its per-tick change.
    prev_world_delta: Vec2,
    /// Set once the rest offsets and anchor have been captured.
    pub ready: bool,
    /// Master enable; when false the pin emits nothing.
    pub enabled: bool,
    /// Rigid-frame attitude in radians.
    pub theta: f32,
    /// Whether a jump was held last step, for edge detection at fixed `dt`.
    jump_was_held: bool,
    /// Sandbox↔leg bridge seed, captured lazily.
    leg_seed: Option<LegBridgeSeed>,
    /// Resolver reference for `P3` in Vello world (y-down).
    pub last_ref_p3: Vec2,
    /// Realized vertical sag of `P3` against the reference, positive = below.
    pub last_sag_y: f32,
}

impl HipPin {
    /// Live hip→foot distance captured at the leg-bridge seed (bevy y-up).
    pub fn stance_height(&self) -> Option<f32> {
        self.leg_seed
            .map(|s| s.live[0].hip.y - s.live[0].foot.y)
    }

    pub fn new(config: &LowerBodyConfig) -> Self {
        Self {
            state: ResolverState::new(config),
            rest_offsets: [Vec2::ZERO; 3],
            seed_hip: Vec2::ZERO,
            anchor_p3: Vec2::ZERO,
            prev_world_delta: Vec2::ZERO,
            ready: false,
            enabled: true,
            theta: 0.0,
            jump_was_held: false,
            leg_seed: None,
            last_ref_p3: Vec2::ZERO,
            last_sag_y: 0.0,
        }
    }

    /// Drop the leg seed so the next leg-driving tick re-captures it from the
    /// live pose (toggle of `hip_drives_legs`, teleport, reset).
    pub fn reseat_leg_bridge(&mut self) {
        self.leg_seed = None;
    }
}

/// Rigid-frame targets: translate the anchor by `world_delta` and rotate each
/// rest offset by `theta`. Bone lengths are preserved.
fn rigid_frame_targets(
    anchor_p3: Vec2,
    rest_offsets: [Vec2; 3],
    world_delta: Vec2,
    theta: f32,
) -> [Vec2; 3] {
    let (sin, cos) = theta.sin_cos();
    rest_offsets.map(|o| {
        let rotated = Vec2::new(o.x * cos - o.y * sin, o.x * sin + o.y * cos);
        anchor_p3 + world_delta + rotated
    })
}

/// Advance the pinned frame by the Raibert hip delta plus a centroid
/// velocity servo.
///
/// `d = hip_delta + (v_hip − v_centroid)·dt + (1 − support_share)·g·dt²`
/// is applied to every particle, so the frame stays rigid and no absolute
/// pose term is used. `support_share` is the gravity fraction the pin cancels;
/// on a released vertical axis the particle keeps its free-flight y
/// (`v·dt + g·dt²`), which leaves gravity ballistic.
fn delta_frame_targets(
    states: [(Vec2, Vec2); 3],
    hip_delta: Vec2,
    v_hip: Vec2,
    gravity: Vec2,
    dt: f32,
    vertical: bool,
    support_share: f32,
) -> [Vec2; 3] {
    let v_c = (states[0].1 + states[1].1 + states[2].1) / 3.0;
    let g_eff = gravity * (1.0 - support_share.clamp(0.0, 1.0));
    let d = hip_delta + (v_hip - v_c) * dt + g_eff * (dt * dt);
    states.map(|(pos, vel)| {
        if vertical {
            pos + d
        } else {
            pos + Vec2::new(d.x, vel.y * dt + gravity.y * (dt * dt))
        }
    })
}

/// Speed-softened compliance: `base·(1 + s·(soft_scale−1))`, `s = |v|/v_ref`
/// clamped to 1. `soft_scale = 1` or `v_ref ≤ 0` returns `base`.
fn softened_compliance(base: f32, soft_scale: f32, v_ref: f32, speed: f32) -> f32 {
    let s = if v_ref > 0.0 {
        (speed / v_ref).min(1.0)
    } else {
        0.0
    };
    base * (1.0 + s * (soft_scale - 1.0))
}

/// Capture the `P3` world anchor and the `P1/P2` rest offsets once the
/// character is assembled.
fn try_seed(
    pin: &mut HipPin,
    spine: &SpineController,
    root: &VelloCharacterPhysicsRoot,
    hip_drop: f32,
    p_q: &Query<&VelloParticle>,
) {
    if pin.ready || root.initial_frame_coordinates.is_none() {
        return;
    }
    // spine.particles == [PH, P0, P1, P2, P3]; the pin drives the last three.
    let p1 = match p_q.get(spine.particles[2]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    let p2 = match p_q.get(spine.particles[3]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    let p3 = match p_q.get(spine.particles[4]) {
        Ok(p) => p.particle.pos,
        Err(_) => return,
    };
    pin.rest_offsets = [p1 - p3, p2 - p3, Vec2::ZERO];
    pin.anchor_p3 = p3;
    pin.seed_hip = pin.state.hip_pos;
    // Bias the first delta so the configured drop is applied exactly once.
    pin.prev_world_delta = Vec2::new(0.0, -hip_drop);
    pin.ready = true;
}

/// Step the resolver and queue the three stiff external position constraints
/// that pin `P1/P2/P3` to the commanded frame.
///
/// Run before `update_character_movement` (the legs read the targets) and
/// before the constraint world is stepped.
pub fn drive_hip_pin(
    time: Res<Time<Fixed>>,
    params: Res<HipPinParams>,
    ground_support: Option<Res<GroundSupport>>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<(&SpineController, &VelloCharacterPhysicsRoot)>,
    mut right_leg_q: Query<&mut RightLegController>,
    mut left_leg_q: Query<&mut LeftLegController>,
    p_q: Query<&VelloParticle>,
    action_q: Query<&ActionState<PlayerAction>>,
    mut world: ResMut<VelloConstraintWorld>,
    mut log_tick: Local<u64>,
) {
    let dt = time.delta_secs();

    for (entity, mut pin) in pin_q.iter_mut() {
        if !pin.enabled {
            continue;
        }
        let Ok((spine, root)) = spine_q.get(entity) else {
            continue;
        };
        try_seed(&mut pin, spine, root, params.hip_drop, &p_q);
        if !pin.ready {
            continue;
        }

        // Game input → resolver input. The jump is edge-triggered here because
        // a held key would re-arm the hop on every landing.
        let mut input_dir = Vec2::ZERO;
        if let Ok(action) = action_q.get(entity) {
            if action.pressed(&PlayerAction::MoveRight) {
                input_dir.x += 1.0;
            }
            if action.pressed(&PlayerAction::MoveLeft) {
                input_dir.x -= 1.0;
            }
            let jump_held = action.pressed(&PlayerAction::MoveUp);
            if jump_held && !pin.jump_was_held {
                input_dir.y += 1.0;
            }
            pin.jump_was_held = jump_held;
            if action.pressed(&PlayerAction::MoveDown) {
                input_dir.y -= 1.0;
            }
        }

        let (pose, _) = step_lower_body(input_dir, &params.config, &mut pin.state, dt);

        // Live P1/P2/P3 states, in the same order as `idx`.
        let idx = [2usize, 3, 4];
        let mut states = [(Vec2::ZERO, Vec2::ZERO); 3];
        let mut all_ready = true;
        for (k, i) in idx.iter().enumerate() {
            match p_q.get(spine.particles[*i]) {
                Ok(p) => states[k] = (p.particle.pos, p.particle.velocity),
                Err(_) => {
                    all_ready = false;
                    break;
                }
            }
        }
        if !all_ready {
            continue;
        }

        // Resolver hip (y-up) → Vello world (y-down). Only the delta from the
        // seed is used, so the resolver's absolute frame never leaks.
        let world_delta = Vec2::new(pose.hip.x - pin.seed_hip.x, -(pose.hip.y - pin.seed_hip.y));
        let hip_delta = world_delta - pin.prev_world_delta;
        pin.prev_world_delta = world_delta;
        let v_hip = Vec2::new(pin.state.hip_vel.x, -pin.state.hip_vel.y);

        let mut pose_targets =
            rigid_frame_targets(pin.anchor_p3, pin.rest_offsets, world_delta, pin.theta);
        if params.hip_drop != 0.0 {
            for t in pose_targets.iter_mut() {
                t.y += params.hip_drop;
            }
        }

        let g_vello = world.gravity();

        // Vertical authority: supported, or a ballistic resolver phase (where
        // tracking the reference preserves the jump).
        let grounded = ground_support.as_deref().map(|s| s.grounded).unwrap_or(true);
        let vertical_tracking = matches!(
            pin.state.vs.phase,
            VerticalPhase::Launch | VerticalPhase::Flight
        );
        let vertical_allowed = grounded || vertical_tracking;
        let support_share = if grounded {
            params.support_share
        } else {
            0.0
        };

        let targets = delta_frame_targets(
            states,
            hip_delta,
            v_hip,
            g_vello,
            dt,
            vertical_allowed,
            support_share,
        );

        // Exactly 3 position constraints; nothing angular.
        for (k, i) in idx.iter().enumerate() {
            let (pos, vel) = states[k];
            let pose_t = pose_targets[k];
            let target = targets[k];
            if *i == 4 {
                pin.last_ref_p3 = pose_t;
                pin.last_sag_y = pos.y - pose_t.y;
            }
            let compliance = softened_compliance(
                params.compliance,
                params.soft_scale,
                params.v_ref,
                vel.length(),
            );

            world.queue_character_one_time_external_position_constraint(
                &CharacterExternalPositionConstraintEvent {
                    character_entity: entity,
                    joint_entity: spine.particles[*i],
                    config: ExternalPositionConstraintConfig {
                        target,
                        compliance,
                        damping: params.damping,
                    },
                },
            );
        }

        // Opt-in telemetry (`PIN_LOG=1`).
        let pin_log = std::env::var("PIN_LOG")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);
        if pin_log {
            *log_tick += 1;
            if *log_tick % 90 == 0 {
                let p3 = p_q
                    .get(spine.particles[4])
                    .map(|p| p.particle.pos)
                    .unwrap_or(pin.last_ref_p3);
                info!(
                    "pin vertical={} grounded={} share={:.2} P3 actual=({:.1},{:.1}) ref=({:.1},{:.1}) sag_y={:.1}",
                    vertical_allowed,
                    grounded,
                    support_share,
                    p3.x,
                    p3.y,
                    pin.last_ref_p3.x,
                    pin.last_ref_p3.y,
                    pin.last_sag_y,
                );
            }
        }

        // Sandbox → legs: write the resolver poses into the leg controllers.
        // `update_character_movement` (ordered after) reads them for the IK.
        if params.hip_drives_legs {
            let (Ok(mut right_leg), Ok(mut left_leg)) =
                (right_leg_q.get_mut(entity), left_leg_q.get_mut(entity))
            else {
                continue;
            };
            // The pin owns P2/P3 while the legs are driven; drop them from the
            // leg PD's angular mass matrix.
            right_leg.config.anchor_chain = true;
            left_leg.config.anchor_chain = true;

            let vello_to_bevy = |p: &VelloParticle| Vec2::new(p.particle.pos.x, -p.particle.pos.y);
            let live_pose = |particles: [Entity; 4],
                             hip_fb: Vec2,
                             knee_fb: Vec2,
                             foot_fb: Vec2|
             -> LegPoseSeed {
                let get = |e: Entity, fallback: Vec2| {
                    p_q.get(e)
                        .map(&vello_to_bevy)
                        .unwrap_or(fallback)
                };
                LegPoseSeed {
                    hip: get(particles[1], hip_fb),
                    knee: get(particles[2], knee_fb),
                    foot: get(particles[3], foot_fb),
                }
            };
            let live_left = live_pose(
                left_leg.particles,
                left_leg.hip_target,
                left_leg.knee_target,
                left_leg.target,
            );
            let live_right = live_pose(
                right_leg.particles,
                right_leg.hip_target,
                right_leg.knee_target,
                right_leg.target,
            );
            let seed = *pin.leg_seed.get_or_insert(LegBridgeSeed {
                live: [live_left, live_right],
                resolver: [
                    LegPoseSeed {
                        hip: pose.hip,
                        knee: pose.left_knee,
                        foot: pose.left_foot,
                    },
                    LegPoseSeed {
                        hip: pose.hip,
                        knee: pose.right_knee,
                        foot: pose.right_foot,
                    },
                ],
            });

            // Follow the live P3 so impulses move the character.
            let actual_p3_bevy = Vec2::new(states[2].0.x, -states[2].0.y);
            let hip_t = actual_p3_bevy - Vec2::Y * params.hip_drop;
            let knee_l = hip_t
                + transfer_segment(
                    seed.live[0].knee - seed.live[0].hip,
                    seed.resolver[0].knee - seed.resolver[0].hip,
                    pose.left_knee - pose.hip,
                );
            let foot_l = knee_l
                + transfer_segment(
                    seed.live[0].foot - seed.live[0].knee,
                    seed.resolver[0].foot - seed.resolver[0].knee,
                    pose.left_foot - pose.left_knee,
                );
            let knee_r = hip_t
                + transfer_segment(
                    seed.live[1].knee - seed.live[1].hip,
                    seed.resolver[1].knee - seed.resolver[1].hip,
                    pose.right_knee - pose.hip,
                );
            let foot_r = knee_r
                + transfer_segment(
                    seed.live[1].foot - seed.live[1].knee,
                    seed.resolver[1].foot - seed.resolver[1].knee,
                    pose.right_foot - pose.right_knee,
                );

            left_leg.hip_target = hip_t;
            right_leg.hip_target = hip_t;
            left_leg.knee_target = knee_l;
            right_leg.knee_target = knee_r;
            left_leg.target = foot_l;
            right_leg.target = foot_r;
            // Joint-rate feed-forward for the leg PD (bevy y-up, px/s).
            left_leg.hip_vel = pose.hip_vel;
            left_leg.knee_vel = pose.left_knee_vel;
            left_leg.target_vel = pose.left_foot_vel;
            right_leg.hip_vel = pose.hip_vel;
            right_leg.knee_vel = pose.right_knee_vel;
            right_leg.target_vel = pose.right_foot_vel;
        } else {
            // No pin ⇒ the frame is free; use the full mass matrix.
            if let Ok(mut right_leg) = right_leg_q.get_mut(entity) {
                right_leg.config.anchor_chain = false;
            }
            if let Ok(mut left_leg) = left_leg_q.get_mut(entity) {
                left_leg.config.anchor_chain = false;
            }
        }
    }
}

/// Reset the pin to its resting stance and re-arm the seed (`R`).
pub fn reset_hip_pin(
    keys: Res<ButtonInput<KeyCode>>,
    params: Res<HipPinParams>,
    mut pin_q: Query<&mut HipPin>,
) {
    if !keys.just_pressed(KeyCode::KeyR) {
        return;
    }
    for mut pin in pin_q.iter_mut() {
        pin.state = ResolverState::new(&params.config);
        pin.ready = false;
        pin.leg_seed = None;
        pin.prev_world_delta = Vec2::ZERO;
    }
}
