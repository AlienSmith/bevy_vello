use bevy::{ecs::error::info, math::VectorSpace, prelude::*};
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, CharacterPivotImpulseEvent, VelloCharacterPhysicsRoot,
    VelloConstraintWorld, VelloJoint, VelloParticle,
};
use vello_physics::{
    utility::{cos_sin, BalancedCoreFrame},
    ConnectionConstraint,
};

use crate::character::{
    ArmConfig, IkMode, LeftArmController, ResetArmControlConstraintsEvent, RightArmController,
    SpineConfig, SpineController,
};

pub fn tick_spine_drive(
    time: Res<Time>,
    spine_q: Query<(&SpineController, &VelloCharacterPhysicsRoot)>,
    p_q: Query<&VelloParticle>,
    p_j: Query<&VelloJoint>,
    mut world: ResMut<VelloConstraintWorld>,
) {
    let dt = time.delta_secs();
    for (spine, p_root) in &spine_q {
        if p_root.initial_frame_coordinates.is_none() {
            continue;
        }
        let entities: Vec<Entity> = spine.particles.to_vec();
        let particles: Vec<VelloParticle> = entities
            .iter()
            .map(|e| p_q.get(*e).unwrap().clone())
            .collect();
        let angular_entities: Vec<Entity> = spine.angulars.to_vec();
        let angulars: Vec<VelloJoint> = angular_entities
            .iter()
            .map(|e| p_j.get(*e).unwrap().clone())
            .collect();
        // Same pure math as the legacy path, with rate-converted gains.
        let config = spine.config.clone();

        let events = calculate_spine_drive(
            &entities,
            &particles,
            &angular_entities,
            &angulars,
            &config,
            bevy_to_vello(spine.move_vector),
            dt,
        );

        events
            .0
            .iter()
            .for_each(|e| world.queue_character_external_force(e));
        events
            .1
            .iter()
            .for_each(|e| world.queue_character_angular_constraints(e));
    }
}

use super::ik::compute_ik_positions;

/// Convert from Bevy (y-up) to Vello physics (y-down) coordinates.
///
/// # Coordinate convention (Vello physics)
///
/// ```text
///   x-right, y-down
///   atan2(y, x): 0° → right, +90° → down, ±180° → left, -90° → up
///   CW-on-screen = positive atan2
///   sin_cos(): (sin θ, cos θ), Vec2::new(cos θ, sin θ) → direction at angle θ
/// ```
#[inline]
fn bevy_to_vello(point: Vec2) -> Vec2 {
    Vec2::new(point.x, -point.y)
}
#[inline]
pub fn cross(a: Vec2, b: Vec2) -> f32 {
    (a.x * b.y) - (a.y * b.x)
}

/// Curvature tangent at a spine particle: the normalized bisector of its two
/// adjacent bones.
///
/// For an interior particle the tangent is the average direction of the
/// incoming and outgoing bone:
///
/// ```text
///   tangent = normalize( normalize(Pi - P(i-1)) + normalize(P(i+1) - Pi) )
/// ```
///
/// This is smoother than using a single bone's direction (e.g. a straight
/// neck/head cresting over two joints keeps a well-defined tangent even when
/// the adjacent bones are not collinear).
///
/// At the spine endpoints only one bone exists, so callers should pass the
/// sole neighbor for the missing side (e.g. `tangent(P0, P0, P1)` for the
/// first particle) — the zero-length side is skipped and the single bone's
/// direction is returned. Returns `Vec2::ZERO` if both sides are degenerate.
#[inline]
pub fn spine_tangent(prev: Vec2, current: Vec2, next: Vec2) -> Vec2 {
    let d1 = current - prev;
    let d2 = next - current;

    let len1 = d1.length();
    let len2 = d2.length();

    let u1 = if len1 > f32::EPSILON {
        d1 / len1
    } else {
        Vec2::ZERO
    };
    let u2 = if len2 > f32::EPSILON {
        d2 / len2
    } else {
        Vec2::ZERO
    };

    let bisector = u1 + u2;
    let len_b = bisector.length();
    if len_b > f32::EPSILON {
        bisector / len_b
    } else {
        Vec2::ZERO
    }
}

/// Per-particle curvature tangents along the spine.
///
/// Interior particles use the bisector of their two adjacent bones. The two
/// end particles have only one bone, so we synthesise an imaginary neighbour
/// anchored on the tangent of the closest interior particle: the head gets a
/// virtual particle behind it (`pos - t_inner`), the tail one beyond it
/// (`pos + t_inner`). This keeps the end tangents well-defined and continuous
/// with the body instead of snapping to a single bone's axis.
fn compute_spine_tangents(particles: &[VelloParticle]) -> Vec<Vec2> {
    let n = particles.len();
    let mut tangents = vec![Vec2::ZERO; n];
    if n < 2 {
        return tangents;
    }

    let pos = |i: usize| particles[i].particle.pos;

    // Interior bisector tangents.
    for i in 1..n - 1 {
        tangents[i] = spine_tangent(pos(i - 1), pos(i), pos(i + 1));
    }

    // Head: imaginary particle reflected from the first interior tangent.
    let inner_head = tangents[1];
    if inner_head != Vec2::ZERO {
        tangents[0] = spine_tangent(pos(0) - inner_head, pos(0), pos(1));
    }

    // Tail: imaginary particle reflected from the last interior tangent.
    let inner_tail = tangents[n - 2];
    if inner_tail != Vec2::ZERO {
        tangents[n - 1] = spine_tangent(pos(n - 2), pos(n - 1), pos(n - 1) + inner_tail);
    }

    tangents
}

fn remap_spine_control(input: Vec2, heading: Vec2) -> Vec2 {
    let move_axis = input.y;
    let steer_axis = input.x.clamp(-1.0, 1.0);
    Vec2::new(move_axis, steer_axis)
}

const FOREARM_ANGULAR_EPSILON: f32 = 0.003;
const SIGN: [f32; 5] = [-1.0, -1.0, 0.0, 1.0, 1.0];
fn calculate_spine_drive(
    entities: &Vec<Entity>,
    particles: &Vec<VelloParticle>,
    angular_entities: &Vec<Entity>,
    angulars: &Vec<VelloJoint>,
    config: &SpineConfig,
    move_vector_vello: Vec2,
    dt: f32,
) -> (
    Vec<CharacterPivotImpulseEvent>,
    Vec<CharacterAngularConstraintEvent>,
) {
    let mut angular_events = vec![];
    let mut impulse_events = vec![];
    let control = remap_spine_control(move_vector_vello, Vec2::ZERO);
    // let tangent_impulse_magnitude = control.x * dt * config.tangent_impulse_scaler;
    // let normal_impulse_magnituide = control.y * dt * config.normal_impulse_scaler;
    // let tangents = compute_spine_tangents(particles);

    // for i in 0..particles.len() {
    //     let sign = SIGN[i];
    //     let e = entities[i];
    //     let p = &particles[i];
    //     let tangent = tangents[i];
    //     let normal = Vec2::new(-tangent.y, tangent.x);
    //     let tangent_impulse = tangent * tangent_impulse_magnitude / p.particle.inv_mass;
    //     let normal_impulse = normal * sign * normal_impulse_magnituide / p.particle.inv_mass;
    //     let impulse = tangent_impulse + normal_impulse;
    //     info!("{}", impulse);
    //     impulse_events.push(CharacterPivotImpulseEvent {
    //         character_entity: p.root_entity,
    //         joint_entity: e,
    //         impulse: tangent_impulse + normal_impulse,
    //     })
    // }

    let steer_angle = control.y * config.steer_angle;
    let target = Vec2::new(steer_angle.cos(), steer_angle.sin());

    for (i, joint_entity) in angular_entities.iter().enumerate() {
        let compliance = match angulars[i].init_config {
            vello_physics::ConnectionConstraintInitConfig::Angular(_, _, _, c) => c,
            _ => 0.0,
        };

        angular_events.push(CharacterAngularConstraintEvent {
            character_entity: particles[0].root_entity,
            joint_entity: *joint_entity,
            config: vello_physics::AngularConstraintConfig {
                rest_cos: target.x,
                rest_sin: target.y,
                compliance,
            },
        });
    }

    (impulse_events, angular_events)
}

/// 2-bone IK for the arm, driven by angular constraints + shape matching in local space.
///
/// # Arm topology
/// ```text
/// P1 (spine) --[distance]--> P12 (shoulder) --[distance]--> P13 (elbow) --[distance]--> PRLA (wrist)
/// ```
///
/// Angular joints (pivot in bold):
/// - `P1_P12_P13`   → shoulder angle at **P12** between P1→P12 and P12→P13
/// - `P12_P13_PRLA` → elbow angle at **P13** between P12→P13 and P13→PRLA
///
/// # Strategy
///
/// Combined angular + shape-matching approach. The IK solve runs entirely in the
/// character's local frame (via `blend_core`), so both constraint types operate
/// in the same coordinate space and don't fight each other.
///
/// Two IK modes are supported, selected via `config.ik_mode`:
///
/// - **Reach** (default): Law-of-cosines position IK. Places the wrist at the target
///   position. Good for grabbing objects, melee attacks.
///
/// - **Aim**: Least-action forearm alignment IK. Constrains the forearm direction to
///   point at the target. The weapon offset angle rotates the aim direction before
///   solving, so a weapon attached at a fixed angle to the forearm still points at
///   the target. The arm never goes fully straight even for out-of-range targets.
///
/// - **Disabled**: No events emitted. Use for death, ragdoll, cutscenes.
///
/// In both modes, the IK runs in local space and uses `rotate_toward` damping on
/// the angular constraint rest angles to prevent sudden jumps that cause body wobble.
fn calculate_arm_ik(
    target: Vec2,
    dt: f32,
    particles_entity: &Vec<Entity>,
    particles: &Vec<VelloParticle>,
    joints_entity: &Vec<Entity>,
    _joints: &Vec<VelloJoint>,
    config: &ArmConfig,
    blend_core: &BalancedCoreFrame,
) -> Vec<CharacterAngularConstraintEvent> {
    const ARM_PARTICLE_COUNT: usize = 4; // P1, P12, P13, PRLA
    const ARM_JOINT_COUNT: usize = 2; // shoulder, elbow

    let mut angular_events = vec![];

    // Early exit if disabled or missing particles/joints
    if matches!(config.ik_mode, IkMode::Disabled)
        || particles.len() < ARM_PARTICLE_COUNT
        || joints_entity.len() < ARM_JOINT_COUNT
    {
        return angular_events;
    }

    let p1 = particles[0].particle.pos; // spine base
    let p12 = particles[1].particle.pos; // shoulder (IK root)
    let p13 = particles[2].particle.pos; // elbow
    let prla = particles[3].particle.pos; // wrist

    let true_target = bevy_to_vello(target);
    let dist_to_target = (prla - true_target).length();

    if dist_to_target <= config.convergence_threshold {
        return angular_events;
    }

    // Convert everything to local space so angular + shape matching agree
    let local_target = blend_core.world_to_local(true_target);
    let p1_local = blend_core.world_to_local(p1);
    let p12_local = blend_core.world_to_local(p12);
    let p13_local = blend_core.world_to_local(p13);
    let prla_local = blend_core.world_to_local(prla);

    let upper_len = (p13_local - p12_local).length();
    let forearm_len = (prla_local - p13_local).length();
    if upper_len <= f32::EPSILON || forearm_len <= f32::EPSILON {
        return angular_events;
    }

    // Compute the frame's max angular delta from the rate
    let max_delta = config.max_angle_rate * dt;
    let character_entity = particles[0].root_entity;

    // Compute IK positions directly each frame.  The FOREARM_ANGULAR_EPSILON dead
    // zone below prevents the feedback loop between IK and physics — the cache was
    // previously needed to avoid that same loop, but the dead zone is a cleaner
    // solution that doesn't hide behind a cache abstraction.
    let (desired_p13_local, desired_prla_local) = compute_ik_positions(
        &config,
        local_target,
        p12_local,
        p13_local,
        prla_local,
        upper_len,
        forearm_len,
    );

    // ---- Determine if this is elbow-only mode ----
    // In elbow-only mode the shoulder is frozen, so desired_p13 == current p13.
    let is_elbow_only = (desired_p13_local - p13_local).length_squared() <= f32::EPSILON;

    // ---- Dead zone: skip ALL events when forearm angular error is trivially small ----
    // Without this dead zone, infinitesimal IK corrections (< 0.003 rad) still emit
    // events each frame.  These cause the physics solver to apply tiny forces, which
    // shift particles, which produce new (equally tiny) corrections next frame — a
    // sustained high-frequency oscillation that never damps out.
    {
        let current_forearm_dir = (prla_local - p13_local).normalize_or_zero();
        let desired_forearm_dir = (desired_prla_local - desired_p13_local).normalize_or_zero();
        let cos_err = current_forearm_dir
            .dot(desired_forearm_dir)
            .clamp(-1.0, 1.0);
        let angle_err = cos_err.acos();
        if angle_err <= FOREARM_ANGULAR_EPSILON {
            return angular_events;
        }
    }

    // ---- Shoulder angular constraint (P1_P12_P13) — SKIP in elbow-only mode ----
    if !is_elbow_only {
        let desired_shoulder_cs = cos_sin(p1_local, p12_local, desired_p13_local);
        let current_shoulder_cs = cos_sin(p1_local, p12_local, p13_local);

        let (blended_cos, blended_sin) = rotate_toward(
            current_shoulder_cs.x,
            current_shoulder_cs.y,
            desired_shoulder_cs.x,
            desired_shoulder_cs.y,
            max_delta,
        );

        angular_events.push(CharacterAngularConstraintEvent {
            character_entity,
            joint_entity: joints_entity[0],
            config: vello_physics::AngularConstraintConfig {
                rest_cos: blended_cos,
                rest_sin: blended_sin,
                compliance: config.angular_compliance,
            },
        });
    }

    // ---- Elbow angular constraint (P12_P13_PRLA) — always emit in Aim mode ----
    {
        let desired_elbow_cs = cos_sin(p12_local, desired_p13_local, desired_prla_local);
        let current_elbow_cs = cos_sin(p12_local, p13_local, prla_local);

        let (blended_cos, blended_sin) = rotate_toward(
            current_elbow_cs.x,
            current_elbow_cs.y,
            desired_elbow_cs.x,
            desired_elbow_cs.y,
            max_delta,
        );

        angular_events.push(CharacterAngularConstraintEvent {
            character_entity,
            joint_entity: joints_entity[1],
            config: vello_physics::AngularConstraintConfig {
                rest_cos: blended_cos,
                rest_sin: blended_sin,
                compliance: config.angular_compliance,
            },
        });
    }

    angular_events
}

/// Rotate unit vector (cos_a, sin_a) toward (cos_b, sin_b) by at most `max_delta` radians.
/// Returns the new (cos, sin) after the rotation.
///
/// Uses the tangent half-angle error metric, matching the angular constraint solver's
/// own error function (`-delta_sin / (1 + delta_cos)`). This ensures the clamped result
/// is compatible with how the constraint interprets the rest angle, avoiding feedback
/// mismatches that cause wobble or sluggish response.
///
/// `max_delta` is clamped to `[0, PI)` internally to keep `tan(max_delta/2)` well-defined
/// (tan has asymptotes at odd multiples of PI/2). An angular change larger than PI radians
/// would go the long way around the circle anyway.
fn rotate_toward(cos_a: f32, sin_a: f32, cos_b: f32, sin_b: f32, max_delta: f32) -> (f32, f32) {
    // sin(θ_b - θ_a) = sin_b*cos_a - cos_b*sin_a = -(sin_a*cos_b - cos_a*sin_b) = -cross
    // cos(θ_b - θ_a) = cos_b*cos_a + sin_b*sin_a = dot
    let cross = sin_a * cos_b - cos_a * sin_b; // sin(θ_a - θ_b)
    let dot = cos_a * cos_b + sin_a * sin_b; // cos(θ_b - θ_a)

    // Tangent half-angle: tan((θ_b - θ_a)/2) = sin(θ_b - θ_a) / (1 + cos(θ_b - θ_a))
    // sin(θ_b - θ_a) = -cross
    let error = -cross / (1.0 + dot);
    let max_error = (max_delta * 0.5).tan();
    // Soft saturation instead of a hard clamp: `tanh` keeps large errors near the
    // max rate (so a moving target is tracked immediately) but smoothly tapers the
    // step as the error shrinks, removing the square-wave feel of the previous hard
    // clamp. Equivalent energy behaviour, just a smooth velocity profile.
    let clamped_error = max_error * (error / max_error).tanh();

    // Convert back: cos(Δ) = (1 - t²) / (1 + t²), sin(Δ) = 2t / (1 + t²)
    let error_sq = clamped_error * clamped_error;
    let denom = 1.0 + error_sq;
    let new_delta_cos = (1.0 - error_sq) / denom;
    let new_delta_sin = 2.0 * clamped_error / denom;

    // Rotate current by the clamped delta: rest = current + clamped_delta
    // cos(θ_a + Δ) = cos_a * cos(Δ) - sin_a * sin(Δ)
    // sin(θ_a + Δ) = sin_a * cos(Δ) + cos_a * sin(Δ)
    let new_cos = cos_a * new_delta_cos - sin_a * new_delta_sin;
    let new_sin = sin_a * new_delta_cos + cos_a * new_delta_sin;

    (new_cos, new_sin)
}

pub fn update_character_movement(
    time: Res<Time>,
    spine_q: Query<(&SpineController, &VelloCharacterPhysicsRoot)>,
    mut right_arm_q: Query<(&mut RightArmController, &VelloCharacterPhysicsRoot)>,
    mut left_arm_q: Query<(&mut LeftArmController, &VelloCharacterPhysicsRoot)>,
    p_q: Query<&VelloParticle>,
    j_q: Query<&VelloJoint>,
    mut velocity_events: EventWriter<CharacterPivotImpulseEvent>,
    mut angular_events: EventWriter<CharacterAngularConstraintEvent>,
) {
    let dt = time.delta_secs();

    // ---- Right arm ----
    for (arm, p_root) in &mut right_arm_q {
        if p_root.initial_frame_coordinates.is_none() {
            continue;
        }

        let p_e: Vec<Entity> = arm.particles.to_vec();
        let j_e: Vec<Entity> = arm.joints.to_vec();
        let particles: Vec<VelloParticle> =
            p_e.iter().map(|e| p_q.get(*e).unwrap().clone()).collect();
        let angular_constraints: Vec<VelloJoint> =
            j_e.iter().map(|e| j_q.get(*e).unwrap().clone()).collect();

        let arm_angular = calculate_arm_ik(
            arm.target,
            dt,
            &p_e,
            &particles,
            &j_e,
            &angular_constraints,
            &arm.config,
            &p_root.frame_coordinates,
        );

        angular_events.write_batch(arm_angular);
    }

    // ---- Left arm ----
    for (arm, p_root) in &mut left_arm_q {
        if p_root.initial_frame_coordinates.is_none() {
            continue;
        }

        let p_e: Vec<Entity> = arm.particles.to_vec();
        let j_e: Vec<Entity> = arm.joints.to_vec();
        let particles: Vec<VelloParticle> =
            p_e.iter().map(|e| p_q.get(*e).unwrap().clone()).collect();
        let angular_constraints: Vec<VelloJoint> =
            j_e.iter().map(|e| j_q.get(*e).unwrap().clone()).collect();

        let arm_angular_events = calculate_arm_ik(
            arm.target,
            dt,
            &p_e,
            &particles,
            &j_e,
            &angular_constraints,
            &arm.config,
            &p_root.frame_coordinates,
        );

        angular_events.write_batch(arm_angular_events);
    }
}

pub fn reset_arm_constraint_event(
    mut reader: EventReader<ResetArmControlConstraintsEvent>,
    query_control: Query<(&LeftArmController, &RightArmController)>,
    query_a: Query<&VelloJoint>,
    mut angular_writer: EventWriter<CharacterAngularConstraintEvent>,
) {
    let mut angular_events = vec![];
    for item in reader.read() {
        let (left, right) = query_control.get(item.character).unwrap();
        let angulars = match item.arm {
            super::WhichArm::Left => left.joints,
            super::WhichArm::Right => right.joints,
        };
        angulars.iter().for_each(|e| {
            let j = query_a.get(*e).unwrap();
            if let ConnectionConstraint::Angular(config) = j.init_constrats.unwrap() {
                angular_events.push(CharacterAngularConstraintEvent {
                    character_entity: item.character,
                    joint_entity: *e,
                    config,
                });
            }
        });
    }
    angular_writer.write_batch(angular_events);
}
