use bevy::prelude::*;
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, CharacterPivotPositionEvent, CharacterPivotVelocityEvent,
    VelloCharacterPhysicsRoot, VelloJoint, VelloParticle,
};
use vello_physics::{
    collision_response::PartcileShapeMatchingConfig,
    utility::{cos_sin, BalancedCoreFrame},
    ConnectionConstraint,
};

use crate::character::{
    ArmConfig, IkMode, LeftArmController, ResetArmControlConstraintsEvent, RightArmController,
    SpineConfig, SpineController,
};

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

/// Angular error dead zone (in radians) for forearm alignment.
/// When the angular error between the current and desired forearm direction
/// is ≤ this value, no IK corrections are emitted at all.  This prevents a
/// feedback loop where infinitesimal IK corrections cause the physics solver
/// to apply tiny forces each frame, which shift particles, which produce new
/// (equally tiny) IK corrections the next frame — a sustained high-frequency
/// oscillation that never fully damps out.
///
/// 0.003 rad ≈ 0.17° — small enough to be invisible, large enough to break
/// the IK↔physics feedback loop near convergence.
const FOREARM_ANGULAR_EPSILON: f32 = 0.003;

fn claculate_velocity_spine(
    entities: &Vec<Entity>,
    particles: &Vec<VelloParticle>,
    vec: Vec2,
    config: &SpineConfig,
) -> Vec<CharacterPivotVelocityEvent> {
    const SPINE_PARTICLE_COUNT: usize = 5;

    let mut result = vec![];

    if entities.len() < SPINE_PARTICLE_COUNT || particles.len() < SPINE_PARTICLE_COUNT {
        return result;
    }

    let dir = bevy_to_vello(vec);
    let length = dir.length();
    if length <= f32::EPSILON {
        return result;
    }

    // Desired movement direction in Vello coordinates (x-right, y-down).
    let desired_dir = dir / length;

    let positions: Vec<Vec2> = particles
        .iter()
        .take(SPINE_PARTICLE_COUNT)
        .map(|p| p.particle.pos)
        .collect();

    // Current spine direction: PH (head, index 0) minus P3 (tail, index 4).
    // spine_dir points from tail toward head.
    let spine_vec = positions[0] - positions[SPINE_PARTICLE_COUNT - 1];
    let spine_len = spine_vec.length();

    if spine_len <= f32::EPSILON {
        // Degenerate spine: all particles collapsed. Push them all forward.
        for i in 0..SPINE_PARTICLE_COUNT {
            result.push(CharacterPivotVelocityEvent {
                character_entity: particles[i].root_entity,
                joint_entity: entities[i],
                velocity: desired_dir * length * config.velocity_scale,
            });
        }
        return result;
    }

    let spine_dir = spine_vec / spine_len;

    // Unit tangent perpendicular to spine_dir (90° CCW in y-down).
    let tangent = Vec2::new(-spine_dir.y, spine_dir.x);

    // Signed rotation between spine_dir and desired_dir.
    let dot_val = spine_dir.dot(desired_dir);
    let cross_val = cross(spine_dir, desired_dir);
    let rotation_error = if dot_val < -0.99 {
        config.rotation_gain * 2.0
    } else {
        cross_val
    };

    // Alignment factor: 0 when spine faces away (dot < 0), ramps to 1 as
    // the spine aligns with desired_dir.
    let alignment = dot_val.clamp(0.0, 1.0);

    for i in 0..SPINE_PARTICLE_COUNT {
        // Normalized position along the spine: +1 at PH (head, index 0),
        // -1 at P3 (tail, index 4).
        let t = 1.0 - 2.0 * (i as f32 / (SPINE_PARTICLE_COUNT - 1) as f32);

        // 1. Translational component: scaled by alignment.
        let translational = desired_dir * length * config.velocity_scale * alignment;

        // 2. Rotational component: tangential velocity creating pure torque.
        let pivot = tangent * t * rotation_error * config.rotation_gain * length;

        let target_velocity = translational + pivot;

        // 3. Smooth blend from current physics velocity toward target.
        let current_vel = particles[i].particle.velocity;
        let blended = current_vel + config.velocity_blending * (target_velocity - current_vel);

        // 4. Hard clamp to max speed.
        let speed = blended.length();
        let velocity = if speed > config.max_speed && speed > f32::EPSILON {
            blended / speed * config.max_speed
        } else {
            blended
        };

        result.push(CharacterPivotVelocityEvent {
            character_entity: particles[i].root_entity,
            joint_entity: entities[i],
            velocity,
        });
    }

    result
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
) -> (
    Vec<CharacterAngularConstraintEvent>,
    Vec<CharacterPivotPositionEvent>,
) {
    const ARM_PARTICLE_COUNT: usize = 4; // P1, P12, P13, PRLA
    const ARM_JOINT_COUNT: usize = 2; // shoulder, elbow

    let mut angular_events = vec![];
    let mut position_events = vec![];

    // Early exit if disabled or missing particles/joints
    if matches!(config.ik_mode, IkMode::Disabled)
        || particles.len() < ARM_PARTICLE_COUNT
        || joints_entity.len() < ARM_JOINT_COUNT
    {
        return (angular_events, position_events);
    }

    let p1 = particles[0].particle.pos; // spine base
    let p12 = particles[1].particle.pos; // shoulder (IK root)
    let p13 = particles[2].particle.pos; // elbow
    let prla = particles[3].particle.pos; // wrist

    let true_target = bevy_to_vello(target);
    let dist_to_target = (prla - true_target).length();

    if dist_to_target <= config.convergence_threshold {
        return (angular_events, position_events);
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
        return (angular_events, position_events);
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
            return (angular_events, position_events);
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

    // ---- Shape matching position constraints (local space, damped) ----
    // Helper: damp a local target direction using rotate_toward so the shape
    // matching target doesn't jump suddenly and fight the solver.
    let damp_local_target = |current_local: Vec2, pivot_local: Vec2, desired_local: Vec2| -> Vec2 {
        let current_offset = current_local - pivot_local;
        let desired_offset = desired_local - pivot_local;
        let current_len = current_offset.length();
        let desired_len = desired_offset.length();
        if current_len > f32::EPSILON && desired_len > f32::EPSILON {
            let current_dir = current_offset / current_len;
            let desired_dir = desired_offset / desired_len;
            let (blended_cos, blended_sin) = rotate_toward(
                current_dir.x,
                current_dir.y,
                desired_dir.x,
                desired_dir.y,
                max_delta,
            );
            let blended_dir = Vec2::new(blended_cos, blended_sin);
            pivot_local + blended_dir * desired_len
        } else {
            desired_local
        }
    };

    // Shoulder SM (particle index 1) — SKIP in elbow-only mode.
    // In elbow-only mode the shoulder is frozen, so emitting any SM target
    // (even snapped to current p13) would exert a lingering force that fights
    // the frozen-shoulder assumption.
    if !is_elbow_only {
        let mut shoulder_sm = particles[1].shape_matching;
        let desired_shoulder_local =
            damp_local_target(shoulder_sm.local_target, p12_local, desired_p13_local);
        shoulder_sm.local_target = desired_shoulder_local;
        shoulder_sm.compliance = config.shape_matching_compliance;
        shoulder_sm.damping = config.shape_matching_damping;
        position_events.push(CharacterPivotPositionEvent {
            character_entity,
            joint_entity: particles_entity[1],
            target: shoulder_sm,
        });
    }

    // Elbow SM (particle index 2) — SKIP in elbow-only mode.
    // The forearm position is driven solely by the elbow angular constraint.
    // Adding an SM target here creates a two-force conflict that fights the
    // angular constraint, producing oscillation between the two systems.
    if !is_elbow_only {
        let mut elbow_sm = particles[2].shape_matching;
        let desired_elbow_local =
            damp_local_target(elbow_sm.local_target, desired_p13_local, desired_prla_local);
        elbow_sm.local_target = desired_elbow_local;
        elbow_sm.compliance = config.shape_matching_compliance;
        elbow_sm.damping = config.shape_matching_damping;
        position_events.push(CharacterPivotPositionEvent {
            character_entity,
            joint_entity: particles_entity[2],
            target: elbow_sm,
        });
    }

    // Wrist SM (particle index 3) — always emit in Aim mode.
    // Uses the IK-computed desired_prla_local (which accounts for weapon offset)
    // as the direction target, NOT the raw local_target.  This ensures the wrist
    // SM guides the hand to the weapon-offset-corrected position.
    let mut wrist_sm = particles[3].shape_matching;
    let pos_e = if is_elbow_only {
        desired_p13_local // frozen elbow as pivot
    } else {
        particles[2].shape_matching.local_target // blended elbow SM target
    };
    let desired_wrist_local = damp_local_target(wrist_sm.local_target, pos_e, desired_prla_local);
    wrist_sm.local_target = desired_wrist_local;
    wrist_sm.compliance = config.shape_matching_compliance;
    wrist_sm.damping = config.shape_matching_damping;
    position_events.push(CharacterPivotPositionEvent {
        character_entity,
        joint_entity: particles_entity[3],
        target: wrist_sm,
    });

    (angular_events, position_events)
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
    mut velocity_events: EventWriter<CharacterPivotVelocityEvent>,
    mut angular_events: EventWriter<CharacterAngularConstraintEvent>,
    mut position_events: EventWriter<CharacterPivotPositionEvent>,
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

        let (arm_angular, arm_position) = calculate_arm_ik(
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
        position_events.write_batch(arm_position);
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

        let (arm_angular_events, arm_position_events) = calculate_arm_ik(
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
        position_events.write_batch(arm_position_events);
    }

    // ---- Spine ----
    for (spine, p_root) in &spine_q {
        if p_root.initial_frame_coordinates.is_none() {
            continue;
        }

        if spine.move_vector.length_squared() <= 0.01 {
            continue;
        }

        let entities: Vec<Entity> = spine.particles.to_vec();
        let particles: Vec<VelloParticle> = entities
            .iter()
            .map(|e| p_q.get(*e).unwrap().clone())
            .collect();
        let velocities =
            claculate_velocity_spine(&entities, &particles, spine.move_vector, &spine.config);
        velocity_events.write_batch(velocities);
    }
}

pub fn reset_arm_constraint_event(
    mut reader: EventReader<ResetArmControlConstraintsEvent>,
    query_control: Query<(&LeftArmController, &RightArmController)>,
    query_p: Query<&VelloParticle>,
    query_a: Query<&VelloJoint>,
    mut angular_writer: EventWriter<CharacterAngularConstraintEvent>,
    mut position_writer: EventWriter<CharacterPivotPositionEvent>,
) {
    let mut pos_events = vec![];
    let mut angular_events = vec![];
    for item in reader.read() {
        let (left, right) = query_control.get(item.character).unwrap();
        let (particles, angulars) = match item.arm {
            super::WhichArm::Left => (left.particles, left.joints),
            super::WhichArm::Right => (right.particles, right.joints),
        };
        particles.iter().for_each(|e| {
            let c = query_p.get(*e).unwrap();

            pos_events.push(CharacterPivotPositionEvent {
                character_entity: item.character,
                joint_entity: *e,
                target: PartcileShapeMatchingConfig {
                    local_target: c.shape_matching_init_local_pos.expect(
                        "please don't drop the weapon the same frame character being assembled",
                    ),
                    compliance: c.shape_matching_init.compliance,
                    damping: c.shape_matching_init.damping,
                },
            });
        });
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
    position_writer.write_batch(pos_events);
}
