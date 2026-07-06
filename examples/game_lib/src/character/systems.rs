use bevy::{ecs::intern::Interned, prelude::*};
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, CharacterPivotVelocityEvent, VelloCharacterPhysicsRoot,
    VelloJoint, VelloParticle,
};
use vello_physics::utility::cos_sin;

use crate::{
    character::{
        ArmController, CharacterController, ConnectivityRoot, SpineController, StringPool,
    },
    utility::nlerp_cos_sin,
};

#[inline]
fn bevy_to_vello(point: Vec2) -> Vec2 {
    Vec2::new(point.x, -point.y)
}
#[inline]
pub fn cross(a: Vec2, b: Vec2) -> f32 {
    (a.x * b.y) - (a.y * b.x)
}

fn claculate_velocity_spine(
    entities: &Vec<Entity>,
    particles: &Vec<VelloParticle>,
    vec: Vec2,
    config: &SpineController,
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

    // Unit tangent perpendicular to spine_dir (90° CCW).
    let tangent = Vec2::new(-spine_dir.y, spine_dir.x);

    // Signed rotation between spine_dir and desired_dir.
    // cross = sin(θ): correct direction sign, 0 at both 0° and 180°.
    // When anti-aligned (dot < -0.99, cross ≈ 0 deadlock), use a fixed
    // positive torque to break out; once moving, cross takes over.
    let dot_val = spine_dir.dot(desired_dir);
    let cross_val = cross(spine_dir, desired_dir);
    let rotation_error = if dot_val < -0.99 {
        config.rotation_gain * 2.0
    } else {
        cross_val
    };

    // Alignment factor: 0 when spine faces away (dot < 0), ramps to 1 as
    // the spine aligns with desired_dir. This gates translation so the
    // character rotates in place before moving significantly.
    let alignment = dot_val.clamp(0.0, 1.0);

    for i in 0..SPINE_PARTICLE_COUNT {
        // Normalized position along the spine: +1 at PH (head, index 0),
        // -1 at P3 (tail, index 4).
        let t = 1.0 - 2.0 * (i as f32 / (SPINE_PARTICLE_COUNT - 1) as f32);

        // 1. Translational component: scaled by alignment so poorly-aligned
        //    spines rotate in place rather than sliding.
        let translational = desired_dir * length * config.velocity_scale * alignment;

        // 2. Rotational component: tangential velocity creating pure torque.
        //    Head (t = +1) rotates toward desired_dir; tail (t = -1) opposite.
        let pivot = tangent * t * rotation_error * config.rotation_gain * length;

        let target_velocity = translational + pivot;

        // 3. Smooth blend: lerp from the particle's current physics velocity
        //    toward the computed target. This prevents sudden velocity jumps
        //    that cause wobble in the XPBD constraint chain.
        let current_vel = particles[i].particle.velocity;
        let blended = current_vel + config.velocity_blending * (target_velocity - current_vel);

        // 4. Hard clamp: enforce max speed to keep the character controllable
        //    even under extreme input or constraint feedback.
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

/// 2-bone IK for the right arm.
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
/// Strategy:
/// - The 2-bone IK chain is P12 (root) → P13 (elbow) → PRLA (end effector).
/// - P1 is the spine attachment point used for computing the shoulder rest angle.
/// - We compute desired shoulder and elbow angles from the target position,
///   then emit angular constraint events to drive the XPBD solver.
fn calculate_arm_ik(
    target: Vec2,
    particles: &Vec<VelloParticle>,
    joints_entity: &Vec<Entity>,
    joints: &Vec<VelloJoint>,
    _arm_controller: &ArmController,
) -> Vec<CharacterAngularConstraintEvent> {
    const ARM_PARTICLE_COUNT: usize = 4; // P1, P12, P13, PRLA
    const ARM_JOINT_COUNT: usize = 2; // P1_P12_P13, P12_P13_PRLA

    let mut angular_events = vec![];

    if particles.len() < ARM_PARTICLE_COUNT || joints.len() < ARM_JOINT_COUNT {
        return angular_events;
    }

    // Positions in Vello coordinate space (x-right, y-down).
    let p1 = particles[0].particle.pos; // spine base
    let p12 = particles[1].particle.pos; // shoulder (IK root / pivot)
    let p13 = particles[2].particle.pos; // elbow
    let prla = particles[3].particle.pos; // wrist (end effector)

    let target_vello = bevy_to_vello(target);

    // Segment lengths.
    let upper_arm_len = (p13 - p12).length(); // L1: shoulder → elbow
    let forearm_len = (prla - p13).length(); // L2: elbow → wrist

    if upper_arm_len <= f32::EPSILON || forearm_len <= f32::EPSILON {
        return angular_events;
    }

    // IK chain: P12 (root) → P13 (elbow) → PRLA (end effector)
    let root_to_target = target_vello - p12;
    let dist = root_to_target.length();

    if dist <= f32::EPSILON {
        return angular_events;
    }

    let root_to_target_dir = root_to_target / dist;

    // Reachable range.
    let reach = upper_arm_len + forearm_len;

    // Cosine law for elbow angle at P13.
    // When the target is out of reach, the arm points straight at the target
    // with the elbow fully extended (elbow_angle = π).
    let (desired_upper_arm_dir, _elbow_angle) = if dist >= reach {
        // Out of reach: upper arm points directly at target, elbow fully extended.
        // The desired P13 is at the end of the upper arm pointing toward target.
        (root_to_target_dir, std::f32::consts::PI)
    } else {
        // Within reach: use cosine law for 2-bone IK.
        let clamped_dist = dist.max(f32::EPSILON);

        let cos_elbow_ik = ((upper_arm_len * upper_arm_len) + (forearm_len * forearm_len)
            - (clamped_dist * clamped_dist))
            / (2.0 * upper_arm_len * forearm_len);
        let cos_elbow_ik = cos_elbow_ik.clamp(-1.0, 1.0);
        let elbow_angle_ik = f32::acos(cos_elbow_ik);

        // Shoulder angle offset (angle from root_to_target_dir to upper arm direction).
        let sin_elbow_ik = f32::sin(elbow_angle_ik);
        let shoulder_offset = f32::atan2(
            forearm_len * sin_elbow_ik,
            upper_arm_len + forearm_len * cos_elbow_ik,
        );

        // Bend direction: for the right arm the elbow bends "inward" (downward
        // in Vello Y-down coords, which is CCW relative to the shoulder→target
        // direction). bend_sign = -1.0 rotates CCW in Vello coordinates.
        let bend_sign = -1.0;

        // Desired upper arm direction (from P12 toward P13).
        let total_angle = shoulder_offset * bend_sign;
        let (sin_total, cos_total) = f32::sin_cos(total_angle);
        let desired_upper_arm_dir = Vec2::new(
            root_to_target_dir.x * cos_total - root_to_target_dir.y * sin_total,
            root_to_target_dir.x * sin_total + root_to_target_dir.y * cos_total,
        );

        (desired_upper_arm_dir, elbow_angle_ik)
    };

    // Desired P13 position (elbow).
    let desired_p13 = p12 + desired_upper_arm_dir * upper_arm_len;

    // ---- Shoulder angular constraint ----
    // The shoulder joint P1_P12_P13 measures the angle at P12.
    // cos_sin(p1, p12, desired_p13) gives the target (cos, sin) at P12.
    // Use nlerp with a small factor to smoothly move the rest angle toward
    // the IK target, preventing violent yanks from the stiff compliance.
    let shoulder_cs = cos_sin(p1, p12, desired_p13);
    if let vello_physics::ConnectionConstraint::Angular(config) = &joints[0].constraint {
        let (cos, sin) = nlerp_cos_sin(
            (config.rest_cos, config.rest_sin),
            (shoulder_cs.x, shoulder_cs.y),
            0.05,
        );
        angular_events.push(CharacterAngularConstraintEvent {
            character_entity: joints[0].root_entity,
            joint_entity: joints_entity[0],
            config: vello_physics::AngularConstraintConfig {
                rest_cos: cos,
                rest_sin: sin,
                compliance: config.compliance,
            },
        });
    }

    // ---- Elbow angular constraint ----
    // The elbow joint P12_P13_PRLA measures the angle at P13.
    // cos_sin(p12, desired_p13, target_vello) gives the target (cos, sin) at P13.
    let elbow_cs = cos_sin(p12, desired_p13, target_vello);
    if let vello_physics::ConnectionConstraint::Angular(config) = &joints[1].constraint {
        let (cos, sin) = nlerp_cos_sin(
            (config.rest_cos, config.rest_sin),
            (elbow_cs.x, elbow_cs.y),
            0.05,
        );
        angular_events.push(CharacterAngularConstraintEvent {
            character_entity: joints[1].root_entity,
            joint_entity: joints_entity[1],
            config: vello_physics::AngularConstraintConfig {
                rest_cos: cos,
                rest_sin: sin,
                compliance: config.compliance,
            },
        });
    }

    angular_events
}

pub fn update_character_movement(
    c_q: Query<(
        &ConnectivityRoot,
        &CharacterController,
        &VelloCharacterPhysicsRoot,
    )>,
    p_q: Query<&VelloParticle>,
    j_q: Query<&VelloJoint>,
    string_pool: ResMut<StringPool>,
    mut velocity_events: EventWriter<CharacterPivotVelocityEvent>,
    mut angular_events: EventWriter<CharacterAngularConstraintEvent>,
) {
    //body control
    let temp = ["PH", "P0", "P1", "P2", "P3"];
    let tokens: Vec<Interned<str>> = temp
        .iter()
        .map(|item| string_pool.pool.intern(&item))
        .collect();
    for (root, control, _p_root) in &c_q {
        //arm control
        let right_arm = ["P1", "P12", "P13", "PRLA", "P1_P12_P13", "P12_P13_PRLA"];
        let right_arm_tokens: Vec<Interned<str>> = right_arm
            .iter()
            .map(|item| string_pool.pool.intern(&item))
            .collect();
        let (p, j) = right_arm_tokens.split_at(4);
        let p_e: Vec<Entity> = p
            .iter()
            .map(|item| root.parts.get(item).unwrap().clone())
            .collect();
        let j_e: Vec<Entity> = j
            .iter()
            .map(|item| root.parts.get(item).unwrap().clone())
            .collect();
        let particles: Vec<VelloParticle> =
            p_e.iter().map(|e| p_q.get(*e).unwrap().clone()).collect();
        let angular_constraints: Vec<VelloJoint> =
            j_e.iter().map(|e| j_q.get(*e).unwrap().clone()).collect();

        let arm_angular_events = calculate_arm_ik(
            control.point_vector,
            &particles,
            &j_e,
            &angular_constraints,
            &control.arm_controller,
        );
        angular_events.write_batch(arm_angular_events);

        if control.move_vector.length_squared() <= 0.01 {
            continue;
        }
        let entities: Vec<Entity> = tokens
            .iter()
            .map(|item| root.parts.get(item).unwrap().clone())
            .collect();
        let particles: Vec<VelloParticle> = entities
            .iter()
            .map(|e| p_q.get(*e).unwrap().clone())
            .collect();
        let velocities = claculate_velocity_spine(
            &entities,
            &particles,
            control.move_vector,
            &control.spine_controller,
        );
        velocity_events.write_batch(velocities);
    }
}
