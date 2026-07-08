use bevy::{ecs::intern::Interned, prelude::*};
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, CharacterPivotVelocityEvent, VelloCharacterPhysicsRoot,
    VelloJoint, VelloParticle,
};
use vello_physics::utility::cos_sin;

use crate::character::{
    ArmController, CharacterController, ConnectivityRoot, SpineController, StringPool,
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

/// 2-bone IK for the right arm, driven entirely by angular constraints.
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
/// Angular-constraint-only approach. The XPBD solver handles all the physics —
/// it distributes forces correctly between elbow, wrist, shoulder, and body.
/// This avoids the non-physical velocity injection that fought the solver.
///
/// 1. **Midpoint target damping**: `virtual_target = lerp(prla, true_target, target_blend)`.
///    Each step is small and self-damping — corrections shrink as the wrist
///    approaches the target.
///
/// 2. **Analytic 2-bone IK**: Cosine law on the virtual target → desired angles.
///
/// 3. **Angular constraint events only**: Set the rest angles and a softer
///    compliance (`angular_compliance`, default 0.1 instead of 0.000001).
///    The XPBD constraint solver moves particles in physically correct ways,
///    naturally pushing the body in the opposite direction.
///
/// 4. **Convergence dead zone**: When the wrist is within `convergence_threshold`
///    of the true target, we stop emitting events.
///
/// 5. **Rotate-toward damping**: The rest angle is not snapped to the IK result
///    directly. Instead, it rotates toward the desired angle at a maximum rate
///    of `arm_controller.max_angle_rate` radians/second. This prevents sudden
///    jumps in the constraint rest condition that cause arm overshoot and body
///    wobble.
///
///    Small angular differences pass through at full speed (no slow creep like
///    lerp), while large jumps are capped to a fixed angular velocity.
fn calculate_arm_ik(
    target: Vec2,
    dt: f32,
    _particles_entity: &Vec<Entity>,
    particles: &Vec<VelloParticle>,
    joints_entity: &Vec<Entity>,
    joints: &Vec<VelloJoint>,
    arm_controller: &ArmController,
) -> Vec<CharacterAngularConstraintEvent> {
    const ARM_PARTICLE_COUNT: usize = 4; // P1, P12, P13, PRLA
    const ARM_JOINT_COUNT: usize = 2; // shoulder, elbow

    let mut angular_events = vec![];

    if particles.len() < ARM_PARTICLE_COUNT || joints_entity.len() < ARM_JOINT_COUNT {
        return angular_events;
    }

    let p1 = particles[0].particle.pos; // spine base
    let p12 = particles[1].particle.pos; // shoulder (IK root)
    let p13 = particles[2].particle.pos; // elbow
    let prla = particles[3].particle.pos; // wrist

    let true_target = bevy_to_vello(target);
    let dist_to_target = (prla - true_target).length();

    if dist_to_target <= arm_controller.convergence_threshold {
        return angular_events;
    }

    let upper_len = (p13 - p12).length();
    let forearm_len = (prla - p13).length();
    if upper_len <= f32::EPSILON || forearm_len <= f32::EPSILON {
        return angular_events;
    }

    let reach = upper_len + forearm_len;
    let mut target_pos = true_target;
    let root_to_target = target_pos - p12;
    let dist = root_to_target.length();
    if dist > reach {
        target_pos = p12 + root_to_target.normalize() * (reach - 0.001);
    }
    let to_target = target_pos - p12;
    let target_dist = to_target.length().max(f32::EPSILON);
    let target_dir = to_target / target_dist;

    // Law of cosines for shoulder offset
    let cos_shoulder = (upper_len * upper_len + target_dist * target_dist
        - forearm_len * forearm_len)
        / (2.0 * upper_len * target_dist);
    let cos_shoulder = cos_shoulder.clamp(-1.0, 1.0);
    let shoulder_offset = cos_shoulder.acos();

    // Choose bend direction for the shoulder (negative = one branch, positive = the other)
    let bend_sign = -1.0;

    // Rotate target direction by shoulder_offset * bend_sign to get the desired upper arm direction
    let total_shoulder_angle = shoulder_offset * bend_sign;
    let (sin_sh, cos_sh) = total_shoulder_angle.sin_cos();
    let desired_upper_dir = Vec2::new(
        target_dir.x * cos_sh - target_dir.y * sin_sh,
        target_dir.x * sin_sh + target_dir.y * cos_sh,
    );
    let desired_p13 = p12 + desired_upper_dir * upper_len;

    // Compute the frame's max angular delta from the rate
    let max_delta = arm_controller.max_angle_rate * dt;

    // ---- Shoulder constraint with rotate-toward damping ----
    let desired_shoulder_cs = cos_sin(p1, p12, desired_p13);

    // Compute the current actual shoulder angle from particle positions.
    // This is more reliable than reading the rest angle from the joint,
    // which may be stale or already blended from a previous frame.
    let current_shoulder_cs = cos_sin(p1, p12, p13);

    let (blended_cos, blended_sin) = rotate_toward(
        current_shoulder_cs.x,
        current_shoulder_cs.y,
        desired_shoulder_cs.x,
        desired_shoulder_cs.y,
        max_delta,
    );

    angular_events.push(CharacterAngularConstraintEvent {
        character_entity: particles[0].root_entity,
        joint_entity: joints_entity[0],
        config: vello_physics::AngularConstraintConfig {
            rest_cos: blended_cos,
            rest_sin: blended_sin,
            compliance: arm_controller.angular_compliance,
        },
    });

    // ---- Elbow constraint with rotate-toward damping ----
    // The elbow pivot is at desired_p13. We need the signed angle between
    //   desired_p13 -> p12   (toward the shoulder)
    //   desired_p13 -> target_pos (which is the desired forearm direction)
    // This exactly matches the geometry of the solved pose.
    let desired_elbow_cs = cos_sin(p12, desired_p13, target_pos);

    // Compute the current actual elbow angle from particle positions.
    let current_elbow_cs = cos_sin(p12, p13, prla);

    let (blended_cos, blended_sin) = rotate_toward(
        current_elbow_cs.x,
        current_elbow_cs.y,
        desired_elbow_cs.x,
        desired_elbow_cs.y,
        max_delta,
    );

    angular_events.push(CharacterAngularConstraintEvent {
        character_entity: particles[0].root_entity,
        joint_entity: joints_entity[1],
        config: vello_physics::AngularConstraintConfig {
            rest_cos: blended_cos,
            rest_sin: blended_sin,
            compliance: arm_controller.angular_compliance,
        },
    });

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
    let error = -cross / (1.0 + dot).max(1e-6);

    // Clamp max_delta to [0, PI) so tan(max_delta/2) is always valid and non-negative.
    // tan(θ) has asymptotes at θ = PI/2 + n*PI, and max_delta > PI would go the long way.
    let clamped_delta = max_delta.clamp(0.0, std::f32::consts::PI * 0.9999);

    // Max error in tangent half-angle space, matching the constraint's error metric
    let max_error = (clamped_delta / 2.0).tan();

    // Clamp the error
    let clamped_error = error.clamp(-max_error, max_error);

    // Reconstruct delta cos/sin from clamped tangent half-angle using:
    //   cos(θ) = (1 - tan²(θ/2)) / (1 + tan²(θ/2))
    //   sin(θ) = 2*tan(θ/2) / (1 + tan²(θ/2))
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
    let dt = time.delta_secs();

    //arm control
    let right_arm = ["P1", "P12", "P13", "PRLA", "P1_P12_P13", "P12_P13_PRLA"];
    let right_arm_tokens: Vec<Interned<str>> = right_arm
        .iter()
        .map(|item| string_pool.pool.intern(&item))
        .collect();

    let left_arm = ["P1", "P11", "P10", "PLLA", "P1_P11_P10", "P11_P10_PLLA"];
    let left_arm_tokens: Vec<Interned<str>> = left_arm
        .iter()
        .map(|item| string_pool.pool.intern(&item))
        .collect();

    for (root, control, _p_root) in &c_q {
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
            dt,
            &p_e,
            &particles,
            &j_e,
            &angular_constraints,
            &control.arm_controller,
        );
        angular_events.write_batch(arm_angular_events);

        let (p, j) = left_arm_tokens.split_at(4);
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
            dt,
            &p_e,
            &particles,
            &j_e,
            &angular_constraints,
            &control.arm_controller,
        );
        angular_events.write_batch(arm_angular_events);

        //body control
        let temp = ["PH", "P0", "P1", "P2", "P3"];
        let tokens: Vec<Interned<str>> = temp
            .iter()
            .map(|item| string_pool.pool.intern(&item))
            .collect();

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
