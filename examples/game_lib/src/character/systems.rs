use bevy::{ecs::intern::Interned, prelude::*};
use bevy_vello::integrations::physics::{
    CharacterFrameForceEvent, CharacterPivotForceEvent, CharacterPivotVelocityEvent,
    VelloCharacterPhysicsRoot, VelloJoint, VelloParticle,
};
use vello_physics::{
    soft_body::{ExternalForce, ParticleInfo},
    Particle,
};

use crate::character::{CharacterController, ConnectivityRoot, SpineController, StringPool};

#[inline]
fn bevy_to_vello(point: Vec2) -> Vec2 {
    Vec2::new(point.x, -point.y)
}
#[inline]
pub fn cross(a: Vec2, b: Vec2) -> f32 {
    (a.x * b.y) - (a.y * b.x)
}
#[inline]
fn get_normal_of_b_away_from_a(b: &Vec2, cross_result: f32) -> Vec2 {
    if cross_result < 0.0 {
        // Clockwise winding: Rotate b 90° Clockwise to point further away
        // (x, y) -> (y, -x)
        Vec2::new(b.y, -b.x)
    } else if cross_result > 0.0 {
        // Counter-Clockwise winding: Rotate b 90° Counter-Clockwise to point further away
        // (x, y) -> (-y, x)
        Vec2::new(-b.y, b.x)
    } else {
        // Collinear: Vectors are parallel; any perpendicular works or return zero
        Vec2::new(-b.y, b.x)
    }
}

const WEIGHT_RATIO: f32 = 1.0;
const FRAME_TO_JOINT_PARTICLE_MASS_RATIO: f32 = 2.0;
//generate force to move whist to head direction point to vec direction
fn claculate_force(
    e_h: Entity,
    e_w: Entity,
    p_h: &VelloParticle,
    p_w: &VelloParticle,
    vec: Vec2,
) -> (Vec<CharacterPivotForceEvent>, CharacterFrameForceEvent) {
    let mut result = vec![];
    let dir = bevy_to_vello(vec);
    let length = dir.length();
    let dir = dir / length;
    let pos_h = p_h.particle.pos;
    let pos_w = p_w.particle.pos;
    let delta = (pos_h - pos_w).normalize();
    let proj = delta.dot(dir);
    let (d_h, d_w) = if proj < 0.0 {
        let normal = get_normal_of_b_away_from_a(&delta, cross(dir, delta)) * length;
        (-normal * WEIGHT_RATIO, normal)
    } else {
        let diff = delta - proj * dir;
        let d_h = (proj * dir - diff) * length;
        let d_w = (proj * dir + diff) * length;
        (d_h * WEIGHT_RATIO, d_w)
    };
    let v_h = vec2(d_h.x, d_h.y);
    let v_w = vec2(d_w.x, d_w.y);
    result.push(CharacterPivotForceEvent {
        character_entity: p_h.root_entity,
        joint_entity: e_h,
        force: v_h,
    });
    result.push(CharacterPivotForceEvent {
        character_entity: p_h.root_entity,
        joint_entity: e_w,
        force: v_w,
    });
    let weights_h = p_h.get_weights();
    let weights_w = p_w.get_weights();
    let frame_force: Vec<Vec2> = weights_h
        .iter()
        .zip(weights_w.iter())
        .map(|(h, w)| FRAME_TO_JOINT_PARTICLE_MASS_RATIO * (h * v_h + w * v_w))
        .collect();

    let temp = CharacterFrameForceEvent {
        character_entity: p_h.root_entity,
        forces: frame_force,
    };
    (result, temp)
}

//generate force to move whist to head direction point to vec direction
fn claculate_velocity(
    e_h: Entity,
    e_w: Entity,
    p_h: &VelloParticle,
    p_w: &VelloParticle,
    vec: Vec2,
) -> Vec<CharacterPivotVelocityEvent> {
    let vec = vec * 5.0;
    let mut result = vec![];
    let dir = bevy_to_vello(vec);
    let length = dir.length();
    let dir = dir / length;
    let pos_h = p_h.particle.pos;
    let pos_w = p_w.particle.pos;
    let delta = (pos_h - pos_w).normalize();
    let proj = delta.dot(dir);
    let (d_h, d_w) = if proj < 0.0 {
        let normal = get_normal_of_b_away_from_a(&delta, cross(dir, delta)) * length;
        (-normal * WEIGHT_RATIO, normal)
    } else {
        let diff = delta - proj * dir;
        let d_h = (proj * dir - diff) * length;
        let d_w = (proj * dir + diff) * length;
        (d_h * WEIGHT_RATIO, d_w)
    };
    let v_h = vec2(d_h.x, d_h.y);
    let v_w = vec2(d_w.x, d_w.y);
    result.push(CharacterPivotVelocityEvent {
        character_entity: p_h.root_entity,
        joint_entity: e_h,
        velocity: v_h,
    });
    result.push(CharacterPivotVelocityEvent {
        character_entity: p_h.root_entity,
        joint_entity: e_w,
        velocity: v_w,
    });
    result
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

pub fn update_character_movement(
    c_q: Query<(
        &ConnectivityRoot,
        &CharacterController,
        &VelloCharacterPhysicsRoot,
    )>,
    j_q: Query<&VelloParticle>,
    string_pool: ResMut<StringPool>,
    mut force_events: EventWriter<CharacterPivotForceEvent>,
    mut frame_force_events: EventWriter<CharacterFrameForceEvent>,
    mut velocity_events: EventWriter<CharacterPivotVelocityEvent>,
) {
    let temp = ["PH", "P0", "P1", "P2", "P3"];
    let tokens: Vec<Interned<str>> = temp
        .iter()
        .map(|item| string_pool.pool.intern(&item))
        .collect();
    for (root, control, p_root) in &c_q {
        if control.move_vector.length_squared() <= 0.01 {
            continue;
        }
        let entities: Vec<Entity> = tokens
            .iter()
            .map(|item| root.parts.get(item).unwrap().clone())
            .collect();
        let particles: Vec<VelloParticle> = entities
            .iter()
            .map(|e| j_q.get(*e).unwrap().clone())
            .collect();
        let velocities = claculate_velocity_spine(
            &entities,
            &particles,
            control.move_vector,
            &control.spine_controller,
        );
        velocity_events.write_batch(velocities);
        // // 3. Resolve entities and components using a clean chain
        // let data = root
        //     .parts
        //     .get(&pivot_h)
        //     .and_then(|&e_h| root.parts.get(&pivot_w).map(|&e_w| (e_h, e_w)))
        //     .and_then(|(e_h, e_w)| {
        //         j_q.get_many([e_h, e_w])
        //             .ok()
        //             .map(|[j_h, j_w]| (e_h, e_w, j_h, j_w))
        //     });

        // 4. Use if-let to execute the logic only if all requirements are met
        // if let Some((e_h, e_w, j_h, j_w)) = data {
        //     let forces = claculate_force(e_h, e_w, &j_h, &j_w, control.move_vector);

        //     // 5. Send events directly (no need for .drain() unless reusing the Vec)
        //     force_events.write_batch(forces.0);
        //     frame_force_events.write(forces.1);
        // }
        // if let Some((e_h, e_w, j_h, j_w)) = data {
        //     let velocities = claculate_velocity(e_h, e_w, &j_h, &j_w, control.move_vector);

        //     // 5. Send events directly (no need for .drain() unless reusing the Vec)
        //     velocity_events.write_batch(velocities);
        // }
    }
}
