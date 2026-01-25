use bevy::prelude::*;
use bevy_vello::integrations::physics::{CharacterPivotForceEvent, VelloJoint, VelloParticle};
use nalgebra::Vector2;
use vello_physics::soft_body::{ExternalForce, ParticleInfo};

use crate::character::{CharacterController, ConnectivityRoot, StringPool};

#[inline]
fn bevy_to_vello(point: Vec2) -> Vector2<f32> {
    Vector2::new(point.x, -point.y)
}
#[inline]
pub fn cross(a: Vector2<f32>, b: Vector2<f32>) -> f32 {
    (a.x * b.y) - (a.y * b.x)
}
#[inline]
fn get_normal_of_b_away_from_a(b: &Vector2<f32>, cross_result: f32) -> Vector2<f32> {
    if cross_result < 0.0 {
        // Clockwise winding: Rotate b 90° Clockwise to point further away
        // (x, y) -> (y, -x)
        Vector2::new(b.y, -b.x)
    } else if cross_result > 0.0 {
        // Counter-Clockwise winding: Rotate b 90° Counter-Clockwise to point further away
        // (x, y) -> (-y, x)
        Vector2::new(-b.y, b.x)
    } else {
        // Collinear: Vectors are parallel; any perpendicular works or return zero
        Vector2::new(-b.y, b.x)
    }
}

const WEIGHT_RATIO: f32 = 1.0;
//generate force to move whist to head direction point to vec direction
fn claculate_force(
    e_h: Entity,
    e_w: Entity,
    p_h: &VelloParticle,
    p_w: &VelloParticle,
    vec: Vec2,
) -> Vec<CharacterPivotForceEvent> {
    let mut result = vec![];
    let dir = bevy_to_vello(vec);
    let length = dir.magnitude();
    let dir = dir / length;
    let pos_h = p_h.pos;
    let pos_w = p_w.pos;
    let delta = (pos_h - pos_w).normalize();
    let proj = delta.dot(&dir);
    let (d_h, d_w) = if proj < 0.0 {
        let normal = get_normal_of_b_away_from_a(&delta, cross(dir, delta)) * length;
        (-normal * WEIGHT_RATIO, normal)
    } else {
        let diff = delta - proj * dir;
        let d_h = (proj * dir - diff) * length;
        let d_w = (proj * dir + diff) * length;
        (d_h * WEIGHT_RATIO, d_w)
    };
    result.push(CharacterPivotForceEvent {
        joint_entity: e_h,
        force: vec2(d_h.x, d_h.y),
    });
    result.push(CharacterPivotForceEvent {
        joint_entity: e_w,
        force: vec2(d_w.x, d_w.y),
    });
    result
}

pub fn update_character_movement(
    c_q: Query<(&ConnectivityRoot, &CharacterController)>,
    j_q: Query<&VelloParticle>,
    string_pool: ResMut<StringPool>,
    mut force_events: EventWriter<CharacterPivotForceEvent>,
) {
    // 1. Intern strings once outside the loop
    let pivot_h = string_pool.pool.intern("P0");
    let pivot_w = string_pool.pool.intern("P3");

    for (root, control) in &c_q {
        // 2. Early exit for dead-zone check
        if control.move_vector.length_squared() <= 0.01 {
            continue;
        }

        // 3. Resolve entities and components using a clean chain
        let data = root
            .parts
            .get(&pivot_h)
            .and_then(|&e_h| root.parts.get(&pivot_w).map(|&e_w| (e_h, e_w)))
            .and_then(|(e_h, e_w)| {
                j_q.get_many([e_h, e_w])
                    .ok()
                    .map(|[j_h, j_w]| (e_h, e_w, j_h, j_w))
            });

        // 4. Use if-let to execute the logic only if all requirements are met
        if let Some((e_h, e_w, j_h, j_w)) = data {
            let forces = claculate_force(e_h, e_w, &j_h, &j_w, control.move_vector);

            // 5. Send events directly (no need for .drain() unless reusing the Vec)
            force_events.write_batch(forces);
        }
    }
}
