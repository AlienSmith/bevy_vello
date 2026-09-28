//! Spine position-constraint control.
//!
//! Replaces the impulse-based spine drive with *external position constraints*:
//! each frame it computes three world-space target points from the user's
//! control indicator (its translation + rotation applied to the local P1/P2/P3
//! offsets), then queues a damped position constraint on each of the quad spine
//! particles P1/P2/P3 to pull them toward those targets.
//!
//! This is the "virtual handle" approach: instead of applying an impulse (which
//! inevitably overshoots), the solver softly repositions the particles toward
//! the indicator every fixed step with a configured compliance + damping.

use crate::spine_indicator::SpineIndicator;
use bevy::math::Vec2;
use bevy::prelude::*;
use bevy_vello::integrations::physics::{
    CharacterExternalPositionConstraintEvent, VelloCharacterPhysicsRoot, VelloConstraintWorld,
    VelloParticle,
};
use game_lib::{CharacterRoot, SpineController};

/// Damping for the external position constraints (0..1). Higher = more damping.
const DEFAULT_POSITION_DAMPING: f32 = 0.1;
/// Compliance for the external position constraints. Lower = stiffer. Soft
/// default so the constraint doesn't fight the rest of the solver.
const DEFAULT_POSITION_COMPLIANCE: f32 = 0.01;

/// Which spine-particles are driven by the indicator within
/// `SpineController.particles`. `SpineController.particles` is
/// `[PH, P0, P1, P2, P3]`, so P1/P2/P3 are at indices 2, 3, 4.
const QUAD_INDICES: [usize; 3] = [2, 3, 4];

/// Convert from Bevy (y-up) to Vello physics (y-down) coordinates.
#[inline]
fn bevy_to_vello(point: Vec2) -> Vec2 {
    Vec2::new(point.x, -point.y)
}

/// Compute the three world-space control points in BEVY (y-up) coordinates.
///
/// `local_points` are authored in vello (y-down) coords centred on the indicator
/// (P0/P1/P2 offsets). We convert each offset to bevy y-up, scale it, rotate it
/// by the indicator Z-rotation, then translate by the indicator position.
fn compute_bevy_targets(
    indicator_pos: Vec2,
    rotation: f32,
    local_points: &[Vec2; 3],
    scale: f32,
) -> [Vec2; 3] {
    let (sin, cos) = rotation.sin_cos();
    let mut out = [Vec2::ZERO; 3];
    for (i, lp) in local_points.iter().enumerate() {
        // local_points are vello (y-down); convert to bevy (y-up).
        let local_bevy = Vec2::new(lp.x, -lp.y) * scale;
        let rotated = Vec2::new(
            local_bevy.x * cos - local_bevy.y * sin,
            local_bevy.x * sin + local_bevy.y * cos,
        );
        out[i] = indicator_pos + rotated;
    }
    out
}

/// Pull the quad spine particles P1/P2/P3 toward the indicator points via
/// external position constraints (damped, default compliance).
pub fn spine_position_constraint(
    indicator_q: Query<(&SpineIndicator, &Transform), Without<CharacterRoot>>,
    spine_q: Query<(Entity, &SpineController, &VelloCharacterPhysicsRoot)>,
    particle_q: Query<&VelloParticle>,
    mut world: ResMut<VelloConstraintWorld>,
) {
    let Ok((indicator, indicator_tf)) = indicator_q.single() else {
        return;
    };
    let indicator_pos = indicator_tf.translation.truncate();
    let rotation = indicator_tf.rotation.to_euler(EulerRot::XYZ).2;

    let targets_bevy = compute_bevy_targets(
        indicator_pos,
        rotation,
        &indicator.local_points,
        // `local_points` are derived from the real assembled particle positions
        // (already in world/scaled space at spawn), so no extra scaling is needed.
        1.0,
    );
    let targets_vello: [Vec2; 3] = targets_bevy.map(bevy_to_vello);

    for (character_entity, spine, _p_root) in &spine_q {
        for (k, i) in QUAD_INDICES.iter().enumerate() {
            let particle_entity = spine.particles[*i];
            // Guard: only queue when the particle is fully assembled.
            if particle_q.get(particle_entity).is_err() {
                continue;
            }
            world.queue_character_one_time_external_position_constraint(
                &CharacterExternalPositionConstraintEvent {
                    character_entity,
                    joint_entity: particle_entity,
                    config: vello_physics::ExternalPositionConstraintConfig {
                        target: targets_vello[k],
                        compliance: spine.config.compliance,
                        damping: spine.config.damping,
                    },
                },
            );
        }
    }
}
