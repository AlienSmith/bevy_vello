use bevy::{prelude::*, time::Fixed};
use bevy_vello::integrations::physics::{
    VelloCharacterPhysicsRoot, VelloParticle,
};

use crate::character::SpineController;
use crate::lower_body::measure::{
    update_measurements, ParticleSample,
};
use super::state::*;


pub(crate) fn try_seed(
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

/// Measurement time constant for the EMA-filtered `v_CoM` and `omega` (plan §3:
/// "EMA-filtered (2–4 frame time constant)"). At 90 Hz, 0.03 s ≈ 3 frames.
pub const MEASURE_TAU: f32 = 0.03;

/// Gather every particle owned by the character and update the whole-body CoM
/// measurement (plan §1 / changes §1).
///
/// Scans all [`VelloParticle`]s whose `root_entity` is the character root (the
/// `HipPin` entity), mass-weights position and velocity over **all** parts
/// (including arms), EMA-filters `v_CoM` and `omega`, and stores the measured
/// torso `theta` + `z_CoM` on the pin. Runs before [`drive_hip_pin`] so the
/// resolver's loops can read the measurement this same fixed step.
pub fn measure_body(
    time: Res<Time<Fixed>>,
    mut pin_q: Query<(Entity, &mut HipPin)>,
    spine_q: Query<&SpineController>,
    p_q: Query<&VelloParticle>,
) {
    let dt = time.delta_secs();
    for (entity, mut pin) in pin_q.iter_mut() {
        if !pin.enabled {
            continue;
        }
        let Ok(spine) = spine_q.get(entity) else {
            continue;
        };
        // Gather all particles of this character (root_entity == entity).
        let mut samples: Vec<ParticleSample> = Vec::with_capacity(16);
        for p in p_q.iter() {
            if p.root_entity == entity {
                samples.push(ParticleSample {
                    pos: p.particle.pos,
                    vel: p.particle.velocity,
                    inv_mass: p.particle.inv_mass,
                });
            }
        }
        if samples.is_empty() {
            continue;
        }
        // Torso axis: v10 spine = [PH, P0, P1, P2, P3]; hip = P3, shoulder = P1.
        let (hip, shoulder) = match (p_q.get(spine.particles[4]), p_q.get(spine.particles[2])) {
            (Ok(hip), Ok(shoulder)) => (hip.particle.pos, shoulder.particle.pos),
            _ => continue,
        };
        // Floor line in the Vello (y-down) world frame: the live foot line below
        // the hip anchor by the realized standing height. Previously this was the
        // hardcoded `floor_y = 0.0`, which pinned the CoM-height channel to zero
        // (floor − com clamped negative) and starved loop-1/loop-2 apex feedback.
        // Using the seeded anchor + standing_height ties the measurement to the
        // same real floor the collision response enforces.
        let floor_y = pin.anchor_p3.y + pin.standing_height;
        pin.measured = update_measurements(
            &mut pin.measure_filter,
            &samples,
            (hip, shoulder),
            floor_y,
            dt,
            MEASURE_TAU,
        );
    }
}
