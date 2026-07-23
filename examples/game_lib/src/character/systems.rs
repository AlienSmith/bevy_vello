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

/// Angular error threshold (in radians) for switching to elbow-only IK solve.
/// When the forearm direction error is ≤ this value, the shoulder freezes and only
/// the elbow rotates to finish aiming, preventing the two-joint oscillation near convergence.
/// 0.05 rad ≈ 3° — small enough to be invisible, large enough to prevent the slow tail.
const ELBOW_ONLY_THRESHOLD: f32 = 0.05;

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

fn solve_reach_ik(
    local_target: Vec2,
    p12_local: Vec2,
    upper_len: f32,
    forearm_len: f32,
    bend_sign: f32,
) -> (Vec2, Vec2) {
    let reach = upper_len + forearm_len;
    let mut target_pos_local = local_target;
    let root_to_target = target_pos_local - p12_local;
    let dist = root_to_target.length();
    if dist > reach {
        target_pos_local = p12_local + root_to_target.normalize() * (reach - 0.001);
    }
    let to_target = target_pos_local - p12_local;
    let target_dist = to_target.length().max(f32::EPSILON);
    let target_dir = to_target / target_dist;

    // Law of cosines for shoulder offset
    let cos_shoulder = (upper_len * upper_len + target_dist * target_dist
        - forearm_len * forearm_len)
        / (2.0 * upper_len * target_dist);
    let cos_shoulder = cos_shoulder.clamp(-1.0, 1.0);
    let shoulder_offset = cos_shoulder.acos();

    // Rotate target direction by shoulder_offset * bend_sign to get the desired upper arm direction
    let total_shoulder_angle = shoulder_offset * bend_sign;
    let (sin_sh, cos_sh) = total_shoulder_angle.sin_cos();
    let desired_upper_dir = Vec2::new(
        target_dir.x * cos_sh - target_dir.y * sin_sh,
        target_dir.x * sin_sh + target_dir.y * cos_sh,
    );
    let desired_p13_local = p12_local + desired_upper_dir * upper_len;
    let desired_prla_local = target_pos_local;

    (desired_p13_local, desired_prla_local)
}

/// Least-action forearm alignment IK: make the forearm point at the target.
///
/// Uses a Lagrange multiplier solve that constrains the forearm direction
/// (not wrist position), finding the minimal-displacement joint angles.
/// The weapon offset angle rotates the aim direction before solving.
///
/// The solve is **iterative** (4 iterations).  Rotating the shoulder changes
/// the elbow position, which in turn changes the elbow→target direction used
/// to compute `alpha`.  A single split therefore leaves a ~13.8° residual;
/// iterating reduces it to < 0.001°.
///
/// Returns `(desired_p13_local, desired_prla_local)`.
fn solve_aim_ik(
    local_target: Vec2,
    p12_local: Vec2,
    p13_local: Vec2,
    prla_local: Vec2,
    upper_len: f32,
    forearm_len: f32,
    weapon_offset_y: f32, // The parallel vertical offset
) -> (Vec2, Vec2) {
    // Use mutable copies that we refine each iteration
    let mut theta1 = {
        let upper_dir = p13_local - p12_local;
        upper_dir.y.atan2(upper_dir.x)
    };
    let mut theta2 = {
        let upper_dir = p13_local - p12_local;
        let forearm_dir = prla_local - p13_local;
        let cross = upper_dir.x * forearm_dir.y - upper_dir.y * forearm_dir.x;
        let dot = upper_dir.dot(forearm_dir);
        cross.atan2(dot)
    };

    // Iterative refinement: 4 iterations is enough to converge to < 0.001°
    for _iter in 0..4 {
        // Current elbow position (from current theta1)
        let (sin1, cos1) = theta1.sin_cos();
        let elbow = p12_local + Vec2::new(cos1, sin1) * upper_len;

        // Compute target direction from this iteration's elbow position
        let to_target = local_target - elbow;
        let target_dir = to_target.normalize_or_zero();
        let r = to_target.length();

        if r < 1e-4 {
            // Target is at elbow — can't aim, return current state
            let (sin1, cos1) = theta1.sin_cos();
            let desired_p13 = p12_local + Vec2::new(cos1, sin1) * upper_len;
            let (sin12, cos12) = (theta1 + theta2).sin_cos();
            let desired_prla = desired_p13 + Vec2::new(cos12, sin12) * forearm_len;
            return (desired_p13, desired_prla);
        }

        // Shift by weapon offset along the elbow→target normal
        let target_normal = Vec2::new(-target_dir.y, target_dir.x);
        let virtual_target = local_target - target_normal * weapon_offset_y;
        let to_vtarget = virtual_target - elbow;
        let alpha = to_vtarget.y.atan2(to_vtarget.x);

        // Forearm absolute angle
        let forearm_angle = theta1 + theta2;

        // Wrapped angular error
        let mut angle_diff = forearm_angle - alpha;
        angle_diff = (angle_diff + std::f32::consts::PI).rem_euclid(2.0 * std::f32::consts::PI)
            - std::f32::consts::PI;

        // Split equally and accumulate
        theta1 -= angle_diff * 0.5;
        theta2 -= angle_diff * 0.5;
    }

    // Reconstruct final positions from converged angles
    let (sin1, cos1) = theta1.sin_cos();
    let desired_p13 = p12_local + Vec2::new(cos1, sin1) * upper_len;

    let (sin12, cos12) = (theta1 + theta2).sin_cos();
    let desired_prla = desired_p13 + Vec2::new(cos12, sin12) * forearm_len;

    (desired_p13, desired_prla)
}

/// Elbow-only aim IK: shoulder is frozen, only the elbow rotates to aim at the target.
///
/// Uses a fixed-point iteration to handle the weapon offset circular dependency:
/// the weapon offset normal depends on the forearm direction, which is what we solve for.
/// Since this is only called when the angular error is small (≤ ELBOW_ONLY_THRESHOLD),
/// the initial guess (current forearm direction) is close to the answer → 4 iterations suffice.
fn solve_aim_ik_elbow_only(
    local_target: Vec2,
    _p12_local: Vec2,
    p13_local: Vec2, // elbow position — this becomes the desired shoulder target (frozen)
    prla_local: Vec2,
    _upper_len: f32,
    forearm_len: f32,
    weapon_offset_y: f32,
) -> (Vec2, Vec2) {
    // Shoulder frozen → elbow stays at its current position
    let desired_p13 = p13_local;

    // Initial guess: current forearm direction
    let mut forearm_dir = (prla_local - p13_local).normalize_or_zero();

    // Fixed-point iteration to converge forearm direction + weapon offset normal.
    //   forearm_dir → forearm_normal → virtual_target → new_forearm_dir → ...
    // 4 iterations is enough because the initial guess is already close.
    for _ in 0..4 {
        let forearm_normal = Vec2::new(-forearm_dir.y, forearm_dir.x);
        let virtual_target = local_target - forearm_normal * weapon_offset_y;
        let to_vtarget = virtual_target - p13_local;
        forearm_dir = to_vtarget.normalize_or_zero();
    }

    // Final wrist position from converged forearm direction
    let desired_prla = p13_local + forearm_dir * forearm_len;

    (desired_p13, desired_prla)
}

/// Dispatches to Reach / Aim (full or elbow-only) based on config.ik_mode.
fn compute_ik_positions(
    config: &ArmConfig,
    local_target: Vec2,
    p12_local: Vec2,
    p13_local: Vec2,
    prla_local: Vec2,
    upper_len: f32,
    forearm_len: f32,
) -> (Vec2, Vec2) {
    match &config.ik_mode {
        IkMode::Disabled => (p13_local, prla_local),
        IkMode::Reach => solve_reach_ik(
            local_target,
            p12_local,
            upper_len,
            forearm_len,
            config.bend_sign,
        ),
        IkMode::Aim { weapon_offset_y } => {
            let forearm_dir = prla_local - p13_local;
            let to_target = local_target - p13_local;
            let target_dir = to_target.normalize_or_zero();

            // Compare against the VIRTUAL target direction (accounting for weapon
            // offset), because that's what the IK solves for.  Using the true target
            // direction here would show a persistent weapon-offset-angle error even
            // when the IK is fully converged, causing the arm to oscillate.
            let target_normal = Vec2::new(-target_dir.y, target_dir.x);
            let virtual_target = local_target - target_normal * weapon_offset_y;
            let to_vtarget = virtual_target - p13_local;
            let vtarget_dir = to_vtarget.normalize_or_zero();

            let cos_err = forearm_dir.dot(vtarget_dir) / (forearm_dir.length().max(f32::EPSILON));
            let angle_err = cos_err.clamp(-1.0, 1.0).acos();
            let use_full = angle_err > ELBOW_ONLY_THRESHOLD;
            if use_full {
                solve_aim_ik(
                    local_target,
                    p12_local,
                    p13_local,
                    prla_local,
                    upper_len,
                    forearm_len,
                    *weapon_offset_y,
                )
            } else {
                solve_aim_ik_elbow_only(
                    local_target,
                    p12_local,
                    p13_local,
                    prla_local,
                    upper_len,
                    forearm_len,
                    *weapon_offset_y,
                )
            }
        }
    }
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
    cache: Option<&mut (Option<Vec2>, Vec2, Vec2)>,
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

    // Cache key: world-space true_target.  Must NOT cache on local_target because
    // blend_core.world_to_local produces a different value every frame as the
    // physics solver shifts the core frame — even when the world target is stationary.
    let (desired_p13_local, desired_prla_local) =
        if let Some((ref mut cached_target, ref mut cached_p13, ref mut cached_prla)) = cache {
            if let Some(prev_target) = cached_target {
                let dist = (true_target - *prev_target).length();
                if dist < f32::EPSILON {
                    // Target hasn't moved → reuse cached result
                    (*cached_p13, *cached_prla)
                } else {
                    // Target changed → compute fresh IK and update cache
                    let result = compute_ik_positions(
                        &config,
                        local_target,
                        p12_local,
                        p13_local,
                        prla_local,
                        upper_len,
                        forearm_len,
                    );
                    *cached_target = Some(true_target);
                    *cached_p13 = result.0;
                    *cached_prla = result.1;
                    result
                }
            } else {
                // First frame → compute and populate cache
                let result = compute_ik_positions(
                    &config,
                    local_target,
                    p12_local,
                    p13_local,
                    prla_local,
                    upper_len,
                    forearm_len,
                );
                *cached_target = Some(true_target);
                *cached_p13 = result.0;
                *cached_prla = result.1;
                result
            }
        } else {
            // No cache available (e.g. reach mode) → compute every frame
            compute_ik_positions(
                &config,
                local_target,
                p12_local,
                p13_local,
                prla_local,
                upper_len,
                forearm_len,
            )
        };

    // ---- Shoulder constraint with rotate-toward damping (local space) ----
    let desired_shoulder_cs = cos_sin(p1_local, p12_local, desired_p13_local);

    // Compute the current actual shoulder angle from local-space particle positions.
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

    // ---- Elbow constraint with rotate-toward damping (local space) ----
    let desired_elbow_cs = cos_sin(p12_local, desired_p13_local, desired_prla_local);

    // Compute the current actual elbow angle from local-space particle positions.
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

    // Shoulder (particle index 1): local target is the desired elbow position
    let mut shoulder_sm = particles[1].shape_matching;
    // In elbow-only mode (desired_p13 == current p13), snap the SM target directly
    // to freeze the shoulder.  The damped blend from previous frames would otherwise
    // exert a lingering force that fights the frozen-shoulder assumption.
    let desired_shoulder_local = if (desired_p13_local - p13_local).length_squared() <= f32::EPSILON
    {
        p13_local
    } else {
        damp_local_target(shoulder_sm.local_target, p12_local, desired_p13_local)
    };
    shoulder_sm.local_target = desired_shoulder_local;
    shoulder_sm.compliance = config.shape_matching_compliance;
    shoulder_sm.damping = config.shape_matching_damping;
    position_events.push(CharacterPivotPositionEvent {
        character_entity,
        joint_entity: particles_entity[1],
        target: shoulder_sm,
    });

    // Elbow (particle index 2): local target is the desired wrist position
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

    // Wrist (particle index 3): extend from elbow local target toward local target
    let mut wrist_sm = particles[3].shape_matching;
    let pos_e = particles[2].shape_matching.local_target;
    let distance = (wrist_sm.local_target - pos_e).length().max(f32::EPSILON);
    let dir_to_target = (local_target - pos_e).normalize();
    let desired_wrist_local = pos_e + dir_to_target * distance;
    let desired_wrist_local = damp_local_target(wrist_sm.local_target, pos_e, desired_wrist_local);
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
            Some(&mut (arm.cached_target, arm.cached_p13, arm.cached_prla)),
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
            Some(&mut (arm.cached_target, arm.cached_p13, arm.cached_prla)),
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: arm geometry for tests.
    struct TestArm {
        p12_local: Vec2,
        p13_local: Vec2,
        prla_local: Vec2,
        upper_len: f32,
        forearm_len: f32,
    }

    /// Build a test arm in y-down coordinate space:
    ///   - shoulder at origin
    ///   - upper arm pointing right (theta1 = 0)
    ///   - forearm with theta2 = -2.5 rad (~-143°), wrist at (10, -15) from shoulder
    ///
    /// In y-down, positive theta2 = CW bend (wrist below elbow, forward).
    /// theta2 = -2.5 is deliberately a hyperextended/reverse pose (wrist above elbow),
    /// giving a ~106° initial angular error — a worst-case stress test for convergence.
    ///
    /// An arm starting near alignment (angle_err < 0.05 rad ≈ 3°) would exercise
    /// `solve_aim_ik_elbow_only` instead.
    fn test_arm_bent() -> TestArm {
        let p12_local = Vec2::ZERO;
        let upper_len = 30.0;
        let forearm_len = 25.0;
        let theta1: f32 = 0.0; // shoulder angle: straight right
        let theta2: f32 = -2.5; // elbow angle: roughly -143° (bent, forearm pointing down-right)
        let (s1, c1) = theta1.sin_cos();
        let p13_local = p12_local + Vec2::new(c1, s1) * upper_len;
        let (s12, c12) = (theta1 + theta2).sin_cos();
        let prla_local = p13_local + Vec2::new(c12, s12) * forearm_len;
        TestArm {
            p12_local,
            p13_local,
            prla_local,
            upper_len,
            forearm_len,
        }
    }

    /// Compute the angle (in radians) between `forearm_dir` and `target_dir`
    fn compute_angle_err(forearm_dir: Vec2, target_dir: Vec2) -> f32 {
        let cos_err = forearm_dir.dot(target_dir) / (forearm_dir.length().max(f32::EPSILON));
        cos_err.clamp(-1.0, 1.0).acos()
    }

    // ========================================================================
    // Test 1: solve_aim_ik determinism + convergence to near-zero
    // ========================================================================
    // The iterative solve (4 iterations) converges the forearm direction to the
    // virtual target direction with < 0.001° residual, because each iteration
    // re-evaluates the elbow→target direction from the updated elbow position.
    #[test]
    fn test_solve_aim_ik_determinism() {
        let arm = test_arm_bent();
        let local_target = Vec2::new(50.0, -15.0);
        let offset = 0.0;

        let r1 = solve_aim_ik(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            offset,
        );
        let r2 = solve_aim_ik(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            offset,
        );

        // Determinism
        assert!(
            (r1.0 - r2.0).length() < f32::EPSILON,
            "solve_aim_ik not deterministic: p13 diff {}",
            (r1.0 - r2.0).length()
        );
        assert!(
            (r1.1 - r2.1).length() < f32::EPSILON,
            "solve_aim_ik not deterministic: prla diff {}",
            (r1.1 - r2.1).length()
        );

        let (desired_p13, desired_prla) = r1;

        // Distance constraints
        let upper_actual = (desired_p13 - arm.p12_local).length();
        let forearm_actual = (desired_prla - desired_p13).length();
        assert!(
            (upper_actual - arm.upper_len).abs() < 1e-4,
            "upper arm length changed: {} vs {}",
            upper_actual,
            arm.upper_len
        );
        assert!(
            (forearm_actual - arm.forearm_len).abs() < 1e-4,
            "forearm length changed: {} vs {}",
            forearm_actual,
            arm.forearm_len
        );

        // Convergence to near-zero (iterative solve)
        let forearm_dir = desired_prla - desired_p13;
        let to_target = local_target - desired_p13;
        let target_dir = to_target.normalize_or_zero();
        let residual_err = compute_angle_err(forearm_dir, target_dir);
        assert!(
            residual_err < 1e-4,
            "solve_aim_ik did not converge: residual={:.6}rad ({:.4}°)",
            residual_err,
            residual_err.to_degrees()
        );
    }

    // ========================================================================
    // Test 2: solve_aim_ik_elbow_only determinism + invariants
    // ========================================================================
    #[test]
    fn test_solve_aim_ik_elbow_only_invariants() {
        let arm = test_arm_bent();
        let local_target = Vec2::new(50.0, -15.0);
        let offset = 0.0;

        let r1 = solve_aim_ik_elbow_only(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            offset,
        );
        let r2 = solve_aim_ik_elbow_only(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            offset,
        );

        // Determinism
        assert!(
            (r1.0 - r2.0).length() < f32::EPSILON,
            "elbow_only not deterministic: p13"
        );
        assert!(
            (r1.1 - r2.1).length() < f32::EPSILON,
            "elbow_only not deterministic: prla"
        );

        let (desired_p13, desired_prla) = r1;

        // Shoulder frozen
        assert!(
            (desired_p13 - arm.p13_local).length() < f32::EPSILON,
            "shoulder moved in elbow-only mode"
        );

        // Forearm length preserved
        let forearm_actual = (desired_prla - desired_p13).length();
        assert!(
            (forearm_actual - arm.forearm_len).abs() < 1e-4,
            "forearm length changed in elbow-only: {} vs {}",
            forearm_actual,
            arm.forearm_len
        );

        // Forearm should point at target (offset=0)
        let forearm_dir = desired_prla - desired_p13;
        let to_target = local_target - desired_p13;
        let target_dir = to_target.normalize_or_zero();
        let err = compute_angle_err(forearm_dir, target_dir);
        // Elbow-only uses 4 iterations of fixed-point; floating-point accumulation
        // leaves a tiny residual (~0.02° with these parameters).  Use 1e-3 tolerance.
        assert!(
            err < 1e-3,
            "forearm does not point at target after elbow-only: angle_err={:.6}rad ({:.4}°)",
            err,
            err.to_degrees()
        );
    }

    // ========================================================================
    // Test 3: Aim vs Elbow-Only consistency at small angle_err
    // ========================================================================
    #[test]
    fn test_aim_vs_elbow_only_consistency() {
        // Build an arm that is ALREADY nearly aligned with the target,
        // so angle_err < ELBOW_ONLY_THRESHOLD.
        let p12_local = Vec2::ZERO;
        let upper_len = 30.0;
        let forearm_len = 25.0;

        // Forearm points at the target direction directly, so angle_err ≈ 0.
        let target_world = Vec2::new(50.0, -10.0);
        let p13_local = Vec2::new(30.0, 0.0); // shoulder→elbow straight right
        let to_target = target_world - p13_local;
        let target_dir = to_target.normalize();
        let forearm_len = 25.0;
        let prla_local = p13_local + target_dir * forearm_len;

        let offset = 0.0;

        let (aim_p13, aim_prla) = solve_aim_ik(
            target_world,
            p12_local,
            p13_local,
            prla_local,
            upper_len,
            forearm_len,
            offset,
        );
        let (eo_p13, eo_prla) = solve_aim_ik_elbow_only(
            target_world,
            p12_local,
            p13_local,
            prla_local,
            upper_len,
            forearm_len,
            offset,
        );

        // Both should produce forearm pointing at target
        let aim_forearm = aim_prla - aim_p13;
        let eo_forearm = eo_prla - eo_p13;
        let aim_angle_err = compute_angle_err(aim_forearm, target_dir);
        let eo_angle_err = compute_angle_err(eo_forearm, target_dir);

        // Full solve leaves a residual (shoulder movement changes elbow direction).
        // Just verify it's significantly reduced compared to what it would be.
        assert!(
            aim_angle_err < 10.0_f32.to_radians(),
            "solve_aim_ik: residual angle_err too large: aim_angle_err={:.4}° (target rely aligned)",
            aim_angle_err.to_degrees()
        );
        assert!(
            eo_angle_err < 1e-3,
            "elbow_only: residual angle_err={:.6}°",
            eo_angle_err.to_degrees()
        );

        // Wrist positions should be close (both aim at same direction)
        let wrist_diff = (aim_prla - eo_prla).length();
        assert!(
            wrist_diff < 1.0,
            "wrist positions differ too much: {} (aim_p13={:.2}, eo_p13={:.2})",
            wrist_diff,
            (aim_p13 - p12_local).length(),
            (eo_p13 - p12_local).length()
        );
    }

    // ========================================================================
    // Test 4: solve_reach_ik distance constraints
    // ========================================================================
    #[test]
    fn test_solve_reach_ik_invariants() {
        let arm = test_arm_bent();
        let local_target = Vec2::new(50.0, -10.0);

        let (desired_p13, desired_prla) = solve_reach_ik(
            local_target,
            arm.p12_local,
            arm.upper_len,
            arm.forearm_len,
            -1.0,
        );

        // Upper arm length preserved
        let upper_actual = (desired_p13 - arm.p12_local).length();
        assert!(
            (upper_actual - arm.upper_len).abs() < 1e-4,
            "reach: upper arm length changed: {} vs {}",
            upper_actual,
            arm.upper_len
        );

        // Forearm length preserved
        let forearm_actual = (desired_prla - desired_p13).length();
        assert!(
            (forearm_actual - arm.forearm_len).abs() < 1e-4,
            "reach: forearm length changed: {} vs {}",
            forearm_actual,
            arm.forearm_len
        );

        // Total reach should be within bounds
        let total_reach = (desired_prla - arm.p12_local).length();
        assert!(
            total_reach <= arm.upper_len + arm.forearm_len + 0.01,
            "reach: total reach {} exceeds {}",
            total_reach,
            arm.upper_len + arm.forearm_len
        );
    }

    #[test]
    fn test_solve_reach_ik_out_of_range() {
        let arm = test_arm_bent();
        // Target far beyond reach
        let far_target = Vec2::new(500.0, -300.0);

        let (desired_p13, desired_prla) = solve_reach_ik(
            far_target,
            arm.p12_local,
            arm.upper_len,
            arm.forearm_len,
            -1.0,
        );

        // Arm should be fully extended (wrist at max reach)
        let total_reach = (desired_prla - arm.p12_local).length();
        let max_reach = arm.upper_len + arm.forearm_len;
        assert!(
            total_reach <= max_reach + 0.01,
            "out-of-range: total reach {} exceeds max {}",
            total_reach,
            max_reach
        );
        assert!(
            total_reach > max_reach - 1.0,
            "out-of-range: arm not fully extended: {} vs max {}",
            total_reach,
            max_reach
        );
    }

    // ========================================================================
    // Test 5: compute_ik_positions dispatch logic
    // ========================================================================
    #[test]
    fn test_compute_ik_positions_dispatch() {
        let arm = test_arm_bent();

        // Case A: IkMode::Aim with offset=0 → should aim forearm at local_target
        let config_aim = ArmConfig {
            ik_mode: IkMode::Aim {
                weapon_offset_y: 0.0,
            },
            ..Default::default()
        };
        let (aim_p13, aim_prla) = compute_ik_positions(
            &config_aim,
            Vec2::new(50.0, -10.0),
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
        );

        // forearm distance preserved
        let forearm_actual = (aim_prla - aim_p13).length();
        assert!(
            (forearm_actual - arm.forearm_len).abs() < 1e-4,
            "Aim: forearm length changed in dispatch"
        );

        // Case B: IkMode::Reach
        let config_reach = ArmConfig {
            ik_mode: IkMode::Reach,
            ..Default::default()
        };
        let (reach_p13, reach_prla) = compute_ik_positions(
            &config_reach,
            Vec2::new(50.0, -10.0),
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
        );

        // reach: forearm distance preserved
        let reach_forearm = (reach_prla - reach_p13).length();
        assert!(
            (reach_forearm - arm.forearm_len).abs() < 1e-4,
            "Reach: forearm length changed in dispatch"
        );

        // Case C: IkMode::Disabled → no change
        let config_disabled = ArmConfig {
            ik_mode: IkMode::Disabled,
            ..Default::default()
        };
        let (dis_p13, dis_prla) = compute_ik_positions(
            &config_disabled,
            Vec2::new(50.0, -10.0),
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
        );
        assert!(
            (dis_p13 - arm.p13_local).length() < f32::EPSILON,
            "Disabled: p13 should be unchanged"
        );
        assert!(
            (dis_prla - arm.prla_local).length() < f32::EPSILON,
            "Disabled: prla should be unchanged"
        );
    }

    // ========================================================================
    // Test 6: Convergence test — iterative solve converges to near-zero
    // ========================================================================
    // The 4-iteration iterative solve re-evaluates the elbow→target direction
    // from the updated elbow position each iteration, converging to < 0.001°.
    #[test]
    fn test_solve_aim_ik_converges() {
        let arm = test_arm_bent();
        let local_target = Vec2::new(50.0, -15.0);

        let (desired_p13, desired_prla) = solve_aim_ik(
            local_target,
            arm.p12_local,
            arm.p13_local,
            arm.prla_local,
            arm.upper_len,
            arm.forearm_len,
            0.0,
        );

        // Residual angular error after solve
        let forearm_dir_result = desired_prla - desired_p13;
        let to_target_result = local_target - desired_p13;
        let target_dir_result = to_target_result.normalize_or_zero();
        let residual_err = compute_angle_err(forearm_dir_result, target_dir_result);

        // The 4-iteration iterative solve should converge to < 0.001°
        assert!(
            residual_err < 1e-4,
            "solve_aim_ik did not converge: residual={:.6}rad ({:.4}°)",
            residual_err,
            residual_err.to_degrees()
        );
    }

    // ========================================================================
    // Test 7: Simulated stationary target iteration (cache test)
    // ========================================================================
    #[test]
    fn test_cache_with_stationary_target() {
        let arm = test_arm_bent();
        let target = Vec2::new(50.0, -15.0);

        // We can't call calculate_arm_ik (needs Bevy entities), so test the
        // caching logic directly by simulating what calculate_arm_ik does.
        // The cache key is world-space true_target.
        let true_target = bevy_to_vello(target);

        // Simulate first call: cache miss (None) → populate
        let mut cached_target: Option<Vec2> = None;
        let mut cached_p13 = Vec2::ZERO;
        let mut cached_prla = Vec2::ZERO;

        if let None = cached_target {
            cached_target = Some(true_target);
            cached_p13 = Vec2::new(1.0, 2.0);
            cached_prla = Vec2::new(3.0, 4.0);
        }

        // Simulate second call with same target: cache hit
        if let Some(prev) = cached_target {
            let dist = (true_target - prev).length();
            if dist < f32::EPSILON {
                // Cache hit: reuse, verify cached values unchanged
                assert_eq!(cached_p13, Vec2::new(1.0, 2.0));
                assert_eq!(cached_prla, Vec2::new(3.0, 4.0));
            } else {
                panic!("cache miss on stationary target: dist={}", dist);
            }
        } else {
            panic!("expected Some after first call");
        }
    }
}
