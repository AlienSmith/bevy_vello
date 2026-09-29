use bevy::{ecs::error::info, math::VectorSpace, prelude::*};
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, CharacterExternalPositionConstraintEvent,
    CharacterPivotImpulseEvent, VelloCharacterPhysicsRoot, VelloConstraintWorld, VelloJoint,
    VelloParticle,
};
use vello_physics::{
    utility::{cos_sin, BalancedCoreFrame},
    ConnectionConstraint,
};

use crate::character::{
    ArmConfig, IkMode, LeftArmController, ResetArmControlConstraintsEvent, RightArmController,
    SpineConfig, SpineController, SpineIndicator,
};
use crate::utility::linear_to_angle;

pub fn tick_spine_drive(
    time: Res<Time>,
    spine_q: Query<(&SpineController, &VelloCharacterPhysicsRoot)>,
    mut indicator_q: Query<&mut SpineIndicator>,
    p_q: Query<&VelloParticle>,
    p_j: Query<&VelloJoint>,
    mut world: ResMut<VelloConstraintWorld>,
) {
    let dt = time.delta_secs();
    // The spine indicator carries the commanded pose AND the root entity of the
    // character it drives. We resolve that character's SpineController directly
    // (all 5 particles + 3 angular joints, per the calculate_spine_drive
    // contract) and queue drive events just for those.
    let Ok(mut indicator) = indicator_q.single_mut() else {
        return;
    };
    // Guard: no known character yet (or it was despawned).
    let Ok((spine, p_root)) = spine_q.get(indicator.character) else {
        return;
    };
    if p_root.initial_frame_coordinates.is_none() {
        return;
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
    let config = indicator.config.clone();

    // Re-anchor the desired goal to the live body every fixed tick so the three
    // external position constraints behave as *pure damping* rather than active
    // drag:
    // - While a movement command is active, the desired centre is `P2 +
    //   commanded_dir * command_reach` (a constant distance ahead of the spine) and
    //   the desired angle is the input heading — the controller steers/resists.
    // - When the command is released (player let go), BOTH references are
    //   re-derived from the live body pose (`desired_center` = live P2,
    //   `desired_angle` = live P2→P1 heading). With rest offset ≈ 0 the springs
    //   only damp velocity, so an external knock can freely rotate/translate the
    //   body and that motion *persists* instead of being dragged back to a stale
    //   pose.
    if indicator.command_active {
        let p2_current = p_q.get(spine.particles[3]).map(|p| p.particle.pos);
        if let Ok(p2) = p2_current {
            indicator.desired_center = p2 + indicator.commanded_dir * indicator.command_reach;
        }
    } else {
        // Idle: mirror the P2 re-anchor for the heading, but *predict* the
        // reference a half step ahead of the live body instead of snapping it to
        // the current pose. Setting the target exactly at the live position makes
        // the spring rest offset ~0, so it applies no damping force and the body
        // decelerates abruptly on its own (jitter). Offsetting each reference by
        // `0.5 * velocity * dt` keeps the spring pulling smoothly along the motion.
        //
        // Linear:  predicted_center = P2 + 0.5 * v2 * dt
        // Angular: heading = atan2(P1 - P2), ω = cross(P1-P2, v1-v2)/|P1-P2|^2,
        //          predicted_angle = heading + 0.5 * ω * dt
        let p1 = p_q.get(spine.particles[2]).ok();
        let p2 = p_q.get(spine.particles[3]).ok();
        if let (Some(p1), Some(p2)) = (p1, p2) {
            let pos1 = p1.particle.pos;
            let pos2 = p2.particle.pos;
            let vel1 = p1.particle.velocity;
            let vel2 = p2.particle.velocity;
            let half_dt = 0.5 * dt;
            indicator.desired_center = pos2 + vel2 * half_dt;
            let arm = pos1 - pos2;
            let dvel = vel1 - vel2;
            let len_sq = arm.length_squared();
            let ang_vel = if len_sq > f32::EPSILON {
                (arm.x * dvel.y - arm.y * dvel.x) / len_sq
            } else {
                0.0
            };
            indicator.desired_angle = arm.y.atan2(arm.x) + ang_vel * half_dt;
        }
    }

    let result = calculate_spine_drive(
        &entities,
        &particles,
        &angular_entities,
        &angulars,
        &config,
        indicator.desired_center,
        indicator.desired_angle,
        &indicator.local_points,
        indicator.omega_prev,
        dt,
    );

    result
        .position_events
        .iter()
        .for_each(|e| world.queue_character_one_time_external_position_constraint(e));
    result
        .angular_events
        .iter()
        .for_each(|e| world.queue_character_angular_constraints(e));

    // The virtual pose is exactly what the physics pull toward AND what the
    // indicator visual is drawn at — write it back so they never diverge.
    indicator.center = result.virtual_center;
    indicator.angle = result.virtual_angle;
    // Store the frame motion (re-derived, deterministic) for the lean scale, plus
    // the commanded lean (drawn as a bent P3) and this frame's smoothed omega
    // (becomes `omega_prev` next tick for differentiating angular acceleration).
    let dt_safe = dt.max(f32::EPSILON);
    indicator.linear_speed = result.motion.linear / dt_safe;
    indicator.angular_speed = result.motion.angular / dt_safe;
    indicator.lean_angle = result.lean_angle;
    indicator.omega_prev = result.omega;
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
/// Interpolate `current` partway toward `target` (`current + alpha*step`) and
/// clamp the resulting per-frame displacement to `max_step`. Pure function of
/// `(current, target, max_step, alpha)` — deterministic, unit-testable.
#[inline]
fn interpolate_toward(current: f32, target: f32, alpha: f32, max_step: f32) -> f32 {
    let step = target - current;
    let mid = current + alpha * step;
    let step_len = mid - current;
    let clamped = step_len.clamp(-max_step, max_step);
    current + clamped
}

/// Shortest-path variant of [`interpolate_toward`] for **angles** (radians).
///
/// Naive `target - current` breaks on the ±π wrap: going from Up (-π/2) to
/// Left (π) the raw diff is `3π/2` (the long way) and, once `current` is re-
/// wrapped to just under -π (≈ -3.1) near the target, the diff jumps to ~6.24
/// (≈ 358°), so the pose spins a full turn each frame and never settles. Here
/// the difference is normalised into `(-π, π]` before interpolating + clamping,
/// so it always takes the shortest arc and converges cleanly.
#[inline]
fn interpolate_toward_angle(current: f32, target: f32, alpha: f32, max_step: f32) -> f32 {
    let delta = (target - current).rem_euclid(std::f32::consts::TAU);
    let step = if delta > std::f32::consts::PI {
        delta - std::f32::consts::TAU
    } else {
        delta
    };
    current + (step * alpha).clamp(-max_step, max_step)
}

/// The signed *shortest-path* angular displacement from `current` to `target`
/// (radians), in `(-π, π]`. Used so the per-frame angular *velocity* carries the
/// true rotation direction (e.g. Up → Left is −π/2, not +3π/2), which the lean
/// needs to tilt the correct way.
#[inline]
fn shortest_signed_delta(current: f32, target: f32) -> f32 {
    let delta = (target - current).rem_euclid(std::f32::consts::TAU);
    if delta > std::f32::consts::PI {
        delta - std::f32::consts::TAU
    } else {
        delta
    }
}

/// Mirror of the solver's `P1_P2_P3` angle measurement
/// ([`constraints.rs`](study_vello/integrations/vello_physics/src/constraints.rs:437)):
/// `v1 = P1→P2`, `v2 = P2→P3`, `cos = u1·u2`, `sin = u1.y*u2.x - u1.x*u2.y`.
/// Recomputing the angular joint's rest `cos/sin` from the *same* commanded bent
/// positions the external position constraints target makes the solver's angular
/// error (`current − rest`) zero at the commanded pose, so the position and
/// angular constraints agree by construction and never fight.
#[inline]
fn angle_basis(pos0: Vec2, pos1: Vec2, pos2: Vec2) -> Option<(Vec2, Vec2, f32, f32)> {
    let v1 = pos1 - pos0;
    let v2 = pos2 - pos1;
    let l1 = v1.length();
    let l2 = v2.length();
    if l1 < 1e-6 || l2 < 1e-6 {
        return None;
    }
    let u1 = v1 / l1;
    let u2 = v2 / l2;
    let cos = u1.dot(u2);
    let sin = u1.y * u2.x - u1.x * u2.y;
    Some((u1, u2, cos, sin))
}

/// Vector variant of [`interpolate_toward`]: move `current` partway toward
/// `target` and clamp the per-frame displacement length to `max_step`.
#[inline]
fn interpolate_toward_vec(current: Vec2, target: Vec2, alpha: f32, max_step: f32) -> Vec2 {
    let delta = target - current;
    let step = (delta * alpha).clamp_length_max(max_step);
    current + step
}

/// The motion the drive actually applied this frame (linear distance in px and
/// signed heading change in radians). Re-derived from the poses each frame, so
/// the controller stays deterministic (no accumulated state).
#[derive(Clone, Copy, Default)]
struct FrameMotion {
    linear: f32,
    angular: f32,
}

/// Result of a [`calculate_spine_drive`] tick: the queued constraint events,
/// the virtual pose (which is ALSO what the indicator is drawn at and what the
/// external position constraints pull the spine toward — one shared target),
/// the frame motion used to scale the lean, and the lean itself (the commanded
/// tip deflection + this frame's signed angular velocity for next-frame diff).
struct SpineDriveResult {
    position_events: Vec<CharacterExternalPositionConstraintEvent>,
    angular_events: Vec<CharacterAngularConstraintEvent>,
    virtual_center: Vec2,
    virtual_angle: f32,
    /// Commanded tip (P3) lean deflection (rad), single source of truth for the
    /// draw and the `P1_P2_P3` angular-rest deviation.
    lean_angle: f32,
    /// This frame's signed angular velocity (rad/s) — stored on the indicator to
    /// become `omega_prev` next tick (for differentiating angular acceleration).
    omega: f32,
    motion: FrameMotion,
}

/// Rotate a P2-centred local offset by `angle` (radians). Vello y-down world
/// convention: the same local-space rotation used by the old indicator handle.
#[inline]
fn rotate_local(point: Vec2, angle: f32) -> Vec2 {
    let (sin, cos) = angle.sin_cos();
    Vec2::new(point.x * cos - point.y * sin, point.x * sin + point.y * cos)
}

/// Compute a single drive tick from the *latched desired goal* (set by input +
/// P2 in [`tick_spine_drive`]) down to the queued constraints.
///
/// **Position-only drive.** Every tick the current centre (P2) is interpolated
/// partway toward the latched desired centre and clamped by the per-frame
/// position cap; the resulting **virtual centre** is both (a) written back onto
/// the `SpineIndicator` (so the visual is the same thing the physics see) and
/// (b) turned into the three external position targets. The P1/P2/P3 shape keeps
/// its *current* orientation (no rotational command), so no angular event is
/// emitted and no angular feedback loop can induce a perpetual spin. Pure: given
/// the same desired goal / current pose / caps it returns the same thing, so the
/// "keep last status on no input" latching lives entirely in how the desired
/// goal is stored, not in this function.
fn calculate_spine_drive(
    entities: &Vec<Entity>,
    particles: &Vec<VelloParticle>,
    angular_entities: &Vec<Entity>,
    angulars: &Vec<VelloJoint>,
    config: &SpineConfig,
    desired_center: Vec2,
    desired_angle: f32,
    local_points: &[Vec2; 3],
    omega_prev: f32,
    dt: f32,
) -> SpineDriveResult {
    let mut angular_events = vec![];
    let mut position_events = vec![];

    // Out-of-contract guard: we drive P1/P2/P3, so we need the full particle set
    // (entities[2..=4]) resolvable against the particles list.
    if particles.len() < 5 {
        return SpineDriveResult {
            position_events,
            angular_events,
            virtual_center: Vec2::ZERO,
            virtual_angle: 0.0,
            lean_angle: 0.0,
            omega: 0.0,
            motion: FrameMotion::default(),
        };
    }
    let p2 = particles[3].particle.pos;
    let p1 = particles[2].particle.pos;

    // Rotation + translation drive, all in Vello (y-down) **world space**.
    //
    // `current_angle` is the live world heading of the P2→P1 bone
    // (atan2(y, x), 0 = right, +90 = down, -90 = up). `desired_angle` is the
    // world heading from the arrow keys (8 global dirs). Both are world
    // headings, so interpolating between them yields the world angle the spine
    // should rotate to, with natural damping via alpha 0.5 + the angular cap.
    // The pose is built by rotating the P2-centred rest offsets *relative to*
    // their own rest heading (`virtual_angle - rest_heading`), so we never
    // assume the spine's initial orientation.
    let center = p2;
    let max_pos_step = config.max_pos_speed * dt;
    // Live world heading of the P2→P1 bone (Vello y-down: 0 = right, +90 = down).
    let current_angle = (p1.y - p2.y).atan2(p1.x - p2.x);
    // Rest world heading of the spine, derived from the P2-centred rest offsets
    // (local_points[0] = P2→P1). This is the spine's heading at `angle == 0`, so
    // we never *assume* an initial orientation — we read it from the particle
    // layout. Rotating the pose by `virtual_angle - rest_heading` makes the
    // posed world heading equal `virtual_angle`.
    let rest_heading = local_points[0].y.atan2(local_points[0].x);
    // Lever arm of the rotation about P2 = max |local_point| (P2-to-tip), used
    // to convert the linear speed cap into an angular one.
    let p2_to_tip_radius = local_points
        .iter()
        .map(|p| p.length())
        .fold(0.0_f32, f32::max);
    let max_ang_step = linear_to_angle(config.max_pos_speed * dt, p2_to_tip_radius);
    let virtual_center = interpolate_toward_vec(center, desired_center, 0.5, max_pos_step);
    // Both current and target are world headings, so interpolation yields the
    // world heading the spine should rotate to (natural damping via alpha 0.5 +
    // the angular cap). Angular variant takes the *shortest* arc so it never
    // spins the long way round (e.g. Up → Left must swing 90°, not 270°).
    let virtual_angle = interpolate_toward_angle(current_angle, desired_angle, 0.5, max_ang_step);

    let motion = FrameMotion {
        linear: (desired_center - center).length(),
        // Signed angular displacement (shortest-path sign), so the angular
        // *velocity* below carries the true rotation direction for the lean.
        angular: shortest_signed_delta(desired_angle, current_angle),
    };

    // ---- Lean (P3-only tilt) ----
    // This frame's raw signed angular velocity (shortest-path direction), then
    // EMA-smoothed across frames (omega_prev already holds the previous smoothed
    // value) so the differentiated acceleration below is stable.
    let dt_safe = dt.max(f32::EPSILON);
    let omega_raw = motion.angular / dt_safe;
    let omega = omega_raw * 0.5 + omega_prev * 0.5;
    // Angular acceleration = change in the smoothed signed angular velocity.
    let alpha = (omega - omega_prev) / dt_safe;
    // Reference acceleration at which the lean reads full: the frame-scale
    // angular velocity `max_ang_step / dt` reaching its cap within one frame, so
    // `max_ang_step / dt / dt`.
    let lean_accel_ref = (max_ang_step / dt_safe) / dt_safe;
    let ratio = (alpha / lean_accel_ref).clamp(-1.0, 1.0);
    let theta_lean = ratio * config.lean_max_angle;

    // Command each P1/P2/P3 (indices 2/3/4): P1 (2) and P2 (3) stay on the rigid
    // base heading (`virtual_angle`), **P3 (4) only** is deflected by `theta_lean`
    // about P2 so the spine tilts toward the rotation direction.
    let p1_target = virtual_center + rotate_local(local_points[0], virtual_angle - rest_heading);
    let p2_target = virtual_center;
    let p3_target =
        virtual_center + rotate_local(local_points[2], virtual_angle - rest_heading + theta_lean);
    let bent_targets = [p1_target, p2_target, p3_target];

    for (i, target) in [2usize, 3, 4].into_iter().zip(bent_targets.iter()) {
        position_events.push(CharacterExternalPositionConstraintEvent {
            character_entity: particles[0].root_entity,
            joint_entity: entities[i],
            config: vello_physics::ExternalPositionConstraintConfig {
                target: *target,
                compliance: config.compliance,
                damping: config.damping,
            },
        });
    }

    // Re-rest the driven quad's angular joint at P2 (`angulars[2]` == P1_P2_P3)
    // to the SAME commanded bent positions the external position constraints
    // target. With rest == commanded, the solver's angular error (`current -
    // rest`) is zero at the commanded pose, so the position and angular
    // constraints can never fight. `angle_basis(P1,P2,P3)` below mirrors the
    // solver's measurement (constraints.rs:437).
    if let Some((_, _, rest_cos, rest_sin)) = angle_basis(p1_target, p2_target, p3_target) {
        angular_events.push(CharacterAngularConstraintEvent {
            character_entity: particles[0].root_entity,
            joint_entity: angular_entities[2],
            config: vello_physics::AngularConstraintConfig {
                rest_cos,
                rest_sin,
                compliance: config.lean_compliance,
            },
        });
    }
    let _ = angulars; // reserved for future multi-joint spine pose use.

    SpineDriveResult {
        position_events,
        angular_events,
        virtual_center,
        virtual_angle,
        lean_angle: theta_lean,
        omega,
        motion,
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
