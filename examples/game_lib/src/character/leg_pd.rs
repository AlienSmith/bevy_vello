//! Angular PD leg controller — **no IK**.
//!
//! The walking feet diverged because the old leg path was a position-only
//! tracker ([`calculate_limb_ik`](super::systems::calculate_limb_ik) solving an
//! IK reach target, first-order rest-angle chase). When the Raibert resolver
//! walks the hip, the foot targets move and the position-only PD lags behind by
//! ~`v_target·τ` (37 px peak at cruise).
//!
//! This controller re-aims the hip and knee **angular constraints** to the
//! resolver pose and lets the solver's baked-in implicit spring-damper (the
//! damping ratio `ζ` from [`LimbConfig::zeta`]) supply the D term. The resolver
//! outputs target position **and** target velocity for all five particles
//! (hip/knees/feet); the controller reads the live particles, derives current
//! joint angles + angular rates, converts the resolver targets to target
//! angles, and emits the same [`CharacterAngularConstraintEvent`]s as the arm
//! controller (reusing the XPBD angular constraints and the ω-scheduled
//! compliance). It does NOT know about IK, and it never touches the rigid
//! core-frame machinery.
//!
//! ## Damping lives in the solver
//!
//! Per the locked design there is NO external `kd` term in the game layer. The
//! `AngularConstraint` solver computes a critically-damped `Kd` from the
//! compliance (spring: `Kp = 1.0/compliance`) and the dynamically-varying
//! generalized mass `m = 1/w_sum`:
//!
//! ```text
//! Kd = 2·ζ·sqrt(Kp·m),   γ = Kd·compliance/dt
//! ```
//!
//! `ζ` (default `1.0` = critical damping) is the ONLY damping knob; `ζ <= 0`
//! reduces the solver exactly to the legacy undamped XPBD solve. This controller
//! simply forwards `config.zeta` into every emitted angular config.

use bevy::prelude::*;
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, VelloCharacterPhysicsRoot, VelloConstraintWorld, VelloJoint,
    VelloParticle,
};
use vello_physics::utility::{angular_compliance_at_length, cos_sin};

use crate::character::{
    systems::{bone_angular_velocity, rotate_cs, rotate_toward, FOREARM_ANGULAR_EPSILON},
    LimbConfig,
};

/// Resolver target set for one leg, **already converted to vello y-down**
/// coordinates (the same frame the live particles are read in). Angles and
/// angular rates are frame-agnostic, but keeping one frame avoids a
/// translation/scale mismatch between the measured and target legs.
#[derive(Clone)]
pub struct LegPdTargets {
    /// Target hip position (P3), vello y-down.
    pub hip: Vec2,
    /// Target knee position (P41), vello y-down.
    pub knee: Vec2,
    /// Target foot position (PRLL), vello y-down.
    pub foot: Vec2,
    /// Target hip velocity, vello y-down px/s.
    pub hip_vel: Vec2,
    /// Target knee velocity, vello y-down px/s.
    pub knee_vel: Vec2,
    /// Target foot velocity, vello y-down px/s.
    pub foot_vel: Vec2,
}

/// Bevy driver mirror of [`drive_limb`](super::systems::drive_limb): gathers a
/// leg's 4 particles, runs the pure angular PD, and queues the resulting
/// angular constraints. Legs are `[P2(ref), P3(hip/pivot), knee, foot]` with
/// joints `[P2_P3_knee (hip), P3_knee_foot (knee)]`.
#[allow(clippy::too_many_arguments)]
pub fn drive_leg_pd(
    targets: &LegPdTargets,
    particles_e: &[Entity; 4],
    joints_e: &[Entity; 2],
    config: &LimbConfig,
    dt: f32,
    p_root: &VelloCharacterPhysicsRoot,
    p_q: &Query<&VelloParticle>,
    j_q: &Query<&VelloJoint>,
    world: &mut VelloConstraintWorld,
) {
    if p_root.initial_frame_coordinates.is_none() {
        return;
    }

    let p_e: Vec<Entity> = particles_e.to_vec();
    let j_e: Vec<Entity> = joints_e.to_vec();
    let particles: Vec<VelloParticle> = p_e.iter().map(|e| p_q.get(*e).unwrap().clone()).collect();
    let angular_constraints: Vec<VelloJoint> =
        j_e.iter().map(|e| j_q.get(*e).unwrap().clone()).collect();
    if angular_constraints.len() < 2 {
        return;
    }

    let events = calculate_leg_pd(targets, dt, &particles, joints_e, config);
    events
        .iter()
        .for_each(|e| world.queue_character_angular_constraints(e));
}

/// Pure angular PD for one leg. No IK, no core frame — reads live particles
/// (`[P2, P3, knee, foot]` in vello y-down), derives current joint angles +
/// rates, converts the resolver targets to target angles, and emits angular
/// constraint events for the hip and knee joints. Each event forwards
/// `config.zeta` so the solver applies its implicit spring-damper damping.
fn calculate_leg_pd(
    targets: &LegPdTargets,
    dt: f32,
    particles: &[VelloParticle],
    joints_entity: &[Entity; 2],
    config: &LimbConfig,
) -> Vec<CharacterAngularConstraintEvent> {
    const LEG_PARTICLE_COUNT: usize = 4; // [P2(ref), P3(hip), knee(P41), foot(PRLL)]
    const LEG_JOINT_COUNT: usize = 2; // hip (P2_P3_knee), knee (P3_knee_foot)

    let mut angular_events = vec![];

    if particles.len() < LEG_PARTICLE_COUNT || joints_entity.len() < LEG_JOINT_COUNT {
        return angular_events;
    }

    let p2 = particles[0].particle.pos;
    let p3 = particles[1].particle.pos;
    let knee = particles[2].particle.pos;
    let foot = particles[3].particle.pos;
    let v2 = particles[0].particle.velocity;
    let v3 = particles[1].particle.velocity;
    let v_knee = particles[2].particle.velocity;
    let v_foot = particles[3].particle.velocity;

    // ---- Current joint angles + angular rates (vello y-down) ----
    // `θ = φ₁ − φ₂` (incoming bone angle minus outgoing), so `θ̇ = ω₁ − ω₂`.
    // The rates drive the ω-scheduled compliance (fast swing → softer spring).
    let theta_hip_cs = cos_sin(p2, p3, knee);
    let theta_knee_cs = cos_sin(p3, knee, foot);

    let omega_spine = bone_angular_velocity(p3 - p2, v3 - v2);
    let omega_thigh = bone_angular_velocity(knee - p3, v_knee - v3);
    let omega_calf = bone_angular_velocity(foot - knee, v_foot - v_knee);

    let theta_dot_hip = omega_spine - omega_thigh;
    let theta_dot_knee = omega_thigh - omega_calf;

    // ---- Target angles (same reference) ----
    // `A_t = hip_t + (P2 − P3)` spins the target hip up along the LIVE spine so
    // the measured and target hip angles share one reference — robust even if
    // the frame tilts. P2 itself is never commanded (the resolver assumes the
    // spine is perfectly vertical).
    let a_t = targets.hip + (p2 - p3);
    let theta_hip_t_cs = cos_sin(a_t, targets.hip, targets.knee);
    let theta_knee_t_cs = cos_sin(targets.hip, targets.knee, targets.foot);

    // Position-only rest: the solver's ζ damping (not an external kd offset)
    // supplies the D term, so the rest angle is simply the target angle.
    let rest_hip_cs = theta_hip_t_cs;
    let rest_knee_cs = theta_knee_t_cs;

    // ---- Target joint angular rates (velocity projection) ----
    // `angular_velocity` is the desired joint rate in **rad/s**, independent of
    // the substep count: the solver converts it to the per-substep constraint
    // velocity `c_dot_desired = ω · dt_substep` inside `apply_velocity_projection`
    // (constraints.rs), so the game layer never needs to know `substeps`. The
    // hip reference spins with the LIVE spine (there is no target P2), exactly
    // mirroring how `a_t` reuses `p2 - p3` above, so its rate is
    // `ω_spine_live − ω_thigh_target`. The one-shot reset clears a stale value
    // after each frame. Non-finite targets are gated to `None`.
    let omega_thigh_t = bone_angular_velocity(
        targets.knee - targets.hip,
        targets.knee_vel - targets.hip_vel,
    );
    let omega_calf_t = bone_angular_velocity(
        targets.foot - targets.knee,
        targets.foot_vel - targets.knee_vel,
    );
    // `c_dot_scale` is a unitless gain on the commanded joint rate (default 1.0
    // = command the raw rad/s rate; lower to probe reduced projection
    // authority). It no longer performs any unit conversion — `angular_velocity`
    // is already rad/s and the solver multiplies by the substep dt.
    let angular_velocity_hip = {
        let v = (omega_spine - omega_thigh_t) * config.c_dot_scale;
        if v.is_finite() {
            Some(v)
        } else {
            None
        }
    };
    let angular_velocity_knee = {
        let v = (omega_thigh_t - omega_calf_t) * config.c_dot_scale;
        if v.is_finite() {
            Some(v)
        } else {
            None
        }
    };

    // ---- ω-scheduled compliance keyed on each joint's own rate ----
    // Reuse the arm's back-EMF style gain schedule: fast swing → soft spring
    // (withholds correction authority instead of pumping the motion), at rest →
    // full stiffness so the leg locks crisply. Length-normalized per bone.
    let scheduled_compliance = |omega: f32| -> f32 {
        if config.omega_ref > 0.0 && config.soft_scale > 1.0 {
            let s = (omega.abs() / config.omega_ref).min(1.0);
            config.angular_compliance * (1.0 + s * (config.soft_scale - 1.0))
        } else {
            config.angular_compliance
        }
    };
    let thigh_len = (knee - p3).length();
    let calf_len = (foot - knee).length();
    let hip_compliance = angular_compliance_at_length(
        scheduled_compliance(theta_dot_hip),
        config.compliance_ref_length,
        thigh_len,
    );
    let knee_compliance = angular_compliance_at_length(
        scheduled_compliance(theta_dot_knee),
        config.compliance_ref_length,
        calf_len,
    );

    let max_delta = config.max_angle_rate * dt;
    let character_entity = particles[0].root_entity;
    // Reduced-space masks when the parent frame is pinned by the HipPin:
    //   hip joint  (P2, P3, knee)  -> only the knee is free: 0b100
    //   knee joint (P3, knee, foot)-> P3 anchored, knee+foot free: 0b110
    // Without this, the pinned P2/P3 absorb most of every correction and the
    // pin undoes it on the next frame, leaving only a few percent of authority.
    let (hip_free, knee_free) = if config.anchor_chain {
        (0b100u8, 0b110u8)
    } else {
        (0b111u8, 0b111u8)
    };

    // ---- Hip angular constraint (P2_P3_knee) ----
    {
        let current_cs = theta_hip_cs;
        let rest_cs = rest_hip_cs;
        let cos_err = (current_cs.x * rest_cs.x + current_cs.y * rest_cs.y).clamp(-1.0, 1.0);
        let angle_err = cos_err.acos();
        // Dead zone: at rest with a stationary target, current == rest ⇒ no
        // event. Without it, infinitesimal corrections emit events each frame
        // and feed the target↔physics oscillation.
        if angle_err > FOREARM_ANGULAR_EPSILON {
            let (blended_cos, blended_sin) = rotate_toward(
                current_cs.x,
                current_cs.y,
                rest_cs.x,
                rest_cs.y,
                config.rest_chase_alpha,
                max_delta,
            );
            angular_events.push(CharacterAngularConstraintEvent {
                character_entity,
                joint_entity: joints_entity[0],
                config: vello_physics::AngularConstraintConfig {
                    rest_cos: blended_cos,
                    rest_sin: blended_sin,
                    compliance: hip_compliance,
                    zeta: config.zeta,
                    angular_velocity: angular_velocity_hip,
                    sigma: config.sigma,
                    free_mask: hip_free,
                    max_angle: None,
                },
            });
        }
    }

    // ---- Knee angular constraint (P3_knee_foot) ----
    {
        let current_cs = theta_knee_cs;
        let rest_cs = rest_knee_cs;
        let cos_err = (current_cs.x * rest_cs.x + current_cs.y * rest_cs.y).clamp(-1.0, 1.0);
        let angle_err = cos_err.acos();
        if angle_err > FOREARM_ANGULAR_EPSILON {
            let (blended_cos, blended_sin) = rotate_toward(
                current_cs.x,
                current_cs.y,
                rest_cs.x,
                rest_cs.y,
                config.rest_chase_alpha,
                max_delta,
            );
            angular_events.push(CharacterAngularConstraintEvent {
                character_entity,
                joint_entity: joints_entity[1],
                config: vello_physics::AngularConstraintConfig {
                    rest_cos: blended_cos,
                    rest_sin: blended_sin,
                    compliance: knee_compliance,
                    zeta: config.zeta,
                    angular_velocity: angular_velocity_knee,
                    sigma: config.sigma,
                    free_mask: knee_free,
                    max_angle: None,
                },
            });
        }
    }

    angular_events
}

#[cfg(test)]
mod tests {
    use super::*;
    use vello_physics::collision_response::Particle;

    const THIGH: f32 = 30.0;
    const CALF: f32 = 30.0;

    /// Build a vertical standing leg: P2 (waist) above P3 (hip), straight down
    /// through knee to foot. All velocities settable for rate control.
    fn standing_leg(v_knee: Vec2, v_foot: Vec2, v_p3: Vec2, v_p2: Vec2) -> Vec<VelloParticle> {
        let p2 = Vec2::new(0.0, -40.0);
        let p3 = Vec2::new(0.0, 0.0);
        let knee = Vec2::new(0.0, THIGH);
        let foot = Vec2::new(0.0, THIGH + CALF);
        let mk = |pos: Vec2, vel: Vec2| VelloParticle {
            particle_init: Particle::default(),
            particle: Particle {
                previous_pos: pos - vel * (1.0 / 90.0),
                pos,
                velocity: vel,
                inv_mass: 1.0,
            },
            root_entity: Entity::PLACEHOLDER,
            fk_target: None,
        };
        vec![
            mk(p2, v_p2),
            mk(p3, v_p3),
            mk(knee, v_knee),
            mk(foot, v_foot),
        ]
    }

    fn cfg() -> LimbConfig {
        LimbConfig {
            zeta: 1.5, // distinct from the 1.0 default so tests can assert passthrough
            ..Default::default()
        }
    }

    fn joints() -> [Entity; 2] {
        [Entity::PLACEHOLDER, Entity::PLACEHOLDER]
    }

    /// Standing targets coinciding with the standing leg (all zero velocity).
    fn standing_targets() -> LegPdTargets {
        LegPdTargets {
            hip: Vec2::new(0.0, 0.0),
            knee: Vec2::new(0.0, THIGH),
            foot: Vec2::new(0.0, THIGH + CALF),
            hip_vel: Vec2::ZERO,
            knee_vel: Vec2::ZERO,
            foot_vel: Vec2::ZERO,
        }
    }

    /// Standing leg + standing targets ⇒ rest == current ⇒ the dead zone
    /// suppresses both events (no constraint churn at rest).
    #[test]
    fn standing_no_events() {
        let leg = standing_leg(Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, Vec2::ZERO);
        let targets = standing_targets();
        let c = cfg();
        let events = calculate_leg_pd(&targets, 1.0 / 90.0, &leg, &joints(), &c);
        assert!(
            events.is_empty(),
            "at rest with matching targets no constraints should be emitted: {} events",
            events.len()
        );
    }

    /// A leg perfectly tracking a moving target has rest == current pose, so no
    /// constraints are emitted — the leg is carried along *with* the reference
    /// instead of fighting it (this was the divergence root cause).
    #[test]
    fn perfect_tracking_emits_no_events() {
        let v_knee = Vec2::new(120.0, 0.0);
        let v_foot = Vec2::new(120.0, 0.0);
        let v_p3 = Vec2::new(120.0, 0.0);
        let v_p2 = Vec2::new(120.0, 0.0);
        let leg = standing_leg(v_knee, v_foot, v_p3, v_p2);

        // Targets identical to the live leg, moving with the same rigid-body
        // velocity ⇒ every target angle matches the current angle.
        let targets = LegPdTargets {
            hip: leg[1].particle.pos,
            knee: leg[2].particle.pos,
            foot: leg[3].particle.pos,
            hip_vel: v_p3,
            knee_vel: v_knee,
            foot_vel: v_foot,
        };
        let c = cfg();
        let events = calculate_leg_pd(&targets, 1.0 / 90.0, &leg, &joints(), &c);
        assert!(
            events.is_empty(),
            "perfect tracking must emit no constraints: {} events",
            events.len()
        );
    }

    /// A perturbed target (hip/knee angles differ from the live leg) drives both
    /// joints, and each emitted config carries `zeta` straight through from the
    /// LimbConfig — the solver's damping knob.
    #[test]
    fn perturbed_target_emits_both_joints_with_zeta() {
        let leg = standing_leg(Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, Vec2::ZERO);
        // Bend the target leg forward: knee + foot displaced in +x.
        let targets = LegPdTargets {
            hip: Vec2::new(0.0, 0.0),
            knee: Vec2::new(8.0, THIGH),
            foot: Vec2::new(16.0, THIGH + CALF),
            hip_vel: Vec2::ZERO,
            knee_vel: Vec2::ZERO,
            foot_vel: Vec2::ZERO,
        };
        let c = cfg();
        let events = calculate_leg_pd(&targets, 1.0 / 90.0, &leg, &joints(), &c);
        assert!(
            events.len() == 2,
            "bent target should drive both joints: got {} events",
            events.len()
        );
        // zeta passthrough: both joints must carry the config's zeta.
        for e in &events {
            assert!(
                (e.config.zeta - c.zeta).abs() < 1e-6,
                "zeta must pass through to the solver: got {} want {}",
                e.config.zeta,
                c.zeta
            );
        }
        // Sanity: the emitted hip rest angle differs from the live hip angle
        // (the solver will rotate the thigh toward the bent target).
        let hip_event = &events[0];
        let rest_cs = Vec2::new(hip_event.config.rest_cos, hip_event.config.rest_sin);
        let current_cs = cos_sin(
            leg[0].particle.pos,
            leg[1].particle.pos,
            leg[2].particle.pos,
        );
        let dot = (rest_cs.x * current_cs.x + rest_cs.y * current_cs.y).clamp(-1.0, 1.0);
        assert!(
            dot.acos() > 0.01,
            "rest angle must rotate off the static pose toward the bent target"
        );
    }

    /// ζ <= 0 must still be forwarded so the solver disables damping (reduces
    /// to legacy undamped XPBD) — the controller never clamps or overrides it.
    #[test]
    fn zeta_forwarded_unmodified() {
        for z in [0.0, -1.0, 0.3, 1.0, 2.0] {
            let leg = standing_leg(Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, Vec2::ZERO);
            let targets = LegPdTargets {
                hip: Vec2::new(0.0, 0.0),
                knee: Vec2::new(8.0, THIGH),
                foot: Vec2::new(16.0, THIGH + CALF),
                hip_vel: Vec2::ZERO,
                knee_vel: Vec2::ZERO,
                foot_vel: Vec2::ZERO,
            };
            let mut c = cfg();
            c.zeta = z;
            let events = calculate_leg_pd(&targets, 1.0 / 90.0, &leg, &joints(), &c);
            assert!(!events.is_empty(), "zeta={z}: bent target must emit events");
            for e in &events {
                assert!(
                    (e.config.zeta - z).abs() < 1e-6,
                    "zeta={z}: forwarded value {} must equal the config",
                    e.config.zeta
                );
            }
        }
    }

    /// The ω-scheduled compliance softens the spring on a fast swing: for an
    /// identical pose, a fast-swinging leg emits a strictly larger compliance
    /// than the same leg at rest (back-EMF authority withholding).
    #[test]
    fn omega_schedule_softens_compliance() {
        let rest_leg = standing_leg(Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, Vec2::ZERO);
        let fast_leg = standing_leg(
            Vec2::new(0.0, 900.0),
            Vec2::new(0.0, 900.0),
            Vec2::ZERO,
            Vec2::ZERO,
        );
        let targets = standing_targets();
        let c = cfg();

        // Push the target off the live pose so events are emitted for both.
        let rest_targets = LegPdTargets {
            knee: Vec2::new(6.0, THIGH),
            foot: Vec2::new(12.0, THIGH + CALF),
            ..targets
        };
        let fast_targets = rest_targets.clone();

        let rest_events = calculate_leg_pd(&rest_targets, 1.0 / 90.0, &rest_leg, &joints(), &c);
        let fast_events = calculate_leg_pd(&fast_targets, 1.0 / 90.0, &fast_leg, &joints(), &c);
        assert_eq!(rest_events.len(), 2, "rest leg must emit both joints");
        assert_eq!(fast_events.len(), 2, "fast leg must emit both joints");
        for i in 0..2 {
            assert!(
                fast_events[i].config.compliance > rest_events[i].config.compliance,
                "joint {i}: fast swing must soften compliance ({} vs {})",
                fast_events[i].config.compliance,
                rest_events[i].config.compliance
            );
        }
    }

    /// A static live leg + a target leg with a *rotating* velocity field
    /// produces non-zero `angular_velocity` on the emitted constraints — the
    /// solver's velocity projection is armed with the resolver's per-frame
    /// joint rate. A stationary target (all-zero velocities) keeps it `None`
    /// (or zero), so the projection stays inert when there is nothing to track.
    #[test]
    /// `c_dot_scale` scales the commanded joint rate (default 1.0). Verify a
    /// scaled config reaches a proportionally scaled `angular_velocity`.
    #[test]
    fn c_dot_scale_reaches_desired_rate() {
        let leg = standing_leg(Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, Vec2::ZERO);
        let dt = 1.0 / 90.0;

        // Bent target pose (same as the zeta test) so both joints emit, plus a
        // rigid 90°-per-second spin about the hip to arm the projection.
        let moving = LegPdTargets {
            knee: Vec2::new(8.0, THIGH),
            foot: Vec2::new(16.0, THIGH + CALF),
            knee_vel: Vec2::new(-THIGH, 0.0),
            foot_vel: Vec2::new(-(THIGH + CALF), 0.0),
            ..standing_targets()
        };

        // Full strength (1.0) vs half strength (0.5) on the same target.
        let full = calculate_leg_pd(&moving, dt, &leg, &joints(), &cfg());
        let half_cfg = LimbConfig {
            c_dot_scale: 0.5,
            ..cfg()
        };
        let half = calculate_leg_pd(&moving, dt, &leg, &joints(), &half_cfg);

        assert_eq!(full.len(), 2);
        assert_eq!(half.len(), 2);
        for (f, h) in full.iter().zip(&half) {
            let fv = f.config.angular_velocity.expect("full must arm projection");
            let hv = h.config.angular_velocity.expect("half must arm projection");
            assert!(
                (hv - 0.5 * fv).abs() < 1e-6,
                "c_dot_scale=0.5 must halve angular_velocity: full={fv} half={hv}"
            );
        }
    }

    #[test]
    fn moving_target_arms_velocity_projection() {
        let leg = standing_leg(Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, Vec2::ZERO);
        let c = cfg();
        let dt = 1.0 / 90.0;

        // Bent target pose (same as the zeta test) so both joints emit.
        let base = LegPdTargets {
            hip: Vec2::new(0.0, 0.0),
            knee: Vec2::new(8.0, THIGH),
            foot: Vec2::new(16.0, THIGH + CALF),
            hip_vel: Vec2::ZERO,
            knee_vel: Vec2::ZERO,
            foot_vel: Vec2::ZERO,
        };

        // Stationary target: velocities zero ⇒ desired joint rates zero.
        let still = calculate_leg_pd(&base, dt, &leg, &joints(), &c);
        assert_eq!(still.len(), 2, "bent target must emit both joints");
        for e in &still {
            assert!(
                e.config.angular_velocity.map_or(true, |v| v.abs() < 1e-6),
                "stationary target must keep angular_velocity zero/None, got {:?}",
                e.config.angular_velocity
            );
        }

        // Rotating target: knee/foot sweep a circle about the hip ⇒ the thigh
        // rotates ⇒ a non-zero target hip rate flows into the projection.
        let moving = LegPdTargets {
            // Rotate the whole leg: knee/foot positions unchanged, but the
            // velocities are a rigid 90°-per-second spin about the hip.
            knee_vel: Vec2::new(-THIGH, 0.0),
            foot_vel: Vec2::new(-(THIGH + CALF), 0.0),
            ..base
        };
        let events = calculate_leg_pd(&moving, dt, &leg, &joints(), &c);
        assert_eq!(events.len(), 2, "rotating target must emit both joints");
        for (i, e) in events.iter().enumerate() {
            assert!(
                e.config.angular_velocity.is_some(),
                "joint {i}: rotating target must arm the velocity projection"
            );
        }
    }
}
