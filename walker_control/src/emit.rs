//! The **actuate** emission adapter — apply [`crate::actuate`] commands back to
//! physics through the constraint world.
//!
//! Deliberately thin and honest about what it may do (plan Law 1): it reads
//! live particles, calls [`actuate_tick`], and queues ONLY
//! `CharacterAngularConstraintEvent` / `CharacterDistanceConstraintEvent`.
//! There is no `queue_character_one_time_external_position_constraint` call
//! anywhere in this crate.
//!
//! The entity handles are plain (no `game_lib` component types, so the crate
//! stays cycle-free); the caller passes its leg particle/joint entities.

use bevy::prelude::*;
use bevy_vello::integrations::physics::{
    CharacterAngularConstraintEvent, CharacterDistanceConstraintEvent, VelloConstraintWorld,
    VelloParticle,
};
use vello_physics::{AngularConstraintConfig, DistanceConstraintConfig};

use crate::actuate::{actuate_tick, Actuation, AngularCommand, DistanceCommand, LegLive, Pose};

/// One leg's entity handles in the physics world.
#[derive(Debug, Clone, Copy)]
pub struct LegHandles {
    /// Particles in solver order: `[P2 (waist ref), P3 (hip), knee, foot]`.
    pub parts: [Entity; 4],
    /// Hip angular joint (`P2-P3-knee`; vello joints index `P40-P3-P2` /
    /// `P41-P3-P2` on the character).
    pub hip_joint: Entity,
    /// Knee angular joint (`P3-knee-foot`).
    pub knee_joint: Entity,
    /// Hip↔foot strut distance joint, if spawned (`None` ⇒ no tether).
    pub strut: Option<Entity>,
}

impl LegHandles {
    /// Read the live leg (positions only — no actuator needs velocities).
    pub fn live(&self, p_q: &Query<&VelloParticle>) -> Option<LegLive> {
        let g = |e: Entity| p_q.get(e).ok().map(|p| p.particle.pos);
        Some(LegLive {
            waist: g(self.parts[0])?,
            hip: g(self.parts[1])?,
            knee: g(self.parts[2])?,
            foot: g(self.parts[3])?,
        })
    }
}

/// Map an angular command to the solver config. Fixed compliance + ζ;
/// velocity projection OFF (`angular_velocity: None`), all particles free
/// (`free_mask: 0b111` — there is no pin in this architecture), no hard break.
#[inline]
pub fn angular_config(cmd: &AngularCommand) -> AngularConstraintConfig {
    AngularConstraintConfig {
        rest_cos: cmd.rest_cos,
        rest_sin: cmd.rest_sin,
        compliance: cmd.compliance,
        zeta: cmd.zeta,
        angular_velocity: None,
        sigma: 0.0,
        free_mask: 0b111,
        max_angle: None,
    }
}

/// Map a distance command (the hip↔stance-foot strut) to the solver config.
#[inline]
pub fn distance_config(cmd: &DistanceCommand) -> DistanceConstraintConfig {
    DistanceConstraintConfig {
        rest_length: cmd.rest_length,
        compliance: cmd.compliance,
    }
}

/// One actuate tick end-to-end: read particles, compute commands, queue them.
/// Legs missing particle data contribute nothing (their previous constraints
/// simply expire — the solver state is per-tick).
pub fn drive_legs(
    character: Entity,
    left: &LegHandles,
    right: &LegHandles,
    p_q: &Query<&VelloParticle>,
    pose: &Pose,
    cfg: &crate::actuate::ActuatorConfig,
    dt: f32,
    world: &mut VelloConstraintWorld,
) -> Actuation {
    let l = left.live(p_q).unwrap_or(LEG_FALLBACK);
    let r = right.live(p_q).unwrap_or(LEG_FALLBACK);
    let a = actuate_tick(&l, &r, pose, cfg, dt);
    emit(character, left, right, &a, world);
    a
}

/// Degenerate live state ⇒ `actuate_tick` emits no angular commands (zero bone
/// lengths) but still the struts; keeps the queue call total.
const LEG_FALLBACK: LegLive = LegLive {
    waist: Vec2::ZERO,
    hip: Vec2::ZERO,
    knee: Vec2::ZERO,
    foot: Vec2::ZERO,
};

/// Queue an [`Actuation`]'s events. Pure plumbing; the values come from
/// [`crate::actuate`].
pub fn emit(
    character: Entity,
    left: &LegHandles,
    right: &LegHandles,
    a: &Actuation,
    world: &mut VelloConstraintWorld,
) {
    for (leg, cmds) in [(left, &a.left), (right, &a.right)] {
        if let Some(cmd) = cmds.hip_joint {
            world.queue_character_angular_constraints(&CharacterAngularConstraintEvent {
                character_entity: character,
                joint_entity: leg.hip_joint,
                config: angular_config(&cmd),
            });
        }
        if let Some(cmd) = cmds.knee_joint {
            world.queue_character_angular_constraints(&CharacterAngularConstraintEvent {
                character_entity: character,
                joint_entity: leg.knee_joint,
                config: angular_config(&cmd),
            });
        }
        if let (Some(strut), Some(cmd)) = (leg.strut, cmds.strut) {
            world.queue_character_distance_constraints(&CharacterDistanceConstraintEvent {
                character_entity: character,
                joint_entity: strut,
                config: distance_config(&cmd),
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuate::ActuatorConfig;

    #[test]
    fn angular_config_disables_projection_and_pins() {
        let cmd = AngularCommand {
            rest_cos: 0.6,
            rest_sin: 0.8,
            compliance: 4.2e-7,
            zeta: 0.9,
        };
        let c = angular_config(&cmd);
        assert_eq!(c.angular_velocity, None, "velocity projection stays off");
        assert_eq!(c.sigma, 0.0);
        assert_eq!(c.free_mask, 0b111, "every particle free — no pin modes");
        assert_eq!(c.max_angle, None);
        assert_eq!(c.compliance, 4.2e-7);
        assert_eq!(c.zeta, 0.9);
        assert!((c.rest_cos - 0.6).abs() < 1e-6 && (c.rest_sin - 0.8).abs() < 1e-6);
    }

    #[test]
    fn distance_config_passes_through() {
        let cmd = DistanceCommand {
            rest_length: 111.0,
            compliance: 1e-6,
        };
        let c = distance_config(&cmd);
        assert_eq!(c.rest_length, 111.0);
        assert_eq!(c.compliance, 1e-6);
    }

    #[test]
    fn emit_queues_into_the_world_without_panicking() {
        // Entity::PLACEHOLDER handles are fine at queue time (the solver
        // resolves them later); this proves the plumbing is total for any
        // command set, including dead-zone Nones and no-strut legs.
        let mut world = VelloConstraintWorld::new(Vec2::new(0.0, 980.0));
        let left = LegHandles {
            parts: [Entity::PLACEHOLDER; 4],
            hip_joint: Entity::PLACEHOLDER,
            knee_joint: Entity::PLACEHOLDER,
            strut: Some(Entity::PLACEHOLDER),
        };
        let right = LegHandles {
            strut: None,
            ..left
        };
        let cfg = ActuatorConfig::default();
        let pose = Pose {
            hip: Vec2::new(0.0, -140.0),
            left_foot: crate::actuate::FootPose {
                pos: Vec2::new(60.0, -90.0),
                stance: true,
            },
            right_foot: crate::actuate::FootPose {
                pos: Vec2::new(-20.0, -30.0),
                stance: false,
            },
            facing: 1.0,
            upper_angle: 0.1,
        };
        let live = LegLive {
            waist: Vec2::new(0.0, -180.0),
            hip: Vec2::new(0.0, -140.0),
            knee: Vec2::new(0.0, -85.0),
            foot: Vec2::new(0.0, -30.0),
        };
        let a = actuate_tick(&live, &live, &pose, &cfg, 1.0 / 90.0);
        emit(Entity::PLACEHOLDER, &left, &right, &a, &mut world);
        // No panic + at least the struts and lean commands were produced.
        assert!(a.left.hip_joint.is_some() || a.left.knee_joint.is_some());
        assert!(a.right.strut.is_some());
    }
}
