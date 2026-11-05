//! The **actuate** module — pure core (plan §1: `Actuate`).
//!
//! Input is exactly the five-element pose for the stick-on-two-legs model:
//! hip position, two foot positions (+ stance flag for the strut), facing,
//! and the commanded upper-body angle. Everything else — the two knees — is
//! derived here by 2-bone IK. Output is the constraint-command set: the hip
//! and knee angular rest bases plus the hip↔stance-foot strut distance. The
//! emission adapter maps these to `CharacterAngularConstraintEvent` /
//! `CharacterDistanceConstraintEvent` — this core knows nothing about
//! entities, and the `ExternalPositionConstraint` API is not referenced
//! anywhere in this crate (plan Law 1).
//!
//! # Actuation dynamics: fixed compliance + fixed ζ, nothing else
//! The XPBD angular constraint carries `compliance` (spring, `Kp = 1/c`) and
//! `zeta` (the solver's implicit damping ratio, `Kd = 2·ζ·sqrt(Kp·m)`). We
//! emit both as **fixed config values** — deliberately dropping the two
//! dynamic terms the old `leg_pd` used:
//! - the ω-scheduled (back-EMF) compliance softening, and
//! - the velocity projection (`angular_velocity` + `sigma`): proven better
//!   off, so the emitted config leaves `angular_velocity: None` (and `sigma`
//!   is then irrelevant).
//! Consequently this module is **position-only** (no particle velocities) and
//! stateless per tick: given (live particles, pose), the commands are a pure
//! function.
//!
//! # Loop 3 — the upper-body orientation lives in the hip joints, not the waist
//! For this model the body above `P3` is a **rigid stick**: `P1`/head are tied
//! to `P2` by blueprint distance joints, so the `P1-P2-P3` angular joint is
//! *not* an attitude actuator (it would only fight the rigidity). The stick's
//! orientation is the **`P3 → P2` axis**, set by the two leg hip joints
//! `P40-P3-P2` and `P41-P3-P2` — exactly the `hip_joint` commands emitted
//! here. The commanded `upper_angle` enters by aiming the hip joint's
//! reference bone (`P3 → P2`) at the lean, via `a_t` below. This is loop 3's
//! actuator and the reason there is no separate waist command.
//!
//! Ported from [`calculate_leg_pd`](../../examples/game_lib/src/character/leg_pd.rs)
//! with the dead machinery stripped: no `anchor_chain`/`ext_pin` modes (there
//! is no pin), no thrust/floor-snap target surgery (target shaping is the
//! control module's job).
//!
//! # Frame
//! Vello **y-down**, the physics world frame the live particles are read in
//! (`x` right = travel direction for `facing > 0`, `y` down). The pose's foot
//! heights are *commands*, not measurements — control computes them from the
//! measured stance feet, so no double-delay is introduced.

use glam::Vec2;

// ---------------------------------------------------------------------------
// Types: the pose, the live state, the commands
// ---------------------------------------------------------------------------

/// The five-element pose from the control module. Vello y-down.
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    /// Root target = hip particle (`P3`) position, next-tick predicted.
    pub hip: Vec2,
    /// Commanded left foot (`PLL`) position + stance state.
    pub left_foot: FootPose,
    /// Commanded right foot (`PRL`) position + stance state.
    pub right_foot: FootPose,
    /// Travel direction sign: `+1` faces +x, `-1` faces -x.
    pub facing: f32,
    /// Commanded upper-body (stick) orientation about `P3`, rad,
    /// `+` = forward lean toward `+x`. This is loop 3's actuator value; it is
    /// delivered through the two leg hip joints (`P40-P3-P2`, `P41-P3-P2`) as
    /// the lean of the `P3 → P2` reference axis — never as a waist constraint.
    pub upper_angle: f32,
}

/// A foot target plus its stance state (the strut gate).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FootPose {
    pub pos: Vec2,
    /// `true` while this foot is the loaded support (strut at active
    /// compliance); swing feet get the idle tether.
    pub stance: bool,
}

/// Live leg state read from the particles this tick, Vello y-down.
/// Positions only: with fixed compliance/damping and no velocity projection
/// there is nothing here that needs particle velocities.
#[derive(Debug, Clone, Copy)]
pub struct LegLive {
    /// Waist reference particle (`P2`) — carries the hip joint's incoming bone.
    pub waist: Vec2,
    /// Hip particle (`P3`).
    pub hip: Vec2,
    /// Knee particle (`P40`/`P41`).
    pub knee: Vec2,
    /// Foot particle (`PLL`/`PRL`).
    pub foot: Vec2,
}

impl LegLive {
    fn thigh_len(&self) -> f32 {
        (self.knee - self.hip).length()
    }
    fn calf_len(&self) -> f32 {
        (self.foot - self.knee).length()
    }
}

/// Tuning for the actuation tick. Fixed compliance + damping per the design
/// above — the solver's implicit spring-damper is the entire leg actuation
/// dynamics.
///
/// There is **no rest-angle blend**: the emitted rest *is* the target angle.
/// Blending from the live measured angle (`current + α·(target − current)`)
/// makes the constraint permanently a fraction of a step behind the pose, and
/// because the pose frame rides on the measured hip it turns a sag into a
/// one-way feedback loop — the hip drops, the foot targets drop with it, the
/// leg folds, the hip drops further.
#[derive(Debug, Clone, Copy)]
pub struct ActuatorConfig {
    /// Fixed hip-joint (`P2-P3-knee`) compliance, the solver's `Kp = 1/c`.
    /// Defaults to the v10 blueprint's own `P2_P3_P40` value.
    pub hip_compliance: f32,
    /// Fixed knee-joint (`P3-knee-foot`) compliance. Defaults to the v10
    /// blueprint's own `P3_P40_PLLL` value — one decade softer than the hip,
    /// which is what lets the knee absorb load without folding.
    pub knee_compliance: f32,
    /// Fixed damping ratio `ζ` forwarded to every angular constraint. The
    /// solver derives `Kd = 2·ζ·sqrt(Kp·m)`; the blueprint leaves it at its
    /// serde default of `1.0` (critical damping), so that is the default here.
    pub zeta: f32,
    /// Hip↔stance-foot strut compliance while the foot is the support. The
    /// blueprint's `P3_PLLL`/`P3_PRLL` distance joints use `1e-6`, and the
    /// stand pose depends on that: the strut is what holds the hip off the
    /// foot at the leg's full reach.
    pub strut_active: f32,
    /// Strut compliance while the leg swings (tether rides along, no authority).
    pub strut_idle: f32,
}

impl Default for ActuatorConfig {
    fn default() -> Self {
        Self {
            // Stiff angular rest targets: the boost QA sweep showed a planted
            // foot only TRACKS a commanded position under the hip when the hip
            // and knee joints are near-rigid (5e-8 / 5e-7 let the leg fold in
            // place instead of dragging the foot along the floor).
            hip_compliance: 1e-9,
            knee_compliance: 1e-8,
            zeta: 1.0,
            // The strut keeps the softer 1e-6: hip↔foot distance authority is
            // already adequate, the foot-drag was an angular-stiffness limit.
            strut_active: 1e-6,
            strut_idle: 1e-2,
        }
    }
}

/// One angular constraint command (rest basis + solver params).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngularCommand {
    pub rest_cos: f32,
    pub rest_sin: f32,
    pub compliance: f32,
    pub zeta: f32,
}

/// One distance constraint command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceCommand {
    pub rest_length: f32,
    pub compliance: f32,
}

/// Commands for one leg's hip (`P2-P3-knee` = `P40-P3-P2`) and knee
/// (`P3-knee-foot`) joints plus the hip↔foot strut. `None` angular = inside
/// the dead zone (current == target; emit nothing).
#[derive(Debug, Clone, Copy, Default)]
pub struct LegCommands {
    pub hip_joint: Option<AngularCommand>,
    pub knee_joint: Option<AngularCommand>,
    pub strut: Option<DistanceCommand>,
}

/// The full command set for one tick. The emission adapter maps joints to the
/// character's joint entities; there is no waist joint on this model.
#[derive(Debug, Clone, Copy, Default)]
pub struct Actuation {
    pub left: LegCommands,
    pub right: LegCommands,
}

// ---------------------------------------------------------------------------
// The pure tick
// ---------------------------------------------------------------------------

/// Angular dead zone (rad): below this, current == target ⇒ no emission.
/// Matches the reference `FOREARM_ANGULAR_EPSILON`.
const ANGLE_EPSILON: f32 = 1e-4;

/// One tick of the pure actuate core.
///
/// `dt` is accepted for signature stability (and so callers do not branch on
/// it) but no longer scales anything: the emitted rest angle is the target.
pub fn actuate_tick(
    left: &LegLive,
    right: &LegLive,
    pose: &Pose,
    cfg: &ActuatorConfig,
    _dt: f32,
) -> Actuation {
    Actuation {
        left: leg_commands(left, pose, pose.left_foot, cfg),
        right: leg_commands(right, pose, pose.right_foot, cfg),
    }
}

/// IK knee target for one leg: the chord is `pose.hip → foot_pose.pos`, the
/// bone lengths are the live ones (`|knee−hip|`, `|foot−knee|`), and the
/// anatomical (anterior) mirror is selected along the facing axis.
///
/// This is the exact value [`leg_commands`] aims the knee joint at, exposed so
/// telemetry can report it (`tP40`/`tP41`) without ever disagreeing with the
/// actuation it describes.
///
/// Returns `None` when the bones are degenerate or either chord endpoint is
/// non-finite — the same guard that makes the angular commands inert.
pub fn ik_knee_target(pose: &Pose, foot_pose: FootPose, live: &LegLive) -> Option<Vec2> {
    let thigh = live.thigh_len();
    let calf = live.calf_len();
    if thigh < 1e-3 || calf < 1e-3 || !foot_pose.pos.is_finite() || !pose.hip.is_finite() {
        return None;
    }
    let (knee_raw, _foot_eff) =
        walker_math::solve_leg_ik(pose.hip, foot_pose.pos, thigh, calf, 1.0);
    Some(walker_math::select_anterior_knee(
        pose.hip,
        foot_pose.pos,
        knee_raw,
        pose.facing,
    ))
}

/// Derive the knee by IK from the pose (hip→foot chord, live bone lengths),
/// then hip + knee angular commands and the strut.
fn leg_commands(
    live: &LegLive,
    pose: &Pose,
    foot_pose: FootPose,
    cfg: &ActuatorConfig,
) -> LegCommands {
    let mut out = LegCommands::default();

    // ---- Strut first: hip↔foot is pose-only, needs no knee ----
    if pose.hip.is_finite() && foot_pose.pos.is_finite() {
        out.strut = Some(DistanceCommand {
            rest_length: (pose.hip - foot_pose.pos).length().max(1.0),
            compliance: if foot_pose.stance {
                cfg.strut_active
            } else {
                cfg.strut_idle
            },
        });
    }

    // ---- Angular commands require valid bones ----
    // IK: knee for hip→foot_target, then pick the anatomical (anterior) mirror
    // along the facing axis. The selection makes the bend sign irrelevant.
    let Some(knee) = ik_knee_target(pose, foot_pose, live) else {
        return out;
    };
    // The hip joint's incoming bone is `P3 → P2`; it is also the stick whose
    // orientation loop 3 commands.
    let spine = live.waist - live.hip;
    if spine.length_squared() < 1e-6 {
        return out;
    }

    // ---- Current joint angles (vello y-down) ----
    let theta_hip = angle_at(live.waist, live.hip, live.knee);
    let theta_knee = angle_at(live.hip, live.knee, live.foot);

    // ---- Target angles ----
    // Loop 3 via the hip joints: the thigh reference axis (`P3 → P2`) is aimed
    // at the commanded upper-body lean. `a_t` is the virtual `P2` the hip joint
    // should see — the live `P3→P2` length rotated forward by `upper_angle`
    // (vello y-down: upright is `(0,-1)`, `+x` is forward).
    let len = spine.length();
    let (sin_lean, cos_lean) = pose.upper_angle.sin_cos();
    let a_t = pose.hip + Vec2::new(len * sin_lean, -len * cos_lean);
    let target_hip = angle_at(a_t, pose.hip, knee);
    let target_knee = angle_at(pose.hip, knee, foot_pose.pos);

    // ---- Emit (dead zone, fixed compliance/zeta, rest = target) ----
    out.hip_joint = joint_command(theta_hip, target_hip, cfg.hip_compliance, cfg);
    out.knee_joint = joint_command(theta_knee, target_knee, cfg.knee_compliance, cfg);

    out
}

// ---------------------------------------------------------------------------
// Math helpers (ported from the reference; all pure)
// ---------------------------------------------------------------------------

/// Joint angle at the pivot: θ = φ₁ − φ₂, i.e. `atan2` of the basis returned
/// by [`angle_basis`] (matches the solver's measurement).
#[inline]
fn angle_at(p0: Vec2, pivot: Vec2, p2: Vec2) -> f32 {
    match angle_basis(p0, pivot, p2) {
        Some((c, s)) => s.atan2(c),
        None => 0.0,
    }
}

/// Mirror of the solver's angle measurement: `v1 = pivot−p0`, `v2 = p2−pivot`,
/// `(cos, sin) = (u1·u2, u1.y*u2.x − u1.x*u2.y)`. `None` on degenerate bones.
#[inline]
fn angle_basis(p0: Vec2, pivot: Vec2, p2: Vec2) -> Option<(f32, f32)> {
    let v1 = pivot - p0;
    let v2 = p2 - pivot;
    let l1 = v1.length();
    let l2 = v2.length();
    if l1 < 1e-6 || l2 < 1e-6 {
        return None;
    }
    let u1 = v1 / l1;
    let u2 = v2 / l2;
    Some((u1.dot(u2), u1.y * u2.x - u1.x * u2.y))
}

/// Wrap to (−π, π].
#[inline]
fn wrap_angle(a: f32) -> f32 {
    (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

/// Pack one joint's **target** angle as the constraint rest, with the fixed
/// compliance/damping.
///
/// No interpolation: `rest` is what the pose asked for. `current` is used only
/// for the dead zone (already at the target ⇒ emit nothing).
fn joint_command(
    current: f32,
    target: f32,
    compliance: f32,
    cfg: &ActuatorConfig,
) -> Option<AngularCommand> {
    let err = wrap_angle(target - current).abs();
    if err <= ANGLE_EPSILON {
        return None;
    }
    let (s, c) = target.sin_cos();
    Some(AngularCommand {
        rest_cos: c,
        rest_sin: s,
        compliance,
        zeta: cfg.zeta,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const L: f32 = 55.0; // v10 thigh = calf

    /// A leg built **from a pose** so its live geometry is IK-consistent:
    /// `knee` is the anterior IK solution for `hip → foot`. With `thigh=calf=L`.
    fn leg_from_pose(hip: Vec2, foot: Vec2, facing: f32) -> LegLive {
        let (knee_raw, _) = walker_math::solve_leg_ik(hip, foot, L, L, 1.0);
        let knee = walker_math::select_anterior_knee(hip, foot, knee_raw, facing);
        LegLive {
            waist: hip + Vec2::new(0.0, -40.0), // straight above hip (upright stick)
            hip,
            knee,
            foot,
        }
    }

    /// v10-scale standing leg in y-down: waist y=-180, hip y=-140, straight
    /// column to foot y=-30 (full extension).
    fn standing_leg(x: f32) -> LegLive {
        LegLive {
            waist: Vec2::new(x, -180.0),
            hip: Vec2::new(x, -140.0),
            knee: Vec2::new(x, -85.0),
            foot: Vec2::new(x, -30.0),
        }
    }

    fn pose_with(left: FootPose, right: FootPose, upper_angle: f32) -> Pose {
        Pose {
            hip: Vec2::new(0.0, -140.0),
            left_foot: left,
            right_foot: right,
            facing: 1.0,
            upper_angle,
        }
    }

    #[test]
    fn commands_are_emitted_for_a_moving_pose() {
        // Right foot stepping forward ⇒ its knee/hip targets move.
        let cfg = ActuatorConfig::default();
        let l = standing_leg(-20.0);
        let r = standing_leg(20.0);
        let pose = pose_with(
            FootPose {
                pos: Vec2::new(-20.0, -30.0),
                stance: false,
            },
            FootPose {
                pos: Vec2::new(60.0, -60.0),
                stance: true,
            },
            0.0,
        );
        let a = actuate_tick(&l, &r, &pose, &cfg, 1.0 / 90.0);
        assert!(
            a.right.knee_joint.is_some(),
            "stepping leg must get a knee command"
        );
        assert!(
            a.right.hip_joint.is_some(),
            "stepping leg must get a hip command"
        );
        assert_eq!(a.left.strut.unwrap().compliance, cfg.strut_idle);
        assert_eq!(a.right.strut.unwrap().compliance, cfg.strut_active);
        assert!(
            (a.right.strut.unwrap().rest_length - (pose.hip - pose.right_foot.pos).length()).abs()
                < 1e-4
        );
    }

    #[test]
    fn compliance_and_zeta_are_fixed_config_values() {
        // Simplification law: every emitted angular command carries exactly
        // the configured compliance and zeta — no ω-schedule, no projection.
        let cfg = ActuatorConfig {
            hip_compliance: 3.3e-7,
            knee_compliance: 9.1e-7,
            zeta: 0.8,
            ..Default::default()
        };
        let l = standing_leg(-20.0);
        let r = standing_leg(20.0);
        let pose = pose_with(
            FootPose {
                pos: Vec2::new(-20.0, -30.0),
                stance: true,
            },
            FootPose {
                pos: Vec2::new(60.0, -90.0),
                stance: true,
            },
            0.2,
        );
        let a = actuate_tick(&l, &r, &pose, &cfg, 1.0 / 90.0);
        // Hip and knee carry their OWN compliance (the blueprint's two values
        // differ by a decade); both carry the one shared ζ.
        for (cmd, want) in [
            (a.left.hip_joint, cfg.hip_compliance),
            (a.left.knee_joint, cfg.knee_compliance),
            (a.right.hip_joint, cfg.hip_compliance),
            (a.right.knee_joint, cfg.knee_compliance),
        ]
        .into_iter()
        .filter_map(|(cmd, want)| cmd.map(|cmd| (cmd, want)))
        {
            assert_eq!(cmd.compliance, want);
            assert_eq!(cmd.zeta, cfg.zeta);
        }
    }

    #[test]
    fn strut_rest_length_uses_the_commanded_pose_not_live_parts() {
        // Live hip drifts 10 px behind the pose ⇒ rest length follows the pose.
        let cfg = ActuatorConfig::default();
        let mut l = standing_leg(0.0);
        l.hip = l.hip + Vec2::X * 10.0;
        let r = standing_leg(0.0);
        let pose = pose_with(
            FootPose {
                pos: Vec2::new(0.0, -30.0),
                stance: true,
            },
            FootPose {
                pos: Vec2::new(0.0, -30.0),
                stance: false,
            },
            0.0,
        );
        let a = actuate_tick(&l, &r, &pose, &cfg, 1.0 / 90.0);
        // pose hip (0,-140) → foot (0,-30): length 110 exactly.
        assert!((a.left.strut.unwrap().rest_length - 110.0).abs() < 1e-3);
    }

    #[test]
    fn dead_zone_silences_an_ik_consistent_rest_pose() {
        // A live leg whose geometry is exactly the IK solution of the commanded
        // pose, upright, emits nothing (current == target at every joint).
        let cfg = ActuatorConfig::default();
        let hip = Vec2::new(0.0, -140.0);
        let foot = Vec2::new(0.0, -60.0); // dist 80, well inside reach 2L=110
        let l = leg_from_pose(hip, foot, 1.0);
        let r = l;
        let pose = pose_with(
            FootPose {
                pos: foot,
                stance: true,
            },
            FootPose {
                pos: foot,
                stance: false,
            },
            0.0,
        );
        let a = actuate_tick(&l, &r, &pose, &cfg, 1.0 / 90.0);
        assert!(
            a.left.hip_joint.is_none() && a.left.knee_joint.is_none(),
            "IK-consistent upright leg must be in the dead zone"
        );
    }

    #[test]
    fn lean_tilts_the_hip_target_forward() {
        // A positive upper_angle aims the `P3 → P2` reference forward: with the
        // leg otherwise in an IK-consistent rest pose, that opens a hip error.
        let cfg = ActuatorConfig::default();
        let hip = Vec2::new(0.0, -140.0);
        let foot = Vec2::new(0.0, -60.0);
        let l = leg_from_pose(hip, foot, 1.0);
        let r = l;
        let pose = pose_with(
            FootPose {
                pos: foot,
                stance: true,
            },
            FootPose {
                pos: foot,
                stance: true,
            },
            0.15,
        );
        let a = actuate_tick(&l, &r, &pose, &cfg, 1.0 / 90.0);
        // Loop 3 arrives at the HIP joint (no waist joint exists on this model).
        let cmd = a.right.hip_joint.expect("lean must emit a hip command");
        let theta = cmd.rest_sin.atan2(cmd.rest_cos);
        assert!(theta.abs() > 1e-3, "rest angle must carry the lean");
    }

    #[test]
    fn loop3_has_no_waist_joint() {
        // Structural: the command set carries loop 3 in the two leg hip joints,
        // never a separate `P1-P2-P3` actuator.
        let cfg = ActuatorConfig::default();
        let l = standing_leg(-20.0);
        let r = standing_leg(20.0);
        let pose = pose_with(
            FootPose {
                pos: Vec2::new(-20.0, -30.0),
                stance: true,
            },
            FootPose {
                pos: Vec2::new(60.0, -90.0),
                stance: true,
            },
            0.2,
        );
        let a = actuate_tick(&l, &r, &pose, &cfg, 1.0 / 90.0);
        assert!(a.left.hip_joint.is_some() && a.right.hip_joint.is_some());
    }

    #[test]
    fn degenerate_thigh_yields_no_angular_but_still_strut() {
        let cfg = ActuatorConfig::default();
        let mut r = standing_leg(20.0);
        r.knee = r.hip; // zero-length thigh
        let pose = pose_with(
            FootPose {
                pos: Vec2::new(-20.0, -30.0),
                stance: false,
            },
            FootPose {
                pos: Vec2::new(60.0, -60.0),
                stance: true,
            },
            0.0,
        );
        let a = actuate_tick(&standing_leg(-20.0), &r, &pose, &cfg, 1.0 / 90.0);
        assert!(a.right.hip_joint.is_none() && a.right.knee_joint.is_none());
        // The strut is hip↔foot: it does not need a knee and must still be emitted.
        assert!(a.right.strut.is_some());
    }

    #[test]
    fn facing_mirrors_the_selected_knee() {
        // Foot straight below the hip on a symmetric bent pose: the two IK
        // mirrors have opposite knee x, and with the live knee ON the chord
        // the emitted hip-joint rest basis mirrors too (equal cos, opposite sin).
        let cfg = ActuatorConfig::default();
        let hip = Vec2::new(0.0, -140.0);
        let foot = Vec2::new(0.0, -100.0); // 40 below hip, reachable, bent knee
        let live = LegLive {
            waist: hip + Vec2::new(0.0, -40.0),
            hip,
            knee: hip + Vec2::new(0.0, L), // on the chord: neither mirror wins by x
            foot: hip + Vec2::new(0.0, 110.0),
        };
        let mk = |facing: f32| {
            actuate_tick(
                &live,
                &live,
                &Pose {
                    hip,
                    left_foot: FootPose {
                        pos: foot,
                        stance: true,
                    },
                    right_foot: FootPose {
                        pos: foot,
                        stance: true,
                    },
                    facing,
                    upper_angle: 0.0,
                },
                &cfg,
                1.0 / 90.0,
            )
        };
        let a = mk(1.0);
        let b = mk(-1.0);
        let ka = a.right.hip_joint.expect("hip command facing +1");
        let kb = b.right.hip_joint.expect("hip command facing -1");
        assert!((ka.rest_cos - kb.rest_cos).abs() < 1e-3, "cos must match");
        assert!(
            (ka.rest_sin + kb.rest_sin).abs() < 1e-3,
            "sin must mirror: {} vs {}",
            ka.rest_sin,
            kb.rest_sin
        );
    }

    #[test]
    fn commands_stay_finite_for_extreme_poses() {
        let cfg = ActuatorConfig::default();
        let l = standing_leg(-20.0);
        let r = standing_leg(20.0);
        let pose = pose_with(
            FootPose {
                pos: Vec2::new(-20.0, -30.0),
                stance: false,
            },
            FootPose {
                pos: Vec2::new(-400.0, 1000.0),
                stance: true,
            }, // absurd
            3.1,
        );
        let a = actuate_tick(&l, &r, &pose, &cfg, 1.0 / 90.0);
        for cmd in [
            a.left.hip_joint,
            a.left.knee_joint,
            a.right.hip_joint,
            a.right.knee_joint,
        ]
        .into_iter()
        .flatten()
        {
            assert!(cmd.rest_cos.is_finite() && cmd.rest_sin.is_finite());
            assert!(cmd.compliance.is_finite() && cmd.zeta.is_finite());
        }
        assert!(a.right.strut.unwrap().rest_length.is_finite());
    }

    #[test]
    fn rest_is_the_target_angle_not_a_blend_toward_it() {
        // The emitted rest must be the IK target angle itself. A 50 % blend
        // (the retired `chase` behaviour) would land halfway between the live
        // angle and the target, which is what let the stance sag.
        let cfg = ActuatorConfig::default();
        let hip = Vec2::new(0.0, -140.0);
        let foot = Vec2::new(0.0, -60.0);
        let l = leg_from_pose(hip, foot, 1.0);
        let target_foot = Vec2::new(60.0, -60.0);
        let pose = pose_with(
            FootPose {
                pos: foot,
                stance: true,
            },
            FootPose {
                pos: target_foot,
                stance: true,
            },
            0.0,
        );
        let a = actuate_tick(&l, &l, &pose, &cfg, 1.0 / 90.0);
        let c = a.right.knee_joint.expect("target differs from current");
        let cmd = c.rest_sin.atan2(c.rest_cos);
        let (knee, _) = walker_math::solve_leg_ik(hip, target_foot, L, L, 1.0);
        let knee = walker_math::select_anterior_knee(hip, target_foot, knee, 1.0);
        let want = angle_at(hip, knee, target_foot);
        assert!(
            wrap_angle(cmd - want).abs() < 1e-4,
            "rest must equal the target angle: {cmd} vs {want}"
        );
        // And it must differ from the live angle by the full error, not half.
        let cur = angle_at(l.hip, l.knee, l.foot);
        let half = wrap_angle(cur + 0.5 * wrap_angle(want - cur) - cmd).abs();
        assert!(half > 1e-3, "rest is a blend, not the target");
    }

    #[test]
    fn finite_output_for_finite_input_never_panics() {
        // Sweep lean and foot targets through a wide grid; nothing panics and
        // every emitted basis is unit-length.
        let cfg = ActuatorConfig::default();
        let l = standing_leg(-20.0);
        let r = standing_leg(20.0);
        for i in 0..40 {
            let lean = (i as f32 - 20.0) * 0.08;
            let fx = (i as f32 - 20.0) * 6.0;
            let pose = pose_with(
                FootPose {
                    pos: Vec2::new(-20.0, -30.0),
                    stance: true,
                },
                FootPose {
                    pos: Vec2::new(fx, -30.0 - (i % 7) as f32 * 12.0),
                    stance: true,
                },
                lean,
            );
            let a = actuate_tick(&l, &r, &pose, &cfg, 1.0 / 90.0);
            for cmd in [
                a.left.hip_joint,
                a.left.knee_joint,
                a.right.hip_joint,
                a.right.knee_joint,
            ]
            .into_iter()
            .flatten()
            {
                let norm = (cmd.rest_cos * cmd.rest_cos + cmd.rest_sin * cmd.rest_sin).sqrt();
                assert!(
                    (norm - 1.0).abs() < 1e-3,
                    "rest basis must stay unit-length: {norm}"
                );
            }
        }
    }
}
