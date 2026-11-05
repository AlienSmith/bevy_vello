//! The **control** module — fiction + command → [`ControlOut`] (plan §1).
//!
//! This module is the *only* place the walking algorithm lives. It is split
//! into:
//!
//! - **Contracts** ([`Command`], [`ControlOut`], [`LocoMode`]) — data-only,
//!   no ECS. The pose itself is [`crate::actuate::Pose`]; control builds it
//!   and wraps it in [`ControlOut`] with the non-actuation fields (`hip_vel`,
//!   `mode`, `debug`).
//! - **[`LocoState`]** — the state machine whose *only* job is deciding which
//!   controller is active. It reads command + fiction and nothing else.
//! - **[`Controller`]** — an enum of three controllers sharing one slot:
//!   [`StandingCtrl`] (implemented), [`WalkingCtrl`] (the `walker_math` seam,
//!   currently a documented hold-pose stub), [`BoostStartCtrl`] (ditto, plan
//!   §5).
//!
//! # Law 2 — controller state is schedule + reference only
//! Every *body* field the controller needs arrives in [`Fiction`] (measured by
//! observe). The controller may integrate only what physics cannot measure:
//! the `v_ref` ramp, foot-placement plans, FSM timers. There is no
//! `hip_pos += v·dt` here, by construction — there is nowhere to write it.
//!
//! # Frame
//! Vello **y-down**, same as [`crate::observe`] and [`crate::actuate`]. The
//! `walker_math` seam (y-up) flips at the stamp; nothing else does.
//!
//! # The standing controller is deliberately a dummy
//! The default pose *is* standing, so standing commands essentially "keep
//! doing what you are doing, only balanced": hold the measured feet, hold the
//! measured hip height, drift the hip horizontally toward the foot midpoint at
//! a capped rate, and command an upright attitude. Emitted constraint targets
//! then land inside [`crate::actuate`]'s dead zone whenever the character
//! already stands balanced — zero actuation ticks, which is the correct
//! behaviour for a body at rest.

use glam::Vec2;
use walker_math::{DebugInfo, LowerBodyConfig, ResolverState};

use crate::actuate::{FootPose, Pose};
use crate::observe::Fiction;

// ---------------------------------------------------------------------------
// Contracts
// ---------------------------------------------------------------------------

/// Player/script intent, written by the input system. Units are px/s and
/// Vello y-down. A `Command` is a **held move** (a script cell's duration);
/// "no command" is `None` in [`crate::walker::Walker::command`], which is what
/// ends a run and returns the machine to standing.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    /// Commanded horizontal direction, unit-ish (`+x` = travel right).
    pub dir: Vec2,
    /// Commanded speed magnitude, px/s. `0` = stop / stay put.
    pub speed: f32,
    /// This move is a **launch**: start it through [`LocoMode::BoostStart`]
    /// instead of entering the gait directly. The boost kicks the body up to
    /// speed; the machine hands over to [`LocoMode::Walking`] once the
    /// *measured* velocity clears [`ControlConfig::boost_exit_speed`].
    pub boost: bool,
}

impl Command {
    /// Convenience: a pure horizontal move command (no launch boost).
    pub fn walk(speed: f32) -> Self {
        let dir = Vec2::new(speed.signum(), 0.0);
        Self {
            dir,
            speed,
            boost: false,
        }
    }

    /// Convenience: a boosted launch move — the kick start of the gait.
    pub fn boost(speed: f32) -> Self {
        let dir = Vec2::new(speed.signum(), 0.0);
        Self {
            dir,
            speed,
            boost: true,
        }
    }

    /// Signed horizontal command speed (`dir.x · speed`).
    pub fn vx(&self) -> f32 {
        self.dir.x * self.speed
    }

    /// True when this command actually asks for locomotion: a nonzero signed
    /// speed above `threshold`. A command below it counts as *ended* — the
    /// machine treats it exactly like no command (return to standing).
    pub fn is_move(&self, threshold: f32) -> bool {
        self.vx().abs() > threshold
    }
}

/// The coarse locomotion mode. Deliberately **three** entries: one per
/// controller. Finer phases (`GaitPhase`, `VerticalPhase`) belong to the
/// controller that owns them; they are not modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocoMode {
    /// Balanced quiet stance.
    Standing,
    /// The gait schedule (Raibert / deadbeat DCM).
    Walking,
    /// The swappable launch used to start moving (`plans/walker-control-rewrite.md` §5).
    BoostStart,
}

impl Default for LocoMode {
    /// A fresh character stands. This also lets [`crate::telemetry::TickRecord`]
    /// derive `Default` without a hand-written initializer.
    fn default() -> Self {
        LocoMode::Standing
    }
}

/// One control tick's output.
///
/// The **pose is a field, not a copy**. Control produces [`Pose`] — actuate's
/// own contract type, with no second struct mirroring it — and wraps it with
/// the three values actuate must never see because they are not actuation:
///
/// | field | consumer |
/// |---|---|
/// | [`pose`](Self::pose) | [`crate::actuate::actuate_tick`] (the only input it reads) |
/// | `hip_vel` | telemetry (`cmd_vx`, `tP3vx`) and prediction bookkeeping |
/// | `mode` | diagnostics; [`LocoState::mode`] is the authority |
/// | `debug` | CSV passthrough (plan §3b); `None` outside the gait controller |
///
/// Keeping `Pose` as the actuate layer's own type is deliberate: the bottom
/// layer stays self-contained and knows nothing about commands, modes, or
/// telemetry, while control cannot silently feed it extra fields because the
/// only path is `out.pose`.
///
/// Note the pose has **no torso-tip field**. The old architecture aimed a
/// *position* there (the spine's external position constraint); this one
/// commands the stick's **orientation** as an angle through the leg hip
/// joints (`P40-P3-P2` / `P41-P3-P2`), so a torso point would be write-only.
/// Loop-3's pendulum model still needs a torso length, but that lives in
/// [`walker_math::LowerBodyConfig::torso_len`] — a balance-law parameter, not
/// an actuation target.
#[derive(Debug, Clone, Copy)]
pub struct ControlOut {
    /// Exactly what actuate consumes this tick.
    pub pose: Pose,
    /// Root velocity target (telemetry + prediction).
    pub hip_vel: Vec2,
    /// Which controller produced this.
    pub mode: LocoMode,
    /// Walking-internals telemetry; `None` outside the gait controller.
    pub debug: Option<DebugInfo>,
}

/// Tuning for the control layer. Everything here is a *decision* knob;
/// solver hardware (compliance/ζ) lives in [`crate::actuate::ActuatorConfig`],
/// and rig geometry (bone lengths, torso length, hip height, floor) lives in
/// [`walker_math::LowerBodyConfig`] — seeded once from the rig at spawn, never
/// guessed here.
#[derive(Debug, Clone, Copy)]
pub struct ControlConfig {
    /// Max horizontal hip drift toward the foot midpoint while standing, px/s.
    pub stand_settle_rate: f32,
    /// Speed magnitude (px/s) below which a move command means "stop".
    pub move_threshold: f32,
    /// Measured speed above which the boost hands over to the gait controller,
    /// px/s (plan §7 gate 4: 90 px/s in the boot window).
    pub boost_exit_speed: f32,
    /// Hard cap on the boost window, seconds (fallback if physics never
    /// accelerates — the boost cannot fake its exit, so it must time out).
    pub boost_timeout: f32,
    /// Boost: rate the **right foot's** x-offset slides back toward under-hip,
    /// px/s — the "skate push" that drags the hip forward. Only this one foot
    /// moves; the left foot stays on its standing rest offset.
    pub boost_slide_speed: f32,
    /// Boost: how far (px) the right foot's x-offset slides toward under-hip
    /// before it stops. The foot only ever moves CLOSER to the hip, so the leg
    /// stays within reach and never detaches.
    pub boost_slide_limit: f32,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            stand_settle_rate: 40.0,
            move_threshold: 5.0,
            boost_exit_speed: 90.0,
            boost_timeout: 1.0,
            boost_slide_speed: 260.0,
            // Slide the right foot all the way to directly under the hip
            // (its rest x-offset at scale 0.5 is +39.5).
            boost_slide_limit: 39.5,
        }
    }
}

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

/// A pose that commands exactly what physics currently has: measured hip,
/// measured feet, measured attitude. Emitted targets collide with the live
/// pose inside actuate's dead zone, so a hold is genuinely inert.
///
/// Used by the two unimplemented controllers so the wiring, telemetry, and
/// mode machine can be exercised end-to-end before their algorithms land.
#[inline]
pub fn hold_pose(
    fiction: &Fiction,
    _cfg: &ControlConfig,
    mode: LocoMode,
    facing: f32,
) -> ControlOut {
    ControlOut {
        pose: Pose {
            hip: fiction.hip,
            left_foot: FootPose {
                pos: fiction.feet[0].pos,
                stance: fiction.feet[0].touching,
            },
            right_foot: FootPose {
                pos: fiction.feet[1].pos,
                stance: fiction.feet[1].touching,
            },
            facing,
            upper_angle: fiction.lean,
        },
        hip_vel: fiction.v_hip,
        mode,
        debug: None,
    }
}

// ---------------------------------------------------------------------------
// Controllers
// ---------------------------------------------------------------------------

/// The controller contract. One call per fixed tick.
///
/// `facing` is the character's travel orientation (`+1` = `+x`), a spawn-time
/// constant rather than command state — but it is one of the five pose elements
/// control produces, so it is passed here instead of being patched in later.
pub trait Step {
    fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut;
    fn mode(&self) -> LocoMode;
}

/// Balanced quiet stance: the **v10 rest pose**, hardcoded.
///
/// # The pose is translation-invariant, so only offsets matter
/// The actuate layer never reads a world position: it consumes only the
/// hip→foot chord (IK knee), the hip→knee→foot angle (knee joint), and the
/// `P3→P2` axis rotated by `upper_angle` (hip joint). The hip particle is free
/// (no position constraint). So `pose.hip` can be anything — moving it in world
/// space changes *nothing*. What actuates the legs is the **shape**: the two
/// hip→foot offset vectors and the lean.
///
/// That is why this controller feeds **no physics feedback**: every value it
/// emits is a constant offset from the blueprint's rest pose. Reading the
/// measured hip or feet here would close a loop and let a sag walk away.
///
/// # The constants
/// Taken from `v10.character.json` (SVG units, then × 0.5 spawn scale), all
/// measured from `P3` (the hip):
///
/// | particle | SVG offset from `P3` | scale-0.5 px |
/// |---|---|---|
/// | `P2` (waist ref) | `( 0.0, −70.0)` | `( 0.0, −35.0)` |
/// | `P40` (L knee) | `(−33.0, 106.0)` | `(−16.5, 53.0)` |
/// | `P41` (R knee) | `( 39.0, 103.0)` | `( 19.5, 51.5)` |
/// | `PLLL` (L foot) | `(−66.0, 212.0)` | `(−33.0, 106.0)` |
/// | `PRLL` (R foot) | `( 79.0, 206.0)` | `( 39.5, 103.0)` |
///
/// Note the rest leg is **fully extended** (hip→foot ≈ 111 px ≈
/// `thigh + calf`), not a bent-knee drop pose: that is what the rig stands on
/// with no actuation at all. Commanding any shorter chord asks the knee to fold
/// against two rigid distance bones, which is what collapsed the stance.
#[derive(Debug, Clone, Copy)]
pub struct StandingCtrl {
    /// Seconds spent standing (telemetry; also lets later hysteresis gates read
    /// a settle time).
    pub since: f32,
    /// Hardcoded `P3 → PLLL` offset, px, vello y-down. Left foot.
    left_foot_offset: Vec2,
    /// Hardcoded `P3 → PRLL` offset, px, vello y-down. Right foot.
    right_foot_offset: Vec2,
}

/// v10 rest offsets under `P3`, px at spawn scale 0.5 (y-down: +y is below).
const REST_LEFT_FOOT: Vec2 = Vec2::new(-33.0, 106.0);
const REST_RIGHT_FOOT: Vec2 = Vec2::new(39.5, 103.0);

impl StandingCtrl {
    pub fn new(_lower: &walker_math::LowerBodyConfig) -> Self {
        Self {
            since: 0.0,
            left_foot_offset: REST_LEFT_FOOT,
            right_foot_offset: REST_RIGHT_FOOT,
        }
    }
}

impl Step for StandingCtrl {
    fn step(
        &mut self,
        _cmd: &Command,
        fiction: &Fiction,
        _cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        if dt > 0.0 {
            self.since += dt;
        }
        // Anchor the frame at the measured hip so the *world* pose is right, but
        // emit only hardcoded offsets: the shape is constant, so the emitted
        // joint rests coincide with the v10 rest pose and land in actuate's dead
        // zone. The body then stands on its own rigid bones, exactly as it does
        // with no actuation at all.
        let hip = fiction.hip;
        let left_foot = hip + self.left_foot_offset;
        let right_foot = hip + self.right_foot_offset;

        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: left_foot,
                    stance: fiction.feet[0].touching,
                },
                right_foot: FootPose {
                    pos: right_foot,
                    stance: fiction.feet[1].touching,
                },
                facing,
                // The rest pose's `P3→P2` axis is straight up (offset
                // `(0, −35)`), so the attitude command is upright.
                upper_angle: 0.0,
            },
            hip_vel: Vec2::ZERO,
            mode: LocoMode::Standing,
            debug: None,
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::Standing
    }
}

/// The gait controller (`walker_math::ResolverState` behind the seam).
///
/// **Stub status**: the state is constructed (so the seam's types are proven
/// real) and `step` currently returns [`hold_pose`]. Filling in the body is
/// the next build step (plan §8.3); nothing else in this crate changes when it
/// lands.
#[derive(Debug, Clone)]
pub struct WalkingCtrl {
    pub state: ResolverState,
}

impl WalkingCtrl {
    pub fn new(cfg: &LowerBodyConfig) -> Self {
        Self {
            state: ResolverState::new(cfg),
        }
    }
}

impl Step for WalkingCtrl {
    fn step(
        &mut self,
        _cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        _dt: f32,
        facing: f32,
    ) -> ControlOut {
        // TODO(walk-seam): stamp fiction body fields, call
        // `walker_math::step_lower_body`, map the pose back to y-down.
        hold_pose(fiction, cfg, LocoMode::Walking, facing)
    }

    fn mode(&self) -> LocoMode {
        LocoMode::Walking
    }
}

/// The swappable launch (plan §5) — the stand→walk bridge.
///
/// # The idea: one particle moves, the IK solves the rest
/// The deadbeat gait needs COM velocity at handover; standing has zero. So the
/// boost is a **skate push** in the simplest possible frame: exactly like
/// [`StandingCtrl`] it anchors at the *measured* hip and holds the v10 rest
/// geometry, but **only the right foot's x-offset moves** — it slides toward
/// `0` (directly under the hip) at `boost_slide_speed`, clamped to
/// `boost_slide_limit` px, y held. The left foot keeps its standing rest offset
/// untouched, so that leg can never leave reach; the right foot only ever moves
/// *closer* to the hip, so it never detaches either. Actuate's IK derives the
/// right knee from that single moving target, and the trailing leg pressing
/// under the hip is what drives the body forward. Whether it propels without
/// toppling is what the stand-then-boost QA script measures.
#[derive(Debug, Clone, Copy)]
pub struct BoostStartCtrl {
    pub since: f32,
}

impl BoostStartCtrl {
    pub fn new() -> Self {
        Self { since: 0.0 }
    }
}

impl Default for BoostStartCtrl {
    fn default() -> Self {
        Self::new()
    }
}

impl Step for BoostStartCtrl {
    fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        self.since += dt.max(0.0);
        // Travel sign: command direction if moving, else facing (a boost
        // always carries a nonzero vx through the machine's entry gate).
        let s = if cmd.vx() != 0.0 {
            cmd.vx().signum()
        } else if facing < 0.0 {
            -1.0
        } else {
            1.0
        };
        // One moving particle: only the RIGHT foot's x-offset slides toward
        // under-hip (x -> 0), y held. It only ever closes the gap to the hip,
        // so it stays within reach. The left foot keeps its standing rest
        // offset untouched; the IK solves the right knee from the new chord.
        let slide = (self.since * cfg.boost_slide_speed).min(cfg.boost_slide_limit);
        let rest_right = REST_RIGHT_FOOT * Vec2::new(s, 1.0); // mirror for -x travel
        let right_offset = rest_right - Vec2::new(s * slide, 0.0);

        let hip = fiction.hip;
        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: hip + REST_LEFT_FOOT,
                    stance: fiction.feet[0].touching,
                },
                right_foot: FootPose {
                    pos: hip + right_offset,
                    stance: fiction.feet[1].touching,
                },
                facing,
                upper_angle: 0.0,
            },
            hip_vel: Vec2::new(s * cmd.speed.abs().min(cfg.boost_exit_speed * 1.5), 0.0),
            mode: LocoMode::BoostStart,
            debug: None,
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::BoostStart
    }
}

/// The three controllers, one shared slot (the component field is this enum).
#[derive(Debug, Clone)]
pub enum Controller {
    Standing(StandingCtrl),
    Walking(WalkingCtrl),
    BoostStart(BoostStartCtrl),
}

impl Controller {
    /// Build the controller for a mode. `lower` supplies the gait config.
    pub fn for_mode(mode: LocoMode, lower: &LowerBodyConfig) -> Self {
        match mode {
            LocoMode::Standing => Controller::Standing(StandingCtrl::new(lower)),
            LocoMode::Walking => Controller::Walking(WalkingCtrl::new(lower)),
            LocoMode::BoostStart => Controller::BoostStart(BoostStartCtrl::new()),
        }
    }

    /// Dispatch through the enum; one tick of control.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        match self {
            Controller::Standing(c) => c.step(cmd, fiction, cfg, dt, facing),
            Controller::Walking(c) => c.step(cmd, fiction, cfg, dt, facing),
            Controller::BoostStart(c) => c.step(cmd, fiction, cfg, dt, facing),
        }
    }

    pub fn mode(&self) -> LocoMode {
        match self {
            Controller::Standing(c) => c.mode(),
            Controller::Walking(c) => c.mode(),
            Controller::BoostStart(c) => c.mode(),
        }
    }
}

impl Default for Controller {
    fn default() -> Self {
        Controller::Standing(StandingCtrl::new(&LowerBodyConfig::default()))
    }
}

// ---------------------------------------------------------------------------
// State machine: decides only WHICH controller runs
// ---------------------------------------------------------------------------

/// The mode machine. It owns no gait state — only the current mode and how
/// long it has held; it reads nothing but the command and the fiction.
///
/// Three states, one per controller:
///
/// ```text
///               move + boost flag
///  Standing ────────────────────────► BoostStart
///    ▲  ▲                                  │  measured speed ≥ boost_exit_speed
///    │  │  command ended (None / stop)     │  or boost_timeout
///    │  │                                  ▼
///    │  └──────────── Walking ◄───────── (handover)
///    │                 ▲   │
///    └─────────────────┘   └── command ended → Standing
///               plain move (no boost) goes Standing → Walking directly
/// ```
///
/// The boost exit reads **fiction** (`v_hip.x`, measured), so the launch cannot
/// fake its own completion (plan §5). Command end — `None`, or a command whose
/// signed speed is under [`ControlConfig::move_threshold`] — always returns to
/// Standing from any mode.
#[derive(Debug, Clone, Copy)]
pub struct LocoState {
    pub mode: LocoMode,
    /// Seconds in the current mode.
    pub since: f32,
}

impl Default for LocoState {
    fn default() -> Self {
        Self {
            mode: LocoMode::Standing,
            since: 0.0,
        }
    }
}

impl LocoState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Decide the mode for this tick.
    pub fn decide(
        &mut self,
        cmd: Option<Command>,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
    ) -> LocoMode {
        self.since += dt.max(0.0);
        // A held move command; anything else counts as no command at all.
        let active = cmd.filter(|c| c.is_move(cfg.move_threshold));

        let next = match (self.mode, active) {
            // Command ended (or never started): stand, from any mode.
            (_, None) => LocoMode::Standing,
            // Launch: a boosted move kicks the gait through BoostStart.
            (LocoMode::Standing, Some(c)) if c.boost => LocoMode::BoostStart,
            // A plain move enters the gait directly (already up to speed).
            (LocoMode::Standing, Some(_)) => LocoMode::Walking,
            (LocoMode::BoostStart, Some(_)) => {
                // Exit on measured speed, with a timeout backstop so a stuck
                // physics can never hold the launch mode forever.
                let boosted = fiction.v_hip.x.abs() >= cfg.boost_exit_speed;
                let expired = self.since >= cfg.boost_timeout;
                if boosted || expired {
                    LocoMode::Walking
                } else {
                    LocoMode::BoostStart
                }
            }
            (LocoMode::Walking, Some(_)) => LocoMode::Walking,
        };

        if next != self.mode {
            self.mode = next;
            self.since = 0.0;
        }
        self.mode
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observe::FootState;

    fn fiction(hip: Vec2, feet: [Vec2; 2], touching: [bool; 2], lean: f32, v: Vec2) -> Fiction {
        Fiction {
            dt: 1.0 / 90.0,
            hip,
            v_hip: v,
            lean,
            lean_rate: 0.0,
            com: hip,
            v_com: v,
            feet: [
                FootState {
                    pos: feet[0],
                    vel: Vec2::ZERO,
                    touching: touching[0],
                },
                FootState {
                    pos: feet[1],
                    vel: Vec2::ZERO,
                    touching: touching[1],
                },
            ],
            grounded: touching[0] || touching[1],
        }
    }

    fn centered() -> Fiction {
        // Feet at ±STANCE_SPREAD, exactly on the x-stance: standing holds x and
        // only lifts the hip toward the stand height.
        fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-10.0, -90.0), Vec2::new(10.0, -90.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        )
    }

    #[test]
    fn standing_emits_the_hardcoded_v10_rest_offsets() {
        // The frame is the v10 rest pose: constant offsets under the measured
        // `P3`, identical whatever the measured feet do and whether grounded.
        // This is what makes the emitted joint rests land in actuate's dead
        // zone — no feedback, no sag walk-away.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        for touching in [[true, true], [false, false]] {
            let f = fiction(
                Vec2::new(3.0, -120.0),
                [Vec2::new(-45.0, -11.0), Vec2::new(12.0, -80.0)],
                touching,
                0.0,
                Vec2::new(11.0, -7.0),
            );
            let mut c = StandingCtrl::new(&walker_math::LowerBodyConfig::default());
            let out = c.step(&Command::default(), &f, &cfg, dt, 1.0);
            assert_eq!(out.mode, LocoMode::Standing);
            assert_eq!(
                out.pose.hip, f.hip,
                "anchor: measured hip (world placement)"
            );
            assert_eq!(
                out.pose.left_foot.pos,
                f.hip + REST_LEFT_FOOT,
                "left foot is the hardcoded P3→PLLL offset"
            );
            assert_eq!(
                out.pose.right_foot.pos,
                f.hip + REST_RIGHT_FOOT,
                "right foot is the hardcoded P3→PRLL offset"
            );
            assert_eq!(out.pose.left_foot.stance, touching[0]);
            assert_eq!(out.pose.right_foot.stance, touching[1]);
            assert_eq!(out.pose.upper_angle, 0.0, "the rest P3→P2 axis is upright");
            assert_eq!(out.pose.facing, 1.0);
            assert!(out.debug.is_none(), "no gait internals in standing");
        }
    }

    #[test]
    fn the_rest_offsets_match_the_v10_blueprint_at_scale_half() {
        // Regression guard on the hardcoded numbers: blueprint SVG offsets
        // scaled by the 0.5 spawn factor.
        assert_eq!(REST_LEFT_FOOT, Vec2::new(-33.0, 106.0));
        assert_eq!(REST_RIGHT_FOOT, Vec2::new(39.5, 103.0));
    }

    #[test]
    fn control_out_carries_the_pose_and_the_actuator_never_sees_the_rest() {
        // `pose` is a field, not a copy: the ControlOut IS what actuate reads,
        // and hip_vel/mode/debug sit beside it where actuate cannot reach them.
        let out = ControlOut {
            pose: Pose {
                hip: Vec2::new(4.0, -140.0),
                left_foot: FootPose {
                    pos: Vec2::new(-10.0, -90.0),
                    stance: true,
                },
                right_foot: FootPose {
                    pos: Vec2::new(10.0, -90.0),
                    stance: false,
                },
                facing: 1.0,
                upper_angle: 0.1,
            },
            hip_vel: Vec2::new(50.0, 0.0),
            mode: LocoMode::Standing,
            debug: None,
        };
        assert_eq!(out.pose.hip, Vec2::new(4.0, -140.0));
        assert_eq!(out.pose.left_foot.pos, Vec2::new(-10.0, -90.0));
        assert!(out.pose.left_foot.stance);
        assert!(!out.pose.right_foot.stance);
        assert_eq!(out.pose.facing, 1.0);
        assert_eq!(out.pose.upper_angle, 0.1);
        assert_eq!(
            out.hip_vel.x, 50.0,
            "velocity is beside the pose, not in it"
        );
    }

    #[test]
    fn hold_pose_stubs_are_inert_and_carry_their_mode() {
        let cfg = ControlConfig::default();
        let f = centered();
        let out = hold_pose(&f, &cfg, LocoMode::Walking, 1.0);
        assert_eq!(out.mode, LocoMode::Walking);
        assert_eq!(out.pose.hip, f.hip);
        assert_eq!(
            out.pose.upper_angle, f.lean,
            "hold must not invent an attitude"
        );
        assert_eq!(out.pose.facing, 1.0);
    }

    #[test]
    fn state_machine_starts_standing_and_stays_without_a_command() {
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let f = centered();
        for _ in 0..50 {
            let m = st.decide(None, &f, &cfg, 1.0 / 90.0);
            assert_eq!(m, LocoMode::Standing);
        }
        assert!(st.since > 0.5, "since accumulates while standing");
    }

    #[test]
    fn a_plain_move_starts_the_gait_directly() {
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let f = centered();
        let m = st.decide(Some(Command::walk(120.0)), &f, &cfg, 1.0 / 90.0);
        assert_eq!(m, LocoMode::Walking, "no boost flag ⇒ no launch window");
    }

    #[test]
    fn a_boosted_move_launches_through_booststart_and_exits_on_measured_speed() {
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let rest = centered();
        let m = st.decide(Some(Command::boost(120.0)), &rest, &cfg, 1.0 / 90.0);
        assert_eq!(m, LocoMode::BoostStart);

        // Still at rest: the launch continues (fiction says nothing yet).
        let dt = 1.0 / 90.0;
        for _ in 0..20 {
            let m = st.decide(Some(Command::boost(120.0)), &rest, &cfg, dt);
            assert_eq!(m, LocoMode::BoostStart);
        }

        // The body MEASURABLY reaches the exit speed: hand over to the gait.
        let moving = fiction(
            rest.hip,
            [Vec2::ZERO, Vec2::ZERO],
            [true, true],
            0.0,
            Vec2::new(cfg.boost_exit_speed + 1.0, 0.0),
        );
        let m = st.decide(Some(Command::boost(120.0)), &moving, &cfg, dt);
        assert_eq!(m, LocoMode::Walking, "boost exits on measured speed");
    }

    #[test]
    fn the_boost_times_out_when_physics_never_delivers() {
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let f = centered();
        st.decide(Some(Command::boost(120.0)), &f, &cfg, 1.0 / 90.0);
        // Hold the launch past `boost_timeout` without gaining any speed.
        let mut m = LocoMode::BoostStart;
        let ticks = (cfg.boost_timeout * 90.0) as usize + 2;
        for _ in 0..ticks {
            m = st.decide(Some(Command::boost(120.0)), &f, &cfg, 1.0 / 90.0);
        }
        assert_eq!(m, LocoMode::Walking, "timeout is the backstop exit");
    }

    #[test]
    fn command_end_returns_any_mode_to_standing() {
        let cfg = ControlConfig::default();
        let f = centered();
        // From Walking: command gone ⇒ Standing.
        let mut st = LocoState::new();
        st.decide(Some(Command::walk(120.0)), &f, &cfg, 1.0 / 90.0);
        assert_eq!(st.mode, LocoMode::Walking);
        assert_eq!(st.decide(None, &f, &cfg, 1.0 / 90.0), LocoMode::Standing);

        // A sub-threshold command counts as ended, not held.
        let mut st = LocoState::new();
        st.decide(Some(Command::walk(120.0)), &f, &cfg, 1.0 / 90.0);
        let stop = Command {
            dir: Vec2::X,
            speed: cfg.move_threshold * 0.5,
            boost: false,
        };
        assert_eq!(
            st.decide(Some(stop), &f, &cfg, 1.0 / 90.0),
            LocoMode::Standing,
            "speed under the threshold = command ended"
        );
    }

    #[test]
    fn mode_entry_resets_since() {
        let cfg = ControlConfig::default();
        let mut st = LocoState::new();
        let f = centered();
        for _ in 0..10 {
            st.decide(None, &f, &cfg, 1.0 / 90.0);
        }
        st.decide(Some(Command::boost(120.0)), &f, &cfg, 1.0 / 90.0);
        assert!(
            st.since < 0.02,
            "entering BoostStart restarts the timeout clock"
        );
    }

    #[test]
    fn controller_slot_swaps_with_the_mode_and_dispatches() {
        let low = LowerBodyConfig::default();
        let cfg = ControlConfig::default();
        let mut c = Controller::for_mode(LocoMode::Standing, &low);
        assert_eq!(c.mode(), LocoMode::Standing);
        let f = centered();
        let out = c.step(&Command::default(), &f, &cfg, 1.0 / 90.0, 1.0);
        assert_eq!(out.mode, LocoMode::Standing);

        // Swapping the slot is just a reassignment — the enum shares one field.
        c = Controller::for_mode(LocoMode::BoostStart, &low);
        assert_eq!(c.mode(), LocoMode::BoostStart);
        let out = c.step(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0, 1.0);
        assert_eq!(out.mode, LocoMode::BoostStart);

        c = Controller::for_mode(LocoMode::Walking, &low);
        assert_eq!(c.mode(), LocoMode::Walking);
        let out = c.step(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0, 1.0);
        assert_eq!(out.mode, LocoMode::Walking);
    }

    #[test]
    fn boost_holds_left_rest_offset_and_slides_right_toward_under_hip() {
        // One moving particle: the left foot is EXACTLY the standing rest
        // offset (never touched, so it cannot detach); only the right foot's
        // x-offset closes toward the hip, y held.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = BoostStartCtrl::new();
        let out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        assert_eq!(out.mode, LocoMode::BoostStart);
        assert_eq!(out.pose.hip, f.hip, "frame anchored at measured hip");
        // Left foot pinned to the rest offset, independent of the slide.
        assert_eq!(
            out.pose.left_foot.pos,
            f.hip + REST_LEFT_FOOT,
            "left foot = standing rest offset, untouched"
        );
        // Right foot: x slides toward the hip by one dt's worth, y = rest.
        let slide1 = dt * cfg.boost_slide_speed;
        assert_eq!(
            out.pose.right_foot.pos,
            f.hip + Vec2::new(REST_RIGHT_FOOT.x - slide1, REST_RIGHT_FOOT.y),
            "right foot x closes toward under-hip, y held"
        );
        assert_eq!(out.pose.upper_angle, 0.0, "simple boost: no lean");

        // Long run: the right foot stops at boost_slide_limit (directly under
        // the hip when limit == rest x) and never crosses to the far side.
        let ticks = ((cfg.boost_slide_limit / cfg.boost_slide_speed) / dt) as usize + 10;
        for _ in 0..ticks {
            c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        }
        let out = c.step(&Command::boost(120.0), &f, &cfg, dt, 1.0);
        assert!(
            (out.pose.right_foot.pos.x - (f.hip.x + REST_RIGHT_FOOT.x - cfg.boost_slide_limit))
                .abs()
                < 1e-3,
            "right slide stops at the limit"
        );
        assert!(
            (out.pose.right_foot.pos.y - (f.hip.y + REST_RIGHT_FOOT.y)).abs() < 1e-3,
            "right slide is horizontal only"
        );
    }

    #[test]
    fn boost_uses_the_command_travel_sign() {
        // A -x boost mirrors the right foot to the -x rest offset and closes it
        // toward the hip; the left foot still pins to its standing rest offset.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = BoostStartCtrl::new();
        let west = Command {
            dir: Vec2::new(-1.0, 0.0),
            speed: 120.0,
            boost: true,
        };
        let out = c.step(&west, &f, &cfg, dt, 1.0);
        let slide1 = dt * cfg.boost_slide_speed;
        assert_eq!(
            out.pose.left_foot.pos,
            f.hip + REST_LEFT_FOOT,
            "left foot is the rest offset for any travel sign"
        );
        assert!(
            out.pose.right_foot.pos.x < f.hip.x,
            "traveling -x ⇒ right foot on the -x side"
        );
        assert!(
            (out.pose.right_foot.pos.x - (f.hip.x + (-REST_RIGHT_FOOT.x + slide1))).abs() < 1e-3,
            "right foot closes toward under-hip from the -x rest offset"
        );
    }

    #[test]
    fn control_is_deterministic_for_identical_input_streams() {
        let cfg = ControlConfig::default();
        let low = LowerBodyConfig::default();
        let run = || {
            let mut c = Controller::for_mode(LocoMode::Standing, &low);
            let mut st = LocoState::new();
            let mut outs = Vec::new();
            for i in 0..40 {
                let v = Vec2::new(i as f32, 0.0);
                let f = fiction(
                    Vec2::new(i as f32, -140.0),
                    [Vec2::new(-20.0, -90.0), Vec2::new(20.0, -90.0)],
                    [i % 4 != 0, i % 5 != 0],
                    0.01 * i as f32,
                    v,
                );
                let cmd = if i % 3 == 0 {
                    None
                } else if i < 20 {
                    Some(Command::boost(100.0))
                } else {
                    Some(Command::walk(100.0))
                };
                let mode = st.decide(cmd, &f, &cfg, 1.0 / 90.0);
                if mode != c.mode() {
                    c = Controller::for_mode(mode, &low);
                }
                let out = c.step(&cmd.unwrap_or_default(), &f, &cfg, 1.0 / 90.0, 1.0);
                outs.push((
                    out.mode,
                    (out.pose.hip.x * 1000.0).round() as i64,
                    (out.pose.upper_angle * 1000.0).round() as i64,
                ));
            }
            outs
        };
        let a = run();
        let b = run();
        assert_eq!(a, b, "same inputs ⇒ same outputs (D1)");
    }
}
