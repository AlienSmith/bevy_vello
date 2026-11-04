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
/// Vello y-down; `jump` is **edge-triggered** by the input stage (a held key
/// must not re-arm a hop on every landing).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    /// Commanded horizontal direction, unit-ish (`+x` = travel right).
    pub dir: Vec2,
    /// Commanded speed magnitude, px/s. `0` = stop / stay put.
    pub speed: f32,
    /// One-tick jump request.
    pub jump: bool,
}

impl Command {
    /// Convenience: a pure horizontal move command.
    pub fn walk(speed: f32) -> Self {
        let dir = Vec2::new(speed.signum(), 0.0);
        Self {
            dir,
            speed,
            jump: false,
        }
    }

    /// Signed horizontal command speed (`dir.x · speed`).
    pub fn vx(&self) -> f32 {
        self.dir.x * self.speed
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
    /// Wiring switch for the walking controller. `false` until
    /// [`WalkingCtrl`]'s `step` body is implemented; the state machine still
    /// computes the transition, it just cannot be entered.
    pub walk_ready: bool,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            stand_settle_rate: 40.0,
            move_threshold: 5.0,
            boost_exit_speed: 90.0,
            boost_timeout: 1.0,
            walk_ready: false,
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

/// Balanced quiet stance. Holds the measured stance and drifts the hip toward
/// the foot midpoint; commands an upright attitude.
#[derive(Debug, Clone, Copy, Default)]
pub struct StandingCtrl {
    /// Seconds spent standing (telemetry; also lets later hysteresis gates read
    /// a settle time).
    pub since: f32,
}

impl StandingCtrl {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Step for StandingCtrl {
    fn step(
        &mut self,
        _cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        if dt > 0.0 {
            self.since += dt;
        }

        // Horizontal target: the support midpoint when both feet bear load,
        // otherwise hold. The drift is rate-limited so a widened stance cannot
        // yank the hip across in one tick.
        let both_stance = fiction.feet[0].touching && fiction.feet[1].touching;
        let target_x = if both_stance {
            let mid = (fiction.feet[0].pos.x + fiction.feet[1].pos.x) * 0.5;
            let max_step = cfg.stand_settle_rate.max(0.0) * dt.max(0.0);
            let delta = mid - fiction.hip.x;
            if max_step > 0.0 {
                fiction.hip.x + delta.clamp(-max_step, max_step)
            } else {
                fiction.hip.x
            }
        } else {
            // One foot (or none) bearing load: the midpoint is not a support,
            // so hold exactly. Any real single-support case is the gait
            // controller's job, not standing's.
            fiction.hip.x
        };

        // Height is held: the vertical loop does not exist in standing (the
        // ground reaction already holds the body up; there is nothing to
        // integrate and nothing to command).
        let hip = Vec2::new(target_x, fiction.hip.y);

        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: fiction.feet[0].pos,
                    stance: fiction.feet[0].touching,
                },
                right_foot: FootPose {
                    pos: fiction.feet[1].pos,
                    stance: fiction.feet[1].touching,
                },
                facing,
                // Upright: the reference pose. Loop 3 delivers this through the
                // leg hip joints (actuate), never through the waist.
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

/// The swappable launch (plan §5).
///
/// **Stub status**: returns [`hold_pose`] until its accel/lean command lands.
/// Its *exit* rule is wired in [`LocoState`] and reads **measured** fiction
/// velocity — the boost cannot fake its own completion, which is the whole
/// reason it is a control-module swap rather than a position kick.
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
        _cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        self.since += dt.max(0.0);
        hold_pose(fiction, cfg, LocoMode::BoostStart, facing)
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
            LocoMode::Standing => Controller::Standing(StandingCtrl::new()),
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
        Controller::Standing(StandingCtrl::new())
    }
}

// ---------------------------------------------------------------------------
// State machine: decides only WHICH controller runs
// ---------------------------------------------------------------------------

/// The mode machine. It owns no gait state — only the current mode, how
/// long it has held, and a launch-assist latch; it reads nothing but the
/// command and the fiction.
#[derive(Debug, Clone, Copy)]
pub struct LocoState {
    pub mode: LocoMode,
    /// Seconds in the current mode.
    pub since: f32,
    /// Latch set when a boost window expires without reaching
    /// `boost_exit_speed`. While the move command stays held, boosts are not
    /// retried — otherwise the timeout would re-arm every tick and the machine
    /// would flap `Standing ↔ BoostStart` once per frame (`since` resets on
    /// every re-entry, so the timeout could never take effect). The latch
    /// clears when the command drops below `move_threshold`, so a fresh launch
    /// request gets a fresh assist.
    pub boost_failed: bool,
}

impl Default for LocoState {
    fn default() -> Self {
        Self {
            mode: LocoMode::Standing,
            since: 0.0,
            boost_failed: false,
        }
    }
}

impl LocoState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Decide the mode for this tick, updating `since` on change. Pure: same
    /// inputs ⇒ same mode, which keeps the whole loop deterministic (D1).
    ///
    /// Transitions (all read command + measured fiction only):
    /// - `Standing → Walking` when a move is commanded, the body is supported,
    ///   `walk_ready`, and either it already moves at `boost_exit_speed` or a
    ///   boost attempt already failed (`boost_failed`: the gait controller is
    ///   the real locomotion, so a failed launch assist must not forbid walking).
    /// - `Standing → BoostStart` when a move is commanded, supported,
    ///   `walk_ready`, slower than `boost_exit_speed`, and no boost has failed
    ///   while the command stayed held.
    /// - `BoostStart → Walking` when measured speed reaches `boost_exit_speed`
    ///   (and `walk_ready`), or on timeout with `boost_failed` latched;
    ///   `BoostStart → Standing` if the command was withdrawn.
    /// - `Walking → Standing` when the command drops below `move_threshold`.
    pub fn decide(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
    ) -> LocoMode {
        self.since += dt.max(0.0);

        let vx = fiction.v_hip.x.abs();
        let moving = cmd.vx().abs() > cfg.move_threshold;
        if !moving {
            // A withdrawn command re-arms the launch assist.
            self.boost_failed = false;
        }
        let next = match self.mode {
            LocoMode::Standing => {
                if moving && cfg.walk_ready && fiction.grounded {
                    if vx >= cfg.boost_exit_speed || self.boost_failed {
                        LocoMode::Walking
                    } else {
                        LocoMode::BoostStart
                    }
                } else {
                    LocoMode::Standing
                }
            }
            LocoMode::BoostStart => {
                if !moving {
                    LocoMode::Standing
                } else if cfg.walk_ready && vx >= cfg.boost_exit_speed {
                    LocoMode::Walking
                } else if self.since >= cfg.boost_timeout.max(0.0) {
                    // Failed launch: latch (no re-arm while held) and hand the
                    // body to the gait controller — the real locomotion.
                    self.boost_failed = true;
                    if cfg.walk_ready {
                        LocoMode::Walking
                    } else {
                        LocoMode::Standing
                    }
                } else {
                    LocoMode::BoostStart
                }
            }
            LocoMode::Walking => {
                if moving {
                    LocoMode::Walking
                } else {
                    LocoMode::Standing
                }
            }
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
        fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-20.0, -90.0), Vec2::new(20.0, -90.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        )
    }

    #[test]
    fn standing_on_a_centered_pose_commands_the_measured_pose() {
        // The dummy contract: targets equal reality ⇒ actuate emits nothing.
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        let f = centered();
        let mut c = StandingCtrl::new();
        let out = c.step(&Command::default(), &f, &cfg, dt, 1.0);

        assert_eq!(out.mode, LocoMode::Standing);
        assert_eq!(out.pose.hip, f.hip, "hip is already at the foot midpoint");
        assert_eq!(out.pose.left_foot.pos, f.feet[0].pos);
        assert_eq!(out.pose.right_foot.pos, f.feet[1].pos);
        assert!(out.pose.left_foot.stance && out.pose.right_foot.stance);
        assert_eq!(out.pose.upper_angle, 0.0, "standing commands upright");
        assert_eq!(out.pose.facing, 1.0, "facing is passed through untouched");
        assert!(out.debug.is_none(), "no gait internals in standing");
    }

    #[test]
    fn standing_drifts_the_hip_toward_the_foot_midpoint_rate_limited() {
        let cfg = ControlConfig {
            stand_settle_rate: 40.0,
            ..ControlConfig::default()
        };
        let dt = 1.0 / 90.0;
        // Hip sits 30 px to the left of the support midpoint.
        let f = fiction(
            Vec2::new(-30.0, -140.0),
            [Vec2::new(-10.0, -90.0), Vec2::new(10.0, -90.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let mut c = StandingCtrl::new();
        let out = c.step(&Command::default(), &f, &cfg, dt, 1.0);
        let expected_step = 40.0 * dt;
        assert!(
            (out.pose.hip.x - (-30.0 + expected_step)).abs() < 1e-4,
            "hip must step exactly rate·dt toward mid, got {}",
            out.pose.hip.x
        );
        assert_eq!(out.pose.hip.y, f.hip.y, "height is held, never integrated");
    }

    #[test]
    fn standing_with_one_foot_only_holds_instead_of_drifting() {
        let cfg = ControlConfig::default();
        let dt = 1.0 / 90.0;
        let f = fiction(
            Vec2::new(-30.0, -140.0),
            [Vec2::new(-10.0, -90.0), Vec2::new(10.0, -60.0)],
            [true, false],
            0.0,
            Vec2::ZERO,
        );
        let mut c = StandingCtrl::new();
        let out = c.step(&Command::default(), &f, &cfg, dt, 1.0);
        assert_eq!(out.pose.hip, f.hip, "no slab midpoint without two supports");
        assert!(!out.pose.right_foot.stance, "swing foot is not a strut");
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
            let m = st.decide(&Command::default(), &f, &cfg, 1.0 / 90.0);
            assert_eq!(m, LocoMode::Standing);
        }
        assert!(st.since > 0.5, "since accumulates while standing");
    }

    #[test]
    fn a_move_from_rest_enters_boost_start_when_walking_is_ready() {
        let cfg = ControlConfig {
            walk_ready: true,
            ..ControlConfig::default()
        };
        let mut st = LocoState::new();
        let f = centered();
        let m = st.decide(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0);
        assert_eq!(
            m,
            LocoMode::BoostStart,
            "a launch from rest goes through boost"
        );
        assert_eq!(st.since, 0.0, "entering a mode resets the timer");
    }

    #[test]
    fn walking_is_not_entered_while_walk_ready_is_false() {
        // The walking controller is a stub; the machine must not hand over to
        // it yet. This flag disappears the moment `WalkingCtrl::step` lands.
        let cfg = ControlConfig::default();
        assert!(!cfg.walk_ready);
        let mut st = LocoState::new();
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-20.0, -90.0), Vec2::new(20.0, -90.0)],
            [true, true],
            0.0,
            Vec2::new(120.0, 0.0),
        );
        for _ in 0..10 {
            let m = st.decide(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0);
            assert_eq!(m, LocoMode::Standing);
        }
    }

    #[test]
    fn boost_exits_on_measured_speed_not_on_a_timer() {
        let cfg = ControlConfig {
            walk_ready: true,
            boost_exit_speed: 90.0,
            boost_timeout: 10.0,
            ..ControlConfig::default()
        };
        let mut st = LocoState::new();
        // Enter boost.
        let slow = centered();
        assert_eq!(
            st.decide(&Command::walk(120.0), &slow, &cfg, 1.0 / 90.0),
            LocoMode::BoostStart
        );
        // Still slow, still in boost.
        let m = st.decide(&Command::walk(120.0), &slow, &cfg, 1.0 / 90.0);
        assert_eq!(m, LocoMode::BoostStart);
        // The body actually accelerated ⇒ hand over.
        let fast = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-20.0, -90.0), Vec2::new(20.0, -90.0)],
            [true, true],
            0.0,
            Vec2::new(95.0, 0.0),
        );
        assert_eq!(
            st.decide(&Command::walk(120.0), &fast, &cfg, 1.0 / 90.0),
            LocoMode::Walking,
            "exit must read the measured velocity the boost cannot fake"
        );
    }

    #[test]
    fn a_failed_launch_hands_off_to_the_gait_and_does_not_flap() {
        // The timeout must not return to Standing: with the command still held
        // and the speed still low, Standing would re-enter BoostStart on the
        // very next tick — an infinite one-tick flap, and `since` resets on
        // every re-entry, so the timeout could never take effect at all.
        let cfg = ControlConfig {
            walk_ready: true,
            boost_timeout: 0.05,
            ..ControlConfig::default()
        };
        let mut st = LocoState::new();
        let f = centered();
        let cmd = Command::walk(120.0);
        let first = st.decide(&cmd, &f, &cfg, 1.0 / 90.0);
        assert_eq!(first, LocoMode::BoostStart);

        let mut modes = Vec::new();
        for _ in 0..20 {
            modes.push(st.decide(&cmd, &f, &cfg, 1.0 / 90.0));
        }
        assert_eq!(
            modes[modes.len() - 1],
            LocoMode::Walking,
            "a failed launch hands the body to the gait controller"
        );
        assert!(
            !modes.contains(&LocoMode::Standing),
            "a held command must not flap Standing↔BoostStart, got {modes:?}"
        );
        assert!(st.boost_failed, "the latch stops the boost re-arming");
    }

    #[test]
    fn withdrawing_then_reissuing_the_command_re_arms_the_launch_assist() {
        let cfg = ControlConfig {
            walk_ready: true,
            boost_timeout: 0.05,
            ..ControlConfig::default()
        };
        let mut st = LocoState::new();
        let f = centered();
        let cmd = Command::walk(120.0);
        st.decide(&cmd, &f, &cfg, 1.0 / 90.0);
        for _ in 0..10 {
            st.decide(&cmd, &f, &cfg, 1.0 / 90.0);
        }
        assert!(st.boost_failed);

        // Release: the latch clears.
        st.decide(&Command::default(), &f, &cfg, 1.0 / 90.0);
        assert!(!st.boost_failed, "a released command re-arms the assist");

        // Press again: a fresh launch gets a fresh assist.
        let m = st.decide(&cmd, &f, &cfg, 1.0 / 90.0);
        assert_eq!(m, LocoMode::BoostStart);
    }

    #[test]
    fn withdrawing_the_command_drops_out_of_boost() {
        let cfg = ControlConfig {
            walk_ready: true,
            ..ControlConfig::default()
        };
        let mut st = LocoState::new();
        let f = centered();
        st.decide(&Command::walk(120.0), &f, &cfg, 1.0 / 90.0);
        let m = st.decide(&Command::default(), &f, &cfg, 1.0 / 90.0);
        assert_eq!(m, LocoMode::Standing);
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
    fn control_is_deterministic_for_identical_input_streams() {
        let cfg = ControlConfig {
            walk_ready: true,
            ..ControlConfig::default()
        };
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
                    Command::default()
                } else {
                    Command::walk(100.0)
                };
                let mode = st.decide(&cmd, &f, &cfg, 1.0 / 90.0);
                if mode != c.mode() {
                    c = Controller::for_mode(mode, &low);
                }
                let out = c.step(&cmd, &f, &cfg, 1.0 / 90.0, 1.0);
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
