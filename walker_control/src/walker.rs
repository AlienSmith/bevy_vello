//! The **per-character component** plus the two thin stage systems that glue
//! observe → control → actuate (plan §1; component shape per the design
//! session).
//!
//! ```text
//! Walker { config, command, state, observer, controller, actuator, telemetry }
//!             │        │       │       │          │           │        │
//!             │        │       │       │          │           │        └─ per-tick CSV record
//!             │        │       │       │          │           └─ ControlOut mailbox
//!             │        │       │       │          └─ Standing | Walking | BoostStart
//!             │        │       │       └─ Fiction + EMA/debounce (one observer type)
//!             │        │       └─ LocoState: decides ONLY which controller runs
//!             │        └─ Command: written by the input system
//!             └─ WalkerConfig: all tuning, typed
//! ```
//!
//! Ownership rules (each field is written by exactly one stage):
//! - `observe_stage` → `observer`, and `telemetry`'s reality block
//! - `control_stage` → `state.mode`, `controller`, `actuator.out`, and
//!   `telemetry`'s command block
//! - `actuate_stage` → constraint queues, and `telemetry`'s knee columns
//!
//! Entity handles are **not** stored here: the leg and spine controllers on
//! the character entity already own them, and the adapter builds
//! [`LegHandles`]/[`ObserveHandles`] at the call site. This keeps
//! `game_lib → walker_control` a one-way dependency.

use bevy::prelude::*;
use bevy_vello::integrations::physics::{VelloConstraintWorld, VelloParticle};
use walker_math::LowerBodyConfig;

use crate::actuate::{ActuatorConfig, LegLive};
use crate::control::{Command, ControlConfig, ControlOut, Controller, LocoMode, LocoState};
use crate::emit::{drive_legs, LegHandles};
use crate::observe::{Fiction, ObserveConfig, Observer};
use crate::telemetry::TickRecord;

/// One character's tuning, typed (plan Law 4: config values, not env forks).
#[derive(Debug, Clone, Default)]
pub struct WalkerConfig {
    /// Gait math config (`walker_math`). Only the walking controller reads it.
    pub lower: LowerBodyConfig,
    pub control: ControlConfig,
    pub observe: ObserveConfig,
    pub actuator: ActuatorConfig,
}

/// The actuator's mailbox: the last [`ControlOut`] the controller produced.
/// `None` before the first control tick.
///
/// Control produces the pose itself (see [`ControlOut::pose`]); this mailbox
/// only holds it, so there is no per-tick conversion and no chance for the two
/// layers to disagree about what was commanded.
#[derive(Debug, Clone, Copy, Default)]
pub struct Actuator {
    pub out: Option<ControlOut>,
}

impl Actuator {
    /// The pose to actuate, if a control tick has produced one.
    pub fn pose(&self) -> Option<crate::actuate::Pose> {
        self.out.map(|o| o.pose)
    }
}

/// The whole control loop for one character.
#[derive(Component)]
pub struct Walker {
    pub config: WalkerConfig,
    /// Written by the input system (`view.input.command`).
    pub command: Command,
    /// The mode machine — decides only which controller is active.
    pub state: LocoState,
    /// The one observer type (holds the fiction).
    pub observer: Observer,
    /// The three controllers, one shared slot.
    pub controller: Controller,
    /// The one actuator type (holds the ControlOut).
    pub actuator: Actuator,
    /// Character facing (`+1` = `+x`). Set at spawn (from the blueprint's
    /// orientation); not part of the per-tick command.
    pub facing: f32,
    /// This tick's CSV shows. Lives on the component so multiple characters
    /// need no bookkeeping; each block has exactly one writer (see the stage
    /// docs below), and [`crate::telemetry::TickRecord::format`] is the only
    /// formatter.
    pub telemetry: TickRecord,
}

impl Walker {
    pub fn new(config: WalkerConfig, facing: f32) -> Self {
        let controller = Controller::for_mode(LocoMode::Standing, &config.lower);
        Self {
            config,
            command: Command::default(),
            state: LocoState::new(),
            observer: Observer::new(),
            controller,
            actuator: Actuator::default(),
            facing,
            telemetry: TickRecord::default(),
        }
    }

    /// The current fiction snapshot (observe's product). Read-only.
    pub fn fiction(&self) -> &Fiction {
        &self.observer.fiction
    }

    /// The last control output, if any.
    pub fn out(&self) -> Option<&ControlOut> {
        self.actuator.out.as_ref()
    }
}

// ---------------------------------------------------------------------------
// Stage systems
// ---------------------------------------------------------------------------

/// Input → command. One line of translation so the source (keyboard, script
/// playhead, AI) stays outside this crate: callers write `Walker::command`;
/// this helper only documents the edge-trigger rule for `jump`.
///
/// Expected usage from the game/harness input system:
/// ```ignore
/// w.command = Command { dir, speed, jump: jump_edge };
/// ```
pub fn set_command(w: &mut Walker, dir: Vec2, speed: f32, jump_edge: bool) {
    w.command = Command {
        dir,
        speed,
        jump: jump_edge,
    };
}

/// Observe stage (runs first): gather physics reality, filter it, and update
/// the observer's [`Fiction`]. Also the place a telemetry system would read the
/// raw pre-filter row from `walker.observer.raw` (plan §3b).
///
/// Writes `w.observer` and the reality block of `w.telemetry`; reads only the
/// physics world. It opens the tick first, clearing the blocks the other stages
/// are about to fill — so a stalled stage reports zeros, never last tick's
/// values.
pub fn observe_stage(
    w: &mut Walker,
    handles: &crate::observe::ObserveHandles,
    character: Entity,
    p_q: &Query<&VelloParticle>,
    colliders: &Query<&bevy_vello::VelloCollider>,
    batch: Option<&bevy_vello::collision::CollisionEventBatch>,
    dt: f32,
) {
    w.telemetry.begin_tick();
    crate::observe::observe_tick(
        handles,
        character,
        p_q,
        colliders,
        batch,
        &mut w.observer,
        &w.config.observe,
        dt,
    );
    w.telemetry.write_reality(&w.observer);
}

/// Control stage (runs after observe, before actuate): decide the mode, swap
/// the controller if it changed, and produce the tick's [`ControlOut`].
///
/// This is deliberately the *whole* integration point: it is the only place
/// where modes and controllers meet, so a boost→walk handover happens within
/// one tick — no dead pose between the two.
pub fn control_stage(w: &mut Walker) {
    let dt = w.observer.fiction.dt;
    let mode = w
        .state
        .decide(&w.command, &w.observer.fiction, &w.config.control, dt);

    if mode != w.controller.mode() {
        w.controller = Controller::for_mode(mode, &w.config.lower);
    }

    // Borrow split: the controller mutates its own schedule state; fiction,
    // command, and facing are read-only for this call.
    let command = w.command;
    let fiction = w.observer.fiction;
    let facing = w.facing;
    let out = w
        .controller
        .step(&command, &fiction, &w.config.control, dt, facing);
    w.actuator.out = Some(out);
    // The same `out` the actuator will read, so the command block can never
    // describe a different pose than the one being actuated.
    w.telemetry.write_control(&out, &command);
}

/// Actuate stage (runs after control, before the physics chain): turn the
/// [`ControlOut`] into angular + distance constraint events.
///
/// Law 1 lives here: this system may only call `queue_character_angular_constraints`
/// and `queue_character_distance_constraints` (inside [`drive_legs`]).
///
/// It also records the IK knees it aims at, computed from the same live bone
/// lengths the joint commands use — so `tP40`/`tP41` are a byproduct of the
/// real actuation, not a second derivation that could disagree with it.
pub fn actuate_stage(
    w: &mut Walker,
    character: Entity,
    left: &LegHandles,
    right: &LegHandles,
    p_q: &Query<&VelloParticle>,
    world: &mut VelloConstraintWorld,
) {
    let Some(out) = w.actuator.out else {
        return;
    };
    let (l, r) = (live_leg(left, p_q), live_leg(right, p_q));
    w.telemetry.write_knees([
        crate::actuate::ik_knee_target(&out.pose, out.pose.left_foot, &l),
        crate::actuate::ik_knee_target(&out.pose, out.pose.right_foot, &r),
    ]);
    drive_legs(
        character,
        left,
        right,
        p_q,
        &out.pose,
        &w.config.actuator,
        w.observer.fiction.dt,
        world,
    );
}

/// A live leg snapshot for the actuator, or the degenerate fallback (struts
/// only, no angular commands) when a particle is missing.
pub fn live_leg(handles: &LegHandles, p_q: &Query<&VelloParticle>) -> LegLive {
    handles.live(p_q).unwrap_or(LegLive {
        waist: Vec2::ZERO,
        hip: Vec2::ZERO,
        knee: Vec2::ZERO,
        foot: Vec2::ZERO,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observe::FootState;
    use glam::Vec2;
    use walker_math::LowerBodyConfig;

    fn standing_fiction() -> Fiction {
        Fiction {
            dt: 1.0 / 90.0,
            hip: Vec2::new(0.0, -140.0),
            v_hip: Vec2::ZERO,
            lean: 0.0,
            lean_rate: 0.0,
            com: Vec2::new(0.0, -140.0),
            v_com: Vec2::ZERO,
            feet: [
                FootState {
                    pos: Vec2::new(-20.0, -90.0),
                    vel: Vec2::ZERO,
                    touching: true,
                },
                FootState {
                    pos: Vec2::new(20.0, -90.0),
                    vel: Vec2::ZERO,
                    touching: true,
                },
            ],
            grounded: true,
        }
    }

    #[test]
    fn a_new_walker_starts_standing_with_no_output() {
        let w = Walker::new(WalkerConfig::default(), 1.0);
        assert_eq!(w.state.mode, LocoMode::Standing);
        assert_eq!(w.controller.mode(), LocoMode::Standing);
        assert!(w.actuator.out.is_none(), "no control tick has run yet");
        assert!(w.actuator.pose().is_none());
        assert_eq!(w.facing, 1.0);
        assert_eq!(w.command, Command::default());
        assert!(w.out().is_none());
    }

    #[test]
    fn control_stage_produces_one_output_per_tick_and_holds_standing() {
        let mut w = Walker::new(WalkerConfig::default(), 1.0);
        w.observer.fiction = standing_fiction();
        control_stage(&mut w);
        let out = w.out().expect("control must produce an output");
        assert_eq!(out.mode, LocoMode::Standing);
        assert_eq!(out.pose.hip, w.observer.fiction.hip);
        assert_eq!(out.pose.facing, w.facing);
        // A second tick with no command keeps standing; the mailbox is replaced
        // (not accumulated) so state cannot drift between stages.
        control_stage(&mut w);
        assert_eq!(w.out().unwrap().mode, LocoMode::Standing);
    }

    #[test]
    fn control_stage_swaps_the_controller_slot_when_the_mode_changes() {
        let mut cfg = WalkerConfig::default();
        cfg.control.walk_ready = true;
        cfg.control.boost_exit_speed = 1.0e9; // always slow ⇒ boost
        let mut w = Walker::new(cfg, 1.0);
        w.observer.fiction = standing_fiction();
        set_command(&mut w, Vec2::new(1.0, 0.0), 120.0, false);
        control_stage(&mut w);
        assert_eq!(w.state.mode, LocoMode::BoostStart);
        assert_eq!(
            w.controller.mode(),
            LocoMode::BoostStart,
            "slot swapped in-tick"
        );
        assert_eq!(w.out().unwrap().mode, LocoMode::BoostStart);
    }

    #[test]
    fn control_stage_hands_boost_over_to_walking_without_a_dead_tick() {
        let mut cfg = WalkerConfig::default();
        cfg.control.walk_ready = true;
        cfg.control.boost_exit_speed = 90.0;
        cfg.control.boost_timeout = 10.0;
        let mut w = Walker::new(cfg, 1.0);
        w.observer.fiction = standing_fiction();
        set_command(&mut w, Vec2::new(1.0, 0.0), 120.0, false);

        // Rest ⇒ boost.
        control_stage(&mut w);
        assert_eq!(w.controller.mode(), LocoMode::BoostStart);

        // The body accelerates (measured velocity) ⇒ walking in the SAME tick.
        w.observer.fiction.v_hip = Vec2::new(95.0, 0.0);
        control_stage(&mut w);
        assert_eq!(w.state.mode, LocoMode::Walking);
        assert_eq!(w.controller.mode(), LocoMode::Walking);
        assert_eq!(
            w.out().unwrap().mode,
            LocoMode::Walking,
            "the handover happens before control runs, so no tick of dead pose"
        );
    }

    #[test]
    fn withdrawing_the_command_returns_to_standing_and_swaps_back() {
        let mut cfg = WalkerConfig::default();
        cfg.control.walk_ready = true;
        cfg.control.boost_exit_speed = 90.0;
        cfg.control.boost_timeout = 10.0;
        let mut w = Walker::new(cfg, 1.0);
        w.observer.fiction = standing_fiction();
        set_command(&mut w, Vec2::new(1.0, 0.0), 120.0, false);
        w.observer.fiction.v_hip = Vec2::new(95.0, 0.0);
        control_stage(&mut w);
        assert_eq!(w.controller.mode(), LocoMode::Walking);

        set_command(&mut w, Vec2::ZERO, 0.0, false);
        control_stage(&mut w);
        assert_eq!(w.controller.mode(), LocoMode::Standing);
        assert_eq!(w.out().unwrap().mode, LocoMode::Standing);
    }

    #[test]
    fn set_command_replaces_rather_than_accumulates() {
        let mut w = Walker::new(WalkerConfig::default(), 1.0);
        set_command(&mut w, Vec2::new(1.0, 0.0), 120.0, true);
        assert_eq!(w.command.vx(), 120.0);
        set_command(&mut w, Vec2::new(-1.0, 0.0), 80.0, false);
        assert_eq!(w.command.vx(), -80.0, "direction flips cleanly");
        assert!(!w.command.jump, "jump is an edge, not a latch");
    }

    #[test]
    fn the_fiction_snapshot_is_observes_product_not_controls_input() {
        // Law 2: control may not write fiction. The borrow shape enforces it —
        // `fiction()` hands out an immutable reference only.
        let mut w = Walker::new(WalkerConfig::default(), 1.0);
        w.observer.fiction = standing_fiction();
        let before = *w.fiction();
        control_stage(&mut w);
        let after = *w.fiction();
        assert_eq!(before, after, "control never mutates measured reality");
    }

    #[test]
    fn the_stages_fill_their_own_telemetry_blocks() {
        let mut w = Walker::new(WalkerConfig::default(), 1.0);
        w.observer.fiction = standing_fiction();
        w.observer.fiction.dt = 1.0 / 90.0;
        w.observer.raw.lean = 0.05;
        w.observer.raw.lean_rate = 0.02;

        // `begin_tick` + reality: what `observe_stage` does after its physics
        // read. The observer's debounce is still cold, so `contact` is 0 here,
        // but `hit` comes from the fiction and is live.
        w.telemetry.begin_tick();
        w.telemetry.write_reality(&w.observer);
        assert_eq!(w.telemetry.v_filt, 0.0);
        assert_eq!(w.telemetry.body_theta, 0.05);
        assert!(w.telemetry.hit[0] && w.telemetry.hit[1]);

        // Control block: exactly the pose the actuator holds.
        control_stage(&mut w);
        let out = *w.out().expect("control ran");
        assert_eq!(w.telemetry.mode, out.mode);
        assert_eq!(w.telemetry.hip, out.pose.hip);
        assert_eq!(w.telemetry.feet[0], out.pose.left_foot.pos);
        assert_eq!(w.telemetry.stance[0], out.pose.left_foot.stance);

        // Knees are left `None`: only `actuate_stage` writes them, because
        // only it knows the live bone lengths.
        assert_eq!(w.telemetry.knees, [None; 2]);
    }

    #[test]
    fn lower_body_config_reaches_the_walking_controller() {
        let cfg = WalkerConfig {
            lower: LowerBodyConfig {
                target_speed: 150.0,
                ..LowerBodyConfig::default()
            },
            ..WalkerConfig::default()
        };
        let w = Walker::new(cfg, 1.0);
        assert_eq!(w.config.lower.target_speed, 150.0);
    }
}
