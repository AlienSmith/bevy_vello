//! The jump & landing reward: **+1 on completion, 0 otherwise.**
//!
//! The authored expert already performs the jump; the residual's two jobs are to
//! avoid CoM rotation at liftoff and to survive the landing. The reward says only
//! that: an episode that reaches the end of the recovery window without tipping
//! over pays `r_survive` once, everything else pays nothing.
//!
//! Dense per-tick terms (upright, spin, momentum, a launch receipt) were tried and
//! removed: a stochastic policy exploits its own noise through them, and the
//! deterministic policy degrades while sampled reward rises.
//!
//! ```text
//!   Launch   (feet on the ground, reward 0)
//!            ── both feet off, after the expert's release ─▶ Flight
//!            ── launch window closed and still grounded ─▶ LaunchMissed
//!   Flight   ── FIRST ground contact ─▶ Recovery
//!            ── contact before `min_flight` seconds ─▶ HopTooSmall
//!   Recovery ── `recovery_window` seconds without tipping ─▶ +r_survive, Completed
//! ```
//!
//! Any phase ends as `TiltedOver` when `|lean| > fall_angle`. Failures pay 0.
//!
//! [`Fiction`] is Vello y-down; `tilt` is `fiction.lean`.

use walker_control::observe::Fiction;

use std::f32::consts::FRAC_PI_3;

/// Consecutive airborne ticks needed to confirm liftoff **after** the expert's
/// power stroke releases. The contact channel reads "no contact" for the first
/// few ticks after a respawn, which used to open the window before the jump;
/// gating on the release plus 3 ticks (33 ms) cannot delay a real hop.
const LIFTOFF_CONFIRM_TICKS: u32 = 3;

/// How an episode ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Termination {
    /// Still running.
    None,
    /// The recovery window elapsed without tipping over. Success.
    Completed,
    /// `|lean| > fall_angle`.
    TiltedOver,
    /// The launch window closed and the body never left the floor.
    LaunchMissed,
    /// A non-foot body link touched the floor. Disabled (the live collider set
    /// misreports this while standing); kept so the CSV label stays stable.
    #[allow(dead_code)]
    BodyContact,
    /// Touched down before [`RewardConfig::min_flight`] seconds of flight: a hop
    /// too small to count as the jump (off by default). Pays nothing.
    HopTooSmall,
}

/// One reward tick's product.
#[derive(Debug, Clone, Copy)]
pub struct StepOut {
    pub reward: f32,
    pub events: RewardEvents,
    /// Non-`None` ⇒ the harness must end the episode now.
    pub termination: Termination,
}

/// Milestones the harness logs / branches on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RewardEvents {
    /// Both feet left the floor (Launch → Flight).
    pub lift_off: bool,
    /// First ground contact after liftoff (Flight → Recovery).
    pub touch_down: bool,
    /// The recovery window completed this tick (`+r_survive`).
    pub survived: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct RewardConfig {
    /// Recovery window length, seconds.
    pub recovery_window: f32,
    /// Completion bonus, the only reward.
    pub r_survive: f32,
    /// Torso tilt beyond which the episode is over, radians (60°).
    pub fall_angle: f32,
    /// Grace after the expert's launch window closes before a still-grounded
    /// body is declared a launch miss, seconds.
    pub launch_miss_grace: f32,
    /// Minimum flight time, seconds; `0` = off. Measured as Flight-stage ticks x
    /// dt (liftoff is confirmed over [`LIFTOFF_CONFIRM_TICKS`] first, so real
    /// airtime is ~3 ticks longer).
    pub min_flight: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self {
            recovery_window: 1.5,
            r_survive: 1.0,
            fall_angle: FRAC_PI_3,
            launch_miss_grace: 0.20,
            min_flight: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Launch,
    Flight,
    Recovery,
}

/// Per-character reward state.
#[derive(Debug, Clone)]
pub struct JumpReward {
    cfg: RewardConfig,
    phase: Phase,
    /// Seconds spent still grounded after the expert's launch window closed.
    launch_miss_t: f32,
    /// Consecutive airborne ticks since the launch window closed.
    airborne_ticks: u32,
    /// Seconds elapsed inside the Recovery window.
    recovery_t: f32,
    // ---- introspection (logged, never acted on) ----
    pub launch_ticks: u32,
    pub flight_ticks: u32,
    pub recovery_ticks: u32,
    /// The last non-`None` termination this episode produced (for the CSV).
    pub term_reason: Option<Termination>,
}

impl JumpReward {
    pub fn new(cfg: RewardConfig) -> Self {
        Self {
            cfg,
            phase: Phase::Launch,
            launch_miss_t: 0.0,
            airborne_ticks: 0,
            recovery_t: 0.0,
            launch_ticks: 0,
            flight_ticks: 0,
            recovery_ticks: 0,
            term_reason: None,
        }
    }

    /// True once liftoff has fired this episode.
    pub fn launched(&self) -> bool {
        self.phase != Phase::Launch
    }

    /// Reset at an episode boundary (respawn).
    pub fn reset(&mut self) {
        *self = Self::new(self.cfg);
    }

    /// One tick. `launch_window_over` is the expert's signal that the power
    /// stroke has been released; it is only consulted in Launch.
    pub fn step(&mut self, f: &Fiction, launch_window_over: bool) -> StepOut {
        let mut events = RewardEvents::default();

        if f.lean.abs() > self.cfg.fall_angle {
            return self.finish(0.0, events, Termination::TiltedOver);
        }

        match self.phase {
            Phase::Launch => {
                self.launch_ticks += 1;
                if launch_window_over {
                    if !f.grounded {
                        self.airborne_ticks += 1;
                    } else {
                        self.airborne_ticks = 0;
                        self.launch_miss_t += f.dt;
                    }
                    if self.airborne_ticks >= LIFTOFF_CONFIRM_TICKS {
                        self.phase = Phase::Flight;
                        events.lift_off = true;
                    } else if self.launch_miss_t >= self.cfg.launch_miss_grace {
                        return self.finish(0.0, events, Termination::LaunchMissed);
                    }
                } else {
                    // Still loading: contact flicker here is noise, not a liftoff.
                    self.airborne_ticks = 0;
                    self.launch_miss_t = 0.0;
                }
            }
            Phase::Flight => {
                self.flight_ticks += 1;
                if f.grounded {
                    if (self.flight_ticks as f32) * f.dt < self.cfg.min_flight {
                        return self.finish(0.0, events, Termination::HopTooSmall);
                    }
                    self.phase = Phase::Recovery;
                    self.recovery_t = 0.0;
                    events.touch_down = true;
                }
            }
            Phase::Recovery => {
                self.recovery_ticks += 1;
                self.recovery_t += f.dt;
                if self.recovery_t >= self.cfg.recovery_window {
                    events.survived = true;
                    return self.finish(self.cfg.r_survive, events, Termination::Completed);
                }
            }
        }

        StepOut {
            reward: 0.0,
            events,
            termination: Termination::None,
        }
    }

    /// Record a termination and return its tick.
    fn finish(&mut self, reward: f32, events: RewardEvents, reason: Termination) -> StepOut {
        self.term_reason = Some(reason);
        StepOut {
            reward,
            events,
            termination: reason,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::prelude::Vec2;
    use walker_control::observe::FootState;

    const SURFACE: f32 = 312.0;
    const STAND_Z: f32 = 104.0; // standing CoM height above the floor
    const DT: f32 = 1.0 / 90.0;

    /// `rise` is how far the CoM is above standing; `v_up` is the upward CoM
    /// velocity; `feet_lift` how far the feet are above the floor (0 = planted).
    fn fiction(
        rise: f32,
        v_up: f32,
        lean: f32,
        omega: f32,
        feet_lift: [f32; 2],
        body_contact: bool,
    ) -> Fiction {
        let touching = [feet_lift[0] <= 0.5, feet_lift[1] <= 0.5];
        Fiction {
            dt: DT,
            hip: Vec2::ZERO,
            v_hip: Vec2::ZERO,
            lean,
            lean_rate: 0.0,
            com: Vec2::new(0.0, SURFACE - STAND_Z - rise),
            v_com: Vec2::new(0.0, -v_up), // vello y-down ⇒ upward is −y
            ang_momentum: 0.0,
            omega_com: omega,
            feet: [
                FootState {
                    pos: Vec2::new(-20.0, SURFACE - feet_lift[0]),
                    vel: Vec2::ZERO,
                    touching: touching[0],
                },
                FootState {
                    pos: Vec2::new(20.0, SURFACE - feet_lift[1]),
                    vel: Vec2::ZERO,
                    touching: touching[1],
                },
            ],
            grounded: touching[0] || touching[1],
            body_contact,
        }
    }

    fn standing() -> Fiction {
        fiction(0.0, 0.0, 0.0, 0.0, [0.0, 0.0], false)
    }

    /// Both feet clear of the floor: the physical liftoff event.
    fn airborne(omega: f32) -> Fiction {
        fiction(STAND_Z * 0.2, 100.0, 0.0, omega, [6.0, 6.0], false)
    }

    /// Run `n` ticks with the launch window **open** (the expert is still
    /// loading), so the launch-miss clock never accumulates.
    fn run(rw: &mut JumpReward, f: &Fiction, n: usize) -> Vec<StepOut> {
        (0..n).map(|_| rw.step(f, false)).collect()
    }

    /// Step the same fiction until liftoff confirms (the window is open), and
    /// return the confirming tick.
    fn confirm_with(rw: &mut JumpReward, f: &Fiction) -> StepOut {
        let mut last = None;
        for _ in 0..LIFTOFF_CONFIRM_TICKS {
            last = Some(rw.step(f, true));
        }
        let out = last.unwrap();
        assert!(out.events.lift_off, "liftoff must confirm after the window");
        out
    }

    /// Open the launch window and confirm a liftoff on the standard airborne pose.
    fn confirm_liftoff(rw: &mut JumpReward, omega: f32) -> StepOut {
        let f = airborne(omega);
        confirm_with(rw, &f)
    }

    /// Sum the rewards of a run.
    fn total(outs: &[StepOut]) -> f32 {
        outs.iter().map(|o| o.reward).sum()
    }


    #[test]
    fn nothing_pays_until_the_window_completes() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        let mut outs = run(&mut rw, &standing(), 50);
        assert!(!rw.launched());
        outs.push(confirm_liftoff(&mut rw, 0.0));
        assert!(rw.launched());
        outs.extend(run(&mut rw, &airborne(0.0), 5));
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        let touch = rw.step(&down, false);
        assert!(touch.events.touch_down);
        outs.push(touch);
        let window_ticks = (cfg.recovery_window / DT).round() as usize;
        outs.extend(run(&mut rw, &down, window_ticks - 2));
        assert_eq!(total(&outs), 0.0, "no per-tick reward anywhere");
        // The tick that reaches the window's end pays once.
        let done = rw.step(&down, false);
        assert!(done.events.survived);
        assert_eq!(done.termination, Termination::Completed);
        assert_eq!(done.reward, cfg.r_survive);
        assert_eq!(rw.recovery_ticks, window_ticks as u32);
        assert!(rw.launch_ticks > 0 && rw.flight_ticks >= 2);
    }

    #[test]
    fn the_recovery_window_is_measured_from_the_first_contact() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        confirm_liftoff(&mut rw, 0.0);
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        assert!(rw.step(&down, false).events.touch_down);
        let window_ticks = (cfg.recovery_window / DT).round() as usize;
        for _ in 0..window_ticks - 1 {
            assert_eq!(rw.step(&down, false).termination, Termination::None);
        }
        assert_eq!(rw.step(&down, false).termination, Termination::Completed);
    }

    #[test]
    fn standing_is_free_and_is_not_a_liftoff() {
        let mut rw = JumpReward::new(RewardConfig::default());
        let outs = run(&mut rw, &standing(), 200);
        assert_eq!(total(&outs), 0.0);
        assert!(!rw.launched());
        assert!(outs.iter().all(|o| !o.events.lift_off));
    }

    #[test]
    fn liftoff_is_feet_leaving_the_ground() {
        let cfg = RewardConfig::default();

        // Feet planted while rising: no liftoff.
        let mut rw = JumpReward::new(cfg);
        let tiptoe = fiction(STAND_Z * 0.3, 100.0, 0.0, 0.0, [0.0, 0.0], false);
        for _ in 0..10 {
            assert!(!rw.step(&tiptoe, true).events.lift_off);
        }

        // One foot clear: `grounded` is the OR of the feet, so no liftoff.
        let mut rw = JumpReward::new(cfg);
        let one = fiction(STAND_Z * 0.2, 100.0, 0.0, 0.0, [6.0, 0.0], false);
        for _ in 0..10 {
            assert!(!rw.step(&one, true).events.lift_off);
        }

        // A single airborne tick is a flicker; it needs LIFTOFF_CONFIRM_TICKS.
        let mut rw = JumpReward::new(cfg);
        let clear = fiction(STAND_Z * 0.2, 0.0, 0.0, 0.0, [6.0, 6.0], false);
        for _ in 0..8 {
            assert!(!rw.step(&clear, true).events.lift_off);
            rw.step(&standing(), true);
        }
        assert!(!rw.launched());

        let mut rw = JumpReward::new(cfg);
        assert!(confirm_with(&mut rw, &clear).events.lift_off);
    }

    #[test]
    fn contact_flicker_before_the_launch_window_is_not_a_liftoff() {
        let mut rw = JumpReward::new(RewardConfig::default());
        for _ in 0..30 {
            let out = rw.step(&airborne(0.0), false);
            assert!(!out.events.lift_off, "phantom liftoff before the release");
        }
        assert!(!rw.launched());
    }

    #[test]
    fn touchdown_is_the_first_ground_contact_after_liftoff() {
        let mut rw = JumpReward::new(RewardConfig::default());
        let climb = fiction(10.0, 60.0, 0.0, 0.0, [6.0, 6.0], false);
        assert!(confirm_with(&mut rw, &climb).events.lift_off);
        let flap = fiction(9.0, 0.0, 0.0, 0.0, [0.0, 0.0], false);
        assert!(rw.step(&flap, false).events.touch_down);
    }

    #[test]
    fn tilt_failure_terminates_and_pays_nothing() {
        let cfg = RewardConfig::default();
        let tipped = fiction(
            STAND_Z * 0.2,
            0.0,
            cfg.fall_angle + 0.05,
            0.0,
            [6.0, 6.0],
            false,
        );

        // In flight: no landing, no payment.
        let mut rw = JumpReward::new(cfg);
        confirm_liftoff(&mut rw, 0.0);
        let out = rw.step(&tipped, false);
        assert_eq!(out.reward, 0.0);
        assert_eq!(out.termination, Termination::TiltedOver);
        assert_eq!(rw.term_reason, Some(Termination::TiltedOver));

        // In recovery: tipping before the window ends forfeits the bonus.
        let mut rw = JumpReward::new(cfg);
        confirm_liftoff(&mut rw, 0.0);
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        rw.step(&down, false);
        rw.step(&down, false);
        let out = rw.step(&tipped, false);
        assert_eq!(out.reward, 0.0);
        assert_eq!(out.termination, Termination::TiltedOver);
    }

    #[test]
    fn launch_miss_terminates_after_the_grace() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        let grace_ticks = (cfg.launch_miss_grace / DT).ceil() as usize;

        for _ in 0..10 {
            assert_eq!(rw.step(&standing(), false).termination, Termination::None);
        }
        for _ in 0..grace_ticks.saturating_sub(2) {
            assert_eq!(rw.step(&standing(), true).termination, Termination::None);
        }
        let mut out = rw.step(&standing(), true);
        for _ in 0..4 {
            if out.termination != Termination::None {
                break;
            }
            out = rw.step(&standing(), true);
        }
        assert_eq!(out.termination, Termination::LaunchMissed);
        assert_eq!(out.reward, 0.0);
        assert!(!rw.launched());
    }

    #[test]
    fn a_real_liftoff_before_the_grace_is_not_a_miss() {
        let mut rw = JumpReward::new(RewardConfig::default());
        for _ in 0..5 {
            assert_eq!(rw.step(&standing(), true).termination, Termination::None);
        }
        let out = confirm_liftoff(&mut rw, 0.0);
        assert!(out.events.lift_off);
        assert_eq!(out.termination, Termination::None);
    }

    #[test]
    fn a_hop_shorter_than_min_flight_is_not_a_jump() {
        let cfg = RewardConfig {
            min_flight: 0.40,
            ..Default::default()
        };
        let mut rw = JumpReward::new(cfg);
        confirm_liftoff(&mut rw, 0.0);
        for _ in 0..10 {
            assert_eq!(rw.step(&airborne(0.0), false).termination, Termination::None);
        }
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        let out = rw.step(&down, false);
        assert_eq!(out.termination, Termination::HopTooSmall);
        assert_eq!(out.reward, 0.0);
        assert_eq!(rw.term_reason, Some(Termination::HopTooSmall));
    }

    #[test]
    fn a_full_hop_passes_min_flight_and_it_is_off_by_default() {
        let cfg = RewardConfig {
            min_flight: 0.40,
            ..Default::default()
        };
        let mut rw = JumpReward::new(cfg);
        confirm_liftoff(&mut rw, 0.0);
        let needed = (cfg.min_flight / DT).ceil() as usize + 2;
        for _ in 0..needed {
            rw.step(&airborne(0.0), false);
        }
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        assert!(rw.step(&down, false).events.touch_down);

        // Default: a 2-tick hop lands normally.
        let mut rw = JumpReward::new(RewardConfig::default());
        confirm_liftoff(&mut rw, 0.0);
        for _ in 0..2 {
            rw.step(&airborne(0.0), false);
        }
        let out = rw.step(&down, false);
        assert!(out.events.touch_down);
        assert_eq!(out.termination, Termination::None);
    }

    #[test]
    fn body_contact_is_ignored() {
        let mut rw = JumpReward::new(RewardConfig::default());
        let out = rw.step(&fiction(0.0, 0.0, 0.0, 0.0, [0.0, 0.0], true), false);
        assert_eq!(out.reward, 0.0);
        assert_eq!(out.termination, Termination::None);
    }

    #[test]
    fn reset_starts_a_fresh_episode() {
        let mut rw = JumpReward::new(RewardConfig::default());
        confirm_liftoff(&mut rw, 0.0);
        assert!(rw.launched());
        rw.reset();
        assert!(!rw.launched());
        assert_eq!(rw.launch_ticks, 0);
        assert_eq!(rw.term_reason, None);
    }
}
