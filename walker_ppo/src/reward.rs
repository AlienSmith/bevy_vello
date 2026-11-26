//! The **3-stage jump & landing reward** (`plans/reward_plan.md`), simplified so
//! the expert owns the jump and the residual learns only what it can change.
//!
//! The expert controller already performs a clean, repeatable jump from episode
//! one; it simply cannot land. So the reward must NOT re-gate the launch with
//! velocity / clearance thresholds and it must NOT terminate on a "launch miss":
//! those only kill the expert's own jump. The residual's two real jobs are
//!
//!   1. avoid CoM rotation at liftoff, and
//!   2. survive the landing.
//!
//! ```text
//!   Stage 1 — Launch   (r = 0, feet on the ground)
//!            ── physical liftoff (both feet off, after the expert's release) ─▶
//!            ── launch window closed and still grounded ─▶ LaunchMissed
//!
//!   Stage 2 — Flight
//!            r = (w_upright·exp(−k1·tilt²) − w_ang·min(ω², 4))·dt   (per second)
//!            ── FIRST ground contact after liftoff ─▶ +R_base·exp(−k·ω²), Stage 3
//!
//!   Stage 3 — Recovery (1.5 s window from that first contact)
//!            r = w_upright·exp(−k1·tilt²)·exp(−k2·‖v_xy‖²)·dt   (per second)
//!            ── window survives ─▶ +R_survive, terminated=Completed
//! ```
//!
//! # The launch payment is on TOUCHDOWN, not at liftoff
//!
//! The expert launches every episode, so a payment at the liftoff tick is
//! collectible no matter how the arc ends. A measured run (N=32, 100 updates)
//! showed exactly that failure: **82.7 % of the gross reward came from the
//! liftoff term at 0 % survival** — the policy learned "launch, spin, tip over"
//! because that paid with certainty while the 1.5 s window was never reached.
//! The bonus is now computed from the spin **at** liftoff and released on the
//! first ground contact: a body that never comes down earns nothing. The flight
//! spin penalty is also capped (`min(ω², 4)`, 2 rad/s) so a rotating body cannot
//! drive the per-tick term to an unbounded negative and collapse the critic.
//!
//! # Per-second, not per-frame
//!
//! The per-tick terms are **rates multiplied by `dt`**. The sim runs at 90 Hz,
//! so a raw per-tick weight silently pays 90× per second more than a weight of
//! `1.0` in the plan's equations, and the integral grows with how long a stage
//! lasts: at `w_upright = 0.01/tick`, 1.5 s of recovery alone paid `1.35`,
//! **more than `R_survive = 1.0`** — the policy could earn more by standing in
//! the window than by completing it, and the same policy scored differently if
//! the physics rate changed. Expressing the weights per second removes both
//! problems: the integral over a stage is `rate × duration`, independent of FPS.
//! Defaults keep the shaping secondary to the one-time bonuses
//! (`R_base = R_survive = 1.0`): recovery max `0.4 × 1.5 = 0.6`, flight max
//! `0.4 × ~0.5 s ≈ 0.2`.
//!
//! # Termination (`terminated = true`, §3)
//!
//! The step returns a [`Termination`] and the harness must end the episode on
//! anything other than [`Termination::None`]:
//!
//! - `TiltedOver`   — `|lean| > [`RewardConfig::fall_angle`]`
//! - `LaunchMissed` — the expert's launch window closed
//!   ([`JumpReward::step`]'s `launch_window_over`) and the body stayed grounded
//!   for [`RewardConfig::launch_miss_grace`] seconds: this stroke can no longer
//!   leave the floor, so waiting out the horizon would only dilute the signal
//! - `BodyContact`  — *(disabled for now)* a non-foot body link touches the
//!   floor; the live collider set still misreports this while standing, so it
//!   must not end an episode until the foot-vs-body distinction is reliable
//! - `Completed`    — the recovery window elapsed without tipping (success)
//!
//! There is **no negative crash penalty**: a crash simply cuts off all the
//! remaining positive `r_flight` / `r_land` through `terminated = true`. A
//! failure tick returns `reward = 0`.
//!
//! # Frames and signs
//!
//! [`Fiction`] is Vello **y-down**:
//!
//! - `tilt`  = `fiction.lean` (hip→waist attitude)
//! - `ω`     = `fiction.omega_com` (whole-body spin rate about the CoM — the
//!   rotation a push-off torque excites; `lean_rate` is only the stick's rate)
//! - `v_xy`  = `fiction.v_com.x` (the horizontal component in this 2-D rig)

use walker_control::observe::Fiction;

use std::f32::consts::FRAC_PI_3;

/// Consecutive airborne ticks needed to confirm liftoff **after** the expert's
/// power stroke releases.
///
/// The contact channel is edge-triggered from the previous tick's collision
/// batch, and the observer is reset when a body is respawned, so the first few
/// ticks of an episode read "no contact" even while the character stands. A run
/// with N=32 measured a phantom liftoff at ~4 ticks — 27 ticks before the
/// stroke could possibly release — which opened the 1.5 s window before the
/// jump. Gating on the expert's release plus a 3-tick confirmation (33 ms) is
/// far shorter than a real hop (~0.5 s airborne) and cannot delay it.
const LIFTOFF_CONFIRM_TICKS: u32 = 3;

/// Cap on `ω²` in the flight spin penalty (2 rad/s). The penalty is a per-second
/// rate, so an uncapped `−w_ang·ω²` can go arbitrarily negative while the body
/// rotates; in a measured run that pushed the flight term net-negative and the
/// policy collapsed. Clamping keeps the term bounded while still strongly
/// discouraging spin.
const SPIN_SQ_CAP: f32 = 4.0;

/// How an episode ended, per the [`crate::reward`] module.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Termination {
    /// Still running.
    None,
    /// The 1.5 s recovery window elapsed without tipping over. Success.
    Completed,
    /// `|lean| > fall_angle`.
    TiltedOver,
    /// The launch window closed and the body never left the floor. The stroke
    /// has no propulsion left, so the episode cannot reach Stage 2.
    LaunchMissed,
    /// A non-foot body link touched the floor. Disabled for now (see the module
    /// docs): the check is constructed again once the collider set is reliable.
    #[allow(dead_code)]
    BodyContact,
    /// The body touched down before [`RewardConfig::min_flight`] seconds of flight had passed:
    /// a hop too small to count as the jump (off by default, `min_flight = 0`). Pays nothing.
    HopTooSmall,
}

/// One reward tick's product.
#[derive(Debug, Clone, Copy)]
pub struct StepOut {
    /// The per-tick reward (including one-time bonuses awarded this tick).
    pub reward: f32,
    /// Milestones fired this tick.
    pub events: RewardEvents,
    /// Non-`None` ⇒ the harness must end the episode now.
    pub termination: Termination,
}

/// Milestones the harness logs / branches on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RewardEvents {
    /// Physical liftoff fired this tick (Stage 1 → 2, both feet off the floor).
    pub lift_off: bool,
    /// The body passed apex and regained a foot (Stage 2 → 3).
    pub touch_down: bool,
    /// The recovery window completed this tick (Stage 3 → done, `+R_survive`).
    pub survived: bool,
}

/// The reward knobs. Every field maps to a symbol in `plans/reward_plan.md`.
#[derive(Debug, Clone, Copy)]
pub struct RewardConfig {
    /// Launch payment scale (`R_base`), paid on **touchdown** (scaled by the
    /// spin measured at liftoff), never at liftoff itself.
    pub r_base: f32,
    /// Liftoff bonus sharpness on `ω²` (`k`).
    pub k_spin: f32,
    /// Flight/recovery upright weight (`w_upright`), **per second**: the
    /// per-tick term is `w_upright · upright · dt`, so the integral over a
    /// stage does not depend on the physics rate.
    pub w_upright: f32,
    /// Flight angular-velocity damping weight (`w_ang`), **per second**
    /// (multiplied by `ω² · dt`).
    pub w_ang: f32,
    /// Pre-liftoff spin penalty (`w_launch_spin`), **per second**: in Stage 1 the
    /// tick pays `−w_launch_spin·min(ω²,cap)·dt`. Dense feedback on the one thing
    /// the residual can still change before liftoff — the CoM rotation. Booked
    /// under the `launch` term.
    pub w_launch_spin: f32,
    /// Recovery upright weight (`w_recover`), **per second**. Separate from the
    /// flight `w_upright` so surviving 1.4 s pays clearly more than 0.2 s.
    pub w_recover: f32,
    /// Tilt sharpness on `tilt²` (`k_1`).
    pub k_tilt: f32,
    /// Recovery horizontal-momentum sharpness on `‖v_xy‖²` (`k_2`).
    pub k_mom: f32,
    /// Recovery window length, seconds (1.5 s in the plan).
    pub recovery_window: f32,
    /// Completion bonus (`R_survive`).
    pub r_survive: f32,
    /// Torso tilt beyond which the episode is over, radians (`60°`).
    pub fall_angle: f32,
    /// Grace after the expert's launch window closes before a still-grounded
    /// body is declared a launch miss, seconds. The push-off impulse is released
    /// while the feet are still down, so the body needs a few ticks to actually
    /// leave the floor; this must be long enough for a real launch to clear it
    /// and short enough to still terminate a dead stroke early.
    pub launch_miss_grace: f32,
    /// Minimum flight time, seconds (`MIN_FLIGHT`). A touchdown before this much time has been
    /// spent in the Flight stage ends the episode as [`Termination::HopTooSmall`] and pays no
    /// completion bonus, so a policy cannot earn the landing reward by shrinking the jump to a
    /// micro-hop. Measured as Flight-stage ticks x dt (liftoff itself is confirmed over
    /// [`LIFTOFF_CONFIRM_TICKS`] ticks first, so real airtime is ~3 ticks longer). `0` = off.
    pub min_flight: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self {
            r_base: 1.0,
            k_spin: 0.5,
            // Per-second rates: recovery integral ≤ 0.4·1.5 = 0.6, flight
            // integral ≈ 0.4·0.5 = 0.2 — both below the one-time bonuses, so
            // completing the window (R_survive) stays the dominant payoff.
            w_upright: 0.4,
            w_ang: 0.1,
            w_launch_spin: 2.0,
            w_recover: 1.0,
            k_tilt: 8.0,
            // Characteristic horizontal speed √(1/k₂) ≈ 158 px/s.
            k_mom: 4e-5,
            recovery_window: 1.5,
            r_survive: 2.0,
            fall_angle: FRAC_PI_3,
            launch_miss_grace: 0.20,
            min_flight: 0.0,
        }
    }
}

/// Where the episode is in its one arc.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Stage 1: on the ground before liftoff, `r = 0`.
    Launch,
    /// Stage 2: airborne before touchdown.
    Flight,
    /// Stage 3: the 1.5 s window after touchdown.
    Recovery,
}

/// Per-character 3-stage reward state.
#[derive(Debug, Clone)]
pub struct JumpReward {
    cfg: RewardConfig,
    phase: Phase,
    /// Seconds spent still grounded after the expert's launch window closed.
    launch_miss_t: f32,
    /// Consecutive airborne ticks seen since the launch window opened (liftoff
    /// is confirmed over [`LIFTOFF_CONFIRM_TICKS`]).
    airborne_ticks: u32,
    /// Launch quality latched at liftoff (`exp(−k_spin·ω²)`), paid on touchdown.
    launch_spin: f32,
    /// Seconds elapsed inside the Recovery window.
    recovery_t: f32,
    // ---- introspection (logged, never acted on) ----
    /// Launch payment actually paid (on the touchdown tick), `R_base·exp(−k·ω²)`
    /// with `ω` measured at liftoff.
    pub liftoff_bonus: f32,
    /// Summed Stage-2 reward.
    pub flight_reward: f32,
    /// Summed Stage-3 reward (excluding `R_survive`).
    pub recovery_reward: f32,
    /// `R_survive` paid on the completion tick.
    pub survive_pay: f32,
    /// Ticks evaluated in each stage this episode (introspection, for the
    /// trainer's per-episode reward-mix analysis). The tick a transition happens
    /// on is attributed to the stage whose branch evaluated it; a tick that ended
    /// on tilt/launch-miss pays nothing and is counted in no stage.
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
            launch_spin: 0.0,
            recovery_t: 0.0,
            liftoff_bonus: 0.0,
            flight_reward: 0.0,
            recovery_reward: 0.0,
            survive_pay: 0.0,
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

    /// Reset at an episode boundary (respawn). Carries nothing across.
    pub fn reset(&mut self) {
        *self = Self::new(self.cfg);
    }

    /// One tick: advance the stage machine and return the reward / termination.
    ///
    /// `launch_window_over` is the expert's signal that the power stroke has
    /// been released (`Controller::launch_window_over`). It is only consulted in
    /// Stage 1: once the window has been closed for
    /// [`RewardConfig::launch_miss_grace`] seconds without the feet leaving the
    /// floor, this stroke cannot launch and the episode ends as
    /// [`Termination::LaunchMissed`].
    pub fn step(&mut self, f: &Fiction, launch_window_over: bool) -> StepOut {
        let mut events = RewardEvents::default();
        let mut reward = 0.0;

        let v_xy_sq = f.v_com.x * f.v_com.x;
        let tilt = f.lean;
        let omega = f.omega_com;

        // ---- §3 failure terminations (any phase): cut the positives, pay 0 ----
        if tilt.abs() > self.cfg.fall_angle {
            return self.fail(Termination::TiltedOver);
        }
        // BodyContact termination is deliberately disabled for now: the live
        // collider set attributes the standing feet (or their props) to
        // non-foot colliders, so this fired on every episode's first tick and
        // killed the expert's jump before it could start. `Fiction.body_contact`
        // stays plumbed so the check can return once the collider set is fixed.

        let upright = (-self.cfg.k_tilt * tilt * tilt).exp();

        match self.phase {
            Phase::Launch => {
                self.launch_ticks += 1;
                // Dense spin penalty (bounded, per second × dt). The expert owns
                // the launch timing, so this cannot be dodged by stalling: the
                // window closes and liftoff happens regardless.
                let pen = self.cfg.w_launch_spin * (omega * omega).min(SPIN_SQ_CAP) * f.dt;
                reward -= pen;
                self.liftoff_bonus -= pen;
                // Liftoff is a PHYSICAL event (both feet off the floor), but only
                // once the expert has released the power stroke and the feet stay
                // off for a few ticks: the contact channel is edge-triggered and
                // reads "airborne" during the first ticks after a respawn, which
                // used to open the whole 1.5 s window before the stroke began.
                if launch_window_over {
                    if !f.grounded {
                        self.airborne_ticks += 1;
                    } else {
                        self.airborne_ticks = 0;
                        // The impulse is out and the feet are down: the grace
                        // clock runs toward a launch miss.
                        self.launch_miss_t += f.dt;
                    }
                    if self.airborne_ticks >= LIFTOFF_CONFIRM_TICKS {
                        self.phase = Phase::Flight;
                        events.lift_off = true;
                        // Latch the launch quality now but pay it on TOUCHDOWN:
                        // a payment here is collectible even by a body that never
                        // lands, which is exactly the "launch and die" optimum
                        // measured in run2 (82.7% of gross reward from liftoff,
                        // 0% survival).
                        self.launch_spin = (-self.cfg.k_spin * omega * omega).exp();
                    } else if self.launch_miss_t >= self.cfg.launch_miss_grace {
                        // No propulsion left in this stroke and still grounded.
                        return self.fail(Termination::LaunchMissed);
                    }
                } else {
                    // Still loading: nothing to judge, and any contact flicker
                    // here is noise, not a liftoff.
                    self.airborne_ticks = 0;
                    self.launch_miss_t = 0.0;
                }
            }
            Phase::Flight => {
                self.flight_ticks += 1;
                // Per-second rate × dt, with the spin penalty bounded (see
                // `SPIN_SQ_CAP`), so the stage integral is FPS-independent and
                // cannot collapse the critic on a spinning arc.
                let spin_sq = (omega * omega).min(SPIN_SQ_CAP);
                let r = (self.cfg.w_upright * upright - self.cfg.w_ang * spin_sq) * f.dt;
                reward += r;
                self.flight_reward += r;

                // Touchdown is the FIRST ground contact after liftoff: pay the
                // launch receipt there (so it requires a landing) and open the
                // recovery window, which runs a full `recovery_window` seconds.
                if f.grounded {
                    // A hop shorter than `min_flight` is not the jump: no payment, episode over.
                    if (self.flight_ticks as f32) * f.dt < self.cfg.min_flight {
                        return self.fail(Termination::HopTooSmall);
                    }
                    self.phase = Phase::Recovery;
                    self.recovery_t = 0.0;
                    events.touch_down = true;
                    let bonus = self.cfg.r_base * self.launch_spin;
                    reward += bonus;
                    self.liftoff_bonus += bonus;
                }
            }
            Phase::Recovery => {
                self.recovery_ticks += 1;
                let momentum_decay = (-self.cfg.k_mom * v_xy_sq).exp();
                // Per-second rate × dt (see the module docs): the whole 1.5 s
                // window is worth `w_upright · 1.5` at best, not 135 ticks of it.
                let r = self.cfg.w_recover * upright * momentum_decay * f.dt;
                reward += r;
                self.recovery_reward += r;

                self.recovery_t += f.dt;
                if self.recovery_t >= self.cfg.recovery_window {
                    // Reaching the end of the window means the body never tipped,
                    // never touched a body link, and therefore survived.
                    events.survived = true;
                    reward += self.cfg.r_survive;
                    self.survive_pay = self.cfg.r_survive;
                    return self.finish(reward, events, Termination::Completed);
                }
            }
        }

        StepOut {
            reward,
            events,
            termination: Termination::None,
        }
    }

    /// A failure termination: record it and return a zero-reward tick.
    fn fail(&mut self, reason: Termination) -> StepOut {
        self.term_reason = Some(reason);
        StepOut {
            reward: 0.0,
            events: RewardEvents::default(),
            termination: reason,
        }
    }

    /// A terminal success: record it and return the tick's total reward.
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

    /// Step through the whole arc from liftoff through recovery completion and
    /// assert the milestones fire in order with the expected one-time payments.
    #[test]
    fn the_three_stages_play_in_order_and_pay_each_bonus() {
        let mut rw = JumpReward::new(RewardConfig::default());

        // Stage 1: free, no event.
        assert_eq!(run(&mut rw, &standing(), 50).len(), 50usize);
        assert!(!rw.launched());
        assert_eq!(rw.liftoff_bonus, 0.0);

        // Liftoff: no payment yet. The launch receipt is paid on touchdown,
        // scaled by the spin at liftoff (ω = 0 here ⇒ full R_base).
        let gate = confirm_liftoff(&mut rw, 0.0);
        assert!(gate.events.lift_off);
        assert_eq!(gate.reward, 0.0, "no payment at liftoff");
        assert!(rw.launched());
        assert_eq!(rw.liftoff_bonus, 0.0);
        assert_eq!(gate.termination, Termination::None);

        // Flight: upright per-SECOND rate × dt at ω = 0.
        let flight = rw.step(&airborne(0.0), false);
        let expected = RewardConfig::default().w_upright * DT;
        assert!(
            (flight.reward - expected).abs() < 1e-6,
            "flight tick {} vs {expected}",
            flight.reward
        );
        assert_eq!(flight.termination, Termination::None);

        // Touchdown: the FIRST ground contact pays the launch receipt and opens
        // the window. The same tick still pays its flight shaping.
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        let touch = rw.step(&down, false);
        assert!(touch.events.touch_down);
        let base = RewardConfig::default().r_base;
        assert!(
            (touch.reward - (base + expected)).abs() < 1e-5,
            "touchdown reward {} vs launch {base} + flight {expected}",
            touch.reward
        );
        assert_eq!(rw.liftoff_bonus, base);

        // Recovery: the window is exactly 1.5 s from that contact.
        let window_ticks = (RewardConfig::default().recovery_window / DT).round() as usize;
        let mut last = None;
        for _ in 0..window_ticks {
            last = Some(rw.step(&down, false));
        }
        let done = last.unwrap();
        assert!(done.events.survived);
        assert_eq!(done.termination, Termination::Completed);
        assert_eq!(rw.survive_pay, RewardConfig::default().r_survive);
        assert!(rw.recovery_reward > 0.0);
        // The decomposition counters the trainer logs are consistent with the
        // stage machine: one launch tick before liftoff plus the liftoff tick,
        // every airborne tick in Flight, and exactly the window in Recovery.
        assert!(rw.launch_ticks > 0);
        assert!(rw.flight_ticks >= 2);
        assert_eq!(rw.recovery_ticks, window_ticks as u32);
        // Design invariant: the whole recovery integral must stay below the
        // completion bonus, or standing in the window outbids completing it.
        assert!(
            rw.recovery_reward < rw.survive_pay,
            "recovery integral {} must not outbid R_survive {}",
            rw.recovery_reward,
            rw.survive_pay
        );
    }

    #[test]
    fn the_recovery_window_is_measured_from_the_first_contact() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        confirm_liftoff(&mut rw, 0.0); // enter Flight
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        assert!(rw.step(&down, false).events.touch_down);

        // One tick short of 1.5 s is not a completed landing yet.
        let window_ticks = (cfg.recovery_window / DT).round() as usize;
        for _ in 0..window_ticks - 1 {
            assert_eq!(rw.step(&down, false).termination, Termination::None);
        }
        // The tick that reaches 1.5 s completes it.
        assert_eq!(
            rw.step(&down, false).termination,
            Termination::Completed,
            "the window must close exactly {window_ticks} ticks after contact"
        );
    }

    #[test]
    fn per_tick_terms_are_fps_independent() {
        let cfg = RewardConfig::default();
        let down = fiction(STAND_Z * 0.05, 0.0, 0.1, 0.0, [0.0, 0.0], false);

        // 0.5 s of recovery, once at 90 Hz and once at 180 Hz (half-dt steps).
        let mut at90 = JumpReward::new(cfg);
        at90.step(&standing(), false);
        confirm_liftoff(&mut at90, 0.0);
        assert!(at90.step(&down, false).events.touch_down);
        for _ in 0..45 {
            at90.step(&down, false);
        }

        let mut at180 = JumpReward::new(cfg);
        at180.step(&standing(), false);
        confirm_liftoff(&mut at180, 0.0);
        assert!(at180.step(&down, false).events.touch_down);
        let mut half = down;
        half.dt = DT * 0.5;
        for _ in 0..90 {
            at180.step(&half, false);
        }

        assert!(
            (at90.recovery_reward - at180.recovery_reward).abs() < 1e-5,
            "0.5 s of recovery must pay the same at 90 and 180 Hz: {} vs {}",
            at90.recovery_reward,
            at180.recovery_reward
        );
        assert_eq!(at90.recovery_ticks, 45);
        assert_eq!(at180.recovery_ticks, 90);
    }

    #[test]
    fn standing_is_free_and_is_not_a_liftoff() {
        let mut rw = JumpReward::new(RewardConfig::default());
        let outs = run(&mut rw, &standing(), 200);
        assert_eq!(total(&outs), 0.0, "Stage 1 has no per-tick reward");
        assert!(!rw.launched());
        assert!(outs.iter().all(|o| !o.events.lift_off));
    }

    #[test]
    fn liftoff_is_feet_leaving_the_ground() {
        let cfg = RewardConfig::default();

        // Feet planted even while rising fast and clear: no liftoff.
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        let tiptoe = fiction(STAND_Z * 0.3, 100.0, 0.0, 0.0, [0.0, 0.0], false);
        for _ in 0..10 {
            assert!(!rw.step(&tiptoe, true).events.lift_off);
        }

        // One foot clear: still not a liftoff (`grounded` is the OR of the feet).
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        let one = fiction(STAND_Z * 0.2, 100.0, 0.0, 0.0, [6.0, 0.0], false);
        for _ in 0..10 {
            assert!(!rw.step(&one, true).events.lift_off);
        }

        // A single airborne tick is a flicker, not a liftoff: Stage 2 needs
        // `LIFTOFF_CONFIRM_TICKS` consecutive air ticks.
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        let clear = fiction(STAND_Z * 0.2, 0.0, 0.0, 0.0, [6.0, 6.0], false);
        for _ in 0..8 {
            assert!(!rw.step(&clear, true).events.lift_off);
            rw.step(&standing(), true);
        }
        assert!(!rw.launched());

        // Three consecutive air ticks after the release: liftoff, with no
        // velocity or clearance gate (the expert owns the jump).
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        assert!(confirm_with(&mut rw, &clear).events.lift_off);
    }

    #[test]
    fn contact_flicker_before_the_launch_window_is_not_a_liftoff() {
        // Measured bug (N=32, 100 updates): the contact channel read "airborne"
        // for the first few ticks after each respawn, so the reward declared
        // liftoff at ~4 ticks — ~27 ticks before the expert's stroke could
        // release — and ran the whole 1.5 s window before the jump. Before the
        // release, airborne readings are noise and must never start Stage 2.
        let mut rw = JumpReward::new(RewardConfig::default());
        for _ in 0..30 {
            let out = rw.step(&airborne(0.0), false);
            assert!(!out.events.lift_off, "phantom liftoff before the release");
            assert_eq!(out.reward, 0.0);
        }
        assert!(!rw.launched());
    }

    fn the_launch_bonus_is_scaled_by_spin_and_paid_on_touchdown() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        // ω = 1 rad/s at liftoff ⇒ discount exp(−k).
        let gate = confirm_liftoff(&mut rw, 1.0);
        assert_eq!(gate.reward, 0.0, "no payment at liftoff");

        // Touchdown pays the receipt (plus that tick's flight shaping).
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 1.0, [0.0, 0.0], false);
        let touch = rw.step(&down, false);
        let expected = cfg.r_base * (-cfg.k_spin).exp();
        let flight_tick = (cfg.w_upright - cfg.w_ang * 1.0f32.min(SPIN_SQ_CAP)) * DT;
        assert!(
            (touch.reward - (expected + flight_tick)).abs() < 1e-5,
            "touchdown {:.6} vs launch {expected:.6} + flight {flight_tick:.6}",
            touch.reward
        );
        assert!(
            (rw.liftoff_bonus - expected).abs() < 1e-5,
            "launch payment {:.6} vs {expected:.6}",
            rw.liftoff_bonus
        );
    }

    #[test]
    fn a_launch_that_never_lands_pays_no_launch_bonus() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        confirm_liftoff(&mut rw, 0.0);

        // Tip over in flight: the episode ends before any touchdown, so the
        // launch receipt must never be paid. This is the launch-and-die guard.
        let tipped = fiction(
            STAND_Z * 0.2,
            0.0,
            cfg.fall_angle + 0.05,
            0.0,
            [6.0, 6.0],
            false,
        );
        let out = rw.step(&tipped, false);
        assert_eq!(out.termination, Termination::TiltedOver);
        assert_eq!(out.reward, 0.0);
        assert_eq!(rw.liftoff_bonus, 0.0, "a launch needs a landing to pay");
    }

    #[test]
    fn flight_reward_rewards_upright_and_damps_spin() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        confirm_liftoff(&mut rw, 0.0); // enter Flight

        let calm = rw.step(&airborne(0.0), false).reward;
        let spin = rw
            .step(
                &fiction(STAND_Z * 0.2, 0.0, 0.0, 2.0, [6.0, 6.0], false),
                false,
            )
            .reward;
        let lean = rw
            .step(
                &fiction(STAND_Z * 0.2, 0.0, 0.5, 0.0, [6.0, 6.0], false),
                false,
            )
            .reward;
        assert!(calm > spin, "spin must lower the flight reward");
        assert!(calm > lean, "lean must lower the flight reward");
    }

    #[test]
    fn touchdown_is_the_first_ground_contact_after_liftoff() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        // Enter Flight 10 px above standing.
        let climb = fiction(10.0, 60.0, 0.0, 0.0, [6.0, 6.0], false);
        assert!(confirm_with(&mut rw, &climb).events.lift_off);
        // A foot touches again 1 px below the launch height: that IS the first
        // ground collision, so the window opens (there is no apex gate).
        let flap = fiction(9.0, 0.0, 0.0, 0.0, [0.0, 0.0], false);
        assert!(rw.step(&flap, false).events.touch_down);
    }

    #[test]
    fn tilt_failure_terminates_and_pays_nothing() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        confirm_liftoff(&mut rw, 0.0); // enter Flight

        let tipped = fiction(
            STAND_Z * 0.2,
            0.0,
            cfg.fall_angle + 0.05,
            0.0,
            [6.0, 6.0],
            false,
        );
        let out = rw.step(&tipped, false);
        assert_eq!(out.reward, 0.0, "a crash tick pays nothing");
        assert_eq!(out.termination, Termination::TiltedOver);
        assert_eq!(rw.term_reason, Some(Termination::TiltedOver));
    }

    #[test]
    fn launch_miss_terminates_after_the_grace() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        let grace_ticks = (cfg.launch_miss_grace / DT).ceil() as usize;

        // While the expert is still loading, a grounded body is normal.
        for _ in 0..10 {
            assert_eq!(rw.step(&standing(), false).termination, Termination::None);
        }
        // The window is closed, but the push-off gets its grace to lift the feet.
        for _ in 0..grace_ticks.saturating_sub(2) {
            assert_eq!(
                rw.step(&standing(), true).termination,
                Termination::None,
                "the grace must not be over yet"
            );
        }
        // Past the grace a still-grounded body cannot launch: end early.
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
        assert_eq!(rw.term_reason, Some(Termination::LaunchMissed));
    }

    #[test]
    fn a_hop_shorter_than_min_flight_is_not_a_jump() {
        let mut cfg = RewardConfig::default();
        cfg.min_flight = 0.40;
        let mut rw = JumpReward::new(cfg);
        confirm_liftoff(&mut rw, 0.0);
        // Airborne for 10 ticks (0.11 s), then touch down: a micro-hop.
        for _ in 0..10 {
            assert_eq!(rw.step(&airborne(0.0), false).termination, Termination::None);
        }
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        let out = rw.step(&down, false);
        assert_eq!(out.termination, Termination::HopTooSmall);
        assert_eq!(out.reward, 0.0);
        assert_eq!(rw.term_reason, Some(Termination::HopTooSmall));
        assert_eq!(rw.survive_pay, 0.0);
    }

    #[test]
    fn a_full_hop_passes_min_flight() {
        let mut cfg = RewardConfig::default();
        cfg.min_flight = 0.40;
        let mut rw = JumpReward::new(cfg);
        confirm_liftoff(&mut rw, 0.0);
        let needed = (cfg.min_flight / DT).ceil() as usize + 2;
        for _ in 0..needed {
            assert_eq!(rw.step(&airborne(0.0), false).termination, Termination::None);
        }
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        let out = rw.step(&down, false);
        assert!(out.events.touch_down, "a long enough flight lands normally");
        assert_eq!(out.termination, Termination::None);
    }

    #[test]
    fn min_flight_is_off_by_default() {
        let mut rw = JumpReward::new(RewardConfig::default());
        confirm_liftoff(&mut rw, 0.0);
        for _ in 0..2 {
            rw.step(&airborne(0.0), false);
        }
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        let out = rw.step(&down, false);
        assert!(out.events.touch_down);
        assert_eq!(out.termination, Termination::None);
    }

    #[test]
    fn a_real_liftoff_before_the_grace_is_not_a_miss() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        // The window is "closed" for a few ticks, but the feet leave the floor
        // inside the grace: this is the expert's real launch, not a miss.
        for _ in 0..5 {
            assert_eq!(rw.step(&standing(), true).termination, Termination::None);
        }
        let out = confirm_liftoff(&mut rw, 0.0);
        assert!(out.events.lift_off);
        assert_eq!(out.termination, Termination::None);
        assert!(rw.launched());
    }

    #[test]
    fn body_contact_is_ignored_for_now() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing(), false);
        // Body contact is not yet a termination: the live collider set fires it
        // while standing, and ending an episode on the ground would kill the
        // expert's jump before it starts.
        let out = rw.step(&fiction(0.0, 0.0, 0.0, 0.0, [0.0, 0.0], true), false);
        assert_eq!(out.reward, 0.0, "Stage 1 still pays nothing");
        assert_eq!(out.termination, Termination::None);
    }

    #[test]
    fn reset_starts_a_fresh_episode() {
        let mut rw = JumpReward::new(RewardConfig::default());
        rw.step(&standing(), false);
        confirm_liftoff(&mut rw, 0.0);
        assert!(rw.launched());
        rw.reset();
        assert!(!rw.launched());
        assert_eq!(rw.liftoff_bonus, 0.0);
        assert_eq!(rw.survive_pay, 0.0);
        assert_eq!(rw.term_reason, None);
    }
}
