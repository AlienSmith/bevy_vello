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
//!            ── physical liftoff (both feet off) ─▶ +R_base·exp(−k·ω²)
//!
//!   Stage 2 — Flight
//!            r = w_upright·exp(−k1·tilt²) − w_ang·ω²    (per tick)
//!            ── passed apex AND a foot is down ─▶ Stage 3
//!
//!   Stage 3 — Recovery (1.5 s window after touchdown)
//!            r = w_upright·exp(−k1·tilt²)·exp(−k2·‖v_xy‖²)   (per tick)
//!            ── window survives ─▶ +R_survive, terminated=Completed
//! ```
//!
//! # Termination (`terminated = true`, §3)
//!
//! The step returns a [`Termination`] and the harness must end the episode on
//! anything other than [`Termination::None`]:
//!
//! - `TiltedOver`   — `|lean| > [`RewardConfig::fall_angle`]`
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
//! [`Fiction`] is Vello **y-down**. The plan's vertical axis is "up", so:
//!
//! - `com_pos_z` (upward CoM height)  = `floor − com.y`
//! - `tilt`  = `fiction.lean` (hip→waist attitude)
//! - `ω`     = `fiction.omega_com` (whole-body spin rate about the CoM — the
//!   rotation a push-off torque excites; `lean_rate` is only the stick's rate)
//! - `v_xy`  = `fiction.v_com.x` (the horizontal component in this 2-D rig)
//!
//! `z_standing` and `floor` are captured on the episode's first tick from the
//! actual body, not from world constants, so the reward is rig-agnostic.

use walker_control::observe::Fiction;

use std::f32::consts::FRAC_PI_3;

/// How an episode ended, per the [`crate::reward`] module.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Termination {
    /// Still running.
    None,
    /// The 1.5 s recovery window elapsed without tipping over. Success.
    Completed,
    /// `|lean| > fall_angle`.
    TiltedOver,
    /// A non-foot body link touched the floor. Disabled for now (see the module
    /// docs): the check is constructed again once the collider set is reliable.
    #[allow(dead_code)]
    BodyContact,
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
    /// Liftoff bonus scale (`R_base`).
    pub r_base: f32,
    /// Liftoff bonus sharpness on `ω²` (`k`).
    pub k_spin: f32,
    /// Flight/recovery upright weight (`w_upright`).
    pub w_upright: f32,
    /// Flight angular-velocity damping weight (`w_ang`).
    pub w_ang: f32,
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
    /// How far the CoM must descend from its flight peak before a foot-down
    /// counts as touchdown, px (prevents a flap on the way up from ending flight).
    pub apex_clearance: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self {
            r_base: 1.0,
            k_spin: 0.5,
            w_upright: 0.01,
            w_ang: 0.002,
            k_tilt: 8.0,
            // Characteristic horizontal speed √(1/k₂) ≈ 158 px/s.
            k_mom: 4e-5,
            recovery_window: 1.5,
            r_survive: 1.0,
            fall_angle: FRAC_PI_3,
            apex_clearance: 2.0,
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
    /// Standing floor reference (vello y-down: `min` foot y = the higher foot),
    /// captured on the first tick.
    floor: Option<f32>,
    /// Standing CoM height above the floor (upward), captured on the first tick.
    standing_z: Option<f32>,
    /// Peak upward CoM height seen during Flight (apex tracking).
    peak_z: f32,
    /// Seconds elapsed inside the Recovery window.
    recovery_t: f32,
    // ---- introspection (logged, never acted on) ----
    /// `R_liftoff` paid on the liftoff tick.
    pub liftoff_bonus: f32,
    /// Summed Stage-2 reward.
    pub flight_reward: f32,
    /// Summed Stage-3 reward (excluding `R_survive`).
    pub recovery_reward: f32,
    /// `R_survive` paid on the completion tick.
    pub survive_pay: f32,
    /// The last non-`None` termination this episode produced (for the CSV).
    pub term_reason: Option<Termination>,
}

impl JumpReward {
    pub fn new(cfg: RewardConfig) -> Self {
        Self {
            cfg,
            phase: Phase::Launch,
            floor: None,
            standing_z: None,
            peak_z: 0.0,
            recovery_t: 0.0,
            liftoff_bonus: 0.0,
            flight_reward: 0.0,
            recovery_reward: 0.0,
            survive_pay: 0.0,
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
    pub fn step(&mut self, f: &Fiction) -> StepOut {
        let mut events = RewardEvents::default();
        let mut reward = 0.0;

        // ---- standing references, captured from the body on the first tick ----
        // Vello y-down: smaller y is higher, so `min` picks the higher foot.
        let higher_foot = f.feet[0].pos.y.min(f.feet[1].pos.y);
        let floor = *self.floor.get_or_insert(higher_foot);
        let com_z = floor - f.com.y; // upward CoM height
        let _standing_z = *self.standing_z.get_or_insert(com_z);

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
                // Stage 1 pays nothing: no per-tick term to stall or camp on.
                // Liftoff is a PHYSICAL event (both feet off the floor), not a
                // cheated velocity/clearance gate. The expert owns the jump
                // timing; the residual's job on this frame is a clean, low-spin
                // push-off, which the one-time bonus prices directly.
                if !f.grounded {
                    self.phase = Phase::Flight;
                    self.peak_z = com_z;
                    events.lift_off = true;
                    let bonus = self.cfg.r_base * (-self.cfg.k_spin * omega * omega).exp();
                    reward += bonus;
                    self.liftoff_bonus += bonus;
                }
            }
            Phase::Flight => {
                self.peak_z = self.peak_z.max(com_z);
                let r = self.cfg.w_upright * upright - self.cfg.w_ang * omega * omega;
                reward += r;
                self.flight_reward += r;

                // Touchdown: the body has passed its flight apex and a foot is
                // back on the ground (the debounced `grounded`, not the raw flag).
                let passed_apex = (self.peak_z - com_z) > self.cfg.apex_clearance;
                if passed_apex && f.grounded {
                    self.phase = Phase::Recovery;
                    self.recovery_t = 0.0;
                    events.touch_down = true;
                }
            }
            Phase::Recovery => {
                let momentum_decay = (-self.cfg.k_mom * v_xy_sq).exp();
                let r = self.cfg.w_upright * upright * momentum_decay;
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

    fn run(rw: &mut JumpReward, f: &Fiction, n: usize) -> Vec<StepOut> {
        (0..n).map(|_| rw.step(f)).collect()
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
        rw.step(&standing()); // capture standing references

        // Stage 1: free, no event.
        assert_eq!(run(&mut rw, &standing(), 50).len(), 50usize);
        assert!(!rw.launched());
        assert_eq!(rw.liftoff_bonus, 0.0);

        // Liftoff: one-time bonus, scaled by ω. ω = 0 ⇒ full R_base.
        let gate = rw.step(&airborne(0.0));
        assert!(gate.events.lift_off);
        assert_eq!(gate.reward, RewardConfig::default().r_base);
        assert!(rw.launched());
        assert_eq!(rw.liftoff_bonus, RewardConfig::default().r_base);
        assert_eq!(gate.termination, Termination::None);

        // Flight: upright per-tick reward at ω = 0.
        let flight = rw.step(&airborne(0.0));
        assert_eq!(flight.reward, RewardConfig::default().w_upright);
        assert_eq!(flight.termination, Termination::None);

        // Touchdown: after apex. Apex was set at the liftoff tick's com_z; now
        // the body descends (rise smaller), apex passed, feet down.
        let down = fiction(STAND_Z * 0.05, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        let touch = rw.step(&down);
        assert!(touch.events.touch_down);

        // Recovery: run out the 1.5 s window and complete.
        let mut last = None;
        for _ in 0..(135 + 1) {
            last = Some(rw.step(&down));
        }
        let done = last.unwrap();
        assert!(done.events.survived);
        assert_eq!(done.termination, Termination::Completed);
        assert_eq!(rw.survive_pay, RewardConfig::default().r_survive);
        assert!(rw.recovery_reward > 0.0);
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

        // Feet planted even while rising fast and clear: no liftoff yet.
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        let tiptoe = fiction(STAND_Z * 0.3, 100.0, 0.0, 0.0, [0.0, 0.0], false);
        assert!(!rw.step(&tiptoe).events.lift_off);

        // One foot clear: still not a liftoff.
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        let one = fiction(STAND_Z * 0.2, 100.0, 0.0, 0.0, [6.0, 0.0], false);
        assert!(!rw.step(&one).events.lift_off);

        // Both feet clear with zero upward velocity: liftoff is the physical
        // event, not a velocity/clearance gate. The expert owns the jump timing.
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        let clear = fiction(STAND_Z * 0.2, 0.0, 0.0, 0.0, [6.0, 6.0], false);
        assert!(rw.step(&clear).events.lift_off);
    }

    #[test]
    fn the_liftoff_bonus_is_scaled_by_spin() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        // ω = 1 rad/s ⇒ discount exp(−k).
        let gate = rw.step(&airborne(1.0));
        let expected = cfg.r_base * (-cfg.k_spin).exp();
        assert!(
            (gate.reward - expected).abs() < 1e-5,
            "liftoff bonus {:.6} vs {expected:.6}",
            gate.reward
        );
    }

    #[test]
    fn flight_reward_rewards_upright_and_damps_spin() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        rw.step(&airborne(0.0)); // enter Flight

        let calm = rw.step(&airborne(0.0)).reward;
        let spin = rw
            .step(&fiction(STAND_Z * 0.2, 0.0, 0.0, 2.0, [6.0, 6.0], false))
            .reward;
        let lean = rw
            .step(&fiction(STAND_Z * 0.2, 0.0, 0.5, 0.0, [6.0, 6.0], false))
            .reward;
        assert!(calm > spin, "spin must lower the flight reward");
        assert!(calm > lean, "lean must lower the flight reward");
    }

    #[test]
    fn touchdown_needs_a_real_passed_apex() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        // Enter Flight at 10 px above standing.
        assert!(
            rw.step(&fiction(10.0, 60.0, 0.0, 0.0, [6.0, 6.0], false))
                .events
                .lift_off
        );
        // A foot goes down while the CoM has only descended 1 px from its apex:
        // within `apex_clearance`, so this is a flap (or a second hop), not a
        // real post-apex touchdown.
        let flap = fiction(9.0, 0.0, 0.0, 0.0, [0.0, 0.0], false);
        assert!(!rw.step(&flap).events.touch_down);

        // Now the CoM descends 5 px past the apex: touchdown.
        let down = fiction(5.0, -60.0, 0.0, 0.0, [0.0, 0.0], false);
        assert!(rw.step(&down).events.touch_down);
    }

    #[test]
    fn tilt_failure_terminates_and_pays_nothing() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        rw.step(&airborne(0.0)); // enter Flight

        let tipped = fiction(
            STAND_Z * 0.2,
            0.0,
            cfg.fall_angle + 0.05,
            0.0,
            [6.0, 6.0],
            false,
        );
        let out = rw.step(&tipped);
        assert_eq!(out.reward, 0.0, "a crash tick pays nothing");
        assert_eq!(out.termination, Termination::TiltedOver);
        assert_eq!(rw.term_reason, Some(Termination::TiltedOver));
    }

    #[test]
    fn body_contact_is_ignored_for_now() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        // Body contact is not yet a termination: the live collider set fires it
        // while standing, and ending an episode on the ground would kill the
        // expert's jump before it starts.
        let out = rw.step(&fiction(0.0, 0.0, 0.0, 0.0, [0.0, 0.0], true));
        assert_eq!(out.reward, 0.0, "Stage 1 still pays nothing");
        assert_eq!(out.termination, Termination::None);
    }

    #[test]
    fn reset_starts_a_fresh_episode() {
        let mut rw = JumpReward::new(RewardConfig::default());
        rw.step(&standing());
        rw.step(&airborne(0.0));
        assert!(rw.launched());
        rw.reset();
        assert!(!rw.launched());
        assert_eq!(rw.liftoff_bonus, 0.0);
        assert_eq!(rw.survive_pay, 0.0);
        assert_eq!(rw.term_reason, None);
    }
}
