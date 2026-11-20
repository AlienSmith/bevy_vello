//! The jump reward: **one payment, two knobs.**
//!
//! ```text
//!   reward = R_LAND   if the launch was REAL and the episode did not end in a fall
//!            0        otherwise
//!
//!   REAL launch (the first one in the episode) =
//!       both feet off the floor            (!grounded)
//!   AND the CoM rise-rate >= V_LIFTOFF     (px/s)
//! ```
//!
//! Two numbers: [`RewardConfig::v_liftoff`] and [`RewardConfig::r_land`].
//!
//! "Feet close to the ground before liftoff" needs no third number — a lift starts
//! from contact by construction (`air_ticks` is zeroed on every grounded tick) and
//! the CoM can only gain `V_LIFTOFF` by pushing off the floor. A geometric version
//! was implemented and measured **vacuous** (the only way to fail it is an
//! unphysical one-tick teleport), so it was removed rather than kept as a knob.
//!
//! # Why it is this small
//!
//! Two previous designs were each worth ~1 000 lines and each was gamed within
//! 100 updates. The measured exploits:
//!
//! | exploit | which term paid it |
//! |---|---|
//! | "stand up from the settled pose", never jump (`liftoffs = 0`, `apex_ratio 1.0`) | a per-tick air credit plus an apex measured as a *sampled max* against the *first tick's* CoM |
//! | "bounce a few times before the real jump" (`liftoff_pay 3.0` from 3 takeoffs) | a liftoff credit paid **per** takeoff |
//! | "never jump" (the whole 10 000-update collapse) | a fall penalty larger than every positive term combined |
//!
//! All three came from *extra machinery*, not from the goal being wrong. So this
//! version removes machinery instead of guarding it:
//!
//! - **One payment.** Nothing to accumulate, nothing to farm, nothing to linger in.
//! - **No penalty.** A fall pays 0, the same as standing still. It cannot be worth
//!   more than a landing, and there is no negative term to run away from — which is
//!   what made "never jump" optimal twice.
//! - **The launch is judged by `v_up` at one tick**, not by a max, a baseline or a
//!   filter: `apex = v_up²/(2g)` is conserved through the flight, so `V_LIFTOFF`
//!   *is* the minimum-height gate (`V_LIFTOFF = 300 px/s ⇒ ~46 px of rise`).
//! - **The feet must be near the floor before liftoff**, so a mid-air twitch or a
//!   bounce during descent can never be mistaken for a launch.
//! - **Once per episode**, so a second hop adds nothing.
//!
//! The cost is sparsity: one positive event per episode. If that proves too sparse
//! to learn, the fix is a *potential-based* dense term (`γΦ(s') − Φ(s)`), which is
//! provably optimum-preserving — see `plans/jump-reward-idea2-least-action.md`.
//!
//! Termination is **not** here: the collector owns the fall gate and the horizon,
//! and calls [`JumpReward::pay_land`] on the episode's last tick.

use walker_control::observe::Fiction;

/// The three knobs, plus a fixed-step fallback.
#[derive(Debug, Clone, Copy)]
pub struct RewardConfig {
    /// CoM rise-rate (px/s) a launch must reach to count. This is also the minimum
    /// height, because the rise it buys is ballistic: `apex = v²/(2g)`.
    /// `250 px/s ⇒ ≈32 px`. (This rig: 1 px ≈ 1 cm, `g = 980 px/s²`.)
    pub v_liftoff: f32,
    /// Paid once, when the episode ends **not** in a fall, after a real launch.
    pub r_land: f32,
    /// Fallback fixed step when the fiction carries `dt == 0`.
    pub dt: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self {
            // Calibrated on measurements: an honest expert hop launches at 276-478
            // px/s (mean 377, n=128), so 250 admits every real hop while a twitch
            // (< 100 px/s) or a settle bounce (v_up <= 0) cannot pass.
            v_liftoff: 250.0,
            // The only positive number in the reward.
            r_land: 1.0,
            dt: 1.0 / 90.0,
        }
    }
}

/// Milestones the harness logs / branches on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RewardEvents {
    /// This tick was the first **real** launch of the episode.
    pub launch: bool,
    /// This tick the episode was paid for landing (0 outside that tick).
    pub landed: bool,
}

/// Per-character state: where the floor is, and whether the jump happened.
#[derive(Debug, Clone)]
pub struct JumpReward {
    cfg: RewardConfig,
    /// Previous tick's measured CoM height (world y), for `v_up`.
    prev_com_y: Option<f32>,
    /// A real launch has happened this episode. Latched: the episode pays once.
    launched: bool,
    /// Airborne ticks since the feet last left the floor.
    air_ticks: u32,
    /// Largest `v_up` seen in the launch window of the **current** lift.
    peak_v_up: f32,
    // ---- introspection (logged, never acted on) ----
    /// The launch velocity measured at the (single) real launch, px/s.
    pub launch_v_up: f32,
    /// The ballistic rise that launch bought, px: `v_up²/(2g)`.
    pub launch_apex: f32,
    /// The `R_LAND` paid on the episode's last tick (0 if it was not paid).
    pub land_pay: f32,
}

/// How many airborne ticks the launch velocity is measured over.
///
/// One tick is the wrong instrument: the impulse is applied as the feet leave the
/// floor, so the *first* airborne tick still reads ~0-100 px/s and the real value
/// shows up a tick or two later. Measured on the sim with a single-tick gate:
/// `launch_pct` collapsed to 16 % while the launches that did register averaged
/// ~340 px/s. Five ticks covers the impulse without reaching the apex.
const LAUNCH_WINDOW: u32 = 5;

impl JumpReward {
    pub fn new(cfg: RewardConfig) -> Self {
        Self {
            cfg,
            prev_com_y: None,
            launched: false,
            air_ticks: 0,
            peak_v_up: 0.0,
            launch_v_up: 0.0,
            launch_apex: 0.0,
            land_pay: 0.0,
        }
    }

    #[cfg(test)]
    pub fn cfg(&self) -> &RewardConfig {
        &self.cfg
    }

    /// Whether a real launch happened this episode. `pay_land` and the harness's
    /// "landed" reporting both key off it.
    pub fn launched(&self) -> bool {
        self.launched
    }

    /// Gravity is fixed here because it is a property of the world, not a knob.
    const GRAVITY: f32 = 980.0;

    /// Reset at an episode boundary (respawn). Carries nothing across.
    pub fn reset(&mut self) {
        self.prev_com_y = None;
        self.launched = false;
        self.air_ticks = 0;
        self.peak_v_up = 0.0;
        self.launch_v_up = 0.0;
        self.launch_apex = 0.0;
        self.land_pay = 0.0;
    }

    /// One tick. Always returns **0**: this reward has no per-tick term, which is
    /// precisely why it has nothing to farm. The only signal it produces here is
    /// the launch event.
    pub fn step(&mut self, f: &Fiction) -> (f32, RewardEvents) {
        let dt = if f.dt > 0.0 { f.dt } else { self.cfg.dt };
        let mut events = RewardEvents::default();

        // One-tick CoM rise-rate, from the unfiltered measured CoM. `v_com` is
        // EMA-filtered and reads ~0 on the takeoff tick, which is what broke the
        // previous `apex_pred` (measured: +0.044 over a whole run).
        let v_up = match self.prev_com_y {
            Some(prev) => (prev - f.com.y) / dt,
            None => 0.0,
        };
        self.prev_com_y = Some(f.com.y);

        // ---- the one event: a real launch ----
        //
        // "Feet close to the ground before liftoff" needs **no test**: a lift starts
        // from contact by construction (`air_ticks` is zeroed every grounded tick,
        // so the tick that starts a lift is always preceded by ground contact), and
        // the CoM can only gain 250 px/s by pushing off the floor. A geometric
        // version was tried and measured vacuous — the only way to fail it is an
        // unphysical one-tick teleport. So it is a fact about the trace, not a knob.
        if f.grounded {
            // Back on the floor: this lift is over (it may have been a bounce).
            self.air_ticks = 0;
            self.peak_v_up = 0.0;
        } else {
            self.air_ticks += 1;
            if !self.launched && self.air_ticks <= LAUNCH_WINDOW {
                self.peak_v_up = self.peak_v_up.max(v_up);
                if self.peak_v_up >= self.cfg.v_liftoff {
                    self.launched = true;
                    self.launch_v_up = self.peak_v_up;
                    self.launch_apex = self.peak_v_up * self.peak_v_up / (2.0 * Self::GRAVITY);
                    events.launch = true;
                }
            }
        }

        (0.0, events)
    }

    /// The episode's last tick: pay `r_land` if a real launch happened and the body
    /// did not fall. Called exactly once by the harness.
    ///
    /// A fall pays **0**, the same as standing still: there is no negative term in
    /// this reward at all, so no policy can be pushed into "never jump" to dodge
    /// one (that is what happened in both previous designs).
    pub fn pay_land(&mut self, f: &Fiction, fallen: bool) -> f32 {
        let pay = if self.launched && !fallen && f.grounded {
            self.cfg.r_land
        } else {
            0.0
        };
        self.land_pay = pay;
        pay
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::prelude::Vec2;
    use walker_control::observe::FootState;

    const SURFACE: f32 = 312.0;
    const DT: f32 = 1.0 / 90.0;

    fn com_at(rise: f32) -> Vec2 {
        Vec2::new(0.0, SURFACE - 104.0 - rise)
    }

    fn fiction(com: Vec2, grounded: bool, feet_lift: f32) -> Fiction {
        Fiction {
            dt: DT,
            hip: com,
            v_hip: Vec2::ZERO,
            lean: 0.0,
            lean_rate: 0.0,
            com,
            v_com: Vec2::ZERO,
            ang_momentum: 0.0,
            omega_com: 0.0,
            feet: [
                FootState {
                    pos: Vec2::new(-20.0, SURFACE - feet_lift),
                    vel: Vec2::ZERO,
                    touching: grounded,
                },
                FootState {
                    pos: Vec2::new(20.0, SURFACE - feet_lift),
                    vel: Vec2::ZERO,
                    touching: grounded,
                },
            ],
            grounded,
        }
    }

    fn standing() -> Fiction {
        fiction(com_at(0.0), true, 0.0)
    }

    /// A takeoff tick: `rise` px higher than the previous tick (⇒ `90·rise` px/s),
    /// feet lifted as far as they would be in the air.
    fn takeoff(rise: f32) -> Fiction {
        fiction(com_at(rise), false, 2.0)
    }

    fn cfg() -> RewardConfig {
        RewardConfig::default()
    }

    /// Run a launch and then report what the episode would pay on its last tick.
    fn hop_ends_with(rise: f32, fallen: bool, end_grounded: bool) -> f32 {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&takeoff(rise));
        rw.pay_land(&fiction(com_at(0.0), end_grounded, 0.0), fallen)
    }

    #[test]
    fn standing_still_earns_nothing() {
        let mut rw = JumpReward::new(cfg());
        for _ in 0..200 {
            assert_eq!(rw.step(&standing()).0, 0.0, "no per-tick reward exists");
        }
        assert!(!rw.launched());
        assert_eq!(
            rw.pay_land(&standing(), false),
            0.0,
            "an episode that never launched is worth 0, even standing up"
        );
    }

    /// The exploit that broke the previous design: rising out of a settled crouch
    /// while the feet are still on the floor.
    #[test]
    fn standing_up_is_not_a_launch() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        // 60 px of CoM rise over 20 ticks, feet planted the whole time.
        for k in 0..20 {
            let f = fiction(com_at(k as f32 * 3.0), true, 0.0);
            let (r, ev) = rw.step(&f);
            assert_eq!(r, 0.0);
            assert!(!ev.launch, "a grounded body cannot launch");
        }
        assert!(!rw.launched());
        assert_eq!(rw.pay_land(&standing(), false), 0.0);
    }

    #[test]
    fn a_weak_lift_does_not_count() {
        // 1 px in a tick = 90 px/s, well under V_LIFTOFF (300).
        assert_eq!(hop_ends_with(1.0, false, true), 0.0);
    }

    /// A lift whose velocity arrives **after** the launch window is not a launch —
    /// this is the bounce case: airborne, feet clear, and the upward velocity only
    /// comes from a later jolt.
    #[test]
    fn a_late_jolt_does_not_count() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&fiction(com_at(0.2), false, 1.0)); // 18 px/s: below V_LIFTOFF
        for _ in 0..10 {
            rw.step(&fiction(com_at(0.3), false, 5.0)); // still drifting, no launch
        }
        // Now a jolt, but the launch window (5 ticks) has closed.
        rw.step(&fiction(com_at(8.0), false, 5.0));
        assert!(!rw.launched(), "the window closed before the jolt");
        assert_eq!(rw.pay_land(&standing(), false), 0.0);
    }

    #[test]
    fn a_real_launch_counts_once_per_episode() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        let (_, ev) = rw.step(&takeoff(5.0)); // 450 px/s
        assert!(ev.launch);
        assert!(rw.launched());
        assert!((rw.launch_v_up - 450.0).abs() < 1.0, "{}", rw.launch_v_up);
        assert!(
            (rw.launch_apex - 450.0 * 450.0 / (2.0 * 980.0)).abs() < 1.0,
            "apex must be the ballistic rise: {}",
            rw.launch_apex
        );
        // Land, bounce, and launch again: nothing changes, nothing re-pays.
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        rw.step(&standing());
        let (_, ev) = rw.step(&takeoff(6.0));
        assert!(!ev.launch, "the launch is latched: one hop per episode");
        assert!(
            (rw.launch_v_up - 450.0).abs() < 1.0,
            "the first launch stands"
        );
    }

    #[test]
    fn a_landed_jump_pays_once() {
        assert_eq!(hop_ends_with(5.0, false, true), cfg().r_land);
    }

    #[test]
    fn a_crashed_jump_pays_nothing() {
        assert_eq!(hop_ends_with(5.0, true, true), 0.0);
        // and is exactly as bad as standing still — never worse, so there is no
        // penalty to run away from
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        assert_eq!(rw.pay_land(&standing(), true), 0.0);
    }

    #[test]
    fn a_hop_that_never_comes_down_pays_nothing() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        assert_eq!(
            rw.pay_land(&fiction(com_at(40.0), false, 20.0), false),
            0.0,
            "still airborne at the end is not a landing"
        );
    }

    #[test]
    fn reset_starts_a_fresh_episode() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        assert!(rw.launched());
        rw.reset();
        assert!(!rw.launched());
        assert_eq!(rw.launch_v_up, 0.0);
        rw.step(&standing());
        assert_eq!(rw.pay_land(&standing(), false), 0.0);
    }

    /// The decoy suite: every cheap non-jump must earn exactly the same as standing
    /// still, and strictly less than a real hop.
    #[test]
    fn no_decoy_beats_a_real_hop() {
        let real = hop_ends_with(5.0, false, true);

        // crouch: CoM down, feet planted
        let mut crouch = JumpReward::new(cfg());
        crouch.step(&standing());
        for k in 0..30 {
            crouch.step(&fiction(com_at(-(k as f32) * 1.5), true, 0.0));
        }
        assert_eq!(crouch.pay_land(&standing(), false), 0.0);

        // lift one foot: `grounded` is an OR over feet, but the reward needs BOTH
        // feet off the floor plus a real `v_up`
        let mut lift = JumpReward::new(cfg());
        lift.step(&standing());
        let mut f = fiction(com_at(0.5), false, 6.0);
        f.feet[1].touching = true; // other foot still down
        f.grounded = true;
        lift.step(&f);
        assert!(!lift.launched());
        assert_eq!(lift.pay_land(&standing(), false), 0.0);

        // bounce on the spot: airborne but descending (v_up < 0)
        let mut bounce = JumpReward::new(cfg());
        bounce.step(&standing());
        bounce.step(&takeoff(2.0)); // +180 px/s: below V_LIFTOFF
        bounce.step(&fiction(com_at(-2.0), false, 3.0)); // falling
        assert!(!bounce.launched());
        assert_eq!(bounce.pay_land(&standing(), false), 0.0);

        assert!(
            real > 0.0,
            "the real hop must be the only positive outcome: {real}"
        );
    }
}
