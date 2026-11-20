//! The jump reward: **one payment, one constraint, three knobs.**
//!
//! ```text
//!   LIFTOFF (once per episode) = the FIRST tick on which BOTH feet are off the floor
//!                                (latched; later lift-offs change nothing)
//!
//!   before liftoff, per tick   = −foot_penalty · max(0, height_above_floor − foot_margin)
//!                                (the feet are constrained to the ground; straying costs)
//!
//!   episode ends               = +R_LAND  if it lifted off AND ended on the harness's
//!                                          LANDING verdict (settled and upright)
//!                                0        otherwise (no lift-off / fell / timed out)
//! ```
//!
//! Three knobs: [`RewardConfig::foot_margin`], [`RewardConfig::foot_penalty`],
//! [`RewardConfig::r_land`].
//!
//! # Why these two mechanisms
//!
//! Every previous attempt was gamed by **feet leaving the floor**, because "off the
//! ground" was the one thing the reward responded to:
//!
//! - a per-tick air credit paid for *hang time* → the body learned to hover/bounce;
//! - an apex measured as a sampled max against the first tick's CoM → "stand up out
//!   of the settled crouch" reached the apex without jumping (`liftoffs = 0`);
//! - a liftoff credit paid **per** takeoff → the body bounced first and banked 3×
//!   (`liftoff_pay 3.000` from 3 takeoffs).
//!
//! So the reward now *constrains* the feet instead of paying for them: while the body
//! has not jumped, the feet belong on the floor within `foot_margin`, and every px
//! above that is charged. Wobbling is no longer a strategy that might pay — it is a
//! cost. And the one thing that does pay is the completed landing, judged by the
//! harness's own verdict (`JumpCtrl::landing_complete`), not by "did not fall".
//!
//! The last point matters: a body that launches and then wobbles reaches the horizon
//! without ever landing, and "did not fall" would have paid it (measured: several
//! lift-offs keep `grounded` flickering, which delays the hold and keeps the fall
//! gate quiet). Keying the payment to the landing verdict makes surviving worth 0.
//!
//! There is **no negative terminal**: a fall pays the same as standing still, so no
//! policy can be pushed into "never jump" to dodge a penalty — which is what killed
//! both previous designs (the fall charge was the only term large enough to steer).

use walker_control::observe::Fiction;

/// The three knobs, plus a fixed-step fallback.
#[derive(Debug, Clone, Copy)]
pub struct RewardConfig {
    /// The feet may be at most this far above their standing height (px) before the
    /// lift-off. The floor reference is the higher foot on the episode's first tick,
    /// i.e. the standing pose — no world constant, and no drift from soft-body
    /// penetration (a running minimum was tried and measured wrong).
    pub foot_margin: f32,
    /// The same constraint **after touchdown**, deliberately more generous (px):
    /// the landing absorb folds the knees and rolls the feet, which is legitimate,
    /// while a *second jump* leaves the floor by tens of px. Since the lift-off is
    /// already latched, that second jump can never pay — this makes it cost.
    pub land_margin: f32,
    /// Cost per px per tick of straying above [`Self::foot_margin`], pre-lift-off.
    /// Small on purpose: a 20 px lift for 100 ticks costs ~0.2 against `R_LAND = 1`.
    pub foot_penalty: f32,
    /// Paid once when the episode ends on the harness's landing verdict after a
    /// lift-off.
    pub r_land: f32,
    /// Fallback fixed step when the fiction carries `dt == 0`.
    pub dt: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self {
            // A foot a few px off the floor while standing is physics noise; tens of
            // px is a deliberate flick. Measured in the sim: during a planted stance
            // the feet stay within ~1-2 px of their standing height.
            foot_margin: 12.0,
            // ~2.5x the pre-launch margin: the absorb legitimately folds the legs.
            land_margin: 30.0,
            // Per-tick cap of the margin charge. At 180 ticks an always-offending
            // episode costs 0.36, i.e. a third of the landing payment: it shapes,
            // it cannot dominate.
            foot_penalty: 0.002,
            r_land: 1.0,
            dt: 1.0 / 90.0,
        }
    }
}

/// Milestones the harness logs / branches on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RewardEvents {
    /// This tick was the **first** tick with both feet off the floor.
    pub lift_off: bool,
}

/// Where the episode is in its one arc.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Before the jump: the feet are constrained to `foot_margin`.
    PreLaunch,
    /// The single legitimate flight: the feet are free.
    Flight,
    /// Back on the ground: the feet are constrained again, to `land_margin`.
    Landed,
}

/// Per-character state: where the feet belong, and whether the jump happened.
#[derive(Debug, Clone)]
pub struct JumpReward {
    cfg: RewardConfig,
    /// Standing foot height (world y, vello y-down), captured on the first tick.
    floor: Option<f32>,
    /// One arc: `PreLaunch → Flight → Landed`. No timers, no re-entry to Flight.
    phase: Phase,
    // ---- introspection (logged, never acted on) ----
    /// The summed foot-margin cost this episode (positive number = charged).
    pub foot_pen: f32,
    /// The `R_LAND` paid on the episode's last tick (0 if it was not paid).
    pub land_pay: f32,
}

impl JumpReward {
    pub fn new(cfg: RewardConfig) -> Self {
        Self {
            cfg,
            floor: None,
            phase: Phase::PreLaunch,
            foot_pen: 0.0,
            land_pay: 0.0,
        }
    }

    #[cfg(test)]
    pub fn cfg(&self) -> &RewardConfig {
        &self.cfg
    }

    /// Whether the body has left the floor in this episode.
    pub fn lifted_off(&self) -> bool {
        self.phase != Phase::PreLaunch
    }

    /// Reset at an episode boundary (respawn). Carries nothing across.
    pub fn reset(&mut self) {
        self.floor = None;
        self.phase = Phase::PreLaunch;
        self.foot_pen = 0.0;
        self.land_pay = 0.0;
    }

    /// One tick. Returns the foot-margin charge (≤ 0) and the lift-off event.
    pub fn step(&mut self, f: &Fiction) -> (f32, RewardEvents) {
        let mut events = RewardEvents::default();

        // The floor is where the feet stand on the episode's first tick. Vello is
        // y-DOWN, so `min` of the two foot y's is the HIGHER foot.
        let higher = f.feet[0].pos.y.min(f.feet[1].pos.y);
        let floor = *self.floor.get_or_insert(higher);

        // Height of the HIGHER foot above the standing floor (vello y-down: the
        // smaller y is the higher foot, so `min` picks it).
        let above = (floor - higher).max(0.0);
        let both_off = !f.feet[0].touching && !f.feet[1].touching;

        let mut r = 0.0;
        match self.phase {
            Phase::PreLaunch => {
                // ---- constraint 1: the feet belong on the ground ----
                r += self.charge(above, self.cfg.foot_margin);
                // ---- the one event: the FIRST tick both feet are off the floor ----
                // Raw per-foot contact, not the OR-of-feet `grounded` flag: a single
                // lifted foot must not count as a jump.
                if both_off {
                    self.phase = Phase::Flight;
                    events.lift_off = true;
                }
            }
            Phase::Flight => {
                // The single legitimate flight: feet free, no constraint.
                if f.grounded {
                    self.phase = Phase::Landed;
                }
            }
            Phase::Landed => {
                // ---- constraint 2: keep the feet on the ground again ----
                // More generous, because the absorb folds the knees. A second jump
                // leaves by tens of px, so it costs and can never pay (the lift-off
                // is latched and the verdict is keyed to the harness's hold).
                r += self.charge(above, self.cfg.land_margin);
            }
        }

        (r, events)
    }

    /// Charge for straying above `margin`, and book it. Returns a non-positive
    /// number.
    ///
    /// The charge is **bounded per tick** at `foot_penalty`: it ramps linearly from
    /// 0 at the margin to the cap at `excess == margin`, and never exceeds it. An
    /// unbounded per-px charge was measured at **6.0 per episode against a landing
    /// payment of 1** — a soft body's feet sit tens of px up for hundreds of ticks,
    /// so the constraint swamped the reward and handed the policy a large,
    /// unavoidable negative (the exact trap that made "never jump" optimal before).
    fn charge(&mut self, above: f32, margin: f32) -> f32 {
        let margin = margin.max(1e-6);
        let excess = (above - margin).max(0.0);
        if excess <= 0.0 {
            return 0.0;
        }
        let pen = self.cfg.foot_penalty * (excess / margin).min(1.0);
        self.foot_pen += pen;
        -pen
    }

    /// The episode's last tick: `R_LAND` if the body lifted off and ended on the
    /// harness's **landing verdict** (`landed`, from `JumpCtrl::landing_complete`).
    /// `fallen` only has to veto a contradictory verdict; it carries no penalty.
    pub fn pay_land(&mut self, fallen: bool, landed: bool) -> f32 {
        let pay = if self.lifted_off() && landed && !fallen {
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

    /// `feet_lift` is how far the feet are above the floor; `touching` is set per
    /// foot from the same value so the raw contact flags stay consistent.
    fn fiction(com: Vec2, feet_lift: [f32; 2]) -> Fiction {
        let touching = [feet_lift[0] <= 0.5, feet_lift[1] <= 0.5];
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
        }
    }

    fn standing() -> Fiction {
        fiction(com_at(0.0), [0.0, 0.0])
    }

    fn cfg() -> RewardConfig {
        RewardConfig::default()
    }

    /// Run `f` for `n` ticks and return the summed reward.
    fn run(rw: &mut JumpReward, f: &Fiction, n: usize) -> f32 {
        let mut total = 0.0;
        for _ in 0..n {
            total += rw.step(f).0;
        }
        total
    }

    #[test]
    fn standing_still_is_free_and_is_not_a_lift_off() {
        let mut rw = JumpReward::new(cfg());
        assert_eq!(run(&mut rw, &standing(), 200), 0.0, "no per-tick reward");
        assert!(!rw.lifted_off());
        assert_eq!(rw.foot_pen, 0.0);
        // Never lifted off ⇒ no landing payment, even with a landing verdict.
        assert_eq!(rw.pay_land(false, true), 0.0);
    }

    /// One foot up is **not** a lift-off, and the constraint charges for it.
    #[test]
    fn lifting_one_foot_is_charged_and_is_not_a_lift_off() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        let one_up = fiction(com_at(0.0), [30.0, 0.0]);
        let charged = run(&mut rw, &one_up, 10);
        assert!(charged < 0.0, "a lifted foot must cost: {charged}");
        assert!(!rw.lifted_off(), "one foot down is still on the ground");
        // Bounded per tick: at 30 px with a 12 px margin the ramp saturates, so the
        // cost is exactly `foot_penalty` per tick over the 10 ticks.
        assert!(
            (rw.foot_pen - cfg().foot_penalty * 10.0).abs() < 1e-5,
            "penalty is capped per tick: {}",
            rw.foot_pen
        );
    }

    /// The exploit that broke the previous design: rising out of the settled crouch
    /// with the feet planted. No velocity, no apex, no pay.
    #[test]
    fn standing_up_is_not_a_jump() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        for k in 0..40 {
            rw.step(&fiction(com_at(k as f32 * 2.5), [0.0, 0.0]));
        }
        assert!(!rw.lifted_off());
        assert_eq!(rw.foot_pen, 0.0, "the feet never left the ground");
        assert_eq!(rw.pay_land(false, true), 0.0, "standing up pays nothing");
    }

    /// Both feet off the floor is the lift-off, and it latches: a second hop, a
    /// bounce, or a wobble afterwards changes nothing.
    #[test]
    fn lift_off_is_the_first_both_feet_off_tick_and_latches() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        let (r, ev) = rw.step(&fiction(com_at(1.0), [3.0, 3.0]));
        assert!(ev.lift_off, "both feet off the floor is the lift-off");
        assert!(r <= 0.0, "the lift-off tick is not a reward in itself");
        assert!(rw.lifted_off());
        // Land, bounce, lift again: no second event, nothing paid per hop.
        rw.step(&standing());
        let (_, ev) = rw.step(&fiction(com_at(1.0), [3.0, 3.0]));
        assert!(!ev.lift_off, "the lift-off is latched: one per episode");
    }

    /// A second jump after touchdown is charged (and can never pay): the landed
    /// margin is loose enough for the absorb, tight enough for another hop.
    #[test]
    fn a_second_jump_after_landing_is_charged() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&fiction(com_at(1.0), [3.0, 3.0])); // lift off  → Flight
        rw.step(&fiction(com_at(0.0), [0.0, 0.0])); // touchdown → Landed
        assert!(rw.lifted_off());
        // A second hop: both feet 45 px up, beyond `land_margin` (30).
        let (r, ev) = rw.step(&fiction(com_at(1.0), [45.0, 45.0]));
        assert!(!ev.lift_off, "the lift-off is latched");
        assert!(r < 0.0, "a second jump must cost: {r}");
        // 45 px against a 30 px margin is half a ramp ⇒ half the per-tick cap.
        assert!(
            (rw.foot_pen - 0.5 * cfg().foot_penalty).abs() < 1e-5,
            "charged against the landed margin: {}",
            rw.foot_pen
        );

        // The absorb itself is free: a few px above the floor for a few ticks.
        let mut absorb = JumpReward::new(cfg());
        absorb.step(&standing());
        absorb.step(&fiction(com_at(1.0), [3.0, 3.0]));
        absorb.step(&fiction(com_at(0.0), [0.0, 0.0]));
        assert_eq!(
            run(&mut absorb, &fiction(com_at(-3.0), [8.0, 8.0]), 10),
            0.0,
            "a normal landing absorb must not be charged"
        );
    }

    /// The only positive outcome.
    #[test]
    fn a_landed_jump_pays_once() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&fiction(com_at(1.0), [3.0, 3.0]));
        assert_eq!(rw.pay_land(false, true), cfg().r_land);
    }

    #[test]
    fn a_crashed_jump_pays_nothing() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&fiction(com_at(1.0), [3.0, 3.0]));
        assert_eq!(rw.pay_land(true, false), 0.0);
    }

    /// The reason the payment keys on the landing verdict: a body that lifts off and
    /// then wobbles to the horizon has "not fallen", and must still earn nothing.
    #[test]
    fn a_wobble_that_never_lands_pays_nothing() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&fiction(com_at(1.0), [3.0, 3.0]));
        // 150 ticks of bouncing on the ground, feet flicking on and off.
        for k in 0..150 {
            let up = if k % 4 == 0 { 6.0 } else { 0.0 };
            rw.step(&fiction(com_at(0.0), [up, up]));
        }
        assert_eq!(
            rw.pay_land(false, false),
            0.0,
            "surviving to the horizon is not landing"
        );
    }

    #[test]
    fn reset_starts_a_fresh_episode() {
        let mut rw = JumpReward::new(cfg());
        rw.step(&standing());
        rw.step(&fiction(com_at(1.0), [3.0, 3.0]));
        assert!(rw.lifted_off());
        rw.reset();
        assert!(!rw.lifted_off());
        assert_eq!(rw.foot_pen, 0.0);
        rw.step(&standing());
        assert_eq!(rw.pay_land(false, true), 0.0);
    }

    /// Decoy suite: every cheap non-jump earns 0 and, where the feet stray, is
    /// actually charged — while the only positive outcome is a landed jump.
    #[test]
    fn no_decoy_beats_a_landed_jump() {
        // real jump, landed
        let mut real = JumpReward::new(cfg());
        real.step(&standing());
        real.step(&fiction(com_at(1.0), [3.0, 3.0]));
        let real_pay = real.pay_land(false, true);

        // crouch: CoM down, feet planted
        let mut crouch = JumpReward::new(cfg());
        crouch.step(&standing());
        for k in 0..30 {
            crouch.step(&fiction(com_at(-(k as f32) * 1.5), [0.0, 0.0]));
        }
        assert_eq!(crouch.pay_land(false, true), 0.0);

        // tiptoe: CoM up, feet still touching (the old apex exploit)
        let mut tiptoe = JumpReward::new(cfg());
        tiptoe.step(&standing());
        for k in 0..30 {
            tiptoe.step(&fiction(com_at(k as f32 * 2.0), [0.0, 0.0]));
        }
        assert_eq!(tiptoe.pay_land(false, true), 0.0);

        // bounce on the spot: both feet leave for one tick, then it never lands
        let mut bounce = JumpReward::new(cfg());
        bounce.step(&standing());
        for k in 0..60 {
            let up = if k % 3 == 0 { 8.0 } else { 0.0 };
            bounce.step(&fiction(com_at(0.0), [up, up]));
        }
        assert_eq!(bounce.pay_land(false, false), 0.0);

        assert!(real_pay > 0.0, "only a landed jump pays: {real_pay}");
    }
}
