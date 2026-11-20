//! The **task-outcome jump reward** — reward the physics milestones of a valid
//! hop, and nothing else.
//!
//! ```text
//!  takeoff (both feet off)  R_liftoff = w_liftoff · clamp(v_up / v_jump, 0, 1)
//!                           arms the apex gate iff v_up > v_min  (0.5 m/s)
//!  airborne, per tick       R_air     = w_air · dt · apex_ratio     (apex_ratio monotone)
//!  touchdown                the landing window OPENS only if armed && apex_ratio >= apex_gate
//!  window, per tick         R_hold    = hold_share · w_success · dt/window · quality
//!  window closes            R_success = w_success · quality         (THE VERDICT, paid once)
//!  episode ends, not fallen R_stand   = w_stand · apex_ratio · upright  (still on your feet)
//!  episode ends in a fall   charge    = w_fall + every credit above (claw-back)
//! ```
//!
//! `quality = apex_ratio · upright` — and nothing else. It deliberately does NOT
//! require a quiet CoM: the landing wobble is a property of the physics, and
//! gating on `‖v_com‖` made the whole term dead in the sim (measured: quality
//! 0.000 on every tick of every landing window). "Up, and still up at the end of
//! the window" is the outcome we can actually judge.
//!
//! `R_stand` is the answer to "the hold never finished, but the body kept its
//! feet": a non-fall episode that is still grounded and not collapsed is paid
//! `w_stand · apex_ratio · upright` at the end, whatever ended it. It is *not*
//! free: it scales with the apex, so standing still (apex ≈ 0) still earns exactly
//! 0, and a fall claws it back like every other credit.
//!
//! ## Why the landing is *both* dense and terminal
//!
//! The verdict is paid **once, when the window closes**, because "landed" is a
//! state at a moment: the body has held an upright, settled stance for
//! `land_window_ts` (1.5 s, the same clock the harness ends the episode on). A
//! per-frame payment alone would reward *being in* the window rather than
//! *completing* it, and could not tell "landed, held 1.4 s, toppled" from
//! "landed".
//!
//! But a terminal-only payment is nearly invisible to the policy: the action that
//! earns it is the touchdown absorption, and the close is ~135 ticks later. With
//! `gamma = 0.995, lambda = 0.95` the effective GAE horizon is ~`1/(1-g*l) ~ 18`
//! ticks, so a lone event 135 ticks downstream assigns almost no credit to the
//! action that caused it. Hence a **small** per-frame `R_hold` whose only job is
//! credit assignment: it pays `hold_share` (5 %) of the verdict **across the whole
//! window**, so the per-frame amount is `w_success·hold_share/window_ticks` — at
//! 90 Hz over 1.5 s that is ≈ 0.0011/frame against a 3.0 verdict. Because the
//! budget is a *fraction*, neither the tick rate nor the window length can inflate
//! it (a per-frame constant would grow with both).
//!
//! ## Why a crash is *guaranteed* worse than standing still
//!
//! [`JumpReward::fall_charge`] = `w_fall` + every positive credit the episode
//! banked. The harness subtracts it on the measured-fall tick, so a fallen episode
//! nets **exactly `-w_fall`** whether it hopped 70 px or barely twitched. This is
//! not a weight-tuning coincidence (the previous round's `fall 3.0` vs `apex 2.5`
//! was, and it dominated the whole objective); it is an identity, and
//! `a_crashed_hop_is_strictly_worse_than_standing_still` locks it.
//!
//! The ordering that results:
//!
//! ```text
//!   clean landing  ~ +4.0     (verdict + hold + takeoff + air)
//!   stand still    =   0.0
//!   crash          =  -0.5    (flat, any hop size)
//! ```
//!
//! ## Why this replaced the two-checkpoint design
//!
//! The previous reward was measured over 7 941 episodes / 248 updates
//! (`tmp/walker_ppo/episodes.csv`) to be optimising the wrong thing:
//!
//! ```text
//!   fall-terminated 90.1%   landed 8.8%   mean ep_len 158 of 288 ticks
//!   per episode: fall_pen -2.702, alive +0.792, apex_peak +0.699,
//!                pen -0.312, settle -0.087, land_pay +0.064,
//!                apex_pay +0.044, liftoff_chg -0.031  =>  -1.533
//! ```
//!
//! **The only term large enough to steer the policy was the fall penalty**, so
//! the objective the net actually optimised was "do not fall", not "jump". The
//! one term that priced the goal was broken: of the 3 423 episodes whose hip rose
//! > 0.5 standing, **56 % were paid for a 3.5 px rise while the body flew ~73 px**,
//! because the apex event fired on `v_com.y >= 0`, and `Fiction::v_com` is
//! EMA-filtered and reads ≈ 0 on the takeoff tick (`apex_pay` — which prices
//! `v_com.y²/2g` at liftoff — totalled **+0.044 for the whole run**). The
//! once-per-flight payment was burned one tick after takeoff.
//!
//! Hence three structural rules here:
//!
//! 1. **No per-tick penalty of substance.** A surviving tick is worth exactly
//!    what the behaviour is worth: standing still earns **0**, and the only way
//!    to earn is to leave the ground and come back — so there is no
//!    "do-nothing" degenerate optimum to fall into.
//! 2. **No event may depend on a filtered velocity.** The takeoff velocity is the
//!    one-tick difference of the *measured* CoM position
//!    ([`Fiction::com`], unfiltered). `Fiction::v_com` is EMA'd and is therefore
//!    never used for an event (regression:
//!    `an_ema_lagged_v_com_cannot_kill_the_liftoff_term`).
//! 3. **The apex is a monotone maximum, evaluated at touchdown.** It is tracked
//!    over the whole airborne span, so it cannot be missed by sampling, and it is
//!    the CoM (mass-weighted), not the hip, so tucking the legs cannot raise it.
//!
//! ## What is deliberately NOT here
//!
//! Torque/work/effort and action-smoothness terms — the "principle of least
//! action" half of the design. They are *approximable* from the same particle
//! state the observer already samples, but they are a separate experiment:
//! see `plans/jump-reward-idea2-least-action.md`. The residual-effort term
//! (`‖δ‖²`) is the one that matters most for a residual controller, because it is
//! what makes "suppress the expert's own stroke" cost something.
//!
//! Termination is **not** here: the collector owns the measured-fall gate and
//! the episode horizon. This module owns only the per-tick scalar.

use walker_control::observe::Fiction;

/// Weights, gates and geometry for the task-outcome reward. Every field is a
/// decision knob; the harness overrides a few from env.
#[derive(Debug, Clone, Copy)]
pub struct RewardConfig {
    /// **Takeoff** credit, paid once on the tick both feet leave the ground.
    /// Prices upward CoM momentum, which internal forces cannot manufacture: you
    /// can fold your legs, but you cannot move your own CoM up without the floor.
    pub w_liftoff: f32,
    /// CoM rise-rate (px/s) that saturates [`Self::w_liftoff`]. ≈ 2.5 m/s.
    pub v_jump: f32,
    /// CoM rise-rate (px/s) the takeoff must reach to **arm the apex gate**.
    /// `50 px/s = 0.5 m/s` (this rig is 1 px ≈ 1 cm; `g = 980 px/s²`).
    pub v_min: f32,
    /// CoM rise above the episode's measured standing CoM that saturates
    /// `apex_ratio` (px).
    ///
    /// **Calibrated on the CoM, not the hip.** A measured expert hop reaches
    /// ≈ 70 px of *hip* rise but only ≈ 46–51 px of *CoM* rise (the legs hang
    /// below the hip), so the old `apex_peak_target = 60` — which was a hip
    /// number — made a real hop read 0.77 and miss an 0.8 gate. Measured on the
    /// smoke run: `apex_ratio 0.771 … 0.853` at `apex_target = 60`, i.e. 46–51 px
    /// of CoM rise. 45 px saturates an honest hop with margin.
    pub apex_target: f32,
    /// `apex_ratio` the flight must reach before the landing window opens at all.
    /// This is the anti-micro-hop gate: no height, no landing credit.
    pub apex_gate: f32,
    /// Dense airborne credit, per second, scaled by `apex_ratio`. This is
    /// exactly the goal ("be higher"), so it gives the launch a gradient before
    /// any landing exists. A rate, not a fraction of the verdict, because it is an
    /// objective and not credit assignment.
    pub w_air: f32,
    /// Fraction of [`Self::w_success`] paid **across the whole landing window**,
    /// scaled by `quality`. Credit assignment only — it teaches the touchdown
    /// absorb, it never decides anything.
    ///
    /// Deliberately a fraction and not a per-tick (or per-second) weight: on a
    /// 90 Hz controller a per-frame constant silently grows with the window length
    /// and the tick rate. Here the *total* window credit is `w_success ·
    /// hold_share` regardless, so the per-frame value is
    /// `w_success · hold_share / (land_window_ts · 90)`.
    pub hold_share: f32,
    /// **The verdict.** Paid once, at the window close, scaled by `quality` at
    /// that moment. Must dominate every other term.
    pub w_success: f32,
    /// **Still standing at the end.** Paid once when the episode ends *without* a
    /// fall and the body is grounded and not collapsed, scaled by
    /// `apex_ratio · upright`. Catches the "the hold did not finish, but the
    /// character kept its feet" case — the wobble is physics, and a policy that
    /// stays up is on the right track.
    pub w_stand: f32,
    /// The **net** cost of a measured fall, after every positive credit the
    /// episode banked has been clawed back ([`JumpReward::fall_charge`]). So a
    /// fallen episode is worth exactly `-w_fall`: worse than standing still
    /// (0), at any hop height, by construction rather than by weight tuning.
    pub w_fall: f32,
    /// `|lean|` (rad) at which the upright factor reaches zero. This is a
    /// **collapse** angle, not a "good enough" angle: set it to the range the body
    /// actually occupies while landing (`≈1.0 rad ≈ 57°`), so the factor still
    /// gives a usable gradient at 0.4–0.6 rad instead of cliffing to zero there.
    pub lean_fail: f32,
    /// Landing-window length (s). Matches `ControlConfig::jump_land_hold`, which
    /// is also the harness's "landed" trigger — keep them equal so the reward's
    /// verdict and the episode boundary are the same moment.
    pub land_window_ts: f32,
    /// Fallback fixed-step length when the fiction carries `dt == 0`.
    pub dt: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self {
            // A real hop on this rig launches at ≈ 370 px/s, so 250 saturates for
            // an honest jump and refuses to pay for a wobble.
            w_liftoff: 1.0,
            v_jump: 250.0,
            v_min: 50.0,
            // 45 px of CoM rise = an honest expert hop (measured 46–51 px).
            apex_target: 45.0,
            // 0.8 ⇒ 36 px of CoM rise. A micro-hop (~20 px) cannot unlock the
            // landing; a real hop clears it.
            apex_gate: 0.8,
            // Per SECOND (so the tick rate cannot change the budget): a 0.7 s
            // flight pays ≈ 0.7 at full apex. This is a real objective — height —
            // which is why it is a rate and not a fraction of the verdict.
            w_air: 1.0,
            // 5 % of the verdict, spread across the window, for credit assignment.
            hold_share: 0.05,
            w_success: 3.0,
            // A third of the verdict: "up and not collapsed" is most of the way
            // there, but the completed hold must still be worth more.
            w_stand: 1.0,
            // The flat cost of a fall (after the claw-back). Small on purpose: it
            // only has to break the tie against standing still, and a large flat
            // charge is what made "never jump" the optimum last round.
            w_fall: 0.5,
            // Collapse scale, deliberately WIDE (measured: the body lands at
            // ~0.5 rad lean; a tight band made the quality factor ≲1e-3, i.e. no
            // gradient at all).
            lean_fail: 1.0,
            land_window_ts: 1.5,
            dt: 1.0 / 90.0,
        }
    }
}

/// Milestones the collector can log / branch on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RewardEvents {
    /// This tick was the first airborne tick after ground contact — takeoff.
    pub liftoff: bool,
    /// This tick the flight touched down **and passed the apex gate**, so the
    /// landing window opened. A touchdown that fails the gate reports `false`.
    pub apex_reached: bool,
    /// This tick the landing window closed successfully — the payment tick.
    pub landed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Grounded,
    Flight,
    LandWindow,
}

/// Per-character task-outcome reward state machine.
#[derive(Debug, Clone)]
pub struct JumpReward {
    cfg: RewardConfig,
    phase: Phase,
    /// Previous tick's measured CoM height (world y) — the one-tick rise-rate.
    /// Only `y` is needed, so no vector type crosses this crate boundary.
    prev_com_y: Option<f32>,
    /// The episode's standing CoM height (world y, vello y-down), captured on the
    /// first tick of the episode. Zero-height reference for `apex_ratio`; the
    /// episode always opens on the canonical settled stance.
    standing_com: Option<f32>,
    /// Takeoff cleared `v_min` — the landing window may open this flight.
    armed: bool,
    /// Monotone max `apex_ratio` of the **whole episode** — the gate and the
    /// quality scale, and the metric the checkpoint is chosen on (the harness's
    /// `peak_height`). Episode-wide on purpose: a per-flight value is destroyed
    /// by the settle bounce that follows every touchdown.
    episode_ratio: f32,
    /// Ticks spent inside the current landing window.
    land_ticks: u32,
    /// The `quality` seen on the most recent window tick / close.
    last_quality: f32,
    // ---- introspection (logged, not acted on) ----
    /// Takeoffs this episode (armed or not — this counts the expert's launches).
    pub liftoffs: u32,
    /// Landings this episode: windows that closed with the gate passed.
    pub landings: u32,
    /// The takeoff CoM rise-rate measured on the most recent takeoff (px/s).
    pub last_v_up: f32,
    /// The **largest** takeoff CoM rise-rate of the episode (px/s) — the number
    /// `v_jump` / `v_min` should be calibrated against. Logged per episode.
    pub max_v_up: f32,
    /// Grounded ticks spent inside the landing window this episode. Distinguishes
    /// "the window never opened" (0) from "it opened but never settled" (>0).
    pub window_ticks: u32,
    /// The most recent takeoff credit (0 outside that tick).
    pub last_liftoff_pay: f32,
    /// The most recent dense airborne credit (0 when grounded).
    pub last_air_pay: f32,
    /// The most recent dense landing-window credit.
    pub last_land_pay: f32,
    /// The payment paid on the most recent window close (0 outside that tick).
    pub last_success_pay: f32,
    /// The still-standing credit paid on the most recent episode end (0 outside).
    pub last_stand_pay: f32,
    /// Episode-cumulative component sums (telemetry / `episodes.csv`).
    pub sum_liftoff_pay: f32,
    pub sum_air_pay: f32,
    pub sum_land_pay: f32,
    pub sum_success_pay: f32,
    pub sum_stand_pay: f32,
}

impl JumpReward {
    pub fn new(cfg: RewardConfig) -> Self {
        Self {
            cfg,
            phase: Phase::Grounded,
            prev_com_y: None,
            standing_com: None,
            armed: false,
            episode_ratio: 0.0,
            land_ticks: 0,
            last_quality: 0.0,
            liftoffs: 0,
            landings: 0,
            last_v_up: 0.0,
            max_v_up: 0.0,
            window_ticks: 0,
            last_liftoff_pay: 0.0,
            last_air_pay: 0.0,
            last_land_pay: 0.0,
            last_success_pay: 0.0,
            last_stand_pay: 0.0,
            sum_liftoff_pay: 0.0,
            sum_air_pay: 0.0,
            sum_land_pay: 0.0,
            sum_success_pay: 0.0,
            sum_stand_pay: 0.0,
        }
    }

    #[cfg(test)]
    pub fn cfg(&self) -> &RewardConfig {
        &self.cfg
    }

    /// The tallest CoM rise of the episode as a fraction of `apex_target`
    /// (monotone). This is **the** height metric — the harness logs it as
    /// `peak_height` and selects the best checkpoint on it, so the number the
    /// reward prices and the number the checkpoint is chosen on are identical.
    pub fn apex_ratio(&self) -> f32 {
        self.episode_ratio
    }

    /// The most recent close's `quality` (0..1) — the fraction of `w_success`
    /// the verdict paid.
    pub fn last_quality(&self) -> f32 {
        self.last_quality
    }

    /// Every positive credit this episode has banked (takeoff + airborne + hold
    /// + verdict). A fall claws all of it back.
    pub fn credits(&self) -> f32 {
        self.sum_liftoff_pay
            + self.sum_air_pay
            + self.sum_land_pay
            + self.sum_success_pay
            + self.sum_stand_pay
    }

    /// The charge a measured fall applies: `w_fall` **plus every credit the
    /// episode banked**. Clawing back the credits is what makes a crash worth
    /// exactly `-w_fall` whether the hop was 70 px or barely a twitch — otherwise
    /// a tall hop that crashes out-earns standing still, which is precisely the
    /// degenerate optimum the previous reward fell into.
    ///
    /// The harness subtracts this on the tick it ends the episode.
    pub fn fall_charge(&self) -> f32 {
        self.cfg.w_fall + self.credits()
    }

    /// Reset at an episode boundary (respawn). Carries nothing across: a new
    /// episode re-measures its own standing CoM.
    pub fn reset(&mut self) {
        self.phase = Phase::Grounded;
        self.prev_com_y = None;
        self.standing_com = None;
        self.armed = false;
        self.episode_ratio = 0.0;
        self.land_ticks = 0;
        self.last_quality = 0.0;
        self.liftoffs = 0;
        self.landings = 0;
        self.last_v_up = 0.0;
        self.max_v_up = 0.0;
        self.window_ticks = 0;
        self.last_liftoff_pay = 0.0;
        self.last_air_pay = 0.0;
        self.last_land_pay = 0.0;
        self.last_success_pay = 0.0;
        self.last_stand_pay = 0.0;
        self.sum_liftoff_pay = 0.0;
        self.sum_air_pay = 0.0;
        self.sum_land_pay = 0.0;
        self.sum_success_pay = 0.0;
        self.sum_stand_pay = 0.0;
    }

    /// One tick of reward. Never returns a negative value: every charge lives in
    /// the harness (the measured-fall gate) so that a surviving tick always reads
    /// non-negative and "stand still" is exactly 0.
    pub fn step(&mut self, f: &Fiction) -> (f32, RewardEvents) {
        let c = self.cfg;
        let dt = if f.dt > 0.0 { f.dt } else { c.dt };

        // The standing reference: the episode opens on the canonical settled
        // stance, so the first measured CoM **is** the zero. Captured once.
        let standing = *self.standing_com.get_or_insert(f.com.y);

        // One-tick CoM rise-rate. `Fiction::com` is the unfiltered mass-weighted
        // centre; `Fiction::v_com` is EMA'd and reads ≈ 0 on the takeoff tick,
        // which is exactly the bug that made the old `apex_pred` worth +0.044 for
        // an entire 248-update run.
        let v_up = match self.prev_com_y {
            Some(prev_y) => (prev_y - f.com.y) / dt,
            None => 0.0,
        };
        self.prev_com_y = Some(f.com.y);

        // Rise above the episode's standing CoM (up is -y). Monotone-max'd below.
        let ratio_now = ((standing - f.com.y) / c.apex_target.max(1e-6)).clamp(0.0, 1.0);

        let mut events = RewardEvents::default();
        let mut r = 0.0f32;
        self.last_liftoff_pay = 0.0;
        self.last_air_pay = 0.0;
        self.last_land_pay = 0.0;
        self.last_success_pay = 0.0;

        match self.phase {
            Phase::Grounded => {
                if !f.grounded {
                    // ---- TAKEOFF · both feet off the floor ----
                    // Price the upward momentum the launch actually delivered.
                    // This is the "a lunge is not a jump" term: the expert always
                    // fires its impulse, but only a launch that moved the CoM pays.
                    self.last_v_up = v_up;
                    self.max_v_up = self.max_v_up.max(v_up);
                    if v_up > c.v_min {
                        self.armed = true;
                        let lift = (v_up / c.v_jump.max(1e-6)).clamp(0.0, 1.0);
                        let pay = c.w_liftoff * lift;
                        r += pay;
                        self.last_liftoff_pay = pay;
                        self.sum_liftoff_pay += pay;
                        self.liftoffs += 1;
                    }
                    self.phase = Phase::Flight;
                    events.liftoff = true;
                }
            }
            Phase::Flight => {
                // ---- AIRBORNE · dense credit for the rise ----
                // Instantaneous, not the flight's max: the term means "be high
                // NOW", and paying the max would let a bounce bank the old apex.
                self.episode_ratio = self.episode_ratio.max(ratio_now);
                let pay = c.w_air * dt * ratio_now;
                r += pay;
                self.last_air_pay = pay;
                self.sum_air_pay += pay;

                if f.grounded {
                    // The gate is judged on the **episode's** apex, not this
                    // flight's. Measured on the first smoke run: the touchdown
                    // tick is immediately followed by an airborne bounce, so a
                    // per-flight apex was reset 4× per episode and a 1.0 apex
                    // never opened a window.
                    if self.armed && self.episode_ratio >= c.apex_gate {
                        // ---- TOUCHDOWN · the gate passed, the window opens ----
                        self.phase = Phase::LandWindow;
                        self.land_ticks = 0;
                        events.apex_reached = true;
                    } else {
                        // A micro-hop or a launch that never cleared `v_min`
                        // earns no landing credit at all.
                        self.phase = Phase::Grounded;
                    }
                }
            }
            Phase::LandWindow => {
                if f.grounded {
                    let quality = self.quality(f);
                    self.last_quality = quality;
                    // Rate- and window-invariant: the WHOLE window pays
                    // `w_success * hold_share`, however many ticks that is.
                    let per_tick = c.w_success * c.hold_share * dt / c.land_window_ts.max(1e-6);
                    let pay = per_tick * quality;
                    r += pay;
                    self.last_land_pay = pay;
                    self.sum_land_pay += pay;
                    self.land_ticks += 1;
                    self.window_ticks += 1;
                    if (self.land_ticks as f32) * dt >= c.land_window_ts {
                        // ---- THE VERDICT · paid once, at the close ----
                        let pay = c.w_success * quality;
                        r += pay;
                        self.last_success_pay = pay;
                        self.sum_success_pay += pay;
                        self.landings += 1;
                        self.phase = Phase::Grounded;
                        self.armed = false;
                        events.landed = true;
                    }
                } else {
                    // A settle bounce: the body left the floor again. The window
                    // SURVIVES (the flight already earned the unlock) but the
                    // HOLD CLOCK restarts — "grounded and settled for
                    // `land_window_ts`" is the verdict's definition, so a ricochet
                    // must not count toward it.
                    self.land_ticks = 0;
                }
            }
        }

        (r, events)
    }

    /// Landing quality ∈ `[0,1]`: apex passed × upright × settled.
    ///
    /// ```text
    ///   quality = apex_ratio · (1 − |lean|/lean_fail) · (1 − ‖v_com‖/v_fail)
    /// ```
    ///
    /// Both factors are **linear and wide**, and their cliffs are at *collapse*,
    /// not at "acceptable":
    ///
    /// - `|lean| = lean_fail ≈ 1.0 rad` — past the fall gate's 0.7 rad, i.e. they
    ///   only reach zero once the body is genuinely going over;
    /// - `‖v_com‖ = v_fail ≈ 250 px/s` — the speed of a body that never stopped.
    ///
    /// This shape is measured, not stylistic. Two tighter versions were tried on
    /// the real sim and both made the whole term dead:
    ///
    /// | factor | at the observed landing (0.5 rad, ~100 px/s) |
    /// |---|---|
    /// | `1 − x/0.35` (clamped) | **0** ⇒ no gradient, every episode constant −0.5 |
    /// | `exp(−3(x/0.35)²)` | **≈2e-6** ⇒ still numerically no gradient |
    /// | `1 − x/1.0` (this) | **0.5 × 0.6 = 0.30** ⇒ usable |
    ///
    /// A shaping term may pay little for a bad landing, but it must never be
    /// identically zero over the range the body actually operates in — a zero
    /// cannot be improved on.
    fn quality(&self, f: &Fiction) -> f32 {
        let c = self.cfg;
        let upright = 1.0 - (f.lean.abs() / c.lean_fail.max(1e-6)).min(1.0);
        self.episode_ratio * upright
    }

    /// The **still-standing** credit: paid once when the episode ends without a
    /// fall while the body is on the ground and not collapsed.
    ///
    /// Scales with `apex_ratio · upright`, so it is:
    /// - **0 for standing still** (apex ≈ 0) — no free lunch, no degenerate optimum;
    /// - monotone in both "how high did you get" and "how upright did you end";
    /// - clawed back by [`Self::fall_charge`] if the episode ends in a fall, so it
    ///   cannot make a crash profitable.
    ///
    /// This is the answer to "the 1.5 s hold never finished, but the character kept
    /// its feet". The wobble is a property of the physics; a policy that is still
    /// on its feet at the end of the episode is on the right track and must be told
    /// so, or the only gradient left is the flat crash cost.
    pub fn pay_stand(&mut self, f: &Fiction) -> f32 {
        if !f.grounded {
            return 0.0;
        }
        let upright = 1.0 - (f.lean.abs() / self.cfg.lean_fail.max(1e-6)).min(1.0);
        let pay = self.cfg.w_stand * self.episode_ratio * upright;
        self.last_stand_pay = pay;
        self.sum_stand_pay += pay;
        pay
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::prelude::Vec2;
    use walker_control::observe::FootState;

    const SURFACE: f32 = 312.0;
    const HIP: f32 = 104.0;
    const DT: f32 = 1.0 / 90.0;

    /// A standing CoM position. `rise` is px above standing (up is -y).
    fn com_at(rise: f32) -> Vec2 {
        Vec2::new(0.0, SURFACE - HIP - rise)
    }

    /// Hip follows the CoM (these tests are arithmetic, not kinematics), feet sit
    /// on the surface, `touching == grounded`.
    fn fiction(com: Vec2, grounded: bool, lean: f32, v_com: Vec2) -> Fiction {
        Fiction {
            dt: DT,
            hip: com,
            v_hip: Vec2::ZERO,
            lean,
            lean_rate: 0.0,
            com,
            v_com,
            ang_momentum: 0.0,
            omega_com: 0.0,
            feet: [
                FootState {
                    pos: Vec2::new(-20.0, SURFACE),
                    vel: Vec2::ZERO,
                    touching: grounded,
                },
                FootState {
                    pos: Vec2::new(20.0, SURFACE),
                    vel: Vec2::ZERO,
                    touching: grounded,
                },
            ],
            grounded,
        }
    }

    fn standing() -> Fiction {
        fiction(com_at(0.0), true, 0.0, Vec2::ZERO)
    }

    /// A takeoff tick: `rise` px higher than the previous tick, moving up fast
    /// enough to saturate `w_liftoff`.
    fn takeoff(rise: f32) -> Fiction {
        fiction(com_at(rise), false, 0.0, Vec2::new(0.0, -400.0))
    }

    fn cfg_default() -> RewardConfig {
        RewardConfig::default()
    }

    /// Ride the landing window out on a held stance.
    fn settle_window(rw: &mut JumpReward, f: &Fiction) -> f32 {
        let ticks = (rw.cfg().land_window_ts / DT).ceil() as u32 + 2;
        let mut total = 0.0;
        for _ in 0..ticks {
            total += rw.step(f).0;
        }
        total
    }

    /// The baseline that broke the old reward: standing must be worth **exactly
    /// nothing**, so no "do-nothing" policy can out-earn a jump.
    #[test]
    fn standing_still_pays_nothing() {
        let mut rw = JumpReward::new(cfg_default());
        for _ in 0..200 {
            let (r, ev) = rw.step(&standing());
            assert_eq!(r, 0.0, "a surviving tick must pay exactly 0");
            assert_eq!(ev, RewardEvents::default());
        }
        assert_eq!(rw.liftoffs, 0);
        assert_eq!(rw.landings, 0);
    }

    /// Takeoff is priced from the measured CoM displacement, and a real launch
    /// saturates the credit.
    #[test]
    fn a_real_launch_pays_the_liftoff_term() {
        let cfg = cfg_default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        // 5 px in one tick = 450 px/s > v_jump (250).
        let (r, ev) = rw.step(&takeoff(5.0));
        assert!(ev.liftoff);
        assert_eq!(rw.liftoffs, 1);
        assert!((rw.last_v_up - 450.0).abs() < 1.0, "v_up {}", rw.last_v_up);
        assert!(
            (r - cfg.w_liftoff).abs() < 1e-3,
            "a saturated launch must pay full w_liftoff: {r}"
        );
        assert!(rw.armed);
    }

    /// **The regression that matters.** `Fiction::v_com` is EMA-filtered and
    /// reads ≈ 0 on the takeoff tick; the old design priced `v_com²/2g` there and
    /// banked **+0.044 for a whole 248-update run**. The takeoff term must ride the
    /// measured CoM displacement, so a zero EMA velocity cannot zero it.
    #[test]
    fn an_ema_lagged_v_com_cannot_kill_the_liftoff_term() {
        let cfg = cfg_default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        // The filter says "not moving"; the body measurably rose 5 px this tick.
        let mut f = takeoff(5.0);
        f.v_com = Vec2::ZERO;
        f.v_hip = Vec2::ZERO;
        let (r, ev) = rw.step(&f);
        assert!(ev.liftoff);
        assert!(
            r > cfg.w_liftoff * 0.99,
            "the EMA must not gate an event credit: {r}"
        );
    }

    /// A takeoff that does not clear `v_min` (a leg-tuck: the feet leave the
    /// floor, the CoM barely moves) pays nothing and cannot unlock the landing.
    #[test]
    fn a_suppressed_launch_pays_nothing_and_cannot_unlock_the_landing() {
        let cfg = cfg_default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        // 0.2 px in a tick ≈ 18 px/s < v_min (50).
        let (r, ev) = rw.step(&fiction(com_at(0.2), false, 0.0, Vec2::ZERO));
        assert!(ev.liftoff, "the feet did leave the ground");
        assert_eq!(r, 0.0, "a tuck is not a launch: {r}");
        assert!(!rw.armed);

        // Even if it somehow reached the apex, the landing payout is forfeit.
        for rise in [20.0, 40.0, 60.0, 40.0, 20.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        let (_, ev) = rw.step(&standing());
        assert!(!ev.apex_reached, "an unarmed flight cannot pass the gate");
        let total = settle_window(&mut rw, &standing());
        assert_eq!(rw.sum_success_pay, 0.0, "no payment: {total}");
        assert_eq!(rw.landings, 0);
    }

    /// The apex is the **maximum** over the airborne span, not the last sample:
    /// a flight that peaks at 70 px and lands at 10 px still passes the gate.
    #[test]
    fn the_apex_is_the_maximum_of_the_whole_flight() {
        let mut rw = JumpReward::new(cfg_default());
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        // Climb past the gate, then descend below it.
        for rise in [30.0, 55.0, 70.0, 50.0, 25.0, 8.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        assert!(
            rw.apex_ratio() > 0.99,
            "70 px against a 45 px target must saturate the ratio: {}",
            rw.apex_ratio()
        );
        let (_, ev) = rw.step(&standing());
        assert!(
            ev.apex_reached,
            "the gate is judged on the flight's max, not the touchdown height"
        );
        settle_window(&mut rw, &standing());
        assert_eq!(rw.landings, 1);
        assert!(
            rw.sum_success_pay > rw.cfg().w_success * 0.95,
            "a clean capture must pay the full verdict: {}",
            rw.sum_success_pay
        );
    }

    /// A micro-hop (rise below `apex_gate`) unlocks nothing.
    #[test]
    fn a_micro_hop_unlocks_no_landing_credit() {
        let mut rw = JumpReward::new(cfg_default());
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        for rise in [10.0, 20.0, 10.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        let (_, ev) = rw.step(&standing());
        assert!(!ev.apex_reached, "20/60 px is below the gate");
        let total = settle_window(&mut rw, &standing());
        assert_eq!(rw.sum_success_pay, 0.0);
        assert_eq!(rw.landings, 0);
        assert_eq!(rw.sum_land_pay, 0.0, "no window, no dense credit: {total}");
    }

    /// The landing window pays for a settled, upright capture and then the
    /// verdict pays the big term at the close.
    #[test]
    fn a_clean_landing_pays_the_window_and_the_verdict() {
        let cfg = cfg_default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        for rise in [30.0, 60.0, 30.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        // Touchdown: back on the standing line, upright, at rest.
        let (_, ev) = rw.step(&standing());
        assert!(ev.apex_reached);
        assert_eq!(rw.sum_success_pay, 0.0, "touchdown is not the payment tick");
        settle_window(&mut rw, &standing());
        assert_eq!(rw.landings, 1);
        // The dense hold credit is a FRACTION of the verdict, independent of the
        // tick rate and of the window length.
        let hold_budget = cfg.w_success * cfg.hold_share;
        assert!(
            (rw.sum_land_pay - hold_budget).abs() < hold_budget * 0.1 + 1e-4,
            "the window pays w_success*hold_share over its whole length: {} vs {hold_budget}",
            rw.sum_land_pay
        );
        assert!(
            rw.sum_land_pay < cfg.w_success * 0.1,
            "the dense hold credit must stay an order of magnitude below the verdict: {}",
            rw.sum_land_pay
        );
        // At the defaults this is 3.0 * 0.05 / (1.5 * 90) ≈ 0.0011 per frame — the
        // per-frame amount is a consequence of the fraction, never a knob.
        assert!(
            cfg.w_success * cfg.hold_share / (cfg.land_window_ts * 90.0) < 0.01,
            "the per-frame hold credit must stay a rounding error against the verdict"
        );
        assert!(
            (rw.sum_success_pay - cfg.w_success).abs() < 1e-3,
            "a perfect capture pays full w_success: {}",
            rw.sum_success_pay
        );
    }

    /// Quality must fall **monotonically** with lean across the range the body
    /// actually lands in (measured: ~0.5 rad), and must still be positive there.
    ///
    /// This is the regression for the two dead shaping shapes that were tried on
    /// the real sim: `1 − x/0.35` (zero at 0.35 ⇒ no gradient) and
    /// `exp(−3(x/0.35)²)` (≈2e-6 at 0.5 rad ⇒ numerically no gradient).
    #[test]
    fn quality_falls_off_monotonically_with_lean() {
        let cfg = cfg_default();
        let mut prev = f32::INFINITY;
        for lean in [0.0, 0.2, 0.4, 0.5, 0.7, 0.9] {
            let mut rw = JumpReward::new(cfg);
            rw.step(&standing());
            rw.step(&takeoff(5.0));
            for rise in [30.0, 60.0, 30.0] {
                rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
            }
            let landed = fiction(com_at(0.0), true, lean, Vec2::ZERO);
            rw.step(&landed);
            settle_window(&mut rw, &landed);
            let q = rw.sum_success_pay / cfg.w_success;
            assert!(
                q < prev + 1e-6,
                "quality must decrease with lean: {lean} rad -> {q} (prev {prev})"
            );
            assert!(
                q > 0.05,
                "a body that lands at {lean} rad must still be improvable, got {q}"
            );
            prev = q;
        }
    }

    /// "Ended on your feet" is worth something even if the 1.5 s hold did not
    /// finish — but it is proportional to the apex, so standing still still earns
    /// exactly 0, and a fall still claws it back.
    #[test]
    fn a_hop_that_stays_up_earns_the_stand_credit() {
        let cfg = cfg_default();

        // No jump at all: nothing to reward.
        let mut still = JumpReward::new(cfg);
        still.step(&standing());
        assert_eq!(
            still.pay_stand(&standing()),
            0.0,
            "standing still is worth 0"
        );

        // A hop that ends grounded and merely leaned (the observed landing state).
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        for rise in [30.0, 45.0, 60.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        let leaned = fiction(com_at(0.0), true, 0.5, Vec2::new(80.0, 0.0));
        rw.step(&leaned);
        let pay = rw.pay_stand(&leaned);
        assert!(
            pay > 0.0 && pay <= cfg.w_stand + 1e-6,
            "a still-standing end must pay, got {pay}"
        );
        assert!(
            (pay - cfg.w_stand * 0.5).abs() < 0.05,
            "half the lean budget at 0.5 rad ⇒ ~half the credit, got {pay}"
        );
        assert!(
            rw.fall_charge() >= pay,
            "a fall claws the stand credit back"
        );
    }

    /// At the *collapse* angle the verdict pays nothing — the cliff is at the
    /// physical limit, not at "acceptable".
    #[test]
    fn a_collapsed_landing_earns_nothing() {
        let cfg = cfg_default();
        let mut rw = JumpReward::new(cfg);
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        for rise in [30.0, 60.0, 30.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        let collapsed = fiction(com_at(0.0), true, cfg.lean_fail, Vec2::ZERO);
        rw.step(&collapsed);
        settle_window(&mut rw, &collapsed);
        assert_eq!(rw.landings, 1, "the window still closes");
        assert!(
            rw.sum_success_pay < 1e-6,
            "past the collapse angle the verdict pays nothing: {}",
            rw.sum_success_pay
        );
    }

    /// A settle bounce must **restart the hold clock** without forfeiting the
    /// flight. Measured on the first smoke run: the touchdown tick is immediately
    /// followed by an airborne tick, so treating a bounce as "window broken" meant
    /// the 1.5 s hold never even started (4 liftoffs, apex 1.0, `land_pay` 0).
    #[test]
    fn a_bounce_restarts_the_hold_clock_without_forfeiting_the_flight() {
        let cfg = cfg_default();
        let mut rw = JumpReward::new(cfg);
        let window_ticks = (cfg.land_window_ts / DT).ceil() as u32;
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        for rise in [30.0, 60.0, 30.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        rw.step(&standing()); // touchdown, window opens
                              // Bounce straight back up.
        for _ in 0..10 {
            rw.step(&fiction(com_at(8.0), false, 0.0, Vec2::ZERO));
        }
        assert_eq!(
            rw.landings, 0,
            "a bounce cannot close the window on its own"
        );
        // Hold for one tick less than a full window: still no verdict.
        for _ in 0..window_ticks.saturating_sub(2) {
            rw.step(&standing());
        }
        assert_eq!(rw.landings, 0, "the hold clock restarted at the bounce");
        // Finish the window.
        for _ in 0..4 {
            rw.step(&standing());
        }
        assert_eq!(rw.landings, 1, "the flight still gets its verdict");
        assert!(
            rw.sum_success_pay > cfg.w_success * 0.9,
            "the capture was clean: {}",
            rw.sum_success_pay
        );
    }

    /// A body that never stops bouncing never settles, so it never gets a verdict.
    #[test]
    fn a_perpetual_bounce_earns_no_verdict() {
        let mut rw = JumpReward::new(cfg_default());
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        for rise in [30.0, 60.0, 30.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        rw.step(&standing());
        // 100 ticks of bouncing (5 up, 1 down) can never accumulate the hold.
        for k in 0..100 {
            let grounded = k % 6 == 0;
            rw.step(&fiction(
                com_at(if grounded { 0.0 } else { 8.0 }),
                grounded,
                0.0,
                Vec2::ZERO,
            ));
        }
        assert_eq!(rw.landings, 0, "a ricochet is not a landing");
        assert_eq!(rw.sum_success_pay, 0.0);
    }

    /// The reward stream pays the height at the event and never takes it back
    /// mid-flight: an event credit must land on the tick that earned it (the old
    /// reward ransomed it and the policy learned to stop jumping). What a fall
    /// costs is a separate, explicit accounting step — [`JumpReward::fall_charge`]
    /// — which is what keeps the ordering auditable.
    #[test]
    fn a_crashed_hop_keeps_the_height_it_bought() {
        let mut rw = JumpReward::new(cfg_default());
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        let mut total = 0.0;
        for rise in [20.0, 45.0, 65.0, 40.0] {
            total += rw
                .step(&fiction(com_at(rise), false, 0.0, Vec2::new(0.0, 60.0)))
                .0;
        }
        let banked = rw.sum_air_pay;
        assert!(banked > 0.0, "airborne credit must accrue: {total}");
        assert!(
            total >= banked - 1e-6,
            "flight ticks never charge against the banked height"
        );
        // Tumble away without ever touching down cleanly.
        for k in 0..30 {
            rw.step(&fiction(
                com_at(40.0 - k as f32 * 2.0),
                false,
                0.4,
                Vec2::new(80.0, 300.0),
            ));
        }
        assert_eq!(rw.landings, 0);
        assert_eq!(rw.sum_success_pay, 0.0);
        assert!(
            (rw.sum_air_pay - banked).abs() < 1e-3 || rw.sum_air_pay > banked,
            "the banked airborne credit is never revoked"
        );
    }

    /// **The ordering guarantee.** A fallen episode nets exactly `-w_fall`, at any
    /// hop height, because [`JumpReward::fall_charge`] claws back every credit the
    /// episode banked. So a crash can never out-earn standing still (0), and a
    /// *taller* crash can never out-earn a smaller one — the exact degenerate
    /// optimum the previous reward had (`fall 3.0` against a tall hop's `+2.0`, so
    /// crashing was profitable, and it was learned).
    #[test]
    fn a_crashed_hop_is_strictly_worse_than_standing_still() {
        let cfg = cfg_default();
        let mut nets = Vec::new();
        for apex in [20.0_f32, 45.0, 70.0] {
            let mut rw = JumpReward::new(cfg);
            rw.step(&standing());
            rw.step(&takeoff(5.0));
            for rise in [apex * 0.4, apex, apex * 0.5] {
                rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
            }
            let earned = rw.credits();
            // What the harness subtracts on the measured-fall tick.
            let net = earned - rw.fall_charge();
            assert!(
                (net + cfg.w_fall).abs() < 1e-4,
                "a fall must net exactly -w_fall, got {net} (earned {earned})"
            );
            assert!(
                net < 0.0,
                "a crash must be worse than standing still: {net}"
            );
            nets.push(net);
        }
        assert!(
            nets.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-4),
            "the crash cost must not depend on the hop height: {nets:?}"
        );
    }

    /// Every tick the reward emits is non-negative: the charges live in the
    /// harness's fall gate, so a surviving tick can never read negative (the old
    /// per-tick integral was how "never jump" became optimal).
    #[test]
    fn the_reward_never_charges_a_surviving_tick() {
        let mut rw = JumpReward::new(cfg_default());
        rw.step(&standing());
        let mut total = 0.0;
        for f in [
            takeoff(5.0),
            fiction(com_at(40.0), false, 0.0, Vec2::ZERO),
            fiction(com_at(10.0), false, 0.3, Vec2::new(50.0, 0.0)),
            fiction(com_at(0.0), true, 0.3, Vec2::new(50.0, 0.0)),
        ] {
            let (r, _) = rw.step(&f);
            total += r;
            assert!(r >= 0.0, "per-tick reward must be >= 0, got {r}");
        }
        assert!(total >= 0.0);
    }

    /// The whole hop must out-earn standing still — the degenerate-optimum guard
    /// that the old reward failed.
    #[test]
    fn a_completed_hop_out_earns_standing_still() {
        let cfg = cfg_default();
        let n = 200;
        let mut still = JumpReward::new(cfg);
        let mut still_total = 0.0;
        for _ in 0..n {
            still_total += still.step(&standing()).0;
        }

        let mut hop = JumpReward::new(cfg);
        let mut hop_total = hop.step(&standing()).0;
        hop_total += hop.step(&takeoff(5.0)).0;
        // A ballistic arc: up, apex, down.
        let arc = [25.0, 50.0, 68.0, 68.0, 55.0, 35.0, 15.0];
        for rise in arc {
            hop_total += hop
                .step(&fiction(com_at(rise), false, 0.0, Vec2::new(0.0, -50.0)))
                .0;
        }
        hop_total += hop.step(&standing()).0;
        hop_total += settle_window(&mut hop, &standing());

        assert!(
            hop.sum_success_pay > cfg.w_success * 0.9,
            "a clean settled capture must pay the verdict: {}",
            hop.sum_success_pay
        );
        assert!(
            hop_total > still_total + cfg.w_success * 0.9,
            "hop {hop_total:.3} must beat standing still {still_total:.3}"
        );
    }

    /// `reset` carries nothing into the next episode — including the standing
    /// reference and the CoM history.
    #[test]
    fn reset_starts_a_fresh_episode() {
        let mut rw = JumpReward::new(cfg_default());
        rw.step(&standing());
        rw.step(&takeoff(5.0));
        for rise in [30.0, 60.0, 30.0] {
            rw.step(&fiction(com_at(rise), false, 0.0, Vec2::ZERO));
        }
        assert!(rw.apex_ratio() > 0.0);
        rw.reset();
        assert_eq!(rw.apex_ratio(), 0.0);
        assert_eq!(rw.liftoffs, 0);
        assert_eq!(rw.landings, 0);
        assert!(rw.prev_com_y.is_none());
        assert!(rw.standing_com.is_none());
        assert!(!rw.armed);
        // The next episode re-measures its own standing CoM and pays nothing for
        // simply standing there.
        let (r, ev) = rw.step(&standing());
        assert_eq!(r, 0.0);
        assert_eq!(ev, RewardEvents::default());
    }
}
