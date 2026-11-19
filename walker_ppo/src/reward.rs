//! The **two-checkpoint jump reward** (`plans/jump-ppo-residual-polar-action.md`
//! §12/§13), factored out of the collector so it is pure arithmetic on a
//! [`Fiction`] and can be unit-tested without spinning up physics.
//!
//! The design closes the sparse-signal hole of the old hop: a jump is judged at
//! exactly two moments, because the physics makes those moments decidable.
//!
//! ```text
//!  last grounded tick ──► CHECKPOINT 1 · liftoff       (the conserved purchase)
//!      (v_com, L, ω)        r = +w_apex·apex_pred/target
//!                              − w_L   · |L| / (hip_height·v_scale)
//!                              − w_spin· |ω_com| / omega_scale
//!  airborne ─────────────► flight: alive only, no shaping (§12.3)
//!  first grounded ────────► CHECKPOINT 2 · landing     (single-event CREDIT)
//!                              ξ      = com.x + v_com.x/ω₀,  ω₀ = √(g/hip_height)
//!                              e_DCM  = |ξ − p_center| / leg_len
//!                              quality = (1−e_DCM)·(1−|ω|/ω_s)·(1−½·post)
//!                              r = +w_land · quality
//!  └─ then a ~land_window_ts settle nudge (small negative shaping, a nudge
//!     only — the verdict was already paid at the checkpoint).
//! ```
//!
//! ## Why the landing must be a *payment*, not an *integral*
//!
//! Checkpoint 2 was originally implemented as a per-tick integral of purely
//! negative terms. That silently inverted the objective: a whole landed hop then
//! read *net-negative* against a policy that simply crouched and never launched
//! (which collects `w_alive` every tick and opens neither checkpoint). The
//! measured symptom was a collapse of `mean_peak_height` 0.64 → 0.34 with zero
//! positive-reward episodes. The fix is to **pay one credit** at the landing
//! tick, scaled by capture quality, and keep the window as a small nudge.
//! `w_alive` is also kept *small* (it is a constant for any non-falling policy
//! on a fixed horizon, so it must not dominate the event credits).
//!
//! `z₀ = hip_height` is **nominal** (not the live, knee-compressed `z`), so `ξ`
//! stays a solid balance indicator while the legs do their job of changing
//! height — matching the gait's own `omega0(cfg.hip_height, …)` call sites.
//!
//! Termination is **not** handled here (measured fall only, `§12.5`); the
//! collector keeps that. This module owns only the per-tick scalar.

use walker_control::observe::Fiction;

/// Weights and normalisers for the two-checkpoint reward. Every field has a
/// sane default; the harness overrides a few from env knobs.
#[derive(Debug, Clone, Copy)]
pub struct RewardConfig {
    /// Constant per-tick alive bonus. Kept **well below** the event credits and
    /// strictly above the per-tick penalty cap: it is a constant on a fixed
    /// horizon, so its only job is to make *falling* worse than *surviving* —
    /// it must never out-earn a completed hop.
    pub w_alive: f32,
    /// Liftoff: positive credit for the height bought, `w_apex·apex_pred/target`.
    /// This prices the *purchase* (the ballistic prediction at the launch tick);
    /// the rendered event is paid by [`Self::w_apex_peak`].
    pub w_apex: f32,
    /// Apex **rise above standing** that saturates the credit, px.
    pub apex_target: f32,
    /// **Apex checkpoint**: credit for the rise the body ACTUALLY reached, paid
    /// ONCE at the apex event (the tick the CoM stops climbing), while still
    /// airborne and *independent of what happens afterwards*.
    ///
    /// This is the fix for the measured collapse: the previous version paid the
    /// peak as a TERMINAL bonus gated on `!fallen`, so the only way to collect
    /// it was to avoid falling — and the cheapest way to avoid falling was to
    /// never jump. Over 220 updates that drove `peak_height` from 0.558 to
    /// 0.050 while `mean_ep_reward` was preserved (`corr(update, peak) =
    /// −0.949`), i.e. the policy learned to suppress the expert's own stroke.
    /// A credit for an EVENT must be paid at the event, not ransomed against a
    /// later outcome the policy can dodge.
    pub w_apex_peak: f32,
    /// Measured rise (px above standing) that saturates [`Self::w_apex_peak`].
    pub apex_peak_target: f32,
    /// Liftoff: charge on the conserved angular momentum `|L|`.
    pub w_liftoff: f32,
    /// Liftoff (optional ablation): charge on the immediate `|ω_com|`.
    pub w_liftoff_spin: f32,
    /// **Landing checkpoint**: single-event credit, paid once on the first
    /// grounded tick after flight and scaled by `quality ∈ [0,1]`. This is the
    /// term that makes "hop and land" beat "crouch and never launch".
    pub w_land: f32,
    /// Settle nudge: balance term on `e_DCM` (small — the verdict is `w_land`).
    pub w_dcm: f32,
    /// Settle nudge: finite-differenced `|ξ̇|` term.
    pub w_xi_dot: f32,
    /// Settle nudge: posture term on `|height−target|`.
    pub w_post: f32,
    /// Settle nudge: spin-delivery term on `|ω_com|`.
    pub w_land_spin: f32,
    /// Per-tick horizontal-speed penalty weight (the lateral-flail killer).
    pub w_lateral: f32,
    /// Anti-dither weight on the L1 change of the policy mean between ticks.
    pub w_act: f32,
    /// Hard cap on the summed per-tick penalty (must stay below `w_alive`, so a
    /// surviving tick never reads net-negative — runs 8/9 collapsed otherwise).
    pub pen_cap: f32,
    /// `|v_hip.x|` above which the body is no longer considered "vertical".
    pub v_gate: f32,
    /// Normalisers: leg length, nominal hip height, gravity, px/s, rad/s.
    pub leg_len: f32,
    pub hip_height: f32,
    pub gravity: f32,
    pub v_scale: f32,
    pub omega_scale: f32,
    /// Target hip height during the land window (px above the floor surface).
    pub land_height_target: f32,
    /// Length of the Land window (seconds).
    pub land_window_ts: f32,
    /// `e_DCM` beyond which the land window ends early (measured crash).
    pub land_window_e: f32,
    /// World y of the floor surface (vello y-down), for `height = surface − hip`.
    pub foot_surface_y: f32,
    /// Fallback fixed-step length when the fiction carries `dt == 0`.
    pub dt: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self {
            // Small on purpose: ~175 ticks × 0.005 ≈ +0.88 for a full episode,
            // which the landing + apex + peak credits (order +2..+3) dwarf.
            w_alive: 0.005,
            w_apex: 0.5,
            apex_target: 40.0,
            // The measured-apex payment. This replaces the old terminal
            // `peak × R_PEAK` bonus, which paid ~1.65 for the flat "time out
            // without ever landing" episodes and so *funded* the degenerate
            // solution. `apex_peak_target = 60 px` ≈ a real hop; saturating at
            // 1.0 caps a hop at +2.5, while `fall_penalty` (3.0) is set ABOVE
            // it so hop-then-crash is still strictly worse than not jumping.
            w_apex_peak: 2.5,
            apex_peak_target: 60.0,
            w_liftoff: 0.3,
            w_liftoff_spin: 0.0,
            w_land: 1.5,
            // The settle window is a NUDGE, not the verdict — these used to be
            // 0.6/0.15/0.2/0.1 and summed to a −5..−10 charge over 27 ticks.
            w_dcm: 0.08,
            w_xi_dot: 0.03,
            w_post: 0.05,
            w_land_spin: 0.03,
            w_lateral: 0.02,
            w_act: 0.02,
            pen_cap: 0.002,
            v_gate: 60.0,
            leg_len: 110.0,
            hip_height: 104.0,
            gravity: 980.0,
            v_scale: 200.0,
            omega_scale: 6.0,
            land_height_target: 104.0,
            land_window_ts: 0.3,
            land_window_e: 1.5,
            foot_surface_y: 312.0,
            dt: 1.0 / 90.0,
        }
    }
}

/// Milestones the collector can log / branch on for telemetry.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RewardEvents {
    /// This tick was the first airborne tick after ground contact — the launch
    /// was priced.
    pub liftoff: bool,
    /// This tick was the first grounded tick after flight — the land window
    /// opened.
    pub landed: bool,
    /// This tick was the **apex event**: airborne and the CoM has stopped
    /// climbing (`v_com.y >= 0`, vello y-down). The measured rise is paid
    /// here, once per flight, regardless of what happens afterwards.
    pub apex_reached: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Grounded,
    Flight,
    LandWindow,
}

/// Per-character two-checkpoint reward state machine.
#[derive(Debug, Clone)]
pub struct JumpReward {
    cfg: RewardConfig,
    phase: Phase,
    /// Ticks spent inside the current Land window.
    land_ticks: u32,
    /// Previous `ξ`, for the finite-differenced `ξ̇` settle judge.
    prev_xi: Option<f32>,
    /// Previous tick's policy mean μ(s), for the anti-dither penalty.
    prev_mu: Option<Vec<f32>>,
    // ---- introspection (logged, not acted on) ----
    /// The `L` priced at the most recent liftoff (px²/s per unit mass).
    pub last_liftoff_l: f32,
    /// The predicted apex (px above surface) at the most recent liftoff.
    pub last_apex_pred: f32,
    /// The most recent `e_DCM` inside a Land window.
    pub last_e_dcm: f32,
    /// The **best** (smallest) `e_DCM` seen at any landing this episode.
    pub min_e_dcm: f32,
    /// Tracks the climb inside the current flight, to detect the apex event.
    pub flight_peak: f32,
    /// Highest rise above standing reached this flight, px.
    pub last_apex_rise: f32,
    /// Credit paid at the most recent apex event (0 outside that tick).
    pub last_apex_peak_pay: f32,
    /// Paid `w_apex_peak·saturate(rise)` credits so far this episode.
    pub sum_apex_peak_pay: f32,
    /// This flight's apex has already been paid — at most one apex credit per
    /// flight, so a mid-air bob cannot farm it.
    flight_apex_paid: bool,
    /// The `w_land·quality` credit paid on the most recent landing (0 outside).
    pub last_land_pay: f32,
    /// The `w_apex·apex` credit paid on the most recent liftoff (0 outside).
    pub last_apex_pay: f32,
    /// Episode-cumulative `w_land·quality` paid across all landings.
    pub sum_land_pay: f32,
    /// Episode-cumulative `w_apex·apex` paid across all liftoffs.
    pub sum_apex_pay: f32,
    /// Lifetime count of liftoffs / landings for this episode (reset()).
    pub liftoffs: u32,
    pub landings: u32,
    // ---- episode-cumulative component accounting (for telemetry only) ----
    pub sum_alive: f32,
    pub sum_pen: f32,
    pub sum_settle: f32,
    pub sum_liftoff_charge: f32,
}

impl JumpReward {
    pub fn new(cfg: RewardConfig) -> Self {
        Self {
            cfg,
            phase: Phase::Grounded,
            land_ticks: 0,
            prev_xi: None,
            prev_mu: None,
            last_liftoff_l: 0.0,
            last_apex_pred: 0.0,
            last_e_dcm: 0.0,
            min_e_dcm: f32::MAX,
            flight_peak: 0.0,
            last_apex_rise: 0.0,
            last_apex_peak_pay: 0.0,
            sum_apex_peak_pay: 0.0,
            flight_apex_paid: false,
            last_land_pay: 0.0,
            last_apex_pay: 0.0,
            sum_land_pay: 0.0,
            sum_apex_pay: 0.0,
            liftoffs: 0,
            landings: 0,
            sum_alive: 0.0,
            sum_pen: 0.0,
            sum_settle: 0.0,
            sum_liftoff_charge: 0.0,
        }
    }

    #[cfg(test)]
    pub fn cfg(&self) -> &RewardConfig {
        &self.cfg
    }

    /// Reset at an episode boundary (teleport). Carries no state across.
    pub fn reset(&mut self) {
        self.phase = Phase::Grounded;
        self.land_ticks = 0;
        self.prev_xi = None;
        self.prev_mu = None;
        self.last_liftoff_l = 0.0;
        self.last_apex_pred = 0.0;
        self.last_e_dcm = 0.0;
        self.min_e_dcm = f32::MAX;
        self.flight_peak = 0.0;
        self.last_apex_rise = 0.0;
        self.last_apex_peak_pay = 0.0;
        self.sum_apex_peak_pay = 0.0;
        self.flight_apex_paid = false;
        self.last_land_pay = 0.0;
        self.last_apex_pay = 0.0;
        self.sum_land_pay = 0.0;
        self.sum_apex_pay = 0.0;
        self.liftoffs = 0;
        self.landings = 0;
        self.sum_alive = 0.0;
        self.sum_pen = 0.0;
        self.sum_settle = 0.0;
        self.sum_liftoff_charge = 0.0;
    }

    /// One tick of reward. `mu` is the policy mean that produced the action
    /// returned in the *same* `last` record (for Δμ, the anti-dither term).
    pub fn step(&mut self, f: &Fiction, mu: Option<&[f32]>) -> (f32, RewardEvents) {
        let c = self.cfg;
        let dt = if f.dt > 0.0 { f.dt } else { c.dt };
        let height_px = c.foot_surface_y - f.hip.y;

        // Anti-dither: L1 change of the policy mean since last tick.
        let d_action = match (&self.prev_mu, mu) {
            (Some(p), Some(m)) if p.len() == m.len() => p
                .iter()
                .zip(m.iter())
                .map(|(a, b): (&f32, &f32)| (b - a).abs())
                .sum::<f32>(),
            _ => 0.0,
        };
        if let Some(m) = mu {
            self.prev_mu = Some(m.to_vec());
        }
        let lateral = f.v_hip.x.abs();
        let pen = ((lateral / 100.0) * c.w_lateral + d_action * c.w_act).min(c.pen_cap);

        let mut events = RewardEvents::default();
        let mut r = c.w_alive;
        self.sum_alive += c.w_alive;
        self.last_land_pay = 0.0;
        self.last_apex_pay = 0.0;
        self.last_apex_peak_pay = 0.0;

        // ---- phase transitions ----
        match self.phase {
            Phase::Grounded => {
                if !f.grounded {
                    // Liftoff checkpoint (first airborne tick). `apex_pred` is
                    // the *rise above standing* the launch bought — the quantity
                    // we actually render, and what `apex_target` saturates.
                    let rise = height_px - c.hip_height;
                    let apex_pred = rise + f.v_com.y * f.v_com.y / (2.0 * c.gravity.max(1e-6));
                    let l_norm = (f.ang_momentum.abs() / (c.hip_height * c.v_scale)).max(0.0);
                    let spin = (f.omega_com.abs() / c.omega_scale).max(0.0);
                    let apex = (apex_pred / c.apex_target.max(1e-6)).clamp(0.0, 1.0);
                    let charge = -c.w_liftoff * l_norm - c.w_liftoff_spin * spin;
                    r += c.w_apex * apex + charge;
                    self.sum_liftoff_charge += charge;
                    self.last_liftoff_l = f.ang_momentum;
                    self.last_apex_pred = apex_pred;
                    self.last_apex_pay = c.w_apex * apex;
                    self.sum_apex_pay += self.last_apex_pay;
                    self.liftoffs += 1;
                    events.liftoff = true;
                    // Arm the apex event for THIS flight. `rise` is the height
                    // already bought above standing; the tracker only ever
                    // grows, so the apex payment cannot be gamed by bobbing.
                    self.flight_peak = rise;
                    self.last_apex_rise = 0.0;
                    self.flight_apex_paid = false;
                    self.phase = Phase::Flight;
                }
            }
            Phase::Flight => {
                // ---- CHECKPOINT 1b · the apex EVENT ----
                // Pay the rise the body ACTUALLY reached, on the tick the CoM
                // stops climbing, *while airborne*. Crucially this is not gated
                // on `!fallen`: the credit is for the event, so a hop that is
                // later mis-landed still banked its height. Ransomning an event
                // credit against a later outcome is what taught the policy to
                // suppress the stroke (measured: peak 0.558 → 0.050 over 220
                // updates while reward was preserved).
                let rise = height_px - c.hip_height;
                if rise > self.flight_peak {
                    self.flight_peak = rise;
                }
                // `v_com.y >= 0` ⇒ descending (vello y-down). Guard on having
                // actually risen: a body that never left the standing line (a
                // failed launch, `rise ≈ 0`) must not be paid an apex credit
                // for simply falling back down.
                let apex_now = f.v_com.y >= 0.0 && self.flight_peak > c.apex_peak_target * 0.05;
                if apex_now && !self.flight_apex_paid {
                    let sat = (self.flight_peak / c.apex_peak_target.max(1e-6)).clamp(0.0, 1.0);
                    self.last_apex_peak_pay = c.w_apex_peak * sat;
                    r += self.last_apex_peak_pay;
                    self.sum_apex_peak_pay += self.last_apex_peak_pay;
                    self.last_apex_rise = self.flight_peak;
                    self.flight_apex_paid = true;
                    events.apex_reached = true;
                }
                if f.grounded {
                    // ---- CHECKPOINT 2 OPENS · touchdown ----
                    // No payment here. On the impact tick the CoM is still
                    // travelling forward of the feet (that is *why* the feet
                    // hit the ground), so `e_DCM` is at its worst exactly when
                    // the body looks least captured. Judging here paid ~0 for
                    // every real landing (measured: mean `min_e_dcm` 0.62 —
                    // quality clamped to zero, 1/672 episodes ever paid).
                    // The verdict is taken at the END of the settle window.
                    self.phase = Phase::LandWindow;
                    self.land_ticks = 0;
                    let omega0 = (c.gravity / c.hip_height.max(1e-6)).max(1e-6).sqrt();
                    self.prev_xi = Some(f.com.x + f.v_com.x / omega0);
                    events.landed = true;
                }
            }
            Phase::LandWindow => {
                if !f.grounded {
                    // Bounced back airborne before the capture settled: the
                    // window is BROKEN and earns no landing credit. (Mirrors
                    // the mean-reversion reality: a body that ricochets off the
                    // floor has not landed, it has failed to land.)
                    self.phase = Phase::Flight;
                    self.land_ticks = 0;
                    self.prev_xi = None;
                }
            }
        }

        // ---- CHECKPOINT 2 · the settle window, then one PAYMENT ----
        // Inside the window we only charge a small nudge. The credit is paid
        // ONCE, at the tick the window closes *successfully* (full duration,
        // never crashed) — that is the moment "the body has landed" becomes
        // decidable.
        if self.phase == Phase::LandWindow {
            let omega0 = (c.gravity / c.hip_height.max(1e-6)).max(1e-6).sqrt();
            let xi = f.com.x + f.v_com.x / omega0;
            let p_center = 0.5 * (f.feet[0].pos.x + f.feet[1].pos.x);
            let e_dcm = (xi - p_center).abs() / c.leg_len.max(1e-6);
            let xi_dot = match self.prev_xi {
                Some(prev) => (xi - prev).abs() / dt,
                None => 0.0,
            };
            self.prev_xi = Some(xi);
            let post = (height_px - c.land_height_target).abs() / c.hip_height.max(1e-6);
            let spin = (f.omega_com.abs() / c.omega_scale).max(0.0);
            // Every term is clamped to `[0,1]` *before* weighting, exactly as
            // the verdict's `quality` factors are. `e_DCM` is `|ξ−p|/leg_len`
            // and routinely exceeds 1 (measured mean ≈ 2.0 mid-window), so
            // unclamped it charges −0.17/tick and the window sums to −4.6 per
            // episode — larger than every credit combined. The nudge must be a
            // nudge: it is capped at `pen_cap` per tick like the rest.
            let nudge = -c.w_dcm * e_dcm.min(1.0)
                - c.w_xi_dot * (xi_dot / c.v_scale).min(1.0)
                - c.w_post * post.min(1.0)
                - c.w_land_spin * spin.min(1.0);
            let settle = nudge.max(-c.pen_cap);
            r += settle;
            self.sum_settle += settle;
            self.last_e_dcm = e_dcm;
            self.min_e_dcm = self.min_e_dcm.min(e_dcm);
            self.land_ticks += 1;
            let timed_out = (self.land_ticks as f32) * dt >= c.land_window_ts;
            let crashed = e_dcm > c.land_window_e;
            if crashed || timed_out {
                // ---- the verdict ----
                // A crash pays nothing. A complete settle pays `w_land·quality`
                // where quality is judged on the *settled* state, not the
                // impact state.
                if timed_out && !crashed {
                    let balance = (1.0 - e_dcm).clamp(0.0, 1.0);
                    let settle = (1.0 - spin).clamp(0.0, 1.0);
                    let posture = (1.0 - 0.5 * post).clamp(0.0, 1.0);
                    self.last_land_pay = c.w_land * balance * settle * posture;
                    r += self.last_land_pay;
                    self.sum_land_pay += self.last_land_pay;
                    self.landings += 1;
                }
                self.phase = Phase::Grounded;
                self.prev_xi = None;
                self.land_ticks = 0;
            }
        }

        r -= pen;
        self.sum_pen += pen;
        (r, events)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::prelude::Vec2;
    use walker_control::observe::FootState;

    const SURFACE: f32 = 312.0;
    const HIP: f32 = 104.0;
    /// Standing hip y (vello y-down), directly above the floor surface.
    fn standing_hip() -> Vec2 {
        Vec2::new(0.0, SURFACE - HIP)
    }

    fn fiction(hip: Vec2, com: Vec2, v_com: Vec2, grounded: bool) -> Fiction {
        Fiction {
            dt: 1.0 / 90.0,
            hip,
            v_hip: v_com,
            com,
            v_com,
            ang_momentum: 0.0,
            omega_com: 0.0,
            grounded,
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
            ..Fiction::default()
        }
    }

    #[test]
    fn standing_pays_only_alive() {
        let mut rw = JumpReward::new(RewardConfig::default());
        let f = fiction(standing_hip(), standing_hip(), Vec2::ZERO, true);
        let (r, ev) = rw.step(&f, None);
        assert!((r - rw.cfg().w_alive).abs() < 1e-6, "got {r}");
        assert_eq!(ev, RewardEvents::default());
    }

    #[test]
    fn liftoff_prices_apex_and_spin() {
        let mut rw = JumpReward::new(RewardConfig::default());
        // Start grounded so the next tick is the liftoff transition.
        rw.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        // Airborne 5 px above standing, rising at 200 px/s: apex rise ≈ 5 + 200²/(2g).
        let air = fiction(
            Vec2::new(0.0, SURFACE - HIP - 5.0),
            Vec2::new(0.0, SURFACE - HIP - 5.0),
            Vec2::new(0.0, -200.0),
            false,
        );
        let (r, ev) = rw.step(&air, None);
        assert!(ev.liftoff);
        // Apex credit saturates below target, so reward > alive and < alive+w_apex.
        assert!(r > rw.cfg().w_alive, "apex must pay: {r}");
        assert!(rw.last_apex_pred > 0.0);
        assert!((rw.last_apex_pred - (5.0 + 200.0 * 200.0 / (2.0 * 980.0))).abs() < 1e-3);
    }

    #[test]
    fn a_spun_launch_is_charged() {
        let mut rw = JumpReward::new(RewardConfig::default());
        rw.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        let mut air = fiction(
            Vec2::new(0.0, SURFACE - HIP - 5.0),
            Vec2::new(0.0, SURFACE - HIP - 5.0),
            Vec2::new(0.0, -200.0),
            false,
        );
        air.ang_momentum = 60_000.0; // a huge conserved purchase
        air.omega_com = 3.0;
        let (r, _) = rw.step(&air, None);
        assert!(rw.last_liftoff_l == 60_000.0);
        // A large |L| (l_norm ≈ 2.9) outweighs the apex credit (~0.32): the
        // launch reads negative.
        assert!(r < rw.cfg().w_alive, "spin must dominate the apex: {r}");
    }

    #[test]
    fn flight_is_alive_only_then_the_landing_evaluates_capture() {
        let mut rw = JumpReward::new(RewardConfig::default());
        rw.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        // Liftoff.
        rw.step(
            &fiction(
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, -100.0),
                false,
            ),
            None,
        );
        // Mid-flight: exactly alive.
        let (r, ev) = rw.step(
            &fiction(
                Vec2::new(0.0, SURFACE - HIP + 10.0),
                Vec2::new(0.0, SURFACE - HIP + 10.0),
                Vec2::new(0.0, -10.0),
                false,
            ),
            None,
        );
        assert_eq!(ev, RewardEvents::default());
        assert!((r - rw.cfg().w_alive).abs() < 1e-6, "flight unshaped: {r}");
        // Touchdown does NOT pay — the CoM is still ahead of the feet on the
        // impact tick, so judging here is the bug that paid ~0 for every real
        // landing. It only OPENS the settle window.
        let bad = fiction(
            Vec2::new(0.0, SURFACE - HIP),
            Vec2::new(0.0, SURFACE - HIP),
            Vec2::new(400.0, 0.0),
            true,
        );
        let (r_bad, ev) = rw.step(&bad, None);
        assert!(ev.landed);
        assert!(rw.sum_land_pay == 0.0, "touchdown must not pay yet");
        assert!(
            r_bad > rw.cfg().w_alive - 0.2,
            "the impact tick is not a huge charge either: {r_bad}"
        );

        // A clean settle pays the full credit when the window closes.
        let mut rw2 = JumpReward::new(RewardConfig::default());
        rw2.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        rw2.step(
            &fiction(
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, -100.0),
                false,
            ),
            None,
        );
        let good = fiction(standing_hip(), standing_hip(), Vec2::ZERO, true);
        let (_, ev) = rw2.step(&good, None);
        assert!(ev.landed);
        assert_eq!(rw2.sum_land_pay, 0.0, "not paid on touchdown");
        // Ride out the settle window held perfectly still over the feet.
        let ticks = (rw2.cfg().land_window_ts / (1.0 / 90.0)).ceil() as u32 + 2;
        for _ in 0..ticks {
            rw2.step(&good, None);
        }
        assert_eq!(rw2.landings, 1, "the window must close as one landing");
        assert!(
            rw2.sum_land_pay > rw2.cfg().w_land * 0.9,
            "a clean settled capture must pay: {}",
            rw2.sum_land_pay
        );
    }

    /// The settle nudge must never out-charge the landing credit. `e_DCM` is
    /// unbounded (it is `|ξ−p|/leg_len`), so an unclamped nudge summed to −4.6
    /// per episode against a `w_land` of 1.5 — the same "landing is a net
    /// liability" trap as the original integral, just 4× smaller.
    #[test]
    fn the_settle_window_is_capped_at_the_penalty_cap_per_tick() {
        let c = RewardConfig::default();
        let mut rw = JumpReward::new(c);
        rw.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        rw.step(
            &fiction(
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, -100.0),
                false,
            ),
            None,
        );
        // Touch down in the worst possible state and stay there: huge DCM
        // error, high lateral speed, spinning hard.
        let mut awful = fiction(
            Vec2::new(0.0, SURFACE - HIP + 60.0),
            Vec2::new(0.0, SURFACE - HIP + 60.0),
            Vec2::new(900.0, 0.0),
            true,
        );
        awful.omega_com = 9.0;
        rw.step(&awful, None);
        let ticks = (c.land_window_ts / (1.0 / 90.0)).ceil() as u32 + 2;
        let before = rw.sum_settle;
        for _ in 0..ticks {
            rw.step(&awful, None);
        }
        let per_tick = (rw.sum_settle - before) / ticks as f32;
        assert!(
            per_tick >= -c.pen_cap - 1e-4,
            "settle {per_tick:.4}/tick must be capped at -pen_cap ({}): total {}",
            c.pen_cap,
            rw.sum_settle
        );
    }

    /// The settle window is what makes "landing" decidable: a body that
    /// ricochets straight back off the floor has NOT landed and earns nothing.
    #[test]
    fn a_bounce_off_the_floor_earns_no_landing_credit() {
        let mut rw = JumpReward::new(RewardConfig::default());
        rw.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        rw.step(
            &fiction(
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, -100.0),
                false,
            ),
            None,
        );
        // Touch down, then immediately leave the ground again.
        rw.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        let bounce = fiction(
            Vec2::new(0.0, SURFACE - HIP - 8.0),
            Vec2::new(0.0, SURFACE - HIP - 8.0),
            Vec2::new(0.0, -50.0),
            false,
        );
        for _ in 0..10 {
            rw.step(&bounce, None);
        }
        assert_eq!(rw.landings, 0, "a bounce is not a landing");
        assert_eq!(rw.sum_land_pay, 0.0);
    }

    #[test]
    fn reset_returns_to_grounded_and_clears_introspection() {
        let mut rw = JumpReward::new(RewardConfig::default());
        rw.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        rw.step(
            &fiction(
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, SURFACE - HIP - 5.0),
                Vec2::new(0.0, -200.0),
                false,
            ),
            None,
        );
        assert!(rw.last_apex_pred > 0.0);
        rw.reset();
        assert_eq!(rw.last_apex_pred, 0.0);
        let (r, ev) = rw.step(
            &fiction(standing_hip(), standing_hip(), Vec2::ZERO, true),
            None,
        );
        assert_eq!(ev, RewardEvents::default());
        assert!((r - rw.cfg().w_alive).abs() < 1e-6);
    }

    #[test]
    fn per_tick_penalty_is_capped_below_alive() {
        let mut rw = JumpReward::new(RewardConfig::default());
        assert!(rw.cfg().pen_cap < rw.cfg().w_alive);
        // Fierce lateral flail + wild action change ⇒ penalty capped.
        let mut f = fiction(standing_hip(), standing_hip(), Vec2::new(900.0, 0.0), true);
        f.v_hip = Vec2::new(900.0, 0.0);
        let (r, _) = rw.step(&f, Some(&[0.0, 0.0]));
        assert!(
            r >= rw.cfg().w_alive - rw.cfg().pen_cap - 1e-6,
            "net-negative tick: {r}"
        );
    }

    /// A clean, well-captured hop must out-earn simply crouching and standing
    /// still for the same number of ticks. This is the **degenerate-optimum
    /// guard**: the original per-tick-negative landing integral failed it, and
    /// the policy learned to never launch (`mean_peak_height` collapsed).
    #[test]
    fn a_completed_hop_out_earns_standing_still() {
        let cfg = RewardConfig::default();
        let n = 175; // ≈ one episode at 90 Hz
                     // --- baseline: never launch ---
        let mut still = JumpReward::new(cfg);
        let standing = fiction(standing_hip(), standing_hip(), Vec2::ZERO, true);
        let mut still_total = 0.0;
        for _ in 0..n {
            still_total += still.step(&standing, None).0;
        }
        // --- a hop: launch, apex, descend, clean capture, settle ---
        let mut hop = JumpReward::new(cfg);
        let mut hop_total = 0.0;
        hop_total += hop.step(&standing, None).0; // grounded
                                                  // liftoff, rising fast
        let air_up = fiction(
            Vec2::new(0.0, SURFACE - HIP - 6.0),
            Vec2::new(0.0, SURFACE - HIP - 6.0),
            Vec2::new(0.0, -260.0),
            false,
        );
        hop_total += hop.step(&air_up, None).0;
        // apex / descent
        for k in 0..8 {
            let y = SURFACE - HIP - (30.0 - k as f32 * 4.0);
            let f = fiction(
                Vec2::new(0.0, y),
                Vec2::new(0.0, y),
                Vec2::new(0.0, 20.0 * k as f32),
                false,
            );
            hop_total += hop.step(&f, None).0;
        }
        // touchdown: ξ over the feet (com.x 0, v_com.x 0) ⇒ perfect capture
        let touch = fiction(
            Vec2::new(0.0, SURFACE - HIP),
            Vec2::new(0.0, SURFACE - HIP),
            Vec2::ZERO,
            true,
        );
        let (_, ev) = hop.step(&touch, None);
        assert!(ev.landed, "touchdown must open the landing checkpoint");
        // Ride the settle window to its close — the verdict is paid there.
        for _ in 0..40 {
            hop_total += hop.step(&touch, None).0;
        }
        assert!(
            hop.sum_land_pay > hop.cfg().w_land * 0.9,
            "a clean settled capture must pay most of w_land: {}",
            hop.sum_land_pay
        );
        assert!(
            hop_total > still_total + 0.5,
            "hop {hop_total:.3} must beat standing still {still_total:.3}"
        );
    }

    /// **The regression that matters.** The apex credit must be paid at the
    /// APEX EVENT — while airborne — and must survive whatever happens next.
    ///
    /// The bug this locks out: the peak used to be a terminal bonus gated on
    /// `!fallen`, paid by the caller at episode end. Collected over 220 updates
    /// that drove `peak_height` 0.558 → 0.050 with `corr(update, peak) = −0.949`
    /// while reward was *preserved*, because the cheap way to claim it was to
    /// never jump at all. If this test ever fails, that failure mode is back.
    #[test]
    fn the_apex_is_paid_while_airborne_and_not_ransomed_on_survival() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        let standing = fiction(standing_hip(), standing_hip(), Vec2::ZERO, true);
        rw.step(&standing, None);

        // Launch: airborne and climbing.
        let up = fiction(
            Vec2::new(0.0, SURFACE - HIP - 10.0),
            Vec2::new(0.0, SURFACE - HIP - 10.0),
            Vec2::new(0.0, -260.0),
            false,
        );
        let (_, ev) = rw.step(&up, None);
        assert!(ev.liftoff, "first airborne tick is the liftoff checkpoint");
        assert!(
            !ev.apex_reached,
            "still climbing — the apex event must not fire yet"
        );

        // Climb to a real apex (40 px above standing).
        let apex_y = SURFACE - HIP - 40.0;
        let rising = fiction(
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, -40.0),
            false,
        );
        let (_, ev) = rw.step(&rising, None);
        assert!(!ev.apex_reached, "v_com.y < 0 is still climbing");

        // The tick the CoM stops climbing — the apex EVENT.
        let top = fiction(
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, 0.0),
            false,
        );
        let (r_apex, ev) = rw.step(&top, None);
        assert!(ev.apex_reached, "the apex event must fire at the top");
        assert!(
            rw.last_apex_peak_pay > cfg.w_apex_peak * 0.5,
            "a 40/60 px rise must bank over half the apex credit: {}",
            rw.last_apex_peak_pay
        );
        assert!(
            r_apex > cfg.w_apex_peak * 0.5,
            "the credit lands on THIS tick, not at episode end: {r_apex}"
        );

        // Now the body tumbles and never lands. The apex credit already banked
        // must be untouched — that is the whole point.
        let banked = rw.sum_apex_peak_pay;
        assert!(banked > 0.0);
        for k in 0..30 {
            let y = apex_y + k as f32 * 3.0;
            let falling = fiction(
                Vec2::new(0.0, y),
                Vec2::new(0.0, y),
                Vec2::new(40.0, 300.0),
                false,
            );
            rw.step(&falling, None);
        }
        assert!(
            (rw.sum_apex_peak_pay - banked).abs() < 1e-6,
            "the banked apex credit must never be revoked by a bad landing"
        );
    }

    /// A launch that never actually rises (the stroke was suppressed, `rise ≈ 0`)
    /// must earn NO apex credit. Otherwise "twitch and fall back" would be paid
    /// for, which is the same degenerate solution in a smaller form.
    #[test]
    fn a_failed_launch_earns_no_apex_credit() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        let standing = fiction(standing_hip(), standing_hip(), Vec2::ZERO, true);
        rw.step(&standing, None);

        // Airborne, but the hip never rose: `flight_peak` stays ~0.
        let flat = fiction(
            Vec2::new(0.0, SURFACE - HIP - 0.5),
            Vec2::new(0.0, SURFACE - HIP - 0.5),
            Vec2::new(0.0, 60.0),
            false,
        );
        let (_, ev) = rw.step(&flat, None);
        assert!(ev.liftoff);
        assert!(!ev.apex_reached);
        assert_eq!(
            rw.sum_apex_peak_pay, 0.0,
            "a non-rise must not be paid as an apex"
        );
    }

    /// One credit per flight — a mid-air bob must not farm the apex payment.
    #[test]
    fn the_apex_credit_is_paid_once_per_flight() {
        let cfg = RewardConfig::default();
        let mut rw = JumpReward::new(cfg);
        let standing = fiction(standing_hip(), standing_hip(), Vec2::ZERO, true);
        rw.step(&standing, None);

        let up = fiction(
            Vec2::new(0.0, SURFACE - HIP - 10.0),
            Vec2::new(0.0, SURFACE - HIP - 10.0),
            Vec2::new(0.0, -260.0),
            false,
        );
        rw.step(&up, None);

        let apex_y = SURFACE - HIP - 60.0;
        let top = fiction(
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, 0.0),
            false,
        );
        let (_, ev) = rw.step(&top, None);
        assert!(ev.apex_reached);
        let first = rw.sum_apex_peak_pay;
        assert!(first > 0.0);

        // Bob: descend, then rise again while still airborne.
        for _ in 0..3 {
            let bob = fiction(
                Vec2::new(0.0, apex_y + 2.0),
                Vec2::new(0.0, apex_y + 2.0),
                Vec2::new(0.0, 20.0),
                false,
            );
            rw.step(&bob, None);
        }
        let bob_up = fiction(
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, -30.0),
            false,
        );
        rw.step(&bob_up, None);
        let bob_top = fiction(
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, apex_y),
            Vec2::new(0.0, 0.0),
            false,
        );
        rw.step(&bob_top, None);
        assert!(
            (rw.sum_apex_peak_pay - first).abs() < 1e-6,
            "a mid-air bob must not pay the apex twice: {first} -> {}",
            rw.sum_apex_peak_pay
        );
    }
}
