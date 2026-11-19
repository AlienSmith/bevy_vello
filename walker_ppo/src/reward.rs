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
    pub w_apex: f32,
    /// Apex **rise above standing** that saturates the credit, px.
    pub apex_target: f32,
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
                    r += c.w_apex * apex - c.w_liftoff * l_norm - c.w_liftoff_spin * spin;
                    self.last_liftoff_l = f.ang_momentum;
                    self.last_apex_pred = apex_pred;
                    events.liftoff = true;
                    self.phase = Phase::Flight;
                }
            }
            Phase::Flight => {
                if f.grounded {
                    // ---- CHECKPOINT 2 · landing (single payment) ----
                    // One credit, on this tick, scaled by how well the body
                    // captured its own momentum. Without this the whole hop
                    // reads net-negative and the optimum is "never launch".
                    let omega0 = (c.gravity / c.hip_height.max(1e-6)).max(1e-6).sqrt();
                    let xi = f.com.x + f.v_com.x / omega0;
                    let p_center = 0.5 * (f.feet[0].pos.x + f.feet[1].pos.x);
                    let e_dcm = (xi - p_center).abs() / c.leg_len.max(1e-6);
                    let post = (height_px - c.land_height_target).abs() / c.hip_height.max(1e-6);
                    let spin = (f.omega_com.abs() / c.omega_scale).max(0.0);
                    // quality ∈ [0,1]: perfect capture (ξ over the feet, still,
                    // at standing height) pays the full `w_land`.
                    let balance = (1.0 - e_dcm).clamp(0.0, 1.0);
                    let settle = (1.0 - spin).clamp(0.0, 1.0);
                    let posture = (1.0 - 0.5 * post).clamp(0.0, 1.0);
                    r += c.w_land * balance * settle * posture;
                    self.last_e_dcm = e_dcm;
                    self.phase = Phase::LandWindow;
                    self.land_ticks = 0;
                    self.prev_xi = Some(xi);
                    events.landed = true;
                }
            }
            Phase::LandWindow => {
                if !f.grounded {
                    // Bounced back airborne: no shaping, just carry the window.
                    self.phase = Phase::Flight;
                }
            }
        }

        // ---- Settle window (small NUDGE after the verdict was paid) ----
        // The landing credit above is the reward; this only discourages a body
        // that recovers its balance by toppling sideways after the fact.
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
            r += -c.w_dcm * e_dcm
                - c.w_xi_dot * xi_dot / c.v_scale
                - c.w_post * post
                - c.w_land_spin * spin;
            self.last_e_dcm = e_dcm;
            self.land_ticks += 1;
            // Window closes on timeout, or early on a measured crash.
            let timed_out = (self.land_ticks as f32) * dt >= c.land_window_ts;
            if e_dcm > c.land_window_e || timed_out {
                self.phase = Phase::Grounded;
            }
        }

        r -= pen;
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
        // Touchdown with a big DCM error (flying forward ⇒ ξ far from the feet):
        // the checkpoint's `quality` collapses, so it pays essentially nothing —
        // but it must NOT be a big negative (that was the collapse bug).
        let bad = fiction(
            Vec2::new(0.0, SURFACE - HIP),
            Vec2::new(0.0, SURFACE - HIP),
            Vec2::new(400.0, 0.0),
            true,
        );
        let (r_bad, ev) = rw.step(&bad, None);
        assert!(ev.landed);
        assert!(
            r_bad > rw.cfg().w_alive - 0.2,
            "a bad capture pays ~nothing but is not a huge charge: {r_bad}"
        );
        assert!(rw.last_e_dcm > 0.0);

        // A clean touchdown pays the full landing credit.
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
        let (r_good, ev) = rw2.step(&good, None);
        assert!(ev.landed);
        assert!(
            r_good > rw2.cfg().w_land * 0.9,
            "clean capture must pay: {r_good}"
        );
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
        let (r_land, ev) = hop.step(&touch, None);
        assert!(ev.landed, "touchdown must open the landing checkpoint");
        assert!(
            r_land > hop.cfg().w_land * 0.9,
            "a clean capture must pay most of w_land: {r_land}"
        );
        hop_total += r_land;
        // settle
        for _ in 0..40 {
            hop_total += hop.step(&touch, None).0;
        }
        assert!(
            hop_total > still_total + 0.5,
            "hop {hop_total:.3} must beat standing still {still_total:.3}"
        );
    }
}
