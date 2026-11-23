//! The **jump residual** controller — a PPO policy behind the same [`Step`]
//! seam that corrects the authored [`JumpCtrl`](crate::jump::JumpCtrl) expert
//! in a **polar action space** (`plans/jump-ppo-residual-polar-action.md`
//! §1–§4, §7).
//!
//! # Why a residual, and why polar
//! The expert owns the crouch→extend schedule; it lands a hop but cannot stop
//! the torso rotating at liftoff. The residual is the reactive layer: its job is
//! attitude/shape correction around a near-working hop, not discovering one from
//! noise. The action is **feasible by construction** so there is no reach clamp
//! (a clamp is non-invertible at the wall and breaks PPO's `logπ`):
//!
//! ```text
//! per leg i ∈ {L,R}:  r_i ∈ [0,1]   chord / leg_len      (the strut length)
//!                     s_i ∈ [−s_max, s_max]  sin φ_i, φ = angle of hip→foot
//!                                            off straight-down (+y, vello y-down)
//! torso:              s_θ ∈ [−s_max, s_max]  sin θ,  θ = lean off straight-up
//! ```
//!
//! Five dims = the actuator's five DOF (2 strut lengths, 2 chord directions, 1
//! lean). `s_max = sin 60°` keeps `sin` monotone so `asin` reconstructs the angle
//! unambiguously. `r ≤ 1` ⇒ every reconstructed pose is reachable.
//!
//! # Residual-on-expert
//! Each tick the expert proposes `(r, s, θ)`; the policy adds a bounded
//! `δ`; the final command is `clamp(proposal + scale·δ)`. A zero-output policy
//! (scale 0, or a zero-initialised actor) therefore executes the expert exactly.
//! Stance is **authored** from the expert's phase (`stance = phase != Flight`),
//! so the net does not rediscover ground-stiff / air-soft (`residual` 5-dim
//! action; plan §3 / §6 B).
//!
//! # Observation mirrors the action
//! The measured feet are presented in the **same** `(r, s)` frame the policy
//! commands, the lean as `sin`, plus the linear/rotational CoM state
//! (`v_com`, `omega_com`, `ang_momentum`), contact/grounded, the expert
//! proposal, and the `support_offset` (feet-vs-CoM arm — the measured root cause
//! of every prior topple; plan §8). Width is [`OBS_DIM`] and must match the
//! bundle's `in_dim`.

use std::sync::{Arc, RwLock};

use glam::Vec2;
use walker_math::LowerBodyConfig;
use walker_nn::{PolicyBundle, Rng};

use crate::actuate::{FootPose, Pose};
use crate::control::{Command, ControlConfig, ControlOut, LocoMode, Step};
use crate::jump::JumpCtrl;
use crate::learned::{LastStep, LearnMode};
use crate::observe::Fiction;

/// Observation width (the [`JumpResidual::build_obs`] layout — a contract with
/// the bundle's `in_dim`).
pub const OBS_DIM: usize = 22;
/// Action width: `[r_L, s_L, r_R, s_R, s_θ]`.
pub const ACTION_DIM: usize = 5;

/// Tuning + geometry for the residual. Every field is a decision knob.
#[derive(Debug, Clone)]
pub struct JumpResidualConfig {
    /// Observation width — MUST equal [`OBS_DIM`] for the default layout.
    pub obs_dim: usize,
    /// Action width — MUST equal [`ACTION_DIM`].
    pub action_dim: usize,
    /// Hidden widths of the fresh policy.
    pub hidden: Vec<usize>,
    /// Velocity normalizer (px/s) for `v_com` and `cmd.speed`.
    pub v_scale: f32,
    /// Spin-rate normalizer (rad/s) for `omega_com` / `lean_rate`.
    pub omega_scale: f32,
    /// Leg length `thigh + calf` (px) — `r·leg_len` is the chord (plan §6 A:
    /// config value, not the live stretched length).
    pub leg_len: f32,
    /// Standing hip height above the floor (px) — the height normalizer and the
    /// `ang_momentum` / `support_offset` scale.
    pub hip_height: f32,
    /// Lower bound of the normalized chord `r` (deepest allowed crouch).
    pub r_min: f32,
    /// `sin` bound for `s_i` and `s_θ` — `sin 60° ≈ 0.866` (monotone ⇒ `asin`
    /// invertible). The action clamps to this.
    pub s_max: f32,
    /// Residual authority on `r` (per unit action): `r = proposal_r + r_scale·δ`.
    pub r_scale: f32,
    /// Residual authority on `s_i`.
    pub s_scale: f32,
    /// Residual authority on `s_θ`.
    pub theta_scale: f32,
    /// Seed for the fresh policy + sampling stream.
    pub seed: u64,
}

impl Default for JumpResidualConfig {
    fn default() -> Self {
        Self {
            obs_dim: OBS_DIM,
            action_dim: ACTION_DIM,
            hidden: vec![64, 64],
            // Same normalizers as the walking `LearnedConfig` so one rig scale
            // works for both.
            v_scale: 200.0,
            omega_scale: 6.0,
            leg_len: 110.0,
            hip_height: 104.0,
            // Deep crouch floor: `r = 0.35 ⇒ 38.5 px chord` (well inside reach).
            r_min: 0.35,
            s_max: 0.866,
            // Small residual authority: explore locally around the expert.
            r_scale: 0.15,
            s_scale: 0.30,
            theta_scale: 0.30,
            seed: 0x1A2B,
        }
    }
}

/// The expert's proposal, in polar form, for one tick.
#[derive(Debug, Clone, Copy)]
struct Proposal {
    r: [f32; 2],
    s: [f32; 2],
    s_theta: f32,
    stance: [bool; 2],
    hip: Vec2,
}

/// The residual controller. `Clone` copies the shared-policy `Arc`, the rng and
/// the pending `last` — never the weights (same contract as `LearnedCtrl`).
#[derive(Debug, Clone)]
pub struct JumpResidual {
    /// Shared policy snapshot (harness broadcasts new weights between rollouts).
    pub policy: Arc<RwLock<PolicyBundle>>,
    pub mode: LearnMode,
    pub cfg: JumpResidualConfig,
    /// The authored expert whose schedule this residual corrects.
    expert: JumpCtrl,
    rng: Rng,
    last: Option<LastStep>,
    since: f32,
}

impl JumpResidual {
    /// Adopt a shared policy bundle (harness-injected: fresh net for training,
    /// or a loaded checkpoint for deploy).
    pub fn with_policy(
        policy: Arc<RwLock<PolicyBundle>>,
        mode: LearnMode,
        cfg: JumpResidualConfig,
        lower: &LowerBodyConfig,
    ) -> Self {
        let seed = cfg.seed ^ (mode as u64);
        Self {
            policy,
            mode,
            cfg,
            expert: JumpCtrl::new(lower),
            rng: Rng::new(seed),
            last: None,
            since: 0.0,
        }
    }

    /// Build a controller that owns a freshly randomized bundle of the
    /// `cfg`-shaped policy (fallback when no policy was injected).
    pub fn fresh(cfg: JumpResidualConfig, lower: &LowerBodyConfig, mode: LearnMode) -> Self {
        let mut rng = Rng::new(cfg.seed);
        let mut actor_dims = vec![cfg.obs_dim];
        actor_dims.extend_from_slice(&cfg.hidden);
        actor_dims.push(cfg.action_dim);
        let mut critic_dims = vec![cfg.obs_dim];
        critic_dims.extend_from_slice(&cfg.hidden);
        critic_dims.push(1);
        let actor = walker_nn::Mlp::new(&actor_dims, &mut rng);
        let critic = walker_nn::Mlp::new(&critic_dims, &mut rng);
        let log_std = vec![(0.5f32).ln(); cfg.action_dim];
        let bundle = PolicyBundle {
            actor,
            critic,
            log_std,
        };
        Self::with_policy(Arc::new(RwLock::new(bundle)), mode, cfg, lower)
    }

    /// Take the last transition step for the harness to close. `None` until the
    /// first successful step.
    pub fn take_last(&mut self) -> Option<LastStep> {
        self.last.take()
    }

    pub fn since(&self) -> f32 {
        self.since
    }

    /// Whether the underlying expert has finished landing (§12.2 land window +
    /// the post-landing stabilization hold). Forwarded from the expert so the
    /// harness can close the episode on a *completed landing* rather than on
    /// mere touchdown.
    pub fn landing_complete(&self, cfg: &crate::control::ControlConfig) -> bool {
        self.expert.landing_complete(cfg)
    }

    /// The landing grace window (touchdown → hold elapsed). The harness keeps
    /// the fall gate silent while this is open.
    pub fn landing_window_open(&self, cfg: &crate::control::ControlConfig) -> bool {
        self.expert.landing_window_open(cfg)
    }

    /// Forwarded from the expert: has the authored launch window closed (power
    /// stroke released)? The harness asks the reward to terminate a launch-miss
    /// once this is true and the liftoff gate has still not fired.
    pub fn launch_window_over(&self) -> bool {
        self.expert.launch_window_over()
    }

    fn facing_or_default(facing: f32) -> f32 {
        if facing == 0.0 {
            1.0
        } else {
            facing
        }
    }

    /// Convert a world [`Pose`] (the expert's) into the polar proposal, in the
    /// travel frame (so `s` matches what the action commands).
    fn proposal_from_pose(pose: &Pose, facing: f32, leg_len: f32) -> Proposal {
        let mut r = [0.0f32; 2];
        let mut s = [0.0f32; 2];
        let feet = [pose.left_foot, pose.right_foot];
        for (i, fp) in feet.iter().enumerate() {
            let d = fp.pos - pose.hip;
            let chord = d.length().max(1e-6);
            r[i] = (chord / leg_len).clamp(0.0, 1.0);
            // s = sin φ = horizontal / chord, folded into the travel frame.
            s[i] = (d.x / chord / facing).clamp(-1.0, 1.0);
        }
        Proposal {
            r,
            s,
            s_theta: (pose.upper_angle * facing).sin(),
            stance: [pose.left_foot.stance, pose.right_foot.stance],
            hip: pose.hip,
        }
    }

    /// `Fiction` + `Command` + the expert proposal → the fixed-width observation.
    ///
    /// ```text
    ///  0 v_com.y/v_scale    1 v_com.x/v_scale
    ///  2 sin(lean)          3 lean_rate/omega_scale
    ///  4 height (floor−hip)/hip_height
    ///  5 r_L  6 s_L  7 r_R  8 s_R      (measured polar feet; s=(Δx/chord)/facing)
    ///  9 l_touch 10 r_touch 11 grounded
    /// 12 dir.y  13 speed/v_scale
    /// 14 omega_com/omega_scale
    /// 15 ang_mom/(hip_height·v_scale)
    /// 16 r_prop_L 17 s_prop_L 18 r_prop_R 19 s_prop_R 20 sin(θ_prop)
    /// 21 support_offset = (com.x − support_x)/leg_len
    /// ```
    pub fn build_obs(
        &self,
        cmd: &Command,
        fiction: &Fiction,
        prop: &Proposal,
        facing: f32,
    ) -> Vec<f32> {
        let c = &self.cfg;
        let hip = fiction.hip;
        let floor_y = hip.y + c.hip_height;
        let polar = |f: &crate::observe::FootState| {
            let d = f.pos - hip;
            let chord = d.length().max(1e-6);
            (
                (chord / c.leg_len).clamp(0.0, 1.0),
                (d.x / chord / facing).clamp(-1.0, 1.0),
            )
        };
        let (rl, sl) = polar(&fiction.feet[0]);
        let (rr, sr) = polar(&fiction.feet[1]);
        // support_x = mean x of touching feet (plan §8: the force arm).
        let mut sx = 0.0f32;
        let mut n = 0.0f32;
        for f in &fiction.feet {
            if f.touching {
                sx += f.pos.x;
                n += 1.0;
            }
        }
        let support_x = if n > 0.0 {
            sx / n
        } else {
            0.5 * (fiction.feet[0].pos.x + fiction.feet[1].pos.x)
        };
        let support_offset = (fiction.com.x - support_x) / c.leg_len;

        let obs = [
            fiction.v_com.y / c.v_scale,
            fiction.v_com.x / c.v_scale,
            fiction.lean.sin(),
            fiction.lean_rate / c.omega_scale,
            (floor_y - hip.y) / c.hip_height,
            rl,
            sl,
            rr,
            sr,
            fiction.feet[0].touching as i32 as f32,
            fiction.feet[1].touching as i32 as f32,
            fiction.grounded as i32 as f32,
            cmd.dir.y,
            cmd.speed / c.v_scale,
            fiction.omega_com / c.omega_scale,
            fiction.ang_momentum / (c.hip_height * c.v_scale),
            prop.r[0],
            prop.s[0],
            prop.r[1],
            prop.s[1],
            prop.s_theta,
            support_offset,
        ];
        obs.into_iter()
            .map(|v| if v.is_finite() { v } else { 0.0 })
            .collect()
    }

    /// Polar residual → world [`Pose`] (feasible by construction; plan §2).
    fn reconstruct(&self, delta: &[f32], prop: &Proposal, facing: f32) -> Pose {
        let c = &self.cfg;
        let d = |i: usize| delta.get(i).copied().unwrap_or(0.0);
        let r_l = (prop.r[0] + c.r_scale * d(0)).clamp(c.r_min, 1.0);
        let s_l = (prop.s[0] + c.s_scale * d(1)).clamp(-c.s_max, c.s_max);
        let r_r = (prop.r[1] + c.r_scale * d(2)).clamp(c.r_min, 1.0);
        let s_r = (prop.s[1] + c.s_scale * d(3)).clamp(-c.s_max, c.s_max);
        let s_th = (prop.s_theta + c.theta_scale * d(4)).clamp(-c.s_max, c.s_max);
        // foot offset from the hip: chord = r·leg_len; below the hip (+y vello).
        let foot = |r: f32, s: f32| {
            let chord = r * c.leg_len;
            let vert = (1.0 - s * s).max(0.0).sqrt();
            Vec2::new(facing * chord * s, chord * vert)
        };
        let left = foot(r_l, s_l);
        let right = foot(r_r, s_r);
        let upper_angle = facing * s_th.asin();
        Pose {
            hip: prop.hip,
            left_foot: FootPose {
                pos: prop.hip + left,
                stance: prop.stance[0],
            },
            right_foot: FootPose {
                pos: prop.hip + right,
                stance: prop.stance[1],
            },
            facing,
            upper_angle,
            stance_split: false,
        }
    }
}

impl Step for JumpResidual {
    fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        if dt > 0.0 {
            self.since += dt;
        }
        let facing = Self::facing_or_default(facing);

        // 1) Expert proposal this tick (advances the stroke's phase).
        let proposal_out = self.expert.step(cmd, fiction, cfg, dt, facing);
        let prop = Self::proposal_from_pose(&proposal_out.pose, facing, self.cfg.leg_len);

        // 2) Observe in the same polar frame the action uses.
        let obs = self.build_obs(cmd, fiction, &prop, facing);

        // 3) Sample the residual δ.
        let (action, logp, value) = {
            let guard = match self.policy.read() {
                Ok(g) => g,
                Err(_) => {
                    return crate::control::hold_pose(fiction, cfg, LocoMode::Jumping, facing)
                }
            };
            if guard.obs_dim() != obs.len() || guard.action_dim() != self.cfg.action_dim {
                return crate::control::hold_pose(fiction, cfg, LocoMode::Jumping, facing);
            }
            match self.mode {
                LearnMode::Train => guard.sample(&obs, &mut self.rng),
                LearnMode::Deploy => (guard.act_deterministic(&obs), 0.0, guard.value_of(&obs)),
            }
        };
        self.last = Some(LastStep {
            obs,
            action: action.clone(),
            logp,
            value,
        });

        // 4) Final = clamp(proposal + scale·δ), then feasible FK to a pose.
        let pose = self.reconstruct(&action, &prop, facing);
        ControlOut {
            pose,
            // The residual commands a pose, not a velocity — but the launch
            // impulse belongs to the EXPERT, not the policy: it is the one
            // piece of authority the legs cannot express (only ~5.5 px of
            // extension left at standing). Dropping it here is what left the
            // residual path with no hop at all.
            hip_vel: Vec2::ZERO,
            jump_impulse: proposal_out.jump_impulse,
            plan: [Vec2::new(f32::NAN, f32::NAN); 2],
            mode: LocoMode::Jumping,
            debug: None,
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::Jumping
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observe::FootState;

    fn fiction(hip: Vec2, feet: [Vec2; 2], touching: [bool; 2]) -> Fiction {
        Fiction {
            dt: 1.0 / 90.0,
            hip,
            v_hip: Vec2::ZERO,
            lean: 0.0,
            lean_rate: 0.0,
            com: hip,
            v_com: Vec2::ZERO,
            ang_momentum: 0.0,
            omega_com: 0.0,
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
            body_contact: false,
        }
    }

    fn standing() -> Fiction {
        // Feet inside full reach (chord < leg_len) so the polar `r` does not
        // clamp; the real standing pose sits a hair past reach by construction.
        let hip = Vec2::new(0.0, 203.0);
        fiction(
            hip,
            [Vec2::new(-30.0, 305.0), Vec2::new(30.0, 305.0)],
            [true, true],
        )
    }

    fn cfg_default() -> JumpResidualConfig {
        JumpResidualConfig::default()
    }

    #[test]
    fn obs_is_finite_and_the_declared_width() {
        let cfg = cfg_default();
        let mut c =
            JumpResidual::fresh(cfg.clone(), &LowerBodyConfig::default(), LearnMode::Deploy);
        let f = standing();
        let prop = JumpResidual::proposal_from_pose(
            &c.expert
                .step(
                    &crate::control::Command::jump(),
                    &f,
                    &ControlConfig::default(),
                    1.0 / 90.0,
                    1.0,
                )
                .pose,
            1.0,
            cfg.leg_len,
        );
        let obs = c.build_obs(&crate::control::Command::jump(), &f, &prop, 1.0);
        assert_eq!(obs.len(), OBS_DIM);
        assert_eq!(obs.len(), cfg.obs_dim);
        assert!(obs.iter().all(|v| v.is_finite()), "{obs:?}");
    }

    #[test]
    fn reconstruction_is_reachable_and_below_the_hip() {
        let cfg = cfg_default();
        let c = JumpResidual::fresh(cfg.clone(), &LowerBodyConfig::default(), LearnMode::Deploy);
        let prop = Proposal {
            r: [1.0, 1.0],
            s: [0.0, 0.0],
            s_theta: 0.0,
            stance: [true, true],
            hip: Vec2::new(10.0, 200.0),
        };
        // Extreme residual in every direction must stay feasible.
        for delta in [
            [10.0, 10.0, 10.0, 10.0, 10.0],
            [-10.0, -10.0, -10.0, -10.0, -10.0],
            [0.0; 5],
        ] {
            let pose = c.reconstruct(&delta, &prop, 1.0);
            for fp in [pose.left_foot, pose.right_foot] {
                let d = fp.pos - pose.hip;
                assert!(
                    d.length() <= cfg.leg_len + 1e-3,
                    "chord {} > leg",
                    d.length()
                );
                assert!(d.y > 0.0, "foot must be below the hip (vello y-down)");
            }
            assert!(pose.upper_angle.is_finite());
        }
    }

    #[test]
    fn zero_residual_executes_the_expert() {
        // With zero residual authority the final pose IS the expert proposal.
        let mut cfg = cfg_default();
        cfg.r_scale = 0.0;
        cfg.s_scale = 0.0;
        cfg.theta_scale = 0.0;
        // The jump.rs expert's v10 rest chord is ~110.8 px; use a leg_len at or
        // above it so the feasible-by-construction `r ≤ 1` clamp does not shorten
        // the proposal — this test isolates "zero residual == expert".
        cfg.leg_len = 112.0;
        let mut c =
            JumpResidual::fresh(cfg.clone(), &LowerBodyConfig::default(), LearnMode::Deploy);
        let mut expert = JumpCtrl::new(&LowerBodyConfig::default());
        let f = standing();
        let cmd = crate::control::Command::jump();
        let cc = ControlConfig::default();
        let residual_out = c.step(&cmd, &f, &cc, 1.0 / 90.0, 1.0);
        let expert_out = expert.step(&cmd, &f, &cc, 1.0 / 90.0, 1.0);
        let p = residual_out.pose;
        let e = expert_out.pose;
        assert!(
            (p.hip - e.hip).length() < 1e-3,
            "hip {:?} vs {:?}",
            p.hip,
            e.hip
        );
        assert!(
            (p.left_foot.pos - e.left_foot.pos).length() < 1e-3,
            "L {:?} vs {:?}",
            p.left_foot.pos,
            e.left_foot.pos
        );
        assert!(
            (p.right_foot.pos - e.right_foot.pos).length() < 1e-3,
            "R {:?} vs {:?}",
            p.right_foot.pos,
            e.right_foot.pos
        );
    }

    #[test]
    fn a_step_reports_one_last_step_of_the_right_shape() {
        let cfg = cfg_default();
        let mut c = JumpResidual::fresh(cfg.clone(), &LowerBodyConfig::default(), LearnMode::Train);
        let f = standing();
        let _ = c.step(
            &crate::control::Command::jump(),
            &f,
            &ControlConfig::default(),
            1.0 / 90.0,
            1.0,
        );
        let last = c.take_last().expect("a step records its transition");
        assert_eq!(last.obs.len(), cfg.obs_dim);
        assert_eq!(last.action.len(), cfg.action_dim);
        assert!(c.take_last().is_none(), "take_last drains");
    }
}
