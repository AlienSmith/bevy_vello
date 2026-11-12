//! The **learned** controller — a [`PolicyBundle`] behind the [`Step`] seam.
//!
//! This is the "wrap the trainer result into a controller" contract from the
//! plan (§2/§4): the network + PPO live in `walker_nn`; *here* is only the
//! env-binding that turns a [`Fiction`] + [`Command`] into an observation, runs
//! the policy, and emits a [`Pose`]. It is **move-agnostic** — a jump and a
//! walk share one obs/action contract; only the harness-side reward and the
//! held [`Command`] differ (plan §0: we first learn a straight jump).
//!
//! # The action IS the [`Pose`]
//! Exactly four foot offsets, the stick lean, and the two stance gates — the
//! same five-element pose `actuate` already consumes, so a learned gait drops
//! into the existing chain with zero new actuation code (plan §3b).
//!
//! # Law 2 discipline
//! The controller reads nothing from physics it was not handed in [`Fiction`],
//! and it **never writes fiction**. It records its own last step
//! ([`LearnedCtrl::last`]) as *data* — `(obs, action, logp, value)` — that the
//! training harness reads after the *next* observe tick to close a transition.
//! The transition is completed by the harness (it owns the reward and the reset
//! event); the controller stays pure.
//!
//! # Shared weights (multi-env)
//! `policy` is an `Arc<RwLock<PolicyBundle>>`. During a rollout the harness
//! never writes it (on-policy: weights frozen), so every character reads the
//! same snapshot; between rollouts the harness takes the write lock once and
//! the next rollout sees new weights — the multi-env broadcast, with no
//! per-tick copy.

use std::sync::{Arc, RwLock};

use glam::Vec2;
use walker_nn::{PolicyBundle, Rng};

use crate::actuate::{FootPose, Pose};
use crate::control::{Command, ControlConfig, ControlOut, LocoMode, Step};
use crate::observe::Fiction;

/// How a [`LearnedCtrl`] acts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LearnMode {
    /// Sample `a ~ N(μ,σ)` and record the transition context (collection).
    Train,
    /// Act on the deterministic mean `μ(s)` — deployment (plan §4).
    Deploy,
}

/// The one transition step the controller hands back for the harness to close
/// (it lacks the *next* fiction, so the harness supplies the reward/done).
#[derive(Debug, Clone)]
pub struct LastStep {
    pub obs: Vec<f32>,
    pub action: Vec<f32>,
    pub logp: f32,
    pub value: f32,
}

/// Learned-control tuning + the obs/action geometry. Seeded once from the rig
/// (`LowerBodyConfig`) at spawn; every field is a *decision* knob, not a solver
/// constant.
#[derive(Debug, Clone)]
pub struct LearnedConfig {
    /// Observation width (the [`Self::build_obs`] layout is a contract with the
    /// bundle's `in_dim`; see [`LearnedCtrl::build_obs`]).
    pub obs_dim: usize,
    /// Action width = 7 (two foot offsets, lean, two stance gates) — must equal
    /// the bundle's `out_dim` (plan §3b).
    pub action_dim: usize,
    /// Hidden-layer widths for the fresh policy a [`LearnedCtrl`] starts with
    /// when none is injected.
    pub hidden: Vec<usize>,
    /// Normalizer for all velocities (px/s): the CoM speeds and `cmd.speed`.
    pub v_scale: f32,
    /// Normalizer for the attitude rate (rad/s).
    pub omega_scale: f32,
    /// Leg length `thigh + calf` (px) — the foot-offset and reach scale.
    pub leg_len: f32,
    /// Standing hip height above the floor (px) — the height normalizer.
    pub hip_height: f32,
    /// Lean clamp (rad).
    pub theta_max: f32,
    /// Horizontal foot-offset authority, as a fraction of `leg_len`.
    pub reach_x: f32,
    /// Foot-depth range below the hip, as `[min, max] · leg_len` — the vertical
    /// lever the policy commands (small = crouch/load, near 1 = full extension;
    /// for the jump the extension *is* the power stroke). The resulting
    /// hip→foot chord is additionally **clamped to `leg_len`** (the reach
    /// envelope — IK cannot command a foot further than the leg can reach), so
    /// a fully spread `(x, depth)` action never asks for an unreachable pose.
    pub depth_min: f32,
    pub depth_max: f32,
    /// Seed for the fresh init + sampling stream (determinism, plan §2).
    pub seed: u64,
}

impl Default for LearnedConfig {
    fn default() -> Self {
        // Defaults match the v10 rig at scale 0.5 and the gait's own
        // `theta_max`/`target_speed` (walker_math::LowerBodyConfig::default).
        Self {
            obs_dim: 14,
            action_dim: 7,
            hidden: vec![64, 64],
            v_scale: 200.0,
            omega_scale: 6.0,
            leg_len: 110.0,
            hip_height: 104.0,
            theta_max: 1.30,
            reach_x: 0.6,
            // Re-centered so the **neutral action `a=0` is the standing pose**:
            // the midpoint `(depth_min+depth_max)/2 · leg_len ≈ 0.945·110 ≈ 104
            // = hip_height`, feet on the floor. A fresh (near-zero-mean) policy
            // therefore starts *stable* — it can stand, and only a deliberate
            // negative `a_y` crouches. Without this the neutral pose floats the
            // feet 19 px and the random-policy torso tips past the fall gate in
            // ~2 ticks, so PPO never observes the crouch→extend it must find.
            // `depth_max = leg_len` is the fully-extended reach (the jump's
            // power stroke); IK clamps the chord to `leg_len` regardless.
            depth_min: 0.78,
            depth_max: 1.11,
            seed: 0x5A17,
        }
    }
}

/// The policy executor behind [`Step`]. `Clone` so it slots into the
/// [`Controller`](crate::control::Controller) enum; a clone copies the *Arc*
/// (shared policy), the rng, and the pending `last` — never the weights.
#[derive(Debug, Clone)]
pub struct LearnedCtrl {
    /// Shared snapshot the harness broadcasts (see module docs).
    pub policy: Arc<RwLock<PolicyBundle>>,
    pub mode: LearnMode,
    pub cfg: LearnedConfig,
    rng: Rng,
    last: Option<LastStep>,
    since: f32,
}

impl LearnedCtrl {
    /// Adopt a shared policy bundle (harness-injected: a fresh random net for
    /// training, or a loaded checkpoint for deploy).
    pub fn with_policy(
        policy: Arc<RwLock<PolicyBundle>>,
        mode: LearnMode,
        cfg: LearnedConfig,
    ) -> Self {
        let seed = cfg.seed ^ (mode as u64);
        Self {
            policy,
            mode,
            rng: Rng::new(seed),
            cfg,
            last: None,
            since: 0.0,
        }
    }

    /// Build a controller that *owns* a freshly randomized bundle of the
    /// `cfg`-shaped policy — the fallback when no policy was injected (a
    /// non-learning scene that momentarily enters this mode).
    pub fn fresh(cfg: LearnedConfig, mode: LearnMode) -> Self {
        let mut rng = Rng::new(cfg.seed);
        let actor_dims = fresh_dims(&cfg);
        let critic_dims = {
            let mut d = vec![cfg.obs_dim];
            d.extend_from_slice(&cfg.hidden);
            d.push(1);
            d
        };
        let actor = walker_nn::Mlp::new(&actor_dims, &mut rng);
        let critic = walker_nn::Mlp::new(&critic_dims, &mut rng);
        let log_std = vec![(0.5f32).ln(); cfg.action_dim];
        let bundle = PolicyBundle {
            actor,
            critic,
            log_std,
        };
        Self {
            policy: Arc::new(RwLock::new(bundle)),
            mode,
            rng: Rng::new(cfg.seed),
            cfg,
            last: None,
            since: 0.0,
        }
    }

    /// The last transition step, taken by the harness to build a [`walker_nn::
    /// Transition`] once it knows this tick's reward/done. `None` until the
    /// first successful step.
    pub fn take_last(&mut self) -> Option<LastStep> {
        self.last.take()
    }

    pub fn since(&self) -> f32 {
        self.since
    }

    /// Swap in a new shared policy snapshot (the harness's weight broadcast).
    pub fn set_policy(&mut self, policy: Arc<RwLock<PolicyBundle>>) {
        self.policy = policy;
    }

    /// `Fiction` + `Command` → the fixed-width observation (plan §3, 14 dims,
    /// all hip-relative, rig-normalized, travel-frame-independent so one policy
    /// works for either facing).
    ///
    /// ```text
    /// 0  v_com_y / v_scale        (jump power signal)
    /// 1  v_com_x / v_scale
    /// 2  lean                    θ (fiction.lean)
    /// 3  lean_rate / omega_scale
    /// 4  height  (floor_y − hip.y) / hip_height    (1 = standing)
    /// 5  lfoot (x)/leg_len   6  lfoot (y)/leg_len
    /// 7  rfoot (x)/leg_len   8  rfoot (y)/leg_len
    /// 9  lfoot touching 1/0   10 rfoot touching 1/0
    /// 11 cmd dir.y            12 cmd dir.x          (move intent: +y=jump, ±x=walk)
    /// 13 cmd speed / v_scale
    /// ```
    pub fn build_obs(&self, cmd: &Command, fiction: &Fiction) -> Vec<f32> {
        let c = &self.cfg;
        let hip = fiction.hip;
        let floor_y = hip.y + c.hip_height;
        let to_obs = |f: &crate::observe::FootState| {
            [(f.pos.x - hip.x) / c.leg_len, (f.pos.y - hip.y) / c.leg_len]
        };
        let l = to_obs(&fiction.feet[0]);
        let r = to_obs(&fiction.feet[1]);
        // The move-intent channels report the TRUE intent. `Command::jump`
        // carries `dir.x = 1` only so the FSM's `is_move` gate (which checks
        // `vx = dir.x·speed`) opens for a vertical command — it is not part of
        // the jump. Handing the policy `dir.x = +1` as an observation trained
        // a RUN-jump (measured: forward drift 175→360 px, landings topple), so
        // a nonzero `dir.y` marks the intent as vertical and `dir.x` reads 0.
        let vertical = cmd.dir.y < 0.0;
        let obs = [
            fiction.v_com.y / c.v_scale,
            fiction.v_com.x / c.v_scale,
            fiction.lean,
            fiction.lean_rate / c.omega_scale,
            (floor_y - hip.y) / c.hip_height,
            l[0],
            l[1],
            r[0],
            r[1],
            fiction.feet[0].touching as i32 as f32,
            fiction.feet[1].touching as i32 as f32,
            cmd.dir.y,
            if vertical { 0.0 } else { cmd.dir.x },
            cmd.speed / c.v_scale,
        ];
        // Any non-finite measurement (a degenerate first fiction) reads as 0 so
        // the net never sees a NaN input.
        obs.into_iter()
            .map(|v| if v.is_finite() { v } else { 0.0 })
            .collect()
    }

    /// Map the 6-dim policy action onto [`Pose`] offsets (vello y-down; x
    /// mirrored by `facing`, like every other controller).
    fn action_to_pose(&self, action: &[f32], hip: Vec2, facing: f32) -> Pose {
        let c = &self.cfg;
        // Foot offsets: x in ±reach_x·leg_len, y (depth below hip) in
        // [depth_min, depth_max]·leg_len. The action is squashed to [−1,1] with
        // tanh so the pose is always bounded, mapped to its range, then clamped
        // to the leg's reach (IK cannot aim a foot beyond the leg length).
        let foot_off = |ax: f32, ay: f32| {
            let x = ax.tanh() * c.reach_x * c.leg_len;
            let depth = c.leg_len
                * (c.depth_min + (c.depth_max - c.depth_min) * (ay * 0.5 + 0.5).clamp(0.0, 1.0));
            let mut off = Vec2::new(x, depth);
            if off.length() > c.leg_len {
                off *= c.leg_len / off.length();
            }
            // vello y-down: foot is BELOW the hip ⇒ +y; mirror x by facing.
            Vec2::new(off.x * facing, off.y)
        };
        let left = foot_off(action[0], action[1]);
        let right = foot_off(action[2], action[3]);
        let upper_angle = action[4].clamp(-1.0, 1.0) * c.theta_max * facing;
        // Two independent stance gates (plan §3b), written so the **neutral
        // action `a = 0` plants both feet** (standing): a foot *lifts* only
        // when the policy drives its gate past a positive `LIFT` margin. This
        // polarity is load-bearing for training — with `a > 0` the fresh
        // (≈0-mean) policy would leave each foot in swing ~50 % of ticks, the
        // legs would never hold the body, and every episode would topple in a
        // few ticks before any crouch→extend could be explored. `LIFT` bounds
        // the early swing rate to the tail of `N(0, init_std)`, so a real lift
        // is a *learned, deliberate* push rather than sampling noise.
        const LIFT: f32 = 0.8;
        let l_stance = action[5] < LIFT;
        let r_stance = action[6] < LIFT;
        Pose {
            hip,
            left_foot: FootPose {
                pos: hip + left,
                stance: l_stance,
            },
            right_foot: FootPose {
                pos: hip + right,
                stance: r_stance,
            },
            facing,
            upper_angle,
        }
    }
}

impl Step for LearnedCtrl {
    fn step(
        &mut self,
        cmd: &Command,
        fiction: &Fiction,
        _cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        if dt > 0.0 {
            self.since += dt;
        }
        let obs = self.build_obs(cmd, fiction);
        let (action, logp, value) = {
            let guard = match self.policy.read() {
                Ok(g) => g,
                Err(_) => {
                    // A poisoned lock is unrecoverable; hold the measured pose.
                    return crate::control::hold_pose(fiction, _cfg, LocoMode::Learned, facing);
                }
            };
            if guard.obs_dim() != obs.len() || guard.action_dim() != self.cfg.action_dim {
                // Shape mismatch (a checkpoint built for another rig): inert.
                return crate::control::hold_pose(fiction, _cfg, LocoMode::Learned, facing);
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
        let pose = self.action_to_pose(&action, fiction.hip, facing);
        ControlOut {
            pose,
            // The policy commands a pose, not a velocity; report the measured
            // hip velocity for telemetry (the harness reads real motion).
            hip_vel: fiction.v_hip,
            mode: LocoMode::Learned,
            debug: None,
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::Learned
    }
}

/// Actor dims = `[obs, hidden…, action]` for a fresh net.
fn fresh_dims(cfg: &LearnedConfig) -> Vec<usize> {
    let mut d = vec![cfg.obs_dim];
    d.extend_from_slice(&cfg.hidden);
    d.push(cfg.action_dim);
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observe::FootState;

    fn fiction(hip: Vec2, feet: [Vec2; 2], touching: [bool; 2], lean: f32, v: Vec2) -> Fiction {
        Fiction {
            dt: 1.0 / 90.0,
            hip,
            v_hip: v,
            lean,
            lean_rate: 0.0,
            com: hip,
            v_com: v,
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
        }
    }

    #[test]
    fn build_obs_is_finite_and_correct_width() {
        let c = LearnedCtrl::fresh(LearnedConfig::default(), LearnMode::Deploy);
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.1,
            Vec2::new(10.0, -5.0),
        );
        let cmd = Command::walk(120.0);
        let obs = c.build_obs(&cmd, &f);
        assert_eq!(obs.len(), c.cfg.obs_dim);
        assert!(obs.iter().all(|v| v.is_finite()), "{obs:?}");
        // height reads ≈1 at the standing offset, touching flags are 1.
        assert!((obs[4] - 1.0).abs() < 0.2, "height {}", obs[4]);
        assert_eq!(obs[9], 1.0);
        assert_eq!(obs[10], 1.0);
    }

    #[test]
    fn non_fiction_input_degenerates_to_zero_not_nan() {
        // A first-tick fiction can be all-zero / NaN; obs must still be finite.
        let c = LearnedCtrl::fresh(LearnedConfig::default(), LearnMode::Deploy);
        let mut f = fiction(
            Vec2::new(f32::NAN, 0.0),
            [Vec2::ZERO; 2],
            [false, false],
            f32::NAN,
            Vec2::new(f32::NAN, f32::NAN),
        );
        f.com = Vec2::splat(f32::NAN);
        let obs = c.build_obs(&Command::default(), &f);
        assert!(obs.iter().all(|v| v.is_finite()), "{obs:?}");
    }

    #[test]
    fn action_to_pose_is_within_reach_and_below_hip() {
        let c = LearnedCtrl::fresh(LearnedConfig::default(), LearnMode::Deploy);
        let hip = Vec2::new(0.0, -140.0);
        // Saturating actions map to bounded, in-reach feet below the hip.
        for a in [
            vec![0.0; 7],
            vec![1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0],
            vec![-1.0, -1.0, 1.0, 1.0, -1.0, 0.0, 1.0],
        ] {
            let p = c.action_to_pose(&a, hip, 1.0);
            let l = p.left_foot.pos - hip;
            let r = p.right_foot.pos - hip;
            assert!(
                l.y > 0.0 && r.y > 0.0,
                "feet below hip (y-down): {l:?} {r:?}"
            );
            assert!(l.length() <= c.cfg.leg_len + 1.0, "left chord in reach");
            assert!(r.length() <= c.cfg.leg_len + 1.0, "right chord in reach");
            assert!(p.upper_angle.abs() <= c.cfg.theta_max + 1e-3);
            assert_eq!(p.hip, hip, "pose rides the measured hip");
        }
    }

    #[test]
    fn deploy_is_deterministic_and_train_records_a_step() {
        let cfg = LearnedConfig::default();
        let mut dep = LearnedCtrl::fresh(cfg.clone(), LearnMode::Deploy);
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.1,
            Vec2::new(10.0, -5.0),
        );
        let cmd = Command::walk(120.0);
        // Deploy repeats the same input → same pose every tick.
        let a = dep.step(&cmd, &f, &ControlConfig::default(), 1.0 / 90.0, 1.0);
        let b = dep.step(&cmd, &f, &ControlConfig::default(), 1.0 / 90.0, 1.0);
        assert_eq!(a.pose.left_foot.pos, b.pose.left_foot.pos, "deploy is μ(s)");
        assert_eq!(a.mode, LocoMode::Learned);

        // Train records the transition context for the harness.
        let mut tr = LearnedCtrl::fresh(cfg.clone(), LearnMode::Train);
        tr.step(&cmd, &f, &ControlConfig::default(), 1.0 / 90.0, 1.0);
        let last = tr.take_last().expect("train step records a last");
        assert_eq!(last.obs.len(), cfg.obs_dim);
        assert_eq!(last.action.len(), cfg.action_dim);
        assert!(last.logp.is_finite() && last.value.is_finite());
        assert!(tr.take_last().is_none(), "take is consuming");
    }

    #[test]
    fn shape_mismatch_holds_inertly() {
        // A bundle built for another rig width must not crash or emit garbage:
        // the controller falls back to holding the measured pose.
        let mut cfg = LearnedConfig::default();
        cfg.obs_dim = 8; // bundle was made for 8, controller will demand 14
        let c = LearnedCtrl::fresh(cfg, LearnMode::Deploy);
        // Now wrap the bundle to 14 by overriding the controller cfg width so
        // the guard sees a mismatch.
        let mut ctrl = c;
        ctrl.cfg.obs_dim = 14;
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.0,
            Vec2::ZERO,
        );
        let out = ctrl.step(
            &Command::default(),
            &f,
            &ControlConfig::default(),
            1.0 / 90.0,
            1.0,
        );
        assert_eq!(out.mode, LocoMode::Learned);
        assert_eq!(out.pose.left_foot.pos, f.feet[0].pos, "mismatch ⇒ hold");
    }

    #[test]
    fn shared_policy_broadcasts_to_another_controller() {
        // The harness's weight broadcast: a fresh controller sharing the same
        // Arc sees the policy the first one trained under.
        let cfg = LearnedConfig::default();
        let a = LearnedCtrl::fresh(cfg.clone(), LearnMode::Deploy);
        let shared = a.policy.clone();
        let b = LearnedCtrl::with_policy(shared, LearnMode::Deploy, cfg);
        let f = fiction(
            Vec2::new(0.0, -140.0),
            [Vec2::new(-33.0, -34.0), Vec2::new(39.5, -37.0)],
            [true, true],
            0.1,
            Vec2::ZERO,
        );
        let cmd = Command::walk(120.0);
        let mut a = a;
        let mut b = b;
        let pa = a
            .step(&cmd, &f, &ControlConfig::default(), 1.0 / 90.0, 1.0)
            .pose
            .left_foot
            .pos;
        let pb = b
            .step(&cmd, &f, &ControlConfig::default(), 1.0 / 90.0, 1.0)
            .pose
            .left_foot
            .pos;
        assert_eq!(pa, pb, "same shared snapshot ⇒ same action");
    }
}
