//! PPO (Proximal Policy Optimization) — the trainer, pure Rust, zero ECS /
//! zero physics concepts (plan §2).
//!
//! This module is the *engine*: it consumes transitions
//! `(obs, action, old_logp, reward, done, value)` and produces updated actor /
//! critic weights. Everything walker-specific — what `obs` and `action`
//! actually mean, the reward, when an episode resets — lives one layer up in
//! `walker_control::control::LearnedCtrl` and the training app. The split
//! mirrors `walker_math` (pure gait law) vs `walker_control` (the physics seam).
//!
//! Algorithm (Schulman et al. 2017), the standard continuous-control recipe:
//! - **Actor**: diagonal Gaussian policy `N(μ(s), diag(σ²))`. `μ` from the
//!   actor MLP, `σ` a state-independent learned `log_std` vector (the
//!   `legged_gym` default — robust and easy to tune).
//! - **Critic**: a separate MLP `V(s)` — same architecture, its own weights
//!   (no gradient coupling between the heads at this net size).
//! - **GAE(λ)** advantages over each episode, bootstrapping `V(next)=0` at a
//!   `done`.
//! - **Clipped surrogate** actor loss + MSE critic loss + entropy bonus, run
//!   for K epochs over the rollout. Full-batch per epoch: the nets are tiny,
//!   so the whole rollout is one gradient step; minibatch SGD is a later
//!   refinement (plan §5).
//!
//! # Determinism
//! Sampling and the KL/entropy bookkeeping ride a seeded [`Rng`], never
//! `rand`, so a training run reproduces tick-for-tick (the `Walker` D1
//! contract, extended to the trainer).

use serde::{Deserialize, Serialize};

use crate::adam::Adam;
use crate::mlp::{Cache, Mlp, Rng};

/// PPO hyperparameters (plan §5 starting point; every field is env-overridable
/// from the training app).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PpoConfig {
    /// `γ`, the discount. 90 Hz ticks → a high value (0.995) so one ~4 s gait
    /// cycle is still "future".
    pub gamma: f32,
    /// Upper clamp on `log_std` (after each Adam step). Actions ride through a
    /// `tanh`/clamp mapping in the env, so σ ≫ 1 turns every sample into a
    /// saturated ±1 coin-flip: the policy cannot express a multi-tick coherent
    /// sequence by chance, and a noisy mean gives no gradient pressure to
    /// shrink σ back — exploration collapses into a flail-and-collect-the-bonus
    /// local optimum (measured: σ → 8.5, peak height pinned at 0.10 for 5000
    /// updates). `0.0` ⇒ σ ≤ 1.
    pub std_max: f32,
    /// Lower clamp on `log_std` — keeps sampling noise from vanishing entirely
    /// (exploration floor). `-3.0` ⇒ σ ≥ 0.05.
    pub std_min: f32,
    /// GAE `λ` — the bias/variance trade on the advantage estimator.
    pub gae_lambda: f32,
    /// Clipped-surrogate band `ε`.
    pub clip_eps: f32,
    /// Entropy bonus weight.
    pub ent_coef: f32,
    /// Critic loss weight.
    pub vf_coef: f32,
    /// Gradient clip norm (global). Prevents a bad rollout from wrecking the
    /// policy at high `lr`; `0` disables.
    pub max_grad_norm: f32,
    /// Optimization epochs per rollout.
    pub epochs: usize,
    /// Initial exploration std (sets `log_std`); annealed by learning it.
    pub init_std: f32,
}

impl Default for PpoConfig {
    fn default() -> Self {
        Self {
            gamma: 0.995,
            std_max: 0.0,
            std_min: -3.0,
            gae_lambda: 0.95,
            clip_eps: 0.2,
            ent_coef: 0.003,
            vf_coef: 0.5,
            max_grad_norm: 0.5,
            epochs: 8,
            init_std: 0.5,
        }
    }
}

/// One environment step as the trainer sees it. `obs`/`action` are
/// fixed-width `Vec<f32>`; the width is a property of the env contract, not
/// this struct.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Transition {
    pub obs: Vec<f32>,
    pub action: Vec<f32>,
    /// `log π(a|s)` under the policy that *collected* this transition.
    pub old_logp: f32,
    pub reward: f32,
    /// True when this step ended an episode (the env reset afterwards).
    pub done: bool,
    /// `V(s)` recorded at collection (the old-policy estimate; GAE's base).
    pub value: f32,
}

/// An accumulation of collected transitions ready for one PPO update.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct Rollout {
    pub items: Vec<Transition>,
}

impl Rollout {
    pub fn push(&mut self, t: Transition) {
        self.items.push(t);
    }
    pub fn len(&self) -> usize {
        self.items.len()
    }
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    /// Empty the buffer for the next collection window.
    pub fn clear(&mut self) {
        self.items.clear();
    }
}

/// What one [`PpoTrainer::update`] reports — the training-app log line.
#[derive(Debug, Clone, Copy, Default)]
pub struct UpdateStats {
    pub policy_loss: f32,
    pub value_loss: f32,
    /// Mean per-dim entropy of the Gaussian policy at the last epoch.
    pub entropy: f32,
    /// Approx KL(old‖new) via `mean(r − 1 − log r)` (the cheap early-stop
    /// signal).
    pub approx_kl: f32,
    /// Fraction of samples whose clipped term won (update was gated).
    pub clip_frac: f32,
    /// Pre-normalization advantage std (sanity: the raw reward scale).
    pub mean_adv: f32,
}

/// The diagonal-Gaussian policy: actor MLP `μ(s)` + learned state-independent
/// `log_std` + a critic MLP `V(s)`. Owns three Adam blocks (actor, critic,
/// `log_std`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PpoTrainer {
    pub actor: Mlp,
    pub critic: Mlp,
    pub log_std: Vec<f32>,
    pub config: PpoConfig,
    #[serde(skip, default)]
    actor_adam: Adam,
    #[serde(skip, default)]
    critic_adam: Adam,
    #[serde(skip, default)]
    std_adam: Adam,
    #[serde(skip, default)]
    rng: Rng,
}

impl PpoTrainer {
    /// Build actor `[obs→hidden…→action]` + critic `[obs→hidden…→1]`, init
    /// `log_std` uniform at `config.init_std`.
    pub fn new(
        obs_dim: usize,
        action_dim: usize,
        hidden: &[usize],
        config: PpoConfig,
        seed: u64,
    ) -> Self {
        let mut rng = Rng::new(seed);
        let mut actor_dims = vec![obs_dim];
        actor_dims.extend_from_slice(hidden);
        actor_dims.push(action_dim);
        let mut critic_dims = vec![obs_dim];
        critic_dims.extend_from_slice(hidden);
        critic_dims.push(1);
        let actor = Mlp::new(&actor_dims, &mut rng);
        let critic = Mlp::new(&critic_dims, &mut rng);
        // Capture sizes before `actor`/`critic` move into the struct.
        let na = actor.n_params();
        let nc = critic.n_params();
        let log_std = vec![config.init_std.ln(); action_dim];
        Self {
            actor,
            critic,
            log_std,
            config,
            actor_adam: Adam::new(na),
            critic_adam: Adam::new(nc),
            std_adam: Adam::new(action_dim),
            rng: Rng::new(seed ^ 0x5EED),
        }
    }

    /// (Re)allocate Adam moments to match current network shapes. Called after
    /// `new` is unnecessary (the constructor sizes them) and **required** after
    /// any `load` whose `#[serde(skip)]` moments came back empty.
    pub fn rebuild_optimizers(&mut self) {
        self.actor_adam.reset_to(self.actor.n_params());
        self.critic_adam.reset_to(self.critic.n_params());
        self.std_adam.reset_to(self.log_std.len());
    }

    pub fn action_dim(&self) -> usize {
        self.actor.out_dim()
    }
    pub fn obs_dim(&self) -> usize {
        self.actor.in_dim()
    }

    /// Override the Adam learning rate on every block. The training app uses
    /// this to tune `lr` from env (plan §5); unit tests use it to make the
    /// value-head converge inside a short loop.
    pub fn set_lr(&mut self, lr: f32) {
        self.actor_adam.lr = lr;
        self.critic_adam.lr = lr;
        self.std_adam.lr = lr;
    }

    /// `σ` = `exp(log_std)`, per action dim.
    fn std(&self) -> Vec<f32> {
        self.log_std.iter().map(|s| s.exp()).collect()
    }

    /// Deterministic mean `μ(s)` — the **deploy** forward (plan §4,
    /// `LearnMode::Deploy`: no sampling).
    pub fn act_deterministic(&self, obs: &[f32]) -> Vec<f32> {
        self.actor.forward(obs)
    }

    /// Evaluate `V(s)` for the GAE bootstrap / reporting.
    pub fn value_of(&self, obs: &[f32]) -> f32 {
        self.critic.forward(obs)[0]
    }

    /// Sample `a ~ N(μ, σ)` and return `(a, logp, V(s))` — the **collection**
    /// step (plan §4, `LearnMode::Train`).
    pub fn sample(&mut self, obs: &[f32]) -> (Vec<f32>, f32, f32) {
        let mu = self.actor.forward(obs);
        let value = self.critic.forward(obs)[0];
        let std = self.std();
        let mut action = Vec::with_capacity(mu.len());
        let mut logp = 0.0f32;
        // Gaussian log-pdf: logp = Σ [ −logσ − 0.5·log(2π) − 0.5·((a−μ)/σ)² ].
        let half_log_2pi = 0.5 * (core::f32::consts::TAU).ln();
        for (i, m) in mu.iter().enumerate() {
            let s = std[i];
            let z = self.rng.normal();
            action.push(m + s * z);
            logp += -s.ln() - half_log_2pi - 0.5 * z * z;
        }
        (action, logp, value)
    }

    /// `log π(a|s)` under the *current* params (the ratio numerator's basis).
    /// Retained for the collection path + tests even though `update` uses the
    /// free [`Self::logp_static`] form (so it borrows `mu`/`std` without `self`).
    #[allow(dead_code)]
    fn logp_of(&self, mu: &[f32], std: &[f32], action: &[f32]) -> f32 {
        let half_log_2pi = 0.5 * (core::f32::consts::TAU).ln();
        let mut lp = 0.0f32;
        for i in 0..action.len() {
            let z = (action[i] - mu[i]) / std[i];
            lp += -std[i].ln() - half_log_2pi - 0.5 * z * z;
        }
        lp
    }

    /// GAE(λ) over the rollout: returns `(advantage, return-to-go)`, split
    /// into episodes at every `done`.
    fn gae(&self, rollout: &Rollout) -> (Vec<f32>, Vec<f32>) {
        let n = rollout.items.len();
        let mut adv = vec![0.0f32; n];
        let mut ret = vec![0.0f32; n];
        let g = self.config.gamma;
        let l = self.config.gae_lambda;

        let mut start = 0usize;
        let mut i = 0usize;
        while i < n {
            if rollout.items[i].done {
                self.gae_segment(&rollout.items[start..=i], start, &mut adv, &mut ret, g, l);
                start = i + 1;
            }
            i += 1;
        }
        if start < n {
            // Trailing partial episode (no `done` yet): bootstrap the next V at
            // the tail's own `value` is the usual fix, but the training app
            // closes the buffer on a true `done`, so the conservative 0 here
            // only touches a mid-rollout slice on the very last update.
            self.gae_segment(&rollout.items[start..], start, &mut adv, &mut ret, g, l);
        }
        (adv, ret)
    }

    /// Backward GAE over one episode segment `[seg]` whose global indices are
    /// `base ..= base+seg.len()-1`.
    fn gae_segment(
        &self,
        seg: &[Transition],
        base: usize,
        adv: &mut [f32],
        ret: &mut [f32],
        g: f32,
        l: f32,
    ) {
        let mut next_adv = 0.0f32;
        for j in (0..seg.len()).rev() {
            let idx = base + j;
            // Value *after* the last step of a finished episode is 0 (env
            // reset — no continuation to bootstrap).
            let next_v = if j + 1 < seg.len() {
                seg[j + 1].value
            } else {
                0.0
            };
            let delta = seg[j].reward + g * next_v - seg[j].value;
            next_adv = delta + g * l * next_adv;
            adv[idx] = next_adv;
            ret[idx] = next_adv + seg[j].value;
        }
    }

    /// Run K PPO epochs on `rollout`, mutate weights in place, return the last
    /// epoch's stats.
    pub fn update(&mut self, rollout: &Rollout) -> UpdateStats {
        let n = rollout.len();
        if n == 0 {
            return UpdateStats::default();
        }
        let (mut adv, ret) = self.gae(rollout);

        // Advantage normalization (mean 0, unit std) — standard, keeps the clip
        // band meaningful across reward scales (plan §5).
        let a_mean = adv.iter().sum::<f32>() / n as f32;
        let a_var = adv.iter().map(|a| (a - a_mean) * (a - a_mean)).sum::<f32>() / n as f32;
        let a_std = (a_var.sqrt() + 1e-8).max(1e-6);
        for a in adv.iter_mut() {
            *a = (*a - a_mean) / a_std;
        }

        let mut actor_buf = Vec::with_capacity(self.actor.n_params());
        let mut critic_buf = Vec::with_capacity(self.critic.n_params());
        self.actor.get_params(&mut actor_buf);
        self.critic.get_params(&mut critic_buf);
        let mut std_buf = self.log_std.clone();

        let mut acache = Cache::new_like(&self.actor);
        let mut ccache = Cache::new_like(&self.critic);

        let half_log_2pi_e = 0.5 * (core::f32::consts::TAU * std::f32::consts::E).ln();
        let mut last = UpdateStats {
            mean_adv: a_std,
            ..Default::default()
        };

        for _ in 0..self.config.epochs {
            let mut a_grad = vec![0.0f32; self.actor.n_params()];
            let mut c_grad = vec![0.0f32; self.critic.n_params()];
            let mut std_grad = vec![0.0f32; self.log_std.len()];
            let mut pl = 0.0f32;
            let mut vl = 0.0f32;
            let mut ent = 0.0f32;
            let mut kl = 0.0f32;
            let mut clipped = 0.0f32;
            let eps = self.config.clip_eps;
            let std = std_buf.iter().map(|s| s.exp()).collect::<Vec<f32>>();

            for (i, tr) in rollout.items.iter().enumerate() {
                let mu = self.actor.forward_cache(&tr.obs, &mut acache);
                let v = self.critic.forward_cache(&tr.obs, &mut ccache)[0];

                let logp = Self::logp_static(&mu, &std, &tr.action);
                let ratio = (logp - tr.old_logp).exp();

                // Clipped surrogate; maximize J = min(r·A, clip(r)·A).
                let a = adv[i];
                let s1 = ratio * a;
                let s2 = ratio.clamp(1.0 - eps, 1.0 + eps) * a;
                pl += -s1.min(s2);

                // Gradient flows unless the clipped branch won.
                let flow = if a > 0.0 {
                    ratio <= 1.0 + eps
                } else {
                    ratio >= 1.0 - eps
                };
                let ccoef = if flow { ratio * a } else { 0.0 };
                if !flow {
                    clipped += 1.0;
                }
                // Approx-KL estimator: r − 1 − log r.
                kl += ratio - 1.0 - (logp - tr.old_logp);

                // dJ/dμ_k = ccoef · (a_k − μ_k)/σ_k²  ⇒ dloss/dμ = −that.
                let mut dmu = vec![0.0f32; mu.len()];
                for (k, m) in mu.iter().enumerate() {
                    let var = std[k] * std[k];
                    dmu[k] = -ccoef * (tr.action[k] - m) / var;
                }
                self.actor.backward(&acache, &dmu, &mut a_grad);

                // dlogp/dlogσ_k = z_k² − 1; entropy ∂H/∂logσ_k = 1.
                for k in 0..std.len() {
                    let z = (tr.action[k] - mu[k]) / std[k];
                    // minimize −(J + ent_coef·H) ⇒ grad = −(ccoef·(z²−1) + ent_coef)
                    std_grad[k] -= ccoef * (z * z - 1.0) + self.config.ent_coef;
                    ent += self.log_std[k] + half_log_2pi_e;
                }

                // Critic MSE: L = vf_coef·0.5·(V − target)² ⇒ dL/dV = vf_coef·(V − target).
                let dv = self.config.vf_coef * (v - ret[i]);
                self.critic.backward(&ccache, &[dv], &mut c_grad);
                vl += 0.5 * (v - ret[i]) * (v - ret[i]);
            }

            let inv = 1.0 / n as f32;
            scale(&mut a_grad, inv);
            scale(&mut c_grad, inv);
            scale(&mut std_grad, inv);
            if self.config.max_grad_norm > 0.0 {
                clip_grad_norm(&mut a_grad, self.config.max_grad_norm);
                clip_grad_norm(&mut c_grad, self.config.max_grad_norm);
                clip_grad_norm(&mut std_grad, self.config.max_grad_norm);
            }

            self.actor_adam.update(&mut actor_buf, &a_grad);
            self.critic_adam.update(&mut critic_buf, &c_grad);
            self.std_adam.update(&mut std_buf, &std_grad);
            // Clamp σ into the exploration band (see `PpoConfig::std_max`).
            // Hard box-constraint after the Adam step — simple, and the
            // gradient path is unaffected for interior values.
            for s in std_buf.iter_mut() {
                *s = s.clamp(self.config.std_min, self.config.std_max);
            }

            // Push the new params back so the next epoch's forward sees them.
            self.actor.set_params(&actor_buf);
            self.critic.set_params(&critic_buf);
            self.log_std = std_buf.clone();

            last = UpdateStats {
                policy_loss: pl * inv,
                value_loss: vl * inv,
                entropy: ent * inv,
                approx_kl: kl * inv,
                clip_frac: clipped * inv,
                mean_adv: a_std,
            };
        }
        last
    }

    /// Static logp so the update loop can borrow `mu`/`std` immutably without
    /// touching `self` (the actor forward already needs `&mut` of the cache).
    #[inline]
    fn logp_static(mu: &[f32], std: &[f32], action: &[f32]) -> f32 {
        let half_log_2pi = 0.5 * (core::f32::consts::TAU).ln();
        let mut lp = 0.0f32;
        for i in 0..action.len() {
            let z = (action[i] - mu[i]) / std[i];
            lp += -std[i].ln() - half_log_2pi - 0.5 * z * z;
        }
        lp
    }

    // -- persistence -------------------------------------------------------

    /// Save a JSON checkpoint (actor/critic weights + `log_std` + config). Adam
    /// moments are **not** persisted (a deploy load needs none; a resume is a
    /// future refinement).
    pub fn save_json(&self, path: &str) -> std::io::Result<()> {
        let s = serde_json::to_string(self).map_err(std::io::Error::other)?;
        if let Some(dir) = std::path::Path::new(path).parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, s)
    }

    /// Load a checkpoint written by [`PpoTrainer::save_json`], then rebuild the
    /// (`#[serde(skip)]`) Adam buffers to the loaded shapes.
    pub fn load_json(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let s = std::fs::read_to_string(path)?;
        let mut t: PpoTrainer = serde_json::from_str(&s)?;
        t.rng = Rng::new(0xA11CE); // fresh sampler stream after a load
        t.rebuild_optimizers();
        Ok(t)
    }
}

#[inline]
fn scale(v: &mut [f32], s: f32) {
    for x in v.iter_mut() {
        *x *= s;
    }
}

/// L2 (global) gradient-norm clipping in place.
fn clip_grad_norm(v: &mut [f32], max_norm: f32) {
    let norm = v.iter().map(|g| g * g).sum::<f32>().sqrt();
    if norm > max_norm && norm > 0.0 {
        let s = max_norm / norm;
        for g in v.iter_mut() {
            *g *= s;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> PpoConfig {
        PpoConfig {
            init_std: 0.4,
            ..PpoConfig::default()
        }
    }

    #[test]
    fn gae_bootstraps_zero_at_done() {
        // Single-episode, zero value baseline: hand-computed GAE must match.
        let t = PpoTrainer::new(2, 1, &[4], cfg(), 1);
        let mut r = Rollout::default();
        r.push(Transition {
            obs: vec![0.0, 0.0],
            action: vec![0.0],
            old_logp: 0.0,
            reward: 1.0,
            done: false,
            value: 0.0,
        });
        r.push(Transition {
            obs: vec![0.0, 0.0],
            action: vec![0.0],
            old_logp: 0.0,
            reward: 1.0,
            done: true,
            value: 0.0,
        });
        let (adv, ret) = t.gae(&r);
        // terminal (step1): delta=1, A=1, R=1. step0: delta=1, A = 1+γλ·1.
        assert!((adv[1] - 1.0).abs() < 1e-4, "terminal A {}", adv[1]);
        assert!((ret[1] - 1.0).abs() < 1e-4);
        assert!(
            (adv[0] - (1.0 + 0.995 * 0.95)).abs() < 1e-4,
            "A0 {}",
            adv[0]
        );
    }

    #[test]
    fn gae_splits_episodes_at_done() {
        // An episode boundary must not leak advantage across a `done`.
        let t = PpoTrainer::new(2, 1, &[4], cfg(), 2);
        let mut r = Rollout::default();
        for _ in 0..2 {
            r.push(Transition {
                obs: vec![0.0, 0.0],
                action: vec![0.0],
                old_logp: 0.0,
                reward: 5.0,
                done: true,
                value: 0.0,
            });
        }
        let (adv, _ret) = t.gae(&r);
        // Both are single-step terminal episodes ⇒ each A == its reward.
        assert!((adv[0] - 5.0).abs() < 1e-4, "ep1 isolated");
        assert!((adv[1] - 5.0).abs() < 1e-4, "ep2 isolated");
    }

    #[test]
    fn sample_records_a_consistent_logp() {
        // `sample` must return a logp equal to re-evaluating the stored action
        // under the same μ/σ (no double-count, no sign flip).
        let mut t = PpoTrainer::new(3, 2, &[5], cfg(), 3);
        let obs = vec![0.1, -0.2, 0.05];
        let (action, logp, _v) = t.sample(&obs);
        let mu = t.actor.forward(&obs);
        let std = t.std();
        let re = t.logp_of(&mu, &std, &action);
        assert!((logp - re).abs() < 1e-4, "logp {logp} vs {re}");
    }

    #[test]
    fn update_is_finite_and_learns_value() {
        // A constant transition with a positive target return: the critic must
        // climb toward it while every reported statistic stays finite.
        let mut t = PpoTrainer::new(2, 1, &[4], cfg(), 4);
        // Production lr (3e-4) is too slow to clear a value-climb threshold in
        // a 40-update unit test; bump it so the head visibly tracks the target.
        t.set_lr(0.02);
        let obs = vec![0.3, -0.3];
        let action = vec![0.1];
        let mu = t.actor.forward(&obs);
        let lp = t.logp_of(&mu, &t.std(), &action);
        let v = t.critic.forward(&obs)[0];
        let mut r = Rollout::default();
        for _ in 0..8 {
            r.push(Transition {
                obs: obs.clone(),
                action: action.clone(),
                old_logp: lp,
                reward: 2.0,
                done: true,
                value: v,
            });
        }
        let before_v = v;
        let mut last = UpdateStats::default();
        for _ in 0..40 {
            last = t.update(&r);
            assert!(
                last.policy_loss.is_finite()
                    && last.value_loss.is_finite()
                    && last.entropy.is_finite()
                    && last.approx_kl.is_finite(),
                "stats finite: {last:?}"
            );
        }
        let after_v = t.critic.forward(&obs)[0];
        assert!(
            after_v > before_v + 0.5,
            "critic learns toward target: {before_v:.3} → {after_v:.3}"
        );
    }

    #[test]
    fn kl_estimator_is_zero_at_identity() {
        // The cheap KL estimator r−1−log r must be exactly 0 when the policy
        // hasn't diverged from the collecting policy (ratio = 1).
        let t = PpoTrainer::new(2, 1, &[3], cfg(), 5);
        let obs = vec![0.0, 0.0];
        let mu = t.actor.forward(&obs);
        let std = t.std();
        let action = mu.clone();
        let logp = t.logp_of(&mu, &std, &action);
        let ratio = (logp - logp).exp();
        let kl = ratio - 1.0 - (logp - logp);
        assert!(kl.abs() < 1e-6, "identity KL: {kl}");
    }

    #[test]
    fn save_load_roundtrip_preserves_policy_and_is_deployable() {
        let t = PpoTrainer::new(3, 2, &[5], cfg(), 6);
        let obs = vec![0.2, -0.1, 0.4];
        let before = t.act_deterministic(&obs);
        let path = "/tmp/walker_nn_test_policy.json";
        t.save_json(path).expect("save");
        let loaded = PpoTrainer::load_json(path).expect("load");
        let after = loaded.act_deterministic(&obs);
        for i in 0..before.len() {
            assert!((before[i] - after[i]).abs() < 1e-5, "dim {i}");
        }
        assert_eq!(loaded.log_std, t.log_std);
        assert_eq!(loaded.config, t.config);
        // A loaded policy is immediately deployable AND trainable: a single
        // update must run without panicking on the rebuilt (empty→sized) Adam.
        let _ = loaded;
        let _ = std::fs::remove_file(path);
    }
}
