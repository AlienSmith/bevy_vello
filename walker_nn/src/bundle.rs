//! A frozen snapshot of a trained policy — the artifact a [`LearnedCtrl`] runs.
//!
//! PPO is on-policy: during one rollout the weights are *frozen* and N env
//! characters act with the same policy, then the harness runs one [`super::
//! PpoTrainer::update`] and re-snapshots. So the deploy/train path needs only
//! the three read-only pieces of a trainer — the actor MLP, the critic MLP, and
//! the state-independent `log_std` — not the optimizer. That is exactly what a
//! `PolicyBundle` holds, and why it is `Clone` + `Serialize`: it slots into the
//! `Controller` enum (which is `Clone`) and loads straight from a checkpoint.
//!
//! The bundle owns an `Rng` for sampling. The **train** path samples `a ~
//! N(μ,σ)`; the **deploy** path takes `μ` deterministically (plan §4).

use serde::{Deserialize, Serialize};

use crate::mlp::{Mlp, Rng};

/// Read-only policy weights: actor `μ(s)`, critic `V(s)`, learned `log_std`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyBundle {
    pub actor: Mlp,
    pub critic: Mlp,
    pub log_std: Vec<f32>,
}

impl PolicyBundle {
    /// Freeze the current weights of `trainer` into a fresh snapshot. The
    /// sampling `Rng` is **not** part of the bundle (it lives in the controller
    /// that acts on it), so this is pure weight copying.
    pub fn from_trainer(trainer: &super::PpoTrainer) -> Self {
        Self {
            actor: trainer.actor.clone(),
            critic: trainer.critic.clone(),
            log_std: trainer.log_std.clone(),
        }
    }

    pub fn action_dim(&self) -> usize {
        self.actor.out_dim()
    }
    pub fn obs_dim(&self) -> usize {
        self.actor.in_dim()
    }

    /// `σ = exp(log_std)`.
    fn std(&self) -> Vec<f32> {
        self.log_std.iter().map(|s| s.exp()).collect()
    }

    /// Deterministic mean `μ(s)` — the deploy forward (no sampling).
    pub fn act_deterministic(&self, obs: &[f32]) -> Vec<f32> {
        self.actor.forward(obs)
    }

    /// `V(s)` from the critic (the deploy path records it for telemetry; the
    /// train path gets it from [`Self::sample`] in one pass).
    pub fn value_of(&self, obs: &[f32]) -> f32 {
        self.critic.forward(obs)[0]
    }

    /// Sample `a ~ N(μ, σ)` and return `(action, log π(a|s), V(s))` — the
    /// on-policy collection step. The caller records all three into its
    /// transition; `V(s)` is the GAE baseline frozen at collection time.
    pub fn sample(&self, obs: &[f32], rng: &mut Rng) -> (Vec<f32>, f32, f32) {
        let mu = self.actor.forward(obs);
        let value = self.critic.forward(obs)[0];
        let std = self.std();
        let half_log_2pi = 0.5 * (core::f32::consts::TAU).ln();
        let mut action = Vec::with_capacity(mu.len());
        let mut logp = 0.0f32;
        for (i, m) in mu.iter().enumerate() {
            let s = std[i];
            let z = rng.normal();
            action.push(m + s * z);
            logp += -s.ln() - half_log_2pi - 0.5 * z * z;
        }
        (action, logp, value)
    }

    pub fn save_json(&self, path: &str) -> std::io::Result<()> {
        let s = serde_json::to_string(self).map_err(std::io::Error::other)?;
        if let Some(dir) = std::path::Path::new(path).parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, s)
    }

    pub fn load_json(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let s = std::fs::read_to_string(path)?;
        Ok(serde_json::from_str(&s)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ppo::{PpoConfig, PpoTrainer};

    #[test]
    fn bundle_act_matches_deterministic_trainer() {
        // The deploy forward through the bundle must equal the trainer's own
        // `act_deterministic` (same weights, frozen).
        let mut t = PpoTrainer::new(3, 2, &[4], PpoConfig::default(), 9);
        let obs = vec![0.1, -0.3, 0.2];
        let bundle = PolicyBundle::from_trainer(&t);
        assert_eq!(bundle.act_deterministic(&obs), t.act_deterministic(&obs));
        // Sampling is stochastic but returns a consistent-width action.
        let (a, lp, v) = bundle.sample(&obs, &mut Rng::new(1));
        assert_eq!(a.len(), 2);
        assert!(lp.is_finite() && v.is_finite());
    }

    #[test]
    fn a_trainer_checkpoint_loads_as_a_deployable_bundle() {
        // The real pipeline: `walker_ppo` writes a **PpoTrainer** json; `walker_qa`
        // reads it back as a **PolicyBundle**. Those are different structs (the
        // trainer json carries an extra `config` and skips its Adam buffers), so
        // the cross-format load must be proven, not assumed.
        let mut t = PpoTrainer::new(5, 2, &[8, 8], PpoConfig::default(), 21);
        // Make the weights unmistakably non-default so a silent zero-load fails.
        t.actor.zero_last_layer();
        let obs = vec![0.3, -0.2, 0.5, 0.0, -0.4];
        let expected = t.act_deterministic(&obs);
        let path = "/tmp/walker_nn_trainer_as_bundle.json";
        t.save_json(path).expect("trainer save");

        let b = PolicyBundle::load_json(path).expect("bundle loads a trainer file");
        assert_eq!(b.obs_dim(), 5, "obs width survives the cross-format load");
        assert_eq!(
            b.action_dim(),
            2,
            "action width survives the cross-format load"
        );
        assert_eq!(b.log_std, t.log_std, "log_std survives");
        assert_eq!(
            b.act_deterministic(&obs),
            expected,
            "the bundle deploys the exact trained weights"
        );
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn bundle_save_load_roundtrip() {
        let t = PpoTrainer::new(4, 3, &[6], PpoConfig::default(), 11);
        let b = PolicyBundle::from_trainer(&t);
        let path = "/tmp/walker_nn_bundle.json";
        b.save_json(path).expect("save");
        let l = PolicyBundle::load_json(path).expect("load");
        let obs = vec![0.0; 4];
        assert_eq!(b.actor, l.actor);
        assert_eq!(b.critic, l.critic);
        assert_eq!(b.log_std, l.log_std);
        assert_eq!(
            b.act_deterministic(&obs),
            l.act_deterministic(&obs),
            "loaded bundle is deployable"
        );
        let _ = std::fs::remove_file(path);
    }
}
