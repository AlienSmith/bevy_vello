//! `walker_nn` — a hand-rolled feedforward net + Adam + PPO, no ML framework,
//! no ECS. The standalone "NN core" the plan (§2) extracts so the bridge
//! (`walker_control`) merely *consumes* a policy, and other moves reuse the
//! same trainer.
//!
//! ```text
//! mlp.rs     dense layers, forward/backward, deterministic RNG
//! adam.rs    Adam over a flat param block
//! ppo.rs     Gaussian policy head + critic, rollout buffer, GAE, clipped update
//! dataset.rs `.obs`/`.target` LE f32 matrix IO + manifest sidecar (distill plan §2)
//! distill.rs gen (μ_B + μ_A targets) + fit (clone-of-B student, MSE + Adam)
//! ```
//!
//! The PPO trainer (`crate::ppo`) is intentionally free of any walker / physics
//! concept: it consumes `(obs, action, logp, reward, done, value)` transitions.
//! The env contract (what `obs` and `action` mean, the reward, episode reset)
//! lives in `walker_control::control::LearnedCtrl` and the training app — the
//! same separation `walker_math` keeps from `walker_control`.

pub mod adam;
pub mod bundle;
pub mod dataset;
pub mod distill;
pub mod mlp;
pub mod ppo;

pub use adam::Adam;
pub use bundle::PolicyBundle;
pub use dataset::DatasetMeta;
pub use distill::{fit, generate, DistillError, FitConfig, FitReport, GenConfig};
pub use mlp::{Mlp, Rng};
pub use ppo::{PpoConfig, PpoTrainer, Rollout, Transition, UpdateStats};
