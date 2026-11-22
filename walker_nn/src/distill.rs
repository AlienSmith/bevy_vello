//! Distillation of two residual policies into one network — the **merge** step
//! of `plans/distill-single-model.md`.
//!
//! Two `tanh` MLPs cannot be merged by adding weights (the nonlinearities are
//! non-additive), but they **can** be merged in function space: both `B` (the
//! accumulated expert history) and `A` (the newest accepted residual) output
//! the *same semantic quantity* — a residual δ on the external keyframe
//! proposal — so `δ_authority(s) = δ_B(s) + δ_A(s)` is a valid target function.
//!
//! ```text
//! target(s) = μ_B(s) + μ_A(s)        # Stage 2: distill gen
//! student   ← clone of B's actor     # Stage 3: distill fit
//! student   ≈ μ_B + μ_A after MSE fit (re-anchor in distill form)
//! ```
//!
//! The fitter lives in the *lib* crate (not the bin) because `Mlp::backward`
//! needs the [`crate::mlp::Cache`] constructor, which is `pub(crate)`; the
//! `distill` binary is a thin CLI over these two pure functions.
//!
//! Determinism: given the same `(obs, B, A)` the targets are identical; given
//! the same `FitConfig` the fit replays identically (seeded jitter).

use crate::adam::Adam;
use crate::bundle::PolicyBundle;
use crate::mlp::{Cache, Rng};
use std::fmt;

/// Stage-2 knobs — `distill gen`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GenConfig {
    /// Per-dimension symmetric clamp applied to the *summed* target
    /// (`target[j] = clamp(μB[j] + μA[j], ±max)`). `None` = off (default).
    pub delta_max: Option<f32>,
}

impl Default for GenConfig {
    fn default() -> Self {
        Self { delta_max: None }
    }
}

/// Stage-3 knobs — `distill fit`. Small-MLP distillation is well studied and
/// does not need hand-tuned hyperparameters; the defaults mirror PPO's
/// `epochs`/`lr` and are the values the plan expects.
#[derive(Debug, Clone, PartialEq)]
pub struct FitConfig {
    /// Full-batch epochs over the (optional jitter-thickened) train rows.
    /// Default 8, mirroring PPO's `epochs`.
    pub epochs: usize,
    /// Adam learning rate. Default `3e-4` (PPO's actor lr).
    pub lr: f32,
    /// Fraction of rows held out as the val split (default `0.1`). The split
    /// is **every `round(1/val_frac)`-th row** (deterministic, temporally
    /// spread — not a random slice).
    pub val_frac: f32,
    /// Support-thickening: when set, each train row is replicated once with a
    /// small per-dimension noise of this std (p.d.-scale). The **same target**
    /// is reused for the noisy copy — valid because the teacher is locally
    /// smooth over a `p.d.`-scale perturbation. Default `None` (off).
    pub jitter: Option<f32>,
    /// Val-MSE ceiling for the report's `pass` flag. The *behavioural* gate in
    /// `walker_qa`, not this number, decides promotion.
    pub val_threshold: f32,
    /// Seed for the jitter RNG (reproducible fits).
    pub seed: u64,
}

impl Default for FitConfig {
    fn default() -> Self {
        Self {
            epochs: 8,
            lr: 3e-4,
            val_frac: 0.1,
            jitter: None,
            val_threshold: 0.05,
            seed: 0xD15_7411,
        }
    }
}

/// Per-epoch training + validation MSE for the report CSV and one-line summary.
#[derive(Debug, Clone)]
pub struct FitReport {
    pub rows: usize,
    pub train_rows: usize,
    pub val_rows: usize,
    pub action_dim: usize,
    pub epochs: usize,
    pub lr: f32,
    pub jitter: Option<f32>,
    /// Mean squared error (over the actor output head) per epoch on TRAIN.
    pub train_mse: Vec<f32>,
    /// Same, per epoch on VAL (`NaN` if the split is empty).
    pub val_mse: Vec<f32>,
    /// Last epoch's val MSE.
    pub final_mse: f32,
    pub val_threshold: f32,
    /// `final_mse < val_threshold` (only meaningful when val rows exist).
    pub pass: bool,
}

impl FitReport {
    pub fn one_line(&self) -> String {
        format!(
            "rows={} train={} val={} action={} epochs={} lr={} jitter={:?} \
             final_val_mse={:.6} threshold={} pass={}",
            self.rows,
            self.train_rows,
            self.val_rows,
            self.action_dim,
            self.epochs,
            self.lr,
            self.jitter,
            self.final_mse,
            self.val_threshold,
            self.pass
        )
    }
}

/// Errors that abort a distill step — almost always a shape-consistency
/// violation (plan §7 invariant 2) or an empty input, never a panic.
#[derive(Debug, Clone, PartialEq)]
pub enum DistillError {
    ShapeMismatch(String),
    Empty(String),
    NonFinite(String),
}

impl fmt::Display for DistillError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DistillError::ShapeMismatch(m) => write!(f, "shape mismatch: {m}"),
            DistillError::Empty(m) => write!(f, "empty input: {m}"),
            DistillError::NonFinite(m) => write!(f, "non-finite value: {m}"),
        }
    }
}

impl std::error::Error for DistillError {}

/// Stage 2 — generate the authority targets. `target = μ_B(s) + μ_A(s)` with
/// dim guards (plan §7 invariants 1–2). Pure function: no physics, instant,
/// re-runnable with a different `A*` at zero cost.
pub fn generate(
    obs: &[Vec<f32>],
    b: &PolicyBundle,
    a: &PolicyBundle,
    cfg: &GenConfig,
) -> Result<Vec<Vec<f32>>, DistillError> {
    let obs_dim = b.obs_dim();
    let action_dim = b.action_dim();
    if a.obs_dim() != obs_dim || a.action_dim() != action_dim {
        return Err(DistillError::ShapeMismatch(format!(
            "B obs/act {obs_dim}/{action_dim} != A obs/act {}/{}",
            a.obs_dim(),
            a.action_dim()
        )));
    }
    if b.actor.dims != a.actor.dims {
        return Err(DistillError::ShapeMismatch(format!(
            "actor shape must match across generations (\"model consistent from start\"): \
             B {:?} != A {:?}",
            b.actor.dims, a.actor.dims
        )));
    }
    if obs.is_empty() {
        return Err(DistillError::Empty("no obs rows to label".into()));
    }

    let mut targets = Vec::with_capacity(obs.len());
    for row in obs {
        if row.len() != obs_dim {
            return Err(DistillError::ShapeMismatch(format!(
                "obs row len {} != obs_dim {obs_dim}",
                row.len()
            )));
        }
        let b_mu = b.act_deterministic(row);
        let a_mu = a.act_deterministic(row);
        let mut t = Vec::with_capacity(action_dim);
        for j in 0..action_dim {
            let mut v = b_mu[j] + a_mu[j];
            if let Some(m) = cfg.delta_max {
                v = v.clamp(-m, m);
            }
            if !v.is_finite() {
                return Err(DistillError::NonFinite(format!(
                    "target[{j}] = {v} from B + A on obs len {}",
                    row.len()
                )));
            }
            t.push(v);
        }
        targets.push(t);
    }
    Ok(targets)
}

/// Val-split stride: every `round(1/val_frac)`-th row is held out (deterministic).
fn val_stride(val_frac: f32) -> usize {
    if val_frac <= 0.0 {
        0
    } else {
        (1.0 / val_frac).round().max(1.0) as usize
    }
}

/// Stage 3 — fit a single net to the authority targets.
///
/// Student init = deep clone of `B`'s actor (it already knows the previous
/// expert history; the fit only learns the small `A` addition — the re-anchor
/// in distill form). Loss is MSE over the actor output head; the optimizer is
/// the existing [`Adam`] over `Mlp::get_params`/`set_params`. `B`'s `critic`
/// and `log_std` are copied unchanged (deploy uses `act_deterministic` only).
///
/// Returns the distilled `PolicyBundle` (same shape `walker_qa POLICY=` /
/// `walker_ppo RESUME=` load) and a [`FitReport`].
pub fn fit(
    obs: &[Vec<f32>],
    targets: &[Vec<f32>],
    expert: &PolicyBundle,
    cfg: &FitConfig,
) -> Result<(PolicyBundle, FitReport), DistillError> {
    let obs_dim = expert.obs_dim();
    let action_dim = expert.action_dim();
    if obs.len() != targets.len() {
        return Err(DistillError::ShapeMismatch(format!(
            "obs rows {} != target rows {}",
            obs.len(),
            targets.len()
        )));
    }
    if obs.is_empty() {
        return Err(DistillError::Empty("no (obs, target) pairs to fit".into()));
    }
    if cfg.epochs == 0 {
        return Err(DistillError::Empty("epochs must be > 0".into()));
    }

    // Split deterministically: every `stride`-th row into val (temporally
    // spread, not a random slice), the rest into train.
    let stride = val_stride(cfg.val_frac);
    let mut train: Vec<(Vec<f32>, Vec<f32>)> = Vec::new();
    let mut val: Vec<(Vec<f32>, Vec<f32>)> = Vec::new();
    for (i, row) in obs.iter().enumerate() {
        if row.len() != obs_dim {
            return Err(DistillError::ShapeMismatch(format!(
                "obs row [{i}] len {} != obs_dim {obs_dim}",
                row.len()
            )));
        }
        if targets[i].len() != action_dim {
            return Err(DistillError::ShapeMismatch(format!(
                "target row [{i}] len {} != action_dim {action_dim}",
                targets[i].len()
            )));
        }
        let pair = (row.clone(), targets[i].clone());
        if stride > 0 && i % stride == 0 {
            val.push(pair);
        } else {
            train.push(pair);
        }
    }
    if train.is_empty() {
        // A dataset smaller than the val stride lands every row in val; that
        // leaves nothing to learn from — refuse rather than silently no-op.
        return Err(DistillError::Empty(format!(
            "no train rows (rows={} all fell into the val split at stride {stride})",
            obs.len()
        )));
    }

    // Optional support thickening: replicate each train row once with a small
    // obs perturbation, reusing the (locally smooth) target.
    if let Some(sigma) = cfg.jitter {
        if sigma > 0.0 {
            let mut rng = Rng::new(cfg.seed);
            let mut extra = Vec::with_capacity(train.len());
            for (o, t) in &train {
                let o2: Vec<f32> = o.iter().map(|v| v + sigma * rng.normal()).collect();
                extra.push((o2, t.clone()));
            }
            train.extend(extra);
        }
    }

    // Student = clone of B's actor. The critic + log_std ride along unchanged.
    let mut student = expert.clone();
    let n = student.actor.n_params();
    let mut params = Vec::with_capacity(n);
    student.actor.get_params(&mut params);
    let mut adam = Adam::new(n);
    adam.lr = cfg.lr;

    let train_rows = train.len();
    let val_rows = val.len();
    let mut train_mse = Vec::with_capacity(cfg.epochs);
    let mut val_mse = Vec::with_capacity(cfg.epochs);
    let mut cache = Cache::new_like(&student.actor);

    for _ep in 0..cfg.epochs {
        // One full-batch pass: accumulate the MSE gradient over every train row.
        let mut grad = vec![0.0f32; n];
        let mut sum = 0.0f32;
        for (o, t) in &train {
            let out = student.actor.forward_cache(o, &mut cache);
            let mut dout = vec![0.0f32; action_dim];
            for j in 0..action_dim {
                let e = out[j] - t[j];
                // MSE = (1/act) Σ e² ⇒ dL/dout = (2/act) e.
                dout[j] = 2.0 * e / action_dim as f32;
                sum += e * e / action_dim as f32;
            }
            student.actor.backward(&cache, &dout, &mut grad);
        }
        train_mse.push(sum / train_rows as f32);
        adam.update(&mut params, &grad);
        student.actor.set_params(&params);

        // Val MSE under the just-updated weights (no gradient).
        if val_rows > 0 {
            let mut vsum = 0.0f32;
            for (o, t) in &val {
                let out = student.actor.forward(o);
                for j in 0..action_dim {
                    let e = out[j] - t[j];
                    vsum += e * e / action_dim as f32;
                }
            }
            val_mse.push(vsum / val_rows as f32);
        } else {
            val_mse.push(f32::NAN);
        }
    }

    let final_mse = *val_mse.last().unwrap_or(&f32::NAN);
    let pass = val_rows > 0 && final_mse < cfg.val_threshold;

    let report = FitReport {
        rows: obs.len(),
        train_rows,
        val_rows,
        action_dim,
        epochs: cfg.epochs,
        lr: cfg.lr,
        jitter: cfg.jitter,
        train_mse,
        val_mse,
        final_mse,
        val_threshold: cfg.val_threshold,
        pass,
    };
    Ok((student, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ppo::{PpoConfig, PpoTrainer};

    fn bundle(obs: usize, act: usize, hidden: &[usize], seed: u64) -> PolicyBundle {
        PolicyBundle::from_trainer(&PpoTrainer::new(
            obs,
            act,
            hidden,
            PpoConfig::default(),
            seed,
        ))
    }

    fn rows(n: usize, obs_dim: usize, seed: u64) -> Vec<Vec<f32>> {
        let mut rng = Rng::new(seed);
        (0..n)
            .map(|_| (0..obs_dim).map(|_| rng.uniform(-1.0, 1.0)).collect())
            .collect()
    }

    /// Force `A`'s actor output to a fixed constant (last-layer w = 0, bias =
    /// c). Then `mu_A(s) ≡ c` — a small, exact, nonlinear-free addition `B`
    /// must learn, isolating the "re-anchor" mechanics.
    fn pin_output_const(bundle: &mut PolicyBundle, c: &[f32]) {
        let last = bundle.actor.layers.last_mut().unwrap();
        last.w.iter_mut().for_each(|v| *v = 0.0);
        last.b.clone_from_slice(c);
    }

    #[test]
    fn generate_is_deterministic_and_equals_mu_sum() {
        let b = bundle(6, 5, &[8, 8], 1);
        let a = bundle(6, 5, &[8, 8], 2);
        let obs = rows(200, 6, 7);

        let t1 = generate(&obs, &b, &a, &GenConfig::default()).unwrap();
        let t2 = generate(&obs, &b, &a, &GenConfig::default()).unwrap();
        assert_eq!(t1, t2, "gen must be a pure function of (obs, B, A)");

        // Manual identity: target[j] == muB[j] + muA[j] on every row.
        for (i, row) in obs.iter().enumerate() {
            let bmu = b.act_deterministic(row);
            let amu = a.act_deterministic(row);
            for j in 0..5 {
                assert!((t1[i][j] - (bmu[j] + amu[j])).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn generate_applies_the_delta_max_clamp() {
        let mut b = bundle(4, 2, &[6], 3);
        let mut a = bundle(4, 2, &[6], 4);
        // Both outputs near +2 → sum ≈ 4; a clamp at 1.0 must bind.
        pin_output_const(&mut b, &[2.0, -2.0]);
        pin_output_const(&mut a, &[2.0, -2.0]);
        let obs = rows(10, 4, 5);
        let t = generate(
            &obs,
            &b,
            &a,
            &GenConfig {
                delta_max: Some(1.0),
            },
        )
        .unwrap();
        for row in &t {
            assert!((row[0] - 1.0).abs() < 1e-6);
            assert!((row[1] + 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn generate_rejects_shape_mismatch_across_generations() {
        let b = bundle(6, 5, &[8, 8], 1);
        let a_bad_obs = bundle(7, 5, &[8, 8], 2); // different obs width
        let a_bad_dims = bundle(6, 5, &[16, 8], 2); // same widths, different hidden
        let obs = rows(5, 6, 3);
        assert!(matches!(
            generate(&obs, &b, &a_bad_obs, &GenConfig::default()),
            Err(DistillError::ShapeMismatch(_))
        ));
        assert!(matches!(
            generate(&obs, &b, &a_bad_dims, &GenConfig::default()),
            Err(DistillError::ShapeMismatch(_))
        ));
        assert!(matches!(
            generate(&[], &b, &b, &GenConfig::default()),
            Err(DistillError::Empty(_))
        ));
    }

    #[test]
    fn fit_absorbs_residual_bias_shift_exactly() {
        // B: a non-degenerate random net (not a zero net). A: output pinned to
        // a fixed constant c. target = mu_B(s) + c. The student (clone of B)
        // must learn to add c to its output — an exact re-anchor problem.
        let b = bundle(6, 5, &[8, 8], 11);
        let mut a = bundle(6, 5, &[8, 8], 12);
        let c = [0.1f32, -0.05, 0.2, -0.15, 0.08];
        pin_output_const(&mut a, &c);

        let obs = rows(400, 6, 13);
        let targets = generate(&obs, &b, &a, &GenConfig::default()).unwrap();

        let cfg = FitConfig {
            epochs: 120,
            lr: 0.02, // larger lr so the linear bias shift settles in-test
            val_frac: 0.1,
            ..FitConfig::default()
        };
        let (student, report) = fit(&obs, &targets, &b, &cfg).unwrap();

        // Consistency invariant 2: the student is the same shape as B.
        assert_eq!(
            student.actor.dims, b.actor.dims,
            "shape is stable across generations"
        );
        // Critic + log_std ride along unchanged.
        assert_eq!(student.critic, b.critic);
        assert_eq!(student.log_std, b.log_std);
        // The fit actually learned: val rows are outside the reported val split
        // threshold and mu ≈ target there.
        assert!(report.val_rows > 0);
        assert!(
            report.final_mse < 1e-4,
            "exact bias shift must converge: final val mse {}",
            report.final_mse
        );
        assert!(report.pass, "final val mse under threshold");

        for (i, row) in obs.iter().enumerate() {
            if i % 10 == 0 {
                // val rows — never touched by the optimizer. Per-row ceiling is
                // the rms implied by the mse bound (sqrt(5 · 1e-4) ≈ 0.022):
                // while the output bias shifts, the hidden layers are coupled,
                // so a small residual is expected — the mse is the tight gate.
                let mu = student.act_deterministic(row);
                for j in 0..5 {
                    assert!(
                        (mu[j] - targets[i][j]).abs() < 3e-2,
                        "row {i} act {j}: student {} vs target {}",
                        mu[j],
                        targets[i][j]
                    );
                }
            }
        }

        // The produced bundle round-trips as a deployable PolicyBundle.
        let path = "/tmp/walker_nn_distill_policy_new.json";
        student.save_json(path).expect("save distilled policy");
        let loaded = PolicyBundle::load_json(path).expect("distilled json loads as PolicyBundle");
        assert_eq!(loaded.obs_dim(), b.obs_dim());
        assert_eq!(loaded.action_dim(), b.action_dim());
        assert_eq!(loaded.actor.dims, b.actor.dims);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn fit_jitter_thickens_train_rows() {
        let b = bundle(4, 2, &[6], 21);
        let obs = rows(40, 4, 22);
        let targets = generate(&obs, &b, &b, &GenConfig::default()).unwrap();
        let cfg = FitConfig {
            epochs: 2,
            jitter: Some(0.01),
            val_frac: 0.1,
            ..FitConfig::default()
        };
        let (_, report) = fit(&obs, &targets, &b, &cfg).unwrap();
        // 40 rows, stride 10 ⇒ 36 train; jitter doubles them.
        assert_eq!(report.train_rows, 36 * 2);
        assert_eq!(report.val_rows, 4);
        assert_eq!(report.rows, 40);
    }
}
