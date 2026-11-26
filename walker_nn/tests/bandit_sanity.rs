//! Sanity: can the PPO trainer improve a policy on the simplest possible task?
//! One-step episodes, constant observation, reward = -|a - target|^2 on a 5-dim action
//! (same obs/action widths as the jump residual). A correct PPO must move the mean
//! action toward the target and raise the reward. Also runs a noisier variant where
//! the reward carries a large random term (like landing luck) to see how much it slows.
use walker_nn::{PpoConfig, PpoTrainer, Rollout, Transition};

fn run(noise: f32, std: f32, epochs: usize, updates: usize, n: usize) -> (f32, f32) {
    let target = [0.3f32, -0.2, 0.25, -0.3, 0.15];
    let mut cfg = PpoConfig::default();
    cfg.init_std = std;
    cfg.epochs = epochs;
    cfg.ent_coef = 0.0;
    let mut t = PpoTrainer::new(22, 5, &[64, 64], cfg, 7);
    t.actor.zero_last_layer();
    t.set_lr(1e-3);
    let obs = vec![0.1f32; 22];
    let mut seed = 12345u64;
    let mut rnd = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((seed >> 33) as f32 / (1u64 << 31) as f32) - 0.5
    };
    let mut first = 0.0;
    let mut last = 0.0;
    for u in 0..updates {
        let mut ro = Rollout::default();
        let mut mean_r = 0.0;
        for _ in 0..n {
            let (a, logp, v) = t.sample(&obs);
            let d: f32 = a.iter().zip(&target).map(|(x, y)| (x - y) * (x - y)).sum();
            let r = -d + noise * rnd() * 2.0;
            mean_r += r / n as f32;
            ro.push(Transition { obs: obs.clone(), action: a, old_logp: logp, reward: r, done: true, value: v });
        }
        t.update(&ro);
        if u == 0 { first = mean_r; }
        last = mean_r;
    }
    let mu = t.act_deterministic(&obs);
    let err: f32 = mu.iter().zip(&target).map(|(x, y)| (x - y).abs()).sum::<f32>() / 5.0;
    println!("noise {noise} std {std}: reward {first:.3} -> {last:.3}; mean |mu - target| = {err:.3} (start {:.3})", target.iter().map(|x| x.abs()).sum::<f32>() / 5.0);
    (last - first, err)
}

#[test]
fn ppo_improves_a_trivial_policy() {
    let (gain, err) = run(0.0, 0.3, 8, 60, 256);
    assert!(gain > 0.0 && err < 0.15, "PPO failed the 1-step bandit: gain {gain}, err {err}");
}

#[test]
fn ppo_with_reward_noise_report() {
    // Report-only: a reward with +-1.0 uniform noise (much larger than the signal).
    let _ = run(1.0, 0.3, 8, 60, 256);
    let _ = run(1.0, 0.3, 8, 60, 32);
}
