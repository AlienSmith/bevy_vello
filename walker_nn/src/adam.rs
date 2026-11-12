//! Adam over a flat `Vec<f32>` parameter block (see [`crate::mlp::Mlp`]).
//!
//! Kept separate from the network because the optimizer state (moments) is a
//! *training* concern the deploy path never touches: a frozen `Mlp` needs no
//! `Adam`, and the two share only the flat-param convention from
//! [`Mlp::get_params`]. Standard decoupled-free Adam (Kingma & Ba, 2014):
//!
//! ```text
//! m ← β₁m + (1−β₁)g
//! v ← β₂v + (1−β₂)g²
//! m̂ = m/(1−β₁ᵗ),  v̂ = v/(1−β₂ᵗ)
//! θ ← θ − α·m̂/(√v̂ + ε)
//! ```

use serde::{Deserialize, Serialize};

/// Adam hyperparameters + running moments for a fixed-size block.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Adam {
    pub lr: f32,
    pub beta1: f32,
    pub beta2: f32,
    pub eps: f32,
    pub step: u64,
    pub m: Vec<f32>,
    pub v: Vec<f32>,
}

impl Default for Adam {
    /// Empty moments at the plan's default hyperparameters; sized later by
    /// [`Adam::reset_to`] when the param block length is known.
    fn default() -> Self {
        Self::new(0)
    }
}

impl Adam {
    /// Resize this optimizer's moment blocks to `n` in place, preserving the
    /// hyperparameters (the step counter restarts).
    pub fn reset_to(&mut self, n: usize) {
        if self.m.len() != n || self.v.len() != n {
            self.m = vec![0.0; n];
            self.v = vec![0.0; n];
            self.step = 0;
        }
    }

    /// Defaults from the plan (§5): `lr = 3e-4`, standard betas.
    pub fn new(n_params: usize) -> Self {
        Self {
            lr: 3e-4,
            beta1: 0.9,
            beta2: 0.999,
            eps: 1e-8,
            step: 0,
            m: vec![0.0; n_params],
            v: vec![0.0; n_params],
        }
    }

    /// Apply one update in place: `params ← params − α·AdamStep(grad)`.
    pub fn update(&mut self, params: &mut [f32], grad: &[f32]) {
        debug_assert_eq!(params.len(), self.m.len());
        debug_assert_eq!(grad.len(), self.m.len());
        self.step += 1;
        let t = self.step as f32;
        // Bias-correction denominators.
        let bc1 = 1.0 - self.beta1.powi(self.step as i32);
        let bc2 = 1.0 - self.beta2.powi(self.step as i32);
        let _ = t;
        for i in 0..params.len() {
            let g = grad[i];
            self.m[i] = self.beta1 * self.m[i] + (1.0 - self.beta1) * g;
            self.v[i] = self.beta2 * self.v[i] + (1.0 - self.beta2) * g * g;
            let m_hat = self.m[i] / bc1;
            let v_hat = self.v[i] / bc2;
            params[i] -= self.lr * m_hat / (v_hat.sqrt() + self.eps);
        }
    }

    /// Reset the moments (e.g. between independent training runs).
    pub fn reset(&mut self) {
        self.m.iter_mut().for_each(|x| *x = 0.0);
        self.v.iter_mut().for_each(|x| *x = 0.0);
        self.step = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimize `f(θ) = (θ₀−3)² + (θ₁+2)²`: Adam must walk θ toward the
    /// stationary point at (3, −2) from the origin.
    #[test]
    fn adam_descends_a_quadratic() {
        let mut params = vec![0.0f32, 0.0];
        let mut adam = Adam::new(2);
        adam.lr = 0.1; // larger lr so a handful of steps suffice in a unit test
        for _ in 0..300 {
            let grad = vec![2.0 * (params[0] - 3.0), 2.0 * (params[1] + 2.0)];
            adam.update(&mut params, &grad);
        }
        assert!(
            (params[0] - 3.0).abs() < 0.05 && (params[1] + 2.0).abs() < 0.05,
            "converged to stationary point: {params:?}"
        );
    }

    #[test]
    fn zero_grad_is_a_noop() {
        let mut params = vec![1.0, -2.0, 3.0];
        let before = params.clone();
        let mut adam = Adam::new(3);
        adam.update(&mut params, &[0.0, 0.0, 0.0]);
        assert_eq!(params, before, "no gradient must not move the block");
    }
}
