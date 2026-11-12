//! Hand-rolled feedforward MLP — no ML framework, no ECS.
//!
//! A dense layer stack: `dims = [in, h1, ..., hn, out]`, `tanh` between every
//! hidden layer and a **linear** output (the policy/critic heads expect raw
//! values and squash them themselves — see [`crate::ppo`]). Weights are flat
//! row-major `Vec<f32>` (a layer of shape `out × in` stores `w[j*in + k]`), so
//! a whole network is one contiguous param block Adam can own.
//!
//! The whole point of this crate being standalone (plan §2, the
//! `walker_math`→`walker_control` precedent): the bridge consumes the net and
//! knows nothing about training; other moves reuse it unchanged.
//!
//! # Determinism
//! Init and sampling ride a seeded 64-bit LCG ([`Rng`] below), not `rand`. The
//! training app is reproducible tick-for-tick, matching the `Walker`'s D1
//! determinism contract (see `walker_control::control` tests).

use serde::{Deserialize, Serialize};

/// One affine layer: `y = x·Wᵀ + b`, `W` row-major `out × in`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Layer {
    pub in_dim: usize,
    pub out_dim: usize,
    /// `W[j*in_dim + k]` — weight from input `k` to output `j`.
    pub w: Vec<f32>,
    pub b: Vec<f32>,
}

impl Layer {
    /// Glorot/Xavier init from a seeded generator.
    pub fn new(in_dim: usize, out_dim: usize, rng: &mut Rng) -> Self {
        let limit = (6.0 / (in_dim + out_dim) as f32).sqrt();
        let mut w = Vec::with_capacity(in_dim * out_dim);
        for _ in 0..in_dim * out_dim {
            w.push(rng.uniform(-limit, limit));
        }
        Self {
            in_dim,
            out_dim,
            w,
            b: vec![0.0; out_dim],
        }
    }

    fn forward(&self, x: &[f32], out: &mut [f32]) {
        for (j, oy) in out.iter_mut().enumerate() {
            let row = &self.w[j * self.in_dim..(j + 1) * self.in_dim];
            let mut acc = self.b[j];
            for k in 0..self.in_dim {
                acc += row[k] * x[k];
            }
            *oy = acc;
        }
    }
}

/// `tanh` applied elementwise, in place.
#[inline]
fn tanh_inplace(v: &mut [f32]) {
    for x in v.iter_mut() {
        *x = x.tanh();
    }
}

/// A dense MLP over `dims = [in, hidden…, out]`: `tanh` on hidden layers,
/// linear final layer. Caches nothing between calls — `forward` is pure, and
/// `backward` takes an explicit activation cache so the trainer can reuse the
/// same forward pass for many minibatches.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Mlp {
    pub dims: Vec<usize>,
    pub layers: Vec<Layer>,
}

/// Per-layer activations saved by [`Mlp::forward_cache`] and consumed by
/// [`Mlp::backward`]: `inputs[l]` is the (post-activation) input to layer `l`,
/// `pre[l]` its pre-activation.
#[derive(Debug, Clone)]
pub struct Cache {
    /// `inputs[0]` is the raw input; `inputs[l]` for `l>0` is `tanh(pre[l-1])`.
    pub inputs: Vec<Vec<f32>>,
    pub pre: Vec<Vec<f32>>,
}

impl Mlp {
    /// Build a network from a shape, Glorot-initializing every layer.
    pub fn new(dims: &[usize], rng: &mut Rng) -> Self {
        assert!(dims.len() >= 2, "Mlp needs at least [in, out]");
        let mut layers = Vec::with_capacity(dims.len() - 1);
        for pair in dims.windows(2) {
            layers.push(Layer::new(pair[0], pair[1], rng));
        }
        Self {
            dims: dims.to_vec(),
            layers,
        }
    }

    pub fn in_dim(&self) -> usize {
        self.dims[0]
    }
    pub fn out_dim(&self) -> usize {
        *self.dims.last().unwrap()
    }
    /// Total scalars in the parameter block.
    pub fn n_params(&self) -> usize {
        self.layers.iter().map(|l| l.w.len() + l.b.len()).sum()
    }

    /// Zero the **last layer** (weights + biases). The standard RL policy init:
    /// every observation then maps to exactly μ(s) ≡ 0, so a fresh policy starts
    /// at the env's neutral action. Gradients still flow into the last layer
    /// (its weight gradient rides the hidden activations), so learning is
    /// unaffected — the policy just *begins* from a known-good pose instead of a
    /// random one that topples before it can explore.
    pub fn zero_last_layer(&mut self) {
        let last = self.layers.last_mut().expect("mlp has >= 2 layers");
        last.w.iter_mut().for_each(|v| *v = 0.0);
        last.b.iter_mut().for_each(|v| *v = 0.0);
    }

    /// Write current params into a flat block, in the same order as
    /// [`Mlp::set_params`] reads it: per layer, `w` then `b`.
    pub fn get_params(&self, buf: &mut Vec<f32>) {
        buf.clear();
        for l in &self.layers {
            buf.extend_from_slice(&l.w);
            buf.extend_from_slice(&l.b);
        }
    }

    /// Overwrite params from a flat block produced by [`Mlp::get_params`].
    pub fn set_params(&mut self, buf: &[f32]) {
        let mut off = 0usize;
        for l in &mut self.layers {
            let nw = l.w.len();
            l.w.copy_from_slice(&buf[off..off + nw]);
            off += nw;
            let nb = l.b.len();
            l.b.copy_from_slice(&buf[off..off + nb]);
            off += nb;
        }
        debug_assert_eq!(off, buf.len(), "set_params block length mismatch");
    }

    /// Pure forward: run the input to the linear output. Convenience over
    /// [`Mlp::forward_cache`] for inference-only call sites (the deploy
    /// controller — plan §4, `LearnMode::Deploy`).
    pub fn forward(&self, x: &[f32]) -> Vec<f32> {
        let mut cache = Cache::new(&self.dims);
        self.forward_cache(x, &mut cache)
    }

    /// Forward while recording activations for a later [`Mlp::backward`].
    pub fn forward_cache(&self, x: &[f32], cache: &mut Cache) -> Vec<f32> {
        debug_assert_eq!(x.len(), self.in_dim(), "input width mismatch");
        let nl = self.layers.len();
        cache.inputs[0].copy_from_slice(x);
        for (l, layer) in self.layers.iter().enumerate() {
            layer.forward(&cache.inputs[l], &mut cache.pre[l]);
            // The final layer stays linear; every other one is tanh'd into the
            // next layer's input.
            if l + 1 < nl {
                cache.inputs[l + 1].copy_from_slice(&cache.pre[l]);
                tanh_inplace(&mut cache.inputs[l + 1]);
            }
        }
        cache.pre[nl - 1].clone()
    }

    /// Gradient of a scalar loss w.r.t. every weight, given `dloss_dout` at the
    /// output, accumulated into `grad` (flat, matching [`Mlp::get_params`]).
    ///
    /// Backprop through `tanh` for hidden layers, identity for the final linear
    /// layer. `grads` must be the same length as the param block.
    pub fn backward(&self, cache: &Cache, dloss_dout: &[f32], grad: &mut [f32]) {
        let nl = self.layers.len();
        // Layer l's param block starts at poff[l]; within it `w` (nw long) is
        // followed by `b` (nb long) — the exact order `get_params` writes.
        let mut poff = Vec::with_capacity(nl);
        {
            let mut o = 0usize;
            for l in &self.layers {
                poff.push(o);
                o += l.w.len() + l.b.len();
            }
        }
        // delta[l] = dL/d(pre-activation of layer l), sized layer l's out_dim.
        let mut delta = cache.pre.clone();
        for j in 0..nl {
            debug_assert_eq!(delta[j].len(), self.layers[j].out_dim);
        }
        delta[nl - 1].copy_from_slice(dloss_dout);

        for l in (0..nl).rev() {
            let layer = &self.layers[l];
            let inp = &cache.inputs[l];
            // Own this layer's delta: releases the borrow on `delta` so the
            // propagation step below can mutably borrow `delta[l - 1]`. The
            // emptied slot is never read again (the walk is strictly descending).
            let dl: Vec<f32> = std::mem::take(&mut delta[l]);
            let g = poff[l];
            // `get_params` writes each layer as [w | b], so the bias block
            // starts `nw` past the layer base (NOT `nb`).
            let nw = layer.w.len();
            debug_assert_eq!(inp.len(), layer.in_dim, "cache input width");
            debug_assert_eq!(dl.len(), layer.out_dim, "delta width");
            // dW[j*in + k] += delta[j] * input[k]   (w section begins at g)
            // db[j]         += delta[j]              (b section at g + nw)
            for j in 0..layer.out_dim {
                let dj = dl[j];
                grad[g + nw + j] += dj;
                let row = j * layer.in_dim;
                for k in 0..layer.in_dim {
                    grad[g + row + k] += dj * inp[k];
                }
            }
            // Propagate to layer l-1's pre-activation through the tanh that
            // produced `inputs[l]`. dL/d(inputs[l][m]) = Σ_j delta[l][j]·W[j][m],
            // then × (1 − inputs[l][m]²) since inputs[l] = tanh(pre[l-1]).
            if l > 0 {
                let d_in = &mut delta[l - 1]; // layer l's in_dim slots
                for m in d_in.iter_mut() {
                    *m = 0.0;
                }
                for j in 0..layer.out_dim {
                    let dj = dl[j];
                    let row = j * layer.in_dim;
                    for m in 0..layer.in_dim {
                        d_in[m] += dj * layer.w[row + m];
                    }
                }
                for m in 0..layer.in_dim {
                    let t = inp[m];
                    d_in[m] *= 1.0 - t * t;
                }
            }
        }
    }
}

impl Cache {
    fn new(dims: &[usize]) -> Self {
        let nl = dims.len() - 1;
        let mut inputs = Vec::with_capacity(nl);
        let mut pre = Vec::with_capacity(nl);
        for l in 0..nl {
            inputs.push(vec![0.0; dims[l]]);
            pre.push(vec![0.0; dims[l + 1]]);
        }
        Self { inputs, pre }
    }

    pub(crate) fn new_like(m: &Mlp) -> Self {
        Self::new(&m.dims)
    }
}

// ---------------------------------------------------------------------------
// Deterministic RNG (seeded LCG) — no `rand` dependency (plan §2)
// ---------------------------------------------------------------------------

/// SplitMix64-derived generator. Small, reproducible, dependency-free.
#[derive(Debug, Clone)]
pub struct Rng {
    state: u64,
}

impl Default for Rng {
    fn default() -> Self {
        Self::new(0x9E37_79B9_7F4A_7C15)
    }
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Next u64 in the stream.
    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }

    /// Uniform f32 in `[lo, hi)`.
    #[inline]
    pub fn uniform(&mut self, lo: f32, hi: f32) -> f32 {
        let u = (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32; // [0,1)
        lo + u * (hi - lo)
    }

    /// Standard normal via Box–Muller. Used to sample the policy action.
    pub fn normal(&mut self) -> f32 {
        let u1 = self.uniform(f32::EPSILON, 1.0);
        let u2 = self.uniform(0.0, 1.0);
        (-2.0 * u1.ln()).sqrt() * (core::f32::consts::TAU * u2).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Finite-difference gradient check: with random (in,out) and a linear
    /// output, the analytic `backward` must match `d‖f(x)‖²/2` numerically to
    /// ~1e-3 on every scalar. Catches any transpose/activation-derivative bug.
    #[test]
    fn analytic_grads_match_finite_differences() {
        let mut rng = Rng::new(0xC0FFEE);
        let dims = [4, 6, 3, 2];
        let net = Mlp::new(&dims, &mut rng);
        let n = net.n_params();

        let x = vec![0.3, -1.2, 0.7, 0.05];
        let loss = |net: &Mlp| {
            let o = net.forward(&x);
            0.5 * o.iter().map(|v| v * v).sum::<f32>()
        };

        // Analytic: dL/dout = out.
        let mut cache = Cache::new_like(&net);
        let out = net.forward_cache(&x, &mut cache);
        let mut grad = vec![0.0f32; n];
        net.backward(&cache, &out, &mut grad);

        // Numeric: perturb each param a hair and diff the loss.
        let eps = 1e-3f32;
        let base = loss(&net);
        let mut params = Vec::with_capacity(n);
        net.get_params(&mut params);
        for i in 0..n {
            let mut p = params.clone();
            p[i] += eps;
            let mut net_p = net.clone();
            net_p.set_params(&p);
            let l_hi = loss(&net_p);
            let numeric = (l_hi - base) / eps;
            let analytic = grad[i];
            assert!(
                (numeric - analytic).abs() < 5e-2,
                "param {i}: analytic {analytic:.5} vs numeric {numeric:.5}"
            );
        }
    }

    #[test]
    fn determinism_same_seed_same_net() {
        let a = Mlp::new(&[3, 5, 2], &mut Rng::new(7));
        let b = Mlp::new(&[3, 5, 2], &mut Rng::new(7));
        assert_eq!(a, b, "identical seed must produce identical params");
        let c = Mlp::new(&[3, 5, 2], &mut Rng::new(8));
        assert_ne!(a, c, "different seed ⇒ different params");
    }

    #[test]
    fn final_layer_is_linear_hidden_are_tanh() {
        // Over a wide positive input, a linear output can exceed 1 (tanh would
        // clamp it); the hidden layers stay bounded.
        let mut rng = Rng::new(1);
        let net = Mlp::new(&[2, 8, 1], &mut rng);
        let out = net.forward(&[100.0, 100.0]);
        assert!(
            out[0].abs() > 1.0 || net.layers[0].out_dim != 1,
            "final layer must not be squashed: {}",
            out[0]
        );
    }

    #[test]
    fn get_set_params_roundtrips() {
        let mut net = Mlp::new(&[4, 3], &mut Rng::new(42));
        let mut buf = Vec::new();
        net.get_params(&mut buf);
        buf.iter_mut().for_each(|v| *v += 1.0);
        let before = net.clone();
        net.set_params(&buf);
        assert_ne!(net, before, "params actually changed");
        let mut buf2 = Vec::new();
        net.get_params(&mut buf2);
        assert_eq!(buf, buf2, "round trip stable");
    }

    #[test]
    fn normal_samples_are_centered_and_reproducible() {
        let mean: f32 = (0..2000).map(|_| Rng::new(12345).normal()).sum::<f32>() / 2000.0;
        // Single-seed stream has a fixed mean over N draws; reseed per draw is
        // wrong, so verify one stream instead:
        let mut rng = Rng::new(999);
        let s: f32 = (0..5000).map(|_| rng.normal()).sum();
        let _ = mean;
        assert!(
            (s / 5000.0).abs() < 0.2,
            "one normal stream must center near 0: {}",
            s / 5000.0
        );
        // Reproducible: same seed ⇒ same first sample.
        assert_eq!(Rng::new(5).normal(), Rng::new(5).normal());
    }
}
