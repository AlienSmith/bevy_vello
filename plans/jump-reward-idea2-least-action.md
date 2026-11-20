# Jump reward, part 2 — the principle of least action (effort / smoothness)

Status: **design only, not implemented** · Area: `walker_control` (the residual
observes its own effort) + `walker_ppo` (reward terms) · Follows
[`jump-ppo-residual-polar-action.md`](jump-ppo-residual-polar-action.md) §14 and the
task-outcome reward that replaced the two-checkpoint one in `walker_ppo/src/reward.rs`.

## 0. Why this is a separate experiment

Part 1 (task-outcome) defines **what success is**: takeoff CoM velocity, a
monotone CoM apex gate, a landing window that pays for a settled capture. It is
deliberately minimal and positive-only: standing still pays exactly 0, and the
only charge is a small measured-fall penalty (0.5) in the harness.

Part 2 defines **how the motion should look**: coordinated and elastic rather
than twitchy. It is not needed to make the character hop; it is needed to make
the hop look like a hop instead of a flail. Keeping the two apart matters,
because the previous round proved that mixing "shaping" into the *evaluative*
terms is how a reward stops pointing at the goal:

```
measured over 248 updates / 7 941 episodes with the old reward:
  fall-terminated 90.1%   landed 8.8%   mean ep_len 158 of 288 ticks
  fall_pen -2.702   alive +0.792   apex_peak +0.699   settle -0.087 ...
```

The one big negative term was attached to a *state* (grounded + leaned) rather
than to the *outcome*, and the policy optimised it: it learned not to launch.

## 1. Correction: the effort numbers ARE computable here

An earlier note in this repo claimed torque/work terms need a solver read-back.
That is too strong. The controller is handed **every particle of the body**
(`ParticleSample { pos, vel, inv_mass }`, mass-weighted into
[`observer`](../walker_control/src/observe.rs) → `Fiction./RawReality`), and the
actuator is a compliant position constraint whose commanded target and compliance
are both known to the controller. So:

- **joint rates** follow from the same positions/velocities the solver integrated;
- **the force the solver applied** follows from `(target − measured) × k_eff`, the
  compliance law the actuator already implements in
  [`actuate.rs`](../walker_control/src/actuate.rs) / [`walker.rs`](../walker_control/src/walker.rs);

which is exactly the "same math calculation from the same values" argument. What
we do **not** have is a direct, authoritative force read-out from inside the XPBD
iterations — so every term below is a *reconstruction*, not a measurement. That
distinction only matters for terms where the solver's split between constraint
correction and integration would differ from our reconstruction. For a PD/compliance
actuator it does not: the correction it applies over a tick **is** `k_eff·e + c_eff·ė`.

One genuine gap: the observer samples `hip`, `waist`, `foot_l`, `foot_r`. The
**knee** particle exists (`LegLive { waist, hip, knee, foot }` in
[`actuate.rs`](../walker_control/src/actuate.rs)) but is not in `RealitySample`.
Shin rates need it, so step 0 is one more entity handle in
[`observe.rs`](../walker_control/src/observe.rs).

## 2. The terms

Notation: the actuator has 5 DOF — two strut lengths `r_i`, two foot directions
`s_i`, one spine lean `s_θ` (see §1 of the parent plan). `a_t ∈ R⁵` is the
normalized policy action, `δ` the residual it added on top of the expert proposal,
`Pose` the reconstructed world pose.

### 2.1 Residual effort — `‖δ‖²` (do this first)

```rust
effort.residual2 = delta.iter().map(|d| d * d).sum::<f32>();
```

This is the one term that matters most, and it is specific to a **residual**
controller:

- it is the exact statement of "least action relative to the prior" — deviate from
  the expert only where the deviation buys reward;
- it closes the hole that produced the "swing both legs back / squash" behaviour:
  **suppressing the expert's stroke stops being free**. In the old run the only way
  the policy could control "do I leave the ground and risk −3?" was to destroy the
  crouch→extend pose, because at extension `prop.r ≈ 1.0` and the action clamp
  leaves `r` no room above the proposal
  ([`reconstruct`](../walker_control/src/jump_residual.rs));
- no physics reconstruction needed: it is the policy's own output.

Caveat: it must be the **residual**, never the pose. A penalty on pose velocity
would penalise the jump.

### 2.2 Action smoothness — first and second difference

```rust
effort.jerk1 = ‖a_t − a_{t−1}‖²;
effort.jerk2 = ‖(a_t − a_{t−1}) − (a_{t−1} − a_{t−2})‖²;   // optional, damps "buzz"
```

The existing `w_act` was an L1 version of `jerk1`, but it was clamped by
`pen_cap = 0.002` per tick and contributed ≈ `−0.31` over a *whole episode* —
numerically dead. Use an L2 penalty with its own small weight and no shared cap.
This is the cheapest instrument against the high-frequency flail that exploration
noise (`INIT_STD 0.3` per dim) injects into a 90 Hz controller.

### 2.3 Pose-tracking error — the force proxy

The compliance law makes the applied correction proportional to the tracking
error, so the error *is* the effort up to stiffness:

```rust
// per actuator DOF, measured on the same tick:
e_i = commanded_i − measured_i;             // strut length, foot direction, spine angle
τ_i ≈ k_eff_i · e_i;                        // k_eff from ControlConfig::compliance
effort.torque2  = Σ τ_i²;                   // "spread the force over the joints"
effort.power    = Σ |τ_i · q̇_i| · dt;       // mechanical work, signed magnitude
```

Inputs the controller already has:

- `commanded_i`: the `Pose` it emitted (`pose5` from the clip sample, then
  `reconstruct`), i.e. `r_i·leg_len`, `asin(s_i)`, `asin(s_θ)`;
- `measured_i`: `Fiction.feet[i].pos`, `Fiction.hip`, `Fiction.lean` — already
  polar-projected in `JumpResidual::build_obs`;
- `k_eff_i`: derived from the joint compliance the actuator configures
  (`set_joint_compliance` tooling exists under `tools/physics-qa/`).

### 2.4 Joint rates `q̇`

For a 2D segment `A → B` with particle velocities `v_A`, `v_B`:

```text
q̇ = cross(B − A, v_B − v_A) / |B − A|²          // rad/s, z-component of the cross
```

Applied to `hip→knee` (thigh) and `knee→foot` (shin), and to `hip→waist` (spine,
which `Fiction.lean_rate` already gives us). These feed the `power` term; on their
own they are **not** a penalty target — a jump has large joint rates by
definition.

## 3. Combined reward (part 2 added to part 1)

Part 1 as shipped (`walker_ppo/src/reward.rs`):

```text
R_1 = R_takeoff + R_air + R_hold + R_verdict (+ R_stand)
      quality  = apex_ratio · (1 − |lean|/lean_fail)     // NO velocity gate
      R_hold   = w_success · hold_share · (dt/land_window_ts) · quality
      R_stand  = w_stand · apex_ratio · upright          // episode ended, not fallen
      crash:   −(w_fall + everything R_1 banked)          // claw-back, nets −w_fall
```

`R_hold` pays 5 % of the verdict across the whole 1.5 s window regardless of tick
rate — credit assignment only. `R_stand` is the lenient terminal: the hold need not
have finished, the body only has to still be on its feet. Both scale with the apex,
so "stand still" is still exactly 0.

Measured calibration at `N=32`, 128 episodes, expert-initialised policy:
`mean_ep_reward +0.16` (max +2.77) instead of a flat `−0.5`, `fell 78 %`,
`stand-paid 21 %`, `quality > 0` on 42 % of episodes, `v_up` mean 377 px/s.

Part 2 adds:

```text
R = R_1
  − w_res    · ‖δ‖²
  − w_jerk   · ‖a_t − a_{t−1}‖²
  − w_torque · Σ τ_i²
  − w_power  · Σ |τ_i · q̇_i| · dt
```

with the part 1 ordering preserved — i.e. every effort weight is small enough that

```text
clean hop (~+4)  >  stand still (0)  >  crash (−0.5)  >  jittering in place (−ε)
```

is still true. Two consequences worth stating explicitly:

- The dense effort terms also land in the **claw-back**: a fallen episode nets
  `−w_fall` whatever it did, so effort can never buy back a crash.
- With part 2, "stand still" is no longer exactly 0 — it pays the effort of
  whatever the expert still does, and of the policy's own noise. That is
  desirable: it makes idling cost *something* the moment the policy jitters.

## 4. Normalization (do this before picking weights)

Every effort quantity scales with the rig, so express each in units of a measured
reference rather than raw px²:

| term | normalizer | how to measure |
|---|---|---|
| `‖δ‖²` | mean `‖δ‖²` under pure exploration at `INIT_STD` | one `EXPERT_ONLY` episode, log it |
| `‖Δa‖²` | mean per-tick `‖Δa‖²` | same |
| `Στ²` | `(k_eff·leg_len)²` | config, analytic |
| `Σ|τ·q̇|dt` | `(k_eff·leg_len)·(π/leg_len)·dt` | config, analytic |

A clean hop should pay ≈ **1.0** of total effort cost against a `w_success` of
3.0, i.e. effort never decides *whether* to jump, only *how*. Pick the weights from
the logged means, not by guessing.

## 5. Where it plugs in

- [`jump_residual.rs`](../walker_control/src/jump_residual.rs) `step` — it already
  holds `prop`, `action`, `fiction`, and the reconstructed `pose`. Compute the
  effort there and report it on `LastStep` (extend it with `effort: Effort`), so
  the harness (Law 2: controller reports, harness collects) prices it. No obs
  change and no action-dim change: the `22 / 5` bundle contract is untouched.
- [`reward.rs`](../walker_ppo/src/reward.rs) — add the four weights and subtract
  them in the per-tick scalar. Keep them in their own struct (`EffortConfig`) so
  part 2 can be switched off wholesale with one field.
- [`observe.rs`](../walker_control/src/observe.rs) — add the knee particle to
  `RealitySample` for `q̇_shin` (step 0; everything else is already there).
- `episodes.csv` — one column per effort term, so the first run tells us which one
  is actually doing work.

## 6. Staging

1. **`‖δ‖²` only**, with a small weight. This is the behavioural fix (no more
   stroke suppression) and it needs no new instrumentation at all.
2. **`‖Δa‖²`**, replacing the dead `w_act`. Damped, smooth exploration.
3. **`Στ²` / `Σ|τ·q̇|`** once the knee is sampled and the compliance constants are
   pulled out of the actuator config. Compare "error-based τ" against
   "command-delta-based τ" on one `EXPERT_ONLY` episode before committing to either.

Do not stage 1 and 3 together: they are the same currency (effort) measured two
ways, and if both move at once we will not know which one shaped the gait.

## 7. Open questions

- **`q̇` from particles vs from the IK solution.** The knee particle is integrated
  by the solver; `actuate`'s `live_leg` reads it. Using the particle is "the same
  values the solver had"; using the IK solution is the commanded chain. They differ
  by the tracking error we are trying to price, so use the particle.
- **Should `power` be signed?** `Σ|τ·q̇|` prices all motion; `Σ τ·q̇` prices
  *resisting* muscle work and pays back elastic return. For a hop, the second is
  more physical but noisier over a 1/90 s tick. Start with the absolute value.
- **Does effort fight the impulse?** The expert's launch impulse is applied to the
  particles directly ([`walker.rs`](../walker_control/src/walker.rs)), not through
  the actuator, so it is invisible to every term above. That is the desired
  behaviour: the launch stays free, the shape costs.
