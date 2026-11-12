# PPO-learned walker: hand-rolled MLP + PPO behind the controller seam

Goal: replace the *authored* gait for a **learned** walking controller — a
two-layer MLP trained with PPO against the real softbody physics in the Bevy
app — without a NN framework (glam + plain `Vec<f32>`), wrapped behind the
existing `Controller`/`Step` seam so the trained weights deploy as just
another controller. The pipeline is move-agnostic: walk now, jump/sidestep
later, same trainer.

## 1. Why this shape

- The collider is a **softbody** and contacts come from the GPU narrow phase
  (`run_gpu_collision` → `CollisionEventBatch`), so the env must be the real
  app (hidden window, `WIN_HIDDEN=1` + Xvfb like `tools/physics-qa`). No
  decoupled physics sandbox — accuracy and speed come from training *in* the
  game physics.
- `walker_control` already has the exact env contract for one agent:
  `Fiction` in → `ControlOut`/`Pose` out → `actuate_stage` queues XPBD
  constraints. A `LocoMode::Learned` controller is therefore a *drop-in*
  policy executor; the training harness is the same chain with N characters.
- Input/output spaces are tiny (see §3), so a 2-layer MLP with hand-written
  backprop is ~300 lines and trains fine on CPU. PPO is the standard for
  locomotion (`legged_gym` idiom) and tolerates our small on-policy budget.

## 2. Crate / module layout

**`walker_nn/` — new standalone crate** (pure Rust, no bevy/ECS, mirrors the
`walker_math` precedent: an algorithm core the bridge consumes). This is the
"NN code" isolated so other moves/pipelines reuse it:

- `walker_nn/src/mlp.rs` — the network:
  - `struct Mlp { layers: Vec<Layer> }` (configurable hidden layers, tanh;
    linear output heads). Weights as flat row-major `Vec<f32>` + dims;
    `forward`, `backward` (returns param grads, Adam moments live beside the
    params), `serde` (de)serialization to JSON. Glorot init from a seeded
    in-crate LCG (deterministic, no `rand` dep).
- `walker_nn/src/adam.rs` — Adam optimizer over a flat `Vec<f32>` param block.
- `walker_nn/src/ppo.rs` — the trainer, pure Rust, no ECS:
  - `GaussianPolicy`: actor head μ(s) + per-dim `log_std` (state-independent),
    critic head V(s) — both built on `Mlp`.
  - `Rollout`: ring of (obs, action, logp, reward, done, value).
  - GAE(λ) advantages/returns, clipped-surrogate actor loss + MSE critic loss
    + entropy bonus, K epochs over minibatches. One `PpoTrainer` owns the
    shared `Mlp` + Adam; N env characters share it (standard multi-env PPO).
- `walker_control` gains `walker_nn = { path = "../walker_nn" }` as a
  dependency and only *consumes* it: the controller seam holds a `LearnedCtrl`.
- `walker_control/src/control.rs` — new `LearnedCtrl` implementing `Step`
  (see §4) + `LocoMode::Learned` variant in the existing enum/dispatch/FSM.
- `walker_ppo/` — **new workspace bin crate** (mirrors `walker_qa`): the
  training app. Spawns the floor + N characters, attaches walkers in training
  mode, runs the rollout → update loop, CSV/log telemetry, saves weights.
  `walker_qa` gains a `POLICY=path.json` env to deploy frozen weights.

Keep `nn`/`ppo` inside `walker_control`: no new dependencies, and the deploy
path (frozen weights as a controller) lives next to the contracts it
implements.

## 3. Observation / action (the "small spaces")

All quantities in the vello frame, hip-relative, normalized by rig constants
(`LowerBodyConfig` bone lengths / `hip_height`) so the same weights transfer
across moves:

```
obs (16 dims):
  0    v_err      (cmd.vx - v_com.x) / v_scale          # tracking error
  1    v_y        v_hip.y / v_scale
  2    lean       θ (fiction.lean)
  3    lean_rate  ω / ω_scale
  4-7  foot_off   (feet[i].pos - hip) / leg_len         # x,y per foot
  8-9  foot_vx    feet[i].vel.x / v_scale
  10-11 touching  feet[i].touching -> 1/0
  12   grounded   1/0
  13   height     (floor_y - hip.y) / hip_height
  14-15 cmd_dir    dir (x,y) of the held Command        # generic move intent
```

## 0. First milestone = a straight vertical JUMP (not the walk)

To de-risk the whole stack we first learn a **jump-up-straight**, not a gait.
A jump exercises every piece of the pipeline — `Fiction`→obs, action→`Pose`,
teleport reset, multi-env rollout, GAE, the clipped update, the weight
snapshot — with **none** of the variables that make walking hard: no gait
cadence, no DCM/step placement, no left/right foot alternation, no
speed-tracking reward. The reward is scalar "how high did the hip go." If the
policy can jump, the seam is proven; the walk is then the same harness with a
tracking reward and (later) a phase-1 boost→gait handover. The controller in
`walker_control::learned` is deliberately **move-agnostic** — a jump and a walk
share one obs/action contract; only the harness-side reward + `Command`
interpretation differ.

Jump mechanism in this actuator (what we expect to discover): the hip is not
directly commandable (`pose.hip` rides the measured hip — translation
invariant). Height comes from the **legs**: command a short hip→foot chord with
the feet planted (stance) = crouch/load; then a long straight chord against the
pinned floor = extension that XPBD converts into hip rise. The policy finds
that crouch→extend timing purely from the height reward. If compliance/strut
can't convert an extension into lift, that's a real finding and we retune the
actuator before blaming PPO.

## 3b. (unchanged) Observation / action
**The action IS the controller's fixed output contract — [`crate::actuate::Pose`].**
No bespoke action space: the MLP emits the pose fields directly. `hip` is not
learned (rides the measured hip — translation-invariant, the `StandingCtrl`
trick) and `facing` is the spawn constant, so the *learned* outputs are the
remaining `Pose` elements:

```
action (6 dims, Gaussian, mapped straight onto Pose):
  0-1  left_foot.pos   hip-relative offset (x,y), scaled to leg_len,
                       clamped into the reach envelope (chord ≤ thigh+calf)
  2-3  right_foot.pos  same
  4    upper_angle     clamped to ±θ_max
  5    left_foot.stance   (continuous: >0.5 ⇒ strut active — the DCM
                       pipeline's stance gate is already a boolean; learning
                       lift/hold timing is the point)
  6    right_foot.stance
```

`facing` mirrors x like the other controllers; `hip` is the measured hip
(translation-invariant pose, same trick as `StandingCtrl`). No DCM/ξ anywhere
in this controller — the gait emerges from reward.

## 4. `LearnedCtrl` — one contract, two modes

```rust
pub struct LearnedCtrl {
    policy: Arc<Mutex<SharedPolicy>>, // or Resource handle passed in step
    mode: LearnMode,                  // Train | Deploy
    rng u64,                          // deterministic sampling (seeded)
    last_obs: Vec<f32>, last_action, last_logp   // for the rollout
}
```

- `step(cmd, fiction, cfg, dt, facing) -> ControlOut`: build obs from
  `Fiction` + `Command`, run the MLP forward → μ (+σ) → action; map to `Pose`.
  - `Train`: sample `a ~ N(μ,σ)`; record `(obs, a, logp)` into the episode
    buffer via a callback the harness injects (`&mut dyn RolloutSink`) so
    `walker_control` stays ECS-free (Law 2 spirit: the controller reports, the
    harness collects).
  - `Deploy`: deterministic `a = μ`.
- The harness computes `reward` + `done` after `observe_stage` of the *next*
  tick (needs the realized result of the action), and pushes the finished
  transition. Termination: `|lean| > θ_fall` or hip below floor margin or
  grounded-fraction collapse → `done=true` → harness resets the character
  (see §6) and bootstraps the critic.

FSM wiring: `WalkerConfig.control.use_learned: bool` — when set,
`LocoState::decide` maps `Walking → Learned` (BoostStart stays authored or is
retired for the learned path; standing stays `StandingCtrl`). Zero new
schedules; `control_stage` dispatches through the enum unchanged.

## 5. PPO hyperparameters (starting point, env-overridable)

```
gamma 0.995   # 90 Hz ticks
gae_lambda 0.95
clip_eps 0.2
ent_coef 0.003
vf_coef 0.5
lr 3e-4 Adam, epochs 8, minibatches 16
rollout_len 360 ticks/char (~4 s), N chars 16..64
total: ~5-10 M ticks target for walking
```

Update runs synchronously on the main thread every rollout: 16×360×small MLP
× 8 epochs is well under the tick budget's concern (training ticks don't need
real-time). Optional: chunked (update over `U` app-frames) if a windowed
render can't stall.

## 6. The training app (`walker_ppo` bin)

- Scene = `walker_qa` minus camera-follow: `MinimalPlugins`-style but with the
  full `VelloPlugin` + `VelloCollisionResponsePlugin` (GPU contacts are the
  env), `WIN_HIDDEN=1` under Xvfb (existing `tools/physics-qa` tooling),
  `Time<Fixed>::from_hz(90)`.
- Long floor; N characters spawned at spaced x-offsets, all `collision_group`
  distinct-from-floor (same as QA). Each gets a `Walker` (shared
  `PpoTrainer` Resource handle via a sidecar component, since `Walker` must
  stay config-cloneable).
- Command source: a `RandomWalkCommand` system replaces `script_system` —
  each episode samples a held `speed ∈ [40, 160]`, `dir ∈ {±1}`, occasionally
  0 (stand) so the policy learns to stop; later this is where jump/other
  move-intent cells plug in.
- Reset: **teleport first** — write every character particle back to its
  blueprint rest offsets under the spawn hip and zero velocities (XPBD is
  position-based; `update_collider_from_soft_body` re-syncs colliders).
  Fallback if teleport shows state residue: despawn root → `remove_soft_body`
  chain → respawn → re-`AttachWalker` (handles churn, slower).
- Telemetry: reuse `WalkerTelemetryBuffer` rows (mode column already records
  controllers) + one trainer log line per update: mean reward, v_err, episode
  length, approx-KL, clip fraction → CSV for `tools/physics-qa` scripts.

## 7. Persistence & deploy

- `SharedPolicy.save_json(path)` / `load_json` (serde, already a dep).
  Checkpoints every `UPDATE_EVERY` rollouts to `tmp/walker_ppo/policy.json`.
- `walker_qa`: `POLICY=tmp/walker_ppo/policy.json` → build
  `LearnedCtrl{mode: Deploy}` and set `use_learned`; existing QA CSV plots
  (`plot_p3.py`, `walk_bands.py`) score the learned gait unchanged. A/B: same
  script cells (`walk_in_120.json`) against the DCM baseline.

## 8. Testing ladder (no big-bang)

1. `nn.rs` unit: finite-difference gradient check on a 2-layer net; determinism.
2. `ppo.rs` unit: GAE on a hand-computed 5-step trajectory; clip loss values;
   a trivial bandit-like regression smoke (1-dim obs, known optimum) →
   converges in-process.
3. `control.rs` unit: `LearnedCtrl::step` emits finite, reach-enforced poses
   from a fixed random-weight net (same harness as existing mode tests).
4. `walker_ppo` smoke: 2 chars × 300 ticks, `PPO_SMOKE=1` exits early, writes
   CSV + policy.json; asserts episode-reward improves over baseline-noise
   (loose bound).
5. Real training: 16 chars, walk reward, watch `walk.csv` bands in QA tools.

## 9. Risks / open questions

- **Sim2sim gap = zero** (same engine) but **contact latency**: observe reads
  last tick's batch (1-tick lag by design) — fold that into the env contract;
  the policy sees delayed touches, deploy path has the same delay, so it is
  consistent, just non-perfect-state (fine; add a tiny history later if
  needed).
- Reward shaping is the real work; start velocity-tracking + alive + posture,
  add foot-slip/torque penalties once gait emerges (legged_gym recipe).
- Blocking updates stall the *render* every rollout; acceptable under
  `WIN_HIDDEN` training. Chunked mode later.
- `use_learned` + FSM: standing handover (Learned → StandingCtrl at cmd end)
  must not topple the body; keep the policy trained on stop commands so it
  reaches a quiet stance before `decide` exits to `Standing`.

## 10. Build order

1. `nn.rs` + gradient check  →  2. `ppo.rs` + units  →  3. `LearnedCtrl` +
`LocoMode::Learned` + control tests  →  4. `walker_ppo` bin (spawn, teleport
reset, random commands, rollout sink)  →  5. checkpoint/save + `walker_qa`
`POLICY=` deploy  →  6. smoke run, then real training.
