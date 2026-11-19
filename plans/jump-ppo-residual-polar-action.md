# Jump: PPO residual on the authored expert, in a polar action space

Status: design agreed, open items below · Area: `walker_control` (new `JumpResidual`
controller) + `walker_ppo` (harness) · Builds on [`ppo-learned-walker.md`](ppo-learned-walker.md)
and the committed expert [`walker_control/src/jump.rs`](../walker_control/src/jump.rs)
(`60888f7`, v1 `JumpCtrl`).

## 0. Why we are here

The hand-rolled `JumpCtrl` crouch→extend stroke lands a hop but every variant
so far **falls over** (measured: v1..v10, `th_launch` improved 0.31→0.11 but θ
still ran to −1.5 through the arc). Two structural gaps explain it:

1. **No foot-to-ground anchor.** The stance "pin" is only the hip↔foot distance
   strut; the foot's only link to the floor is finite friction. At launch the
   feet slide forward ~45 px every time, the support line ends up ahead of the
   CoM, and the vertical push becomes backflip torque. Locking the feet (v9)
   made it worse (stale commands fought the sliding body). This is a
   torque/CoM/friction **dynamic**, not a parameterization bug.
2. **No in-air angular authority.** Legs are far too light vs the torso to arrest
   launch spin; any residual tilt at liftoff is permanent.

The authored open-loop stroke cannot close these — a feedback jump needs the
"new algorithm to not fall" the user called out. So: **keep `JumpCtrl` as the
expert PROPOSAL, train a PPO RESIDUAL on top of it** whose job is the
fine-grained anti-fall correction. The residual starts at zero = executes the
expert exactly, so PPO polishes a near-working hop instead of discovering one
from noise.

## 1. The action space (the decision this doc locks)

We rejected the current Cartesian foot-position action
([`learned.rs::action_to_pose`](../walker_control/src/learned.rs) — `x`, `depth`
per foot, reach-enforced by a hard **clamp**) in favour of a **polar** action in
the controller's own native frame. Three user refinements, each adopted:

- **Angles beat positions** — a foot *position* can be unreachable (the current
  `depth_max: 1.11` even asks for 122 px on a 110 leg and clamps it). A clamp is
  **non-invertible at the wall**: PPO samples `a`, records `logπ(a)`, but the
  world executes `clamp(a) ≠ a`, so the gradient points at a pose that was never
  taken. Feasible-by-construction removes that.
- **Per-leg `(distance, sin φ)`** — express the foot as **chord length** `r` and
  **direction** `s = sin φ`, the angle of the hip→foot line off straight-down
  (`+y`, vello y-down). `r ∈ [0,1]`·leg_len makes the reach an **identity** (no
  clamp). `sin` (not the raw angle) is what the horizontal term wants
  linearly — see §2.
- **Same for the torso** — `s_θ = sin θ`, θ = the angle of the hip→upper-body
  line off `-y` (straight up). The actuator *already* uses this frame:
  [`leg_commands`](../walker_control/src/actuate.rs) builds the spine reference
  `a_t = hip + (len·sin_lean, −len·cos_lean)`. `±60°` ⇒ `s_θ ∈ ±0.866`. One
  `asin` reconstructs the angle the interface wants; `sin` is monotone there so
  it's unambiguous. No reason to teach the net two different angle formats.

**Locked learned action = 5 dims (travel frame, `facing` folded once in FK):**

```
i ∈ {L,R}   r_i  ∈ [0,1]     chord / leg_len    (the strut length / power stroke)
            s_i  ∈ [−1,1]    sin φ_i,  φ off +y (fore-aft foot placement = CoM arm)
torso       s_θ  ∈ [−1,1]    sin θ,   θ off −y  (lean; ±60° range)
```

## 2. Reconstruction (polar → world `Pose`) — no trig on the legs

```
leg  (per side):  chord  = r · leg_len
                  foot.x = facing · chord · warp(s)          // horizontal (CoM-arm lever)
                  foot.y =         chord · sqrt(1 − warp(s)²) // below hip (vello y-down)
                  FootPose.pos = hip + (foot.x, foot.y)
                  FootPose.stance = authored (see §3)

torso:            θ = asin(clamp(s_θ, −1, 1))                // range ±60° ⇒ monotone
                  pose.upper_angle = facing · θ              // → a_t at actuate.rs:285
```

- **Knee / bend is NOT a policy output.** The existing IK +
  [`select_anterior_knee(..., facing)`](../walker_control/src/actuate.rs) picks
  the winding from `facing` exactly as it does for the position action today.
  We feed the reconstructed `foot.pos` into the *unchanged* `Pose`/`actuate`
  chain — **zero actuator surgery**. Same trick the plan's §2 promised
  ("a learned gait drops into the existing chain with no new actuation code").
- **Reach is structural:** `r ≤ 1` ⇒ chord ≤ leg_len ⇒ every pose is reachable.
  No clamp ⇒ `logπ(a)` matches the executed pose (PPO-purity, the point of §1).
- **`warp(·)` — nonlinear remap hook (reserved, default identity).** The only
  imperfection: `d(√(1−s²))/ds = −s/√(1−s²)` compresses the vertical term near
  the `|s|=0.87` edge (mild `s→vertical` cross-talk). Harmless for a *vertical*
  jump (`s≈0`, term is flat). If a future track leans hard, `warp` flattens the
  slope over the range — one FK line, no retraining of the shape. We ship the
  hook, not complexity.

## 3. Residual-on-expert wiring

- **Expert proposal `r(t)/s(t)/θ(t)`** comes straight from `JumpCtrl`'s
  schedule. Its chord-height ramp maps *directly* to `r`: `r_prop = h(t)/leg_len`
  (crouch `h≈62 → r≈0.56`, extend `h≈110 → r≈1.0`); `s_prop ≈ 0` (feet under
  hip); `θ_prop = 0` (upright, the expert holds attitude).
- **Zero-init policy** (`μ ≡ 0`) ⇒ residual `δ ≡ 0` ⇒ tick 0 executes the expert
  exactly. PPO adds `δ` on the 5 dims: `final = clamp(proposal + scale·δ)`.
  This is why the exploration-difficulty worry (position vs angle) is moot:
  we start *on* the near-optimal trajectory and correct locally.
- **Stance: authored from the expert's phase**, `stance = phase != Flight`
  (`JumpCtrl` already computes it). Consistent with §1's philosophy — code
  already guarantees ground-stiff/air-soft, so the net shouldn't rediscover it.
  (If we later want the policy to *experiment* with lift timing, add 2 learned
  gates → 7 dims; see open item B.)

## 4. What changes in code (scope)

- **New** `walker_control/src/jump_residual.rs` — `JumpResidual` controller
  implementing `Step`: holds a `JumpCtrl` (the proposal) + a `LearnedCtrl`-style
  policy handle; each tick builds obs, samples `δ`, does `proposal + δ`, runs
  §2 FK, emits `ControlOut{ mode: Jumping }`. Reuses `LearnedCtrl`'s `LastStep`
  reporting contract (Law 2: controller reports, harness collects).
- **Obs**: **action–observation symmetry** (see §7) — present the *measured*
  feet in the SAME `(r, s)` frame as the action, replacing `build_obs`'s four
  Cartesian foot channels, and **append the expert proposal** (`r_prop,
  s_prop_L, s_prop_R, θ_prop, phase`) so the policy sees the baseline it
  corrects against.
- **`LocoMode::Jumping`** stays; `Controller::Jumping` swaps `JumpCtrl` for
  `JumpResidual` when `use_learned && residual_jump` (a new `ControlConfig`
  flag; **default off ⇒ `use_learned=false` walks/deploy and the pure `JumpCtrl`
  expert path are bit-for-bit untouched**).
- **`walker_ppo`** harness: `ppo_command_system` already holds a `Command::jump`;
  `ppo_collect_system` reward is reused (alive + gated climb + peak bonus −
  penalties). `JumpResidual::take_last` feeds the rollout like `LearnedCtrl`.

## 5. Files read for this design (line anchors)

- [`walker_control/src/learned.rs`](../walker_control/src/learned.rs):
  `action_to_pose` `262`–`307` (the clamp we remove), `build_obs` `221`–`258`,
  `depth_max: 1.11` `120`.
- [`walker_control/src/actuate.rs`](../walker_control/src/actuate.rs):
  `leg_commands` `241`–`294` (strut rest = `|hip−foot|` `252`; spine `a_t`
  from `sin/cos_lean` `284`–`285`; knee via
  `select_anterior_knee(...facing)` `231`).
- [`walker_control/src/jump.rs`](../walker_control/src/jump.rs): `JumpPhase`
  `59`, chord rest/crouch consts `46`–`54`, `stance = phase != Flight` `169`,
  ride-along anchor `209`–`221`.
- [`walker_math/src/types.rs`](../walker_math/src/types.rs): `thigh_len`/`calf_len`
  `16`–`19`, `facing` knee-winding rule `118`–`126`.

## 6. Open items — two, to discuss before implementing

- **A. `leg_len` / bone source in FK.** The controller only knows config
  `leg_len = 110` ([`learned.rs:106`](../walker_control/src/learned.rs)); the
  *live* stretched chord is read by `actuate` from the knee particle. A small
  `r` (deep crouch) is fine; the question is whether to normalize `r` by config
  `leg_len` (simple, tiny mismatch the strut absorbs — current position path
  does the same) or expose the live lengths into `Fiction`. Recommend config.
- **B. stance authored (5 dims) vs learned gates (7 dims).** §3 defaults to
  authored (ground=stiff, flight=soft from `JumpCtrl.phase`). Learn it only if
  the residual should *change lift timing* (e.g. tuck earlier). Recommend
  authored for the first train; flip to 7 dims if apex is capped by strut hold.

## 7. Action–observation symmetry (locked)

The policy outputs the feet/torso as `(r, s=sin φ)`. It must **see** them the
same way. Asymmetry (output polar, input Cartesian `(x, y)`) forces the net to
learn a coordinate rotation for free — wasted capacity on exactly the axis the
jump pivots on. So:

```
per leg:  chord    = |foot − hip|                       (live, measured)
          r_obs    = chord / leg_len
          s_obs    = ((foot − hip).x / chord) / facing  // travel frame ⇒ matches action
          (cos hidden: foot is always BELOW hip ⇒ vertical = +√(1−s²), no sign)
torso:    lean_obs = sin(fiction.lean)                  // matches s_θ (replaces raw lean)
```

**Two notes:**

1. **The travel-frame fold is load-bearing.** The action's `facing` is applied
   *forward* in the FK (§2: `foot.x = facing·chord·s`), so the obs must apply it
   *backward* (`s_obs = (Δx/chord)/facing`) or a `-x` character reads its `s`
   sign flipped vs what it commands. Fold consistently or the symmetry inverts.
2. **Achieved-vs-proposed closes the history gap for free.** `r_obs/s_obs` are
   **measured** (live foot), so they are the pose the compliance solver actually
   *reached* — not what was commanded. With the proposal also in obs (§1 note),
   the net sees `commanded (proposal+δ) → achieved` in the same channels ⇒ it
   reads its own action lag directly. That is what the RNN would have supplied
   (§ prior discussion), without a recurrent state: the state is *in* the
   measurement. We therefore do **not** add the previous-action channels — the
   measured polar already carries the achieved signal, cleaner than `prev_μ`.

**Concrete obs layout (~18 dims), built on `build_obs`'s 14:**

```
 0 v_com.y/v_scale   1 v_com.x/v_scale   2 sin(lean) [symmetry, was raw lean]
 3 lean_rate/omega_scale                 4 height (floor−hip)/hip_height
 5 r_L   6 s_L   7 r_R   8 s_R           (measured polar feet; s=(Δx/chord)/facing)
 9 l_touch  10 r_touch  11 grounded [new]      12 dir.y   13 speed/v_scale
14 r_prop  15 s_prop  16 sin(θ_prop) [new: expert proposal, §3]
17 support_offset = (com.x − support_x)/leg_len [new: the force-arm, §8]
```

Drops vs today: 4 Cartesian foot → 2 polar pairs; raw `lean` → `sin lean`;
`dir.x` (a vertical jump has none). Adds: `grounded`, the 3 proposal channels,
`support_offset`. `obs_dim` stays a `LearnedConfig` knob — the proposal (14–16)
and `support_offset` (17) can be pruned to a minimal residual with a one-line
change.

## 8. The `support_offset` feature (why it's in obs)

Every failed v1–v10 jump measured the *same* root cause: the push-support line
(feet) sitting **ahead of** the CoM at launch, so the vertical ground force
becomes backflip torque (arm ~45–55 px, θ→−1.5). It is already computable from
`fiction.com.x` and the touching-feet midpoint (the lean-trim block in
[`jump.rs`](../walker_control/src/jump.rs)). Surfacing it directly
(`support_x = mean x of touching feet`) is the highest-leverage anti-fall input:
cheaper than letting the net re-derive the arm from raw coords, and exactly the
quantity the residual must learn to drive toward ~0 at liftoff.

## 9. Start-state / spawn→controller timing (decided: no change)

A known issue is that the scene starts physics at spawn but a controller is
attached several frames later. Checked against the **training** app:

- The PPO app **re-pauses physics at Startup**
  ([`main.rs:373`](../walker_ppo/src/main.rs) `…pause_physics`) and only
  un-pauses **once all walkers are attached**
  ([`main.rs:668`](../walker_ppo/src/main.rs) `collision.paused = false`). So
  the solver and the controller switch on together — no uncontrolled pre-attach
  frames in training.
- Each episode starts **parked + settle-reset** to the canonical blueprint rest
  (`particle_init.pos`, velocities zeroed) via
  [`ppo_reset_system`](../walker_ppo/src/main.rs:823), then collection opens
  with a **fresh observer** ([`main.rs:838`](../walker_ppo/src/main.rs)) so the
  teleport discontinuity cannot smear tick 0. That *is* "stand, stabilize, then
  train."

**Decision:** keep the canonical deterministic start — **no start randomization
now**. It is already correct, preserves the bit-for-bit reproducibility
contract, and gives the critic a consistent `V(s₀)`. The jump's real robustness
pressure is the **takeoff/landing** dynamics, not the initial pose (we always
begin standing with feet planted), so domain-randomizing the start would slow
the first reward climb for no on-task benefit. If robustness to off-rest starts
is wanted later, add a **seeded, opt-in** small jitter (lean/foot-offset) as a
separate experiment — it must be seeded per-episode to keep determinism.

## 10. Rotational CoM state — the missing anti-fall channel (user detail)

**Gap:** obs has the *linear* CoM motion (`v_com`, `observe.rs:358`) but no
*rotational* state about the CoM. `lean`/`lean_rate`
([`observe.rs:333-344`](../walker_control/src/observe.rs:333)) are the **hip→waist
stick** pitch and its rate — NOT whole-body rotation about the CoM. In flight
they diverge: the torso counter-rotates against the legs, stick ω≈0 while the
body keeps flipping, so the policy can game the lean signal while still toppling.
This is the exact failure we measured but could not see.

**Evidence (v10 in-place):** `th_launch = −0.113` (stick barely leaned at
liftoff) yet `θ → −1.5` mid-arc. The body already carried spin at launch; only
a CoM-frame rotational channel would have shown it.

**Wording note — we observe momentum, not torque.** `τ = dL/dt` needs foot
*force magnitudes*; the solver only exposes `touching: bool`
([`observe.rs:90`](../walker_control/src/observe.rs:90)) — no force sensor. So
the actionable, observable quantity is **angular momentum about the CoM `L`**,
whose *change* is the torque we can't measure. In flight `L` is conserved ⇒
null it at liftoff ⇒ the spin is prevented by construction. This is the
rotational twin of "v_com → 0".

**New channels (computed in `observe.rs`, mass-weighted, same loop as
`com_velocity`, EMA'd with `cfg.tau`, first tick snaps — deterministic):**

```
r_i   = pos_i − com ;  v_i' = vel_i − v_com ;  m_i = 1/inv_mass
L     = Σ m_i (r_i.x·v_i'.y − r_i.y·v_i'.x) / Σ m_i   // ang. momentum / unit mass
I     = Σ m_i |r_i|²                                   // inertia about CoM
ω_com = L / I                                          // body spin rate (analog to v_com)
```

Add both `ω_com` (the interpretable "how fast am I rotating about balance") and
`L` (the invariant the net must zero before feet leave) to `Fiction` + obs. The
obs layout (§7) grows by ~2 dims (indices 18,19).

**Scoped consequences (not free, all principled):**
- `observe.rs` — add the two derivations to `Observer::observe`, `Fiction` +
  `RawReality` gain fields.
- `telemetry.rs` — `SCALAR_COLUMNS` (frozen 56-array, header contract) grows;
  update the array, `format`, and the `header_matches_the_frozen_scalar_columns`
  + rectangularity tests.
- `learned.rs`/`JumpResidual` `build_obs` — append `ω_com/ω_scale`, `L/v_scale·?`
  (pick a normalizer — `L` units px²/s; scale by `hip_height·v_scale`).
- This is an authored-stack-only obs change; the committed walk deploy is not
  retrained, but its obs *width* changes ⇒ gate under `residual_jump`/`use_learned`
  so `use_learned=false` stays bit-for-bit (Law discipline).

**Recommendation:** include it. It is the single highest-value rotational signal
for an anti-fall residual and it was invisible in every prior attempt.

## 11. Is the expert replay or reactive? (audit + one fix)

The residual is only safe if we know what the expert *is*. Audit of committed v1
[`JumpCtrl`](../walker_control/src/jump.rs) (`60888f7`): it is a **hybrid that
leans toward replay**:

| Part of the stroke | Reactive or replay | Code |
|---|---|---|
| Phase clock `Load→Launch→Flight` | **replay** — pure timers `t >= dur` | jump.rs:159–177 |
| Chord shape (106→78→106, x −33/+39.5) | **replay** — fixed curve, reads nothing | jump.rs:46–54,166 |
| Hip anchor | **half** — `y` measured each tick; `x` seeded once + open-loop carry | jump.rs:209–221 |
| `Flight→Land` exit | **reactive** — ends early on foot contact | jump.rs:182 |
| `Land→Load` re-arm | **reactive** — waits for `grounded` | jump.rs:185 |
| `Load→Launch` | **replay** — fires even if the crouch never loaded | the gap |

Measured consequence (this is why it falls over): in-place hops (physics roughly
matched the schedule) reached ~30 px apex; the run-up hop (moving feet NOT
planted when the timer fired Launch) pushed into an unbraced stance → apex
+0.7 px, a lunge. The expert is a **right shape at a moment reality had not
earned** once physics diverges. A replayed proposal can go stale exactly when it
matters.

**Why a replay expert is still the right hook:** the residual *is* the reactive
layer. Obs (§7/§10) carries the measured polar `(r_obs, s_obs)` + contact +
`support_offset`, so the proposal-vs-reality disagreement is visible to the
policy in the same channels it corrects. Expert supplies the prior; observations
supply the truth; PPO learns the gap — the deal of residual-over-replace.

**One expert-side fix we take BEFORE making the policy learn around it:**
**event-trigger `Load→Launch`** the way `Flight→Land` already is — fire when the
measured chord has actually compressed past a threshold (`r_obs < r_fire`,
`r_fire ≈ 0.8`), timer kept as a fallback cap. Cheap, mirrors the transition
that already worked, no policy surgery; then the residual's job is genuinely
attitude/shape, not "wait for reality". This answers the "can replay give
reasonable direction when physics diverges?" worry at its weakest seam.

**Implemented (2026-11-19).** `ControlConfig::jump_fire_r` (default `0.8`;
`≤ 0` disables ⇒ pure clock replay) plus `ControlConfig::jump_load_cap`
(default `0.30 s`). The gate sits at the `Load→Launch` **boundary**, not inside
Load — the crouch itself still plays on time, or there is nothing to measure.
The predicate is a *crossing* test (`t + dt ≥ load_end`), and the latch
(`fired`) re-arms at every re-hop so each hop earns its own launch. Harness
knobs: `JUMP_FIRE_R`, `JUMP_LOAD_CAP`. Tests:
`the_gate_holds_launch_until_the_chord_actually_compresses`,
`the_gate_releases_the_moment_the_measured_chord_compresses`,
`the_gate_cap_fires_even_a_body_that_never_loads`,
`a_disabled_gate_is_the_pure_clock_replay`,
`each_re_hop_has_to_earn_its_own_launch`.

## 12. Ballistic lock-in — two checkpoints, flight reward-free (user framing, revised)

At the instant the feet leave the ground the flight is **fully determined**:
gravity is the only external force, so the CoM parabola AND the angular
momentum `L` (§10) are conserved through the arc. At the last grounded tick
(vello y-down, `g = 980 px/s²`):

```
apex_pred    = h + v_com.y² / (2g)          (v_com.y < 0 = moving up)
t_flight     = ballistic solve from h, v_com.y
x_land_pred  = com.x + v_com.x · t_flight
spin_drift   = ∫ L/I(t) dt  ≈ (L / Ĩ) · t_flight   // sign/rank reliable, Ĩ mid-arc
```

Measured proof: v10 hop1 launched at stick-lean −0.11 but locked L ⇒ θ ran to
−1.5 "with no cause" in flight. The design consequence (user): **the jump is
evaluated at two checkpoints — liftoff and landing — and only there.** The
low-ω-at-liftoff doctrine also makes the one caveat to lock-in (the figure-
skater effect: `ω = L/I` *is* shapable mid-air by changing `I` with leg
config) irrelevant to the reward: a launch with small `L` needs no mid-air
correction at any achievable `I`. Flight is quiet by demand, not by rescue.

### 12.1 Checkpoint 1 — liftoff tick (last grounded → first airborne)

```
r_lock = − w_L · |ang_momentum| / (hip_height·v_scale)     // the conserved purchase
       [+ optional − w_spin · |omega_com| / omega_scale     // what actually topples]
       [+ optional + w_apex · clamp(apex_pred/target, 0, 1)] // the height we bought]
```

`L` is the honest price (it is what is *locked*); `ω_com` prices the immediate
consequence through the current `I`. Start with `L` alone; the two are a cheap
ablation. This is exactly the sparse-signal fix the old PPO hop lacked: the
doomed launch is priced on the tick it happens, not 0.4 s later at the crash.

### 12.2 Checkpoint 2 — the Land window (first ~`land_window_ts` ≈ 0.3 s grounded after flight)

The user's DCM suite, using the repo's own walking math
(`walker_math::placement` — `ξ = x + v/ω₀`, `ω₀ = √(g/z₀)`):

```
ξ        = com.x + v_com.x / ω₀        with ω₀ = √(g / hip_height)   // z₀ NOMINAL
p_center = (feet[0].x + feet[1].x) / 2 // geometric center of the two feet
e_DCM    = |ξ − p_center| / leg_len

r_land (per tick, inside the window) =
    − w_dcm    · e_DCM                        // 1. primary balance judge
    − w_xi     · |fd(ξ̇)| / v_scale           // 2. settle judge, FINITE-DIFFERENCED
    − w_post   · |height − target|            // 3a. knees absorb, don't collapse
    − w_spin   · |omega_com| / omega_scale    // 3b. torso not spinning while feet plant
```

Notes that pin the design:

- **z₀ = nominal standing height, not live z** (user's adjustment). The knees
  compress during absorption; `ξ` built on `z₀` stays a rock-solid balance
  indicator while `z` does its job of changing. This matches the existing
  `omega0(cfg.hip_height, cfg.gravity)` call sites in the gait.
- **Judges 1 and 2 are the same ideal quantity — keep 2 for the transient.**
  Under ideal LIPM, differentiating `ξ = x + v/ω₀` with `a = ω₀²(x − p_foot)`
  gives `ξ̇ = ω₀·(ξ − p_foot) = ω₀·e_DCM`. So `|ξ̇| ≈ 0` is *implied* by
  `e_DCM ≈ 0` — EXCEPT during the impact transient, where the constant-height
  assumption breaks (knee compression, vertical impulse). The finite-differenced
  `ξ̇` is precisely the "absorb smoothly, don't wobble on the ankles" signal.
  Both are cheap; drop judge 2 by ablation if it measures out as dead weight.
- Judge 3b is the same currency as checkpoint 1: the spin you land with was
  purchased at liftoff. The Land-window spin penalty is the *delivery*, the
  liftoff `L` penalty is the *price* — GAE connects them, so both get credit
  for the same launch decision.

### 12.3 Flight ticks: no shaping

Alive bonus only. The landing reward flows back through GAE to the launch
action; nothing else is required (or useful — see the quiet-by-demand framing
above). Obs prediction channels (§12.4) give the critic the trajectory it is
crediting.

### 12.4 Prediction channels (obs only, not reward)

At liftoff, re-estimated per Flight tick from §10 state:
`(apex_pred − hip_height)/hip_height`, `(x_land_pred − p_center)/leg_len`,
`spin_drift` (running integral `Σ (L/I_now)·dt` since liftoff — exact under the
current config, updates as leg config changes; supersedes the `L/Ĩ·t_flight`
placeholder). The net sees the consequence of its launch decision while it is
still correlated with the decision.

### 12.5 Termination: measured fall only

The earlier "optionally end the episode on predicted doom" is **removed**: a
predicted-doom termination invites the degenerate optimum of never launching.
Predictions inform the reward, never the episode boundary.

### 12.6 Authority during Flight

δ is **not acted on** through Flight (the v1 pencil already holds shape —
swinging airborne legs reaction-rotate the torso through the leg↔hip struts,
the only joints that exist). Landing placement is grounded authority: the Land
window is where the capture is earned. If training later shows the feet need
mid-air re-presentation, release the freeze for the last ~0.1 s before
touchdown — a measured decision, not a speculative one.

## 13. One policy, checkpointed credit (decision)

Two *reward* checkpoints, **one** policy — do not train a ground net and an
airborne net:

1. The flight states are *generated by* the launch policy; a separate flight
   net trains on a non-stationary distribution owned by another learner
   (classic coupled-instability), and under §12 the flight has near-zero
   authority anyway — nothing for a dedicated net to do.
2. The three landing decisions split cleanly without model count: **launch**
   sets the conditions (learned — checkpoint 1), **flight** is a receipt
   (no learning needed), **touchdown capture** is earned in the Land window
   (learned — checkpoint 2) and then handed to the existing land-into-step
   gait machinery.
3. The policy is phase-conditioned by its observations (grounded/touching/
   height/prediction channels), not by a phase switch in the network.
   `decide()`'s gating already restricts which controller runs; the residual
   lives inside that one controller.

Escalation path *only if* flight measures out as the bottleneck: a two-stage
curriculum (freeze launch, train Land-window capture, unfreeze) — same net,
staged credit, not a second model.

Dependency order: §10 fields (`L`, `ω_com`) are the inputs to everything here,
so §10 landed first (done, `1f8a0d3`). §12/§13 features ship with the residual
env (step 5 of the build order): reward terms in `ppo_collect_system`, the
prediction channels in the residual obs layout (§7, amended additively).

(Stash of the failed v8–v10 tuning saga: `stash@{0}`, recoverable but not the
base — we build residual on committed v1 `JumpCtrl`, plus the §11 `Load→Launch`
event-trigger fix.)
