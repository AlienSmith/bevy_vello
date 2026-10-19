# Raibert Sandbox — Third Loop (Attitude/Tilt) + Upper-Body Point

> Status: **PLAN — ready for implementation.** Companion to
> [`raibert_loop3_attitude.md`](raibert_loop3_attitude.md) (character-side loop-3
> reference), [`raibert_vertical_loop.md`](raibert_vertical_loop.md) (loop 1), and
> [`raibert_redesign.md`](raibert_redesign.md) (loop 2). Scope: the **sandbox only**.
> The real-physics hook-up is a later step (see §8).
>
> **Step 0 (do this FIRST, before any loop-3 code):** capture a headless baseline
> of the current 5-particle resolver (§7.1) to prove it still resolves, then use
> it as the regression reference once loop 3 lands.

## 1. Goal

The kinetic sandbox currently runs two of Raibert's three loops:

| Loop | Axis | Owner in [`resolver.rs`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs) |
|---|---|---|
| 1 | vertical (height / jump) | [`tick_vertical`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:457) |
| 2 | horizontal (foot placement / velocity clamp) | [`step_lower_body`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:952) |
| 3 | **attitude / tilt (missing)** | — |

Add loop 3: a **rotational PD that holds the torso upright** (`θ_desired = 0`),
modelled as an **inverted pendulum pivoted at the hip**, plus a **new upper-body
particle** on top of the hip so the tilt is visible and the PD can be observed.

The user's intent (confirmed):

> The torso should be upright. Later we hook this to the physics simulation —
> the physics causes the tilt, and Raibert loop 3 gives the target for the
> angular constraints to correct back to vertical.

So the sandbox's loop-3 output is the **attitude command / torque the angular
constraint path will consume**, and the tilt source is a *simulated* physics-ish
disturbance: the hip's horizontal acceleration — exactly what makes a real torso
pitch during walking / braking / jumping.

## 2. What exists today (no change needed to this)

- Pure resolver over **5 particles**: `hip`, `left_knee`, `left_foot`,
  `right_knee`, `right_foot` ([`LowerBodyPose`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:311)).
- `step_lower_body(input_dir, config, state, dt) -> (LowerBodyPose, DebugInfo)`
  runs in `FixedUpdate` at **90 Hz** ([`plugin.rs`](bevy_vello/examples/game_lib/src/lower_body/plugin.rs:12));
  input from arrows/WASD + Space via [`drive_lower_body`](bevy_vello/examples/game_lib/src/lower_body/systems.rs:98).
- Render (5 circles + thigh/calf lines + floor + camera-follow) and the egui
  tuning window live in [`raibert_sandbox.rs`](bevy_vello/examples/collision_detection/src/raibert_sandbox.rs:70).
- `LowerBodyPose` is constructed in exactly **two** places:
  [`resolver.rs:1299`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1299) and
  [`systems.rs:35`](bevy_vello/examples/game_lib/src/lower_body/systems.rs:35).

## 3. The model (pure math, no solver)

### 3.1 State

Add a small `AttitudeState` to `ResolverState`:

```
pub struct AttitudeState {
    pub theta: f32,   // radians from world vertical; 0 = upright
    pub omega: f32,   // d(theta)/dt, rad/s
}
```

Sign convention: **+θ = forward lean** (+x, the direction of travel, y-up).
Consistent with [`raibert_loop3_attitude.md`](raibert_loop3_attitude.md) §2/§3.

### 3.2 The equation of motion (inverted pendulum at the hip)

The upper body is a point mass at distance `L_upper` above the hip. Around the
upright equilibrium (`θ = 0`) the torso is **unstable** (balancing-pole
dynamics), which is precisely why the PD is needed. Normalizing inertia out
(`I = m·L²`, so all terms are angular accelerations rad/s²):

```
θ̈ =  (g̃ · sin θ) / L_upper · GRV      ← gravity: DESTABILIZING, tilts torso away from vertical
   − (a_x · cos θ) / L_upper · ACC      ← hip horizontal accel: inertial coupling (tilt source)
   +  u                                  ← loop-3 control input
```

```
u  =  Kp · (θ_desired − θ) − Kd · ω      with θ_desired = 0 (upright)
```

- `GRV` (`0`/`1`, default `1`) — toggle the gravity term.
- `ACC` (`0..1`, default `1`) — gain on the hip-acceleration coupling.
- `a_x` = measured hip horizontal acceleration, finite-differenced from the
  resolver's own `hip_vel.x`: `(v_now − v_prev) / dt`. This is the *physics-like*
  tilt source: start/stop walking leans the torso backwards/forwards, jump does
  not (v_x is ballistic in flight).
- Outputs to expose: `θ`, `ω`, `u` (as `attitude_torque`), `a_x`
  (`hip_accel_x`).

### 3.3 Integration & stability

Semi-implicit (symplectic) Euler, matching the grounded-spring style:

```
ω += θ̈ · dt
θ += ω · dt
θ  = θ.clamp(±θ_max)          // θ_max default ≈ 1.30 rad (~75°) — never spin past "fallen over"
```

Linearising about upright (`θ ≈ 0`): `θ̈ + Kd·θ̇ + (Kp − g̃/L_upper)·θ ≈ 0`.

- **Needs `Kp > g̃ / L_upper` to hold upright.** With `g̃ = 980 px/s²` and
  `L_upper = 70 px` (the §9 `h_com ≈ 70 px` offset), the minimum is **14 rad/s²/rad**.
- Critical damping: `Kd = 2·sqrt(Kp − g̃/L_upper)`.
- **Recommended defaults:** `Kp = 45`, `Kd = 11` (≈ critical for the 14 floor),
  `L_upper = 70`, `ACC = 1`, `GRV = 1`, `θ_max = 1.30`.
- At 90 Hz, `Kp·dt² ≈ 45/8100 = 0.0056` — far inside the explicit band; no
  stability risk.

## 4. Flight behaviour (matches the reference doc §7.4)

- During `VerticalPhase::Flight` the torso is **weightless** in its own frame →
  set `GRV = 0` (no `g·sinθ` term). `a_x` is also ~0 in flight (ballistic carry),
  so the only authority left is the PD itself → **PD stays ON** as air
  reorientation toward upright.
- This is exactly the "loop 3 is the only attitude authority in flight" rule
  from [`raibert_loop3_attitude.md`](raibert_loop3_attitude.md) §7.4, tested
  cheaply in the sandbox.

## 5. Code changes

### 5.1 [`resolver.rs`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs) — config, state, math, wiring

1. **Config knobs** in `LowerBodyConfig` (defaults in parentheses):
   - `torso_len: f32` (70.0) — upper-body pivot-to-COM distance, px.
   - `attitude_kp: f32` (45.0) — rotational spring gain, rad/s² per rad.
   - `attitude_kd: f32` (11.0) — rotational damper gain, 1/s.
   - `attitude_accel_coupling: f32` (1.0) — gain `ACC` on the hip-accel term.
   - `attitude_gravity_coupling: bool` (true) — `GRV` toggle.
   - `theta_max: f32` (1.30) — tilt clamp, rad.
2. **State**: `AttitudeState { theta, omega }` in `ResolverState`
   (default `{0, 0}`), plus a `prev_hip_vel_x` field for the finite difference
   (guard first tick / `dt ≤ 0`).
3. **Pure math functions** (unit-testable, mirror the `predict_apex_rise` style):
   - `attitude_pd(theta, omega, theta_desired, kp, kd) -> f32` — the `u` law.
   - `tick_attitude(state, config, hip_accel_x, dt)` — EOM + semi-implicit
     Euler + clamp; applies the Flight `GRV` rule by reading `state.vs.phase`.
4. **Wiring** in `step_lower_body` (after the horizontal loop, before IK):
   - compute `a_x = (state.hip_vel.x − prev_hip_vel_x) / dt` (guard first tick);
   - call `tick_attitude`; write `theta`, `omega` into state;
   - compute the upper-body point `upper = hip + torso_len·(sin θ, cos θ)` (y-up)
     and store it in the pose.
5. **Pose**: add `upper: Vec2` to `LowerBodyPose`.
6. **DebugInfo**: add `theta`, `omega`, `attitude_torque` (= `u`), `hip_accel_x`.

### 5.1b [`hip_pin.rs`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs) — the physics hook-up

- In [`drive_hip_pin`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:344),
  after the existing `step_lower_body(...)` call (line 391) and before
  `rigid_frame_targets(...)` (line 418): write `pin.theta = state.attitude.theta`
  (converted to the Vello y-down sign convention, +θ = forward lean in travel
  frame ⇒ −theta in y-down world). The frame then rotates by loop 3's tilt, and
  the character physically leans back on walk-start / forward on stop.
- No change to leg or hip translation terms — loop 3 stays presentational here.

### 5.2 [`systems.rs`](bevy_vello/examples/game_lib/src/lower_body/systems.rs)

- `LowerBodyController::new` pose literal: add `upper` (init `hip + {0, torso_len}`
  since θ = 0 at rest).
- `reset_lower_body` (`R`) already rebuilds `ResolverState::new` — attitude
  resets for free; no change needed beyond the new state default.

### 5.3 [`mod.rs`](bevy_vello/examples/game_lib/src/lower_body/mod.rs)

- Re-export `AttitudeState` and the two new math functions.

### 5.4 [`raibert_sandbox.rs`](bevy_vello/examples/collision_detection/src/raibert_sandbox.rs)

- **Render**: draw a torso line `hip → upper` (e.g. violet, 6 px) and a 6th
  filled point (e.g. violet) at `pose.upper` via `to_vello`. Camera-follow on
  `hip.x` unchanged.
- **Tuning window**: new `"Attitude (loop 3)"` section with sliders `torso_len`,
  `attitude_kp`, `attitude_kd`, `attitude_accel_coupling`, `theta_max`; checkbox
  `attitude_gravity_coupling`; and telemetry `theta (deg/rad)`, `omega`,
  `attitude_torque`, `hip_accel_x`.
- **No manual nudge control.** The torso is already disturbed live by every
  start/stop of walking (via the accel coupling — a natural tuning input) and by
  jumps (flight). Kp/Kd recovery is observable from ordinary input; headless
  verification uses the unit tests, which seed a nonzero `θ₀` directly.

## 6. Unit tests (pure, `cargo test -p game_lib`)

1. `pd_is_zero_at_upright` — `attitude_pd(0, 0, 0, …) == 0`.
2. `pd_restores_forward_lean` — for `θ > 0`, `u < 0` (restoring) and scales with
   `Kp`; for `θ < 0`, `u > 0`.
3. `pendulum_diverges_without_pd` — run `tick_attitude` with `Kp = Kd = 0`,
   `GRV = 1`: `|θ|` grows (unstable upright — proves the model is a balancing
   pole, not a weight-held spring).
4. `settles_upright_with_pd` — `Kp = 45, Kd = 11`, small `θ₀`; step N ticks;
   assert `θ → 0` and no ringing (|θ| never exceeds initial).
5. `accel_coupling_leans_back` — forward `a_x > 0` drives `θ < 0` (backward)
   while `Kp = Kd = 0`, then recovers to `0` once PD is enabled.
6. `upper_point_geometry` — `upper = hip + L·(sin θ, cos θ)`; at `θ = 0` it is
   exactly `torso_len` above the hip; a lean moves it in `+x`.
7. `theta_clamps` — large applied `Δω` doesn't push `θ` past `±θ_max`.
8. `flight_is_weightless` — with `vs.phase = Flight`, `GRV` term is dropped (θ
   does not diverge under gravity; PD alone holds/returns it).

## 7. Definition of done

- `cargo run -p collision_detection --bin raibert_sandbox --release` shows **6
  particles + a torso line**; the torso leans backward under forward acceleration
  (and forward under braking), is disturbed on a jump/flight without diverging,
  and the PD returns it to upright.
- The torso visibly recovers to vertical cleanly (no bobblehead with the default
  `Kp/Kd`) during ordinary start/stop walking and after a jump.
- All loop-3 knobs editable live; telemetry shows `θ`, `ω`, torque, `a_x`.
- All pure math unit tests pass (`cargo test -p game_lib`); `cargo clippy` clean.
- Loop 3 contributes **no hip position or velocity term** — legs / loop 1 / loop 2
  behaviour is byte-identical when attitude knobs are left at defaults.
- Reference doc [`raibert_loop3_attitude.md`](raibert_loop3_attitude.md) §7 gains a
  note: "sandbox attitude state implemented; verification path resolved".

### 7.1 Headless validation via the existing whole-body harness (baseline first)

**No new harness.** Per user guidance, loop 3 is integrated **directly into the
physics path** and verified with the existing [`headless_arm_pd.rs`](bevy_vello/examples/collision_detection/examples/headless_arm_pd.rs)
whole-body mode — its `move`/`jump` JSON commands drive the resolver through
[`drive_hip_pin`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:344) in
the real 90 Hz fixed physics chain, and its telemetry block already carries
`cmd_vx` + `vphase` from `pin.state`
([`headless_arm_pd.rs:1797`](bevy_vello/examples/collision_detection/examples/headless_arm_pd.rs:1797)).

How the loop-3 state reaches physics — the integration point:

- [`HipPin.theta`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:209) is
  already the rigid-frame attitude consumed by
  [`rigid_frame_targets()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:253)
  to rotate `P1/P2/P3` — and is currently **hardcoded `0.0`**. After loop 3,
  `drive_hip_pin` writes `step_lower_body`'s `θ` into `pin.theta`, so the
  character's pinned frame physically tilts with loop 3 (and recovers). That is
  the sandbox→physics hook-up, done without touching the XPBD solver.

Scripts: reuse the existing [`walk_script.json`](bevy_vello/plans/walk_script.json)
(walk → wait) and [`jump_script.json`](bevy_vello/plans/jump_script.json)
(jump → wait).

- **Step 0 — baseline (BEFORE any loop-3 code):** run e.g.
  `SCRIPT=.../walk_script.json OUT_CSV=tmp/headless/att_walk_base.csv cargo run -p collision_detection --example headless_arm_pd --release`
  (and the same for `jump_script.json`). Confirm the particle rows are sane
  (no NaN, bones rigid, jump ballistic) — proves the resolver is healthy today
  and freezes the exact hip/leg rows as the regression reference. At baseline
  `pin.theta` stays `0`, so the frame is unrotated.
- **Telemetry addition (small):** add `theta`, `omega`, `attitude_torque`,
  `hip_accel_x` scalars to the whole-body row block in [`headless_arm_pd.rs`](bevy_vello/examples/collision_detection/examples/headless_arm_pd.rs)
  (beside the existing `cmd_vx`/`vphase` at ~line 1797), read from
  `pin.state.attitude` + `pin.theta` (`NaN` before loop 3 lands).
- **After loop 3 lands — regression + validation:** re-run the identical scripts
  with new `OUT_CSV`. Compare: hip/leg particle columns must be **unchanged** vs
  baseline (loop 3 adds no hip/leg translation term); `theta` must show lean-back
  on walk start, lean-forward on stop, no divergence through flight, and recovery
  to `0`. The sandbox window (`raibert_sandbox_main`) gives the visual twin.

## 8. Future hook-up to the physics (NOT this task)

- The character already has the seam: [`HipPin.theta`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:209)
  + [`rigid_frame_targets()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:253)
  rotate the `P1/P2/P3` frame. The sandbox's `θ` output will feed that path: the
  physics measures the actual torso frame angle, and loop 3 computes the target
  rest angle for the torso angular constraints to correct it to vertical.
- Return to §10 of the reference doc when doing the hook-up (P2/CoM pivot vs the
  sandbox hip P3, `r_hip ≈ (0, −70 px)` conversion).

## 9. Reference: where the numbers come from

- `g̃ = 980 px/s²` — sandbox gravity default, per [`raibert_vertical_loop.md`](raibert_vertical_loop.md) §7
  (the `100 px = 1 m` mapping; the ECS already overwrites this from the live world).
- `L_upper = 70 px` — the upper-body CoM offset above the hip, per the topology
  analysis in [`raibert_loop3_attitude.md`](raibert_loop3_attitude.md) §9
  ("~70 px above the hip (P3)").
- `Kp dogleg at 14 rad/s²` — derived in §3.3 from `g̃/L_upper`; defaults chosen
  ~3.2× above it with near-critical damping so the sandbox visibly recovers
  without ringing.