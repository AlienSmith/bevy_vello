# Walking speed control — the rules, the measured failure, the fix order

Goal: the scripted per-`move` `speed` becomes a tracked setpoint at *any*
value in the walking range (60, 90, 120, …), reachable in ≤1–1.5 s, with the
character upright. This doc replaces `plans/controllable-speed-walk.md` (the
2026-10-29 plan, written against the old `game_lib` hip-pin architecture) as
the live reference; that file stays for history.

## 0. The measured failure (2026-11-12, `tmp/walker_qa/walkv60-120-60.csv`)

Script: stand 1 s → boost 150 (1 s) → move 60 (4 s) → move 120 (4 s) →
move 60 (4 s) → stand 1 s. Settled-segment means:

| segment | cmd_vx | v_filt | P3vx (raw) | cadence |
|---|---|---|---|---|
| move 60 (first) | 63.6 | **151.3** | 152.2 | ~1.6 steps/s |
| move 120 | 118.8 | **150.6** | 151.1 | ~1.8 |
| move 60 (after) | 61.2 | **154.0** | 154.1 | ~1.6 |

Three facts:

1. The body runs a **~150 px/s attractor** — left by the boot kick — and
   tracks *neither* 60 nor 120. It is not "can't go below 120"; it is
   "command speed barely matters once walking."
2. The 120→60 step-down decelerates by **0 px/s over 4 s** (~360 px). The
   brake channel is open where it matters.
3. **Cadence is speed-independent** (~0.6 s stride at every speed) — the
   clock, not the dynamics, schedules the steps.

Stride arithmetic pins the shape: v·T ≈ 150·0.6 ≈ 90 px at every segment.
Whatever the placement law aims, the realized stride is locked to the fixed
clock's product.

## 1. The rules (the doctrine this project commits to)

The body between footsteps is a Linear Inverted Pendulum:
`v̇ = ω₀²·(x − x_foot)`, `ω₀ = √(g/z₀)`, `z₀ = hip_height` (NOMINAL height,
never live-compressed z — knee flex must not pollute the balance read).
Two points decide everything:

- **CoM** `x` — where the mass is.
- **DCM** `ξ = x + v/ω₀` — where the mass is *going*. `ξ̇ = ω₀·(ξ − x_foot)`.

The foot is the only steering wheel (ground friction at the plant converts
placement into hip acceleration). The three-regime placement rule (user):

| regime | plant position | physics |
|---|---|---|
| cruise (`v = v_des`) | at the deadbeat point ≈ `ξ + (v·e^(−ω₀t_c) − v_des)/(ω₀·sinh(ω₀t_c))` | ride the pendulum |
| accelerate (`v < v_des`) | **behind CoM** | `x − x_foot > 0` ⇒ `v̇ > 0` |
| decelerate (`v > v_des`) | **ahead of CoM**, converging toward `ξ` | `v̇ < 0`; `p → ξ` as the one-step-stop limit |
| stop (`v_des = 0`) | at `ξ` (capture point) | deadbeat: zero velocity at the plant |

The shipped law already *is* this table: `walker_math::placement::
capture_point_deadbeat` implements `p = x + α·v − β·v_des` with
`α = 1/(ω₀·tanh(ω₀t_c))`, `β = 1/(ω₀·sinh(ω₀t_c))`. No new math is needed
for placement; the failure is **authority delivery**, not theory.

Corollary rules:

- **R1 (liftoff is a DCM event, not a clock event).** Under
  `GaitAuthority::Full` (`walker_control::WalkingCtrl::new` sets it), the
  stance leg lifts when `ξ` crosses *its own* toe edge **minus the lead
  margin `v_com·swing_ts`** (`stepping.rs::dcm_crossed`): the swing needs
  one swing-time of lead, so the trigger fires early by design;
  `com_crossed` is the emergency; the timer must not fire steps
  (`wants_step` requires intent).
- **R2 (swing time belongs to the stride, not to a stopwatch).** Cadence
  should satisfy `T_step ≈ stride/speed`-consistent bounds derived from
  placement feasibility, not a speed-independent constant (current:
  `swing_duration_ts = 0.28` fixed).
- **R3 (timing must grant the brake its horizon).** A deadbeat promise
  `v(t_c) = v_des` requires stance *lasted* `t_c`. If the step clock cuts
  stance short of `t_c`, the law's deadline is a lie.
- **R4 (impedance is the second steering wheel — user's new rule).** The
  stance leg's stiffness modulates *how hard* a given plant decelerates the
  body: CoM behind the foot during stance ⇒ stiffen (crisp braking); CoM
  ahead, accelerating ⇒ soften (longer push-over, less bounce). Today
  `ActuatorConfig` compliance is a fixed constant — the lever does not
  exist yet. It is also the same mechanism the jump's landing absorption
  wants (`plans/jump-ppo-residual-polar-action.md` §12.2 height judge).

## 2. Why the attractor is ~150 (suspects, in audit order)

The placement law runs every tick with `plan_meas_v = true` (WalkingCtrl::new
closes the velocity loop through the deadbeat lead). At v≈150, v_des=60, the
law aims a plant that promises decay within one stance. Measured: no decay.
So something starves the promise. Ranked:

1. **Stance-timing starvation (R2/R3 violation).** At speed, `dcm_crossed`
   is true the tick after touchdown (`toe_len − margin ≈ −34` at 150 — the
   trigger is effectively unconditional), so stance length is set by the
   *incoming swing's fixed 0.28 s clock*. Stance can never outlast the
   swing to honor a longer horizon, and the stride is clock-locked at ~90
   px. **The clock is scheduling the gait; the DCM is only rubber-stamping
   it.** (Contradicts R1's intent — worth re-reading as the headline bug.)
2. **Fiction integrator re-anchor.** `hip_v_mix = −1` (default): fiction v
   obeys `approach(v_prev, v_desired, a·dt)` — reaches 60 in 0.1 s of pure
   model time while reality coasts at 150; every law fed the fiction on any
   path (`v` at liftoff retargeting, `capture_offset`, ceilings) sees a
   body already at the setpoint. `plan_meas_v` patches the deadbeat lead
   only; `v_law` is fed elsewhere too. The `HIP_V_MIX` knob (blend of
   filtered reality + command) was built exactly for this and is off.
3. **Reach saturation.** At 150, `ξ` sits 45 px ahead of the hip; `x_max ≈
   54`. The brake plant lives in the last 20 % of the workspace; any lag
   between commanded and actual touchdown (measured ~0.2 s leg closure)
   eats it. The `capture_saturated` debug flag is in
   `walker_math::DebugInfo` but **not exported by walker_control's CSV** —
   blind right now (F0 fix below).
4. **Touchdown double-correction.** `td_solve = true` (default) rewrites
   fiction `v` backward at each landing *on top of* the deadbeat's own
   error correction (the config doc says `beta=1` double-corrects; the
   measured 45↔126 sawtooth is attributed there to exactly this). A
   backward-v fiction re-anchors the next placement short — could hold the
   attractor. `TD_SOLVE=0` is the falsification.
5. **Ceilings (likely innocent).** `v_cruise_max` at v_des=60 is ≈234 — not
   binding at 150. Rule out early, keep simple.

## 3. Falsification ladder (run first, cheap → expensive)

All knobs exist as envs (`walker_qa` `apply_lower_env` currently wires only
`STANCE_TS`/`SWING_TS`; the `lower_config` fields `TD_SOLVE`, `HIP_V_MIX`,
`PLAN_MEAS_V`, `DCM_TC`, `V_MEAS_TAU` have *no env wiring in walker_qa* —
add them; they already exist in the sandbox). Each run uses
`tmp/walker_qa/walkv60-120-60.json`; metric = settled `v_filt` per segment.

- **F0 (eyes before levers):** export the `walker_math::DebugInfo` signals
  the CSV lacks — `trigger_reason` per leg, `capture_saturated`, plant
  positions — as new telemetry columns (append, like §10's two columns did).
  Without this every other experiment is half-blind.
- **F1 `TD_SOLVE=0`:** kills the touchdown rewrite. Attractor gone ⇒
  suspect #4.
- **F2 `HIP_V_MIX=0.3/0.5`:** fiction tracks reality. Attractor gone ⇒
  suspect #2.
- **F3 `STANCE_TS=0.5 SWING_TS=0.5`:** grant the brake a real horizon.
  Attractor gone ⇒ suspects #1/#3 (timing/reach).
- **F4 `DCM_TC` window tweak / `V_MEAS_TAU=0.3`** — finer variants.

Interpretation discipline: one lever per run, compare the three settled
segments side by side. If no single lever moves it, the answer is #1
(timing) and the fix is §4-B.

## 4. The swing-time doctrine (user, 2026-11-12) — THE chosen fix design

Core principle: **swing time controls braking.** Because one leg's stance
duration equals the other leg's swing duration (`T_stance ≈ T_swing` in the
alternating gait), controlling swing time directly controls step time — and
step time is the channel the deadbeat law was never granted. This section
supersedes the earlier suspect-ranking guesswork wherever they overlap; the
measured attractor (§0) is exactly what this doctrine predicts.

### 4a. The mechanism (why a fast trailing swing = pure braking)

During stance on the leading (pivot) foot the pendulum has two halves:

- **Phase 1 — hip behind the pivot:** the body decelerates (uphill).
- **Phase 2 — hip ahead of the pivot:** gravity re-accelerates it forward
  (downhill). Every normal step is Phase 1 + Phase 2, which is why cruise
  speed is self-sustaining and hard to bleed.

If the trailing leg lifts **early** and swings **fast** enough to plant the
new foot **before the CoM crosses the old pivot**, the step is delivered in
Phase 1 only: the new plant stays ahead of the CoM, the next stance is again
all-brake. Phase 2 is skipped entirely ⇒ **pure deceleration**. This is
physically just Rule R3: the brake horizon must be *granted*, and here it is
granted by shortening the swing, not lengthening the stance.

### 4b. Why 120 px/s got stuck (the user's diagnosis, confirmed by §0)

1. **CoM-triggered lifting was too late** (the old gait): waiting for the
   CoM to cross the leading foot meant the swing started mid-stance; at
   120+ px/s the hip travels far during a fixed 0.28 s swing, so every
   step landed the new plant behind the advancing CoM — every step kept a
   long Phase 2. The gait could only ever re-cruise, never brake.
2. **Fixed step time starved the deadbeat:** passing a slow fixed
   `T_step` (0.28) into the deadbeat equation as the horizon demanded a
   foot placement further forward than the leg could physically reach
   (`x_max ≈ 54`, `ξ` 45 px ahead of the hip at 150 px/s — the §2-suspect-3
   saturation, now understood as *structural*, not incidental).

### 4c. The three-point update scheme

**Point 1 — trigger the lift on the DCM, not the CoM (ALREADY SHIPPED, keep
it).** `dcm_crossed` (`stepping.rs:172`) fires the rear-foot lift when `ξ`
passes the support edge — that is exactly this rule; the lead margin makes
it fire ~one swing early, which the doctrine *wants* (early lift). What it
lacks is the **double-support debounce**: a 20–40 ms lockout after a
measured touchdown so landing bounce cannot re-trigger a step. At high
speed `ξ` is already ahead of the foot at touchdown, so liftoff is
immediate — and with a short swing the *cadence*, not the trigger, sets
step time.

**Point 2 — parameterize the swing by a DYNAMIC duration (the implementation
focus).** Drive the swing trajectory from a target `T_swing`:

```
T_cruise = 0.35 s     # normal walking
T_brake  = 0.15..0.18 s  # while v_com > v_des (fast forward delivery of the brake foot)
```

scaled between the two by the velocity error (e.g. lerp on
`(v−v_des)/v_des`, clamped), and pass **the same** `T_swing` into the
deadbeat law as the convergence deadline `t_c` (clamped
`T_step ≥ 0.15 s` so the law's coefficients cannot blow up — α
`≈ 1/(ω₀²·t_c)` as `t_c→0`). With this, the horizon promise and the
cadence that grants it are one number: R3 is satisfied *by construction*.

Honest note on the coefficient behavior: shortening `t_c` *increases* the
demanded forward plant (α ≈ 0.66 at 0.15 s vs 0.37 at 0.35 s ⇒ brake demand
`x+63` px vs reach 54 px). That is not a bug — it is the mechanism: the
demand saturates against the reach clamp, the foot plants at maximum
forward reach = **maximum available braking**, and the short swing delivers
that saturated plant *before the CoM crosses the pivot*. The reach clamp
becomes a deliberate authority limiter instead of an error-stealing silent
failure.

**Point 3 — two-phase stance impedance (NOTED, DEFERRED).** Firm strut
while the hip is behind the leading foot (Phase 1 braking), softened knee/
ankle if the hip crosses into Phase 2 (rob gravity of its acceleration
boost). This is the §1-R4 lever. We deliberately do **not** hand-build it:
it is a continuous stiffness schedule over the stance cycle — precisely the
kind of fine-grained profile the jump PPO residual (and possibly a walk
residual) is better positioned to discover. Keep as a documented future
action; the actuation path (`actuate::leg_commands` →
`emit::distance_config` per-tick compliance) is already identified in §4c
of the pre-revision text if RL ever wants it.

### 4d. The per-tick update loop (pseudocode, points 1+2 combined)

```
every tick (walking mode):
  # 1. reality
  measure com_x, v_com (EMA-filtered); z0 := hip_height (NOMINAL)
  omega0 := sqrt(g / z0)
  xi     := com_x + v_com / omega0

  # 2. intent → cadence
  v_des := cmd.speed * dir_sign
  err   := (|v_com| − v_des) / max(v_des, 1)          # signed overspeed
  T_sw  := lerp(T_cruise=0.35, T_brake=0.15, clamp(err, 0, 1))
  t_c   := max(T_sw, 0.15)                            # law floor

  # 3. placement (deadbeat on the SAME horizon)
  p     := xi + (v_com·e^(−omega0·t_c) − v_des) / (omega0·sinh(omega0·t_c))
  p     := clamp(p, hip ± x_max)                      # saturation = max brake

  # 4. liftoff (point 1, shipped + debounce)
  if stance_leg and not debouncing:
      if xi crossed plant toe (dir-aware, margin lead as today):
          begin swing: target p, DURATION T_sw, existing arc
  on touchdown: plant := measured foot_x; debounce timer := 30 ms

  # 5. fiction velocity (keep plan_meas_v lead; hip_v_mix / td_solve per F1-F2)
```

Scope for this implementation round: **point 2** (dynamic `T_sw` as both
swing duration and deadbeat `t_c`) + the 20–40 ms double-support debounce;
point 1 is already `dcm_crossed`; point 3 deferred to RL. The F-ladder (§3)
still runs, but its meaning changes: with point 2 in, `STANCE_TS`/
`SWING_TS` envs become the A/B arms (fixed vs dynamic cadence), and F1/F2
decide whether `td_solve`/`hip_v_mix` coexist with the new cadence or fight it.

## 5. Files & anchors (verified 2026-11-12, post-§10 tree `1f8a0d3`)

- `walker_math::placement::capture_point_deadbeat` — the law, [`placement.rs:266`](../walker_math/src/placement.rs:266)
- `walker_math::laws::compute_v_max` — cruise ceiling, [`laws.rs:41`](../walker_math/src/laws.rs:41)
- `walker_math::stepping::step_leg` — R1 triggers, [`stepping.rs:172`](../walker_math/src/stepping.rs:172)
- `walker_math::walk::step_lower_body` — fiction velocity, stop-settle, hip
  stance clamp [`walk.rs:154`](../walker_math/src/walk.rs:154), clamp [`walk.rs:263`](../walker_math/src/walk.rs:263)
- `walker_control::WalkingCtrl::new` — `plan_meas_v = true`,
  `GaitAuthority::Full` [`control.rs:488`](../walker_control/src/control.rs:488)
- `walker_control::WalkingCtrl::step` — stamps (capture measure, contact,
  first-tick plant seed) [`control.rs:522`](../walker_control/src/control.rs:522)
- `walker_control::telemetry::SCALAR_COLUMNS` — what the CSV sees
  [`telemetry.rs:67`](../walker_control/src/telemetry.rs:67) (F0 extends this)
- `walker_qa::apply_lower_env` — env wiring to extend [`main.rs:226`](../walker_qa/src/main.rs:226)
- defaults: `target_speed 120, accel 800, stance/swing 0.28, kv 0.12, toe_len 8,
  dcm_tc 0 (=stance), td_solve true, plan_meas_v false (WalkerCtrl overrides),
  hip_v_mix −1` [`types.rs:294`](../walker_math/src/types.rs:294)

## 6. Dependency on the jump track

The jump's landing checkpoint (`jump-ppo-residual-polar-action.md` §12.2)
hands off to this gait after the Land window; a gait that cannot hold a
slow speed will mis-credit the jump policy for landing errors it doesn't
own. Order: speed fix → jump residual env wiring. §10/§12 plan work is
committed (`1f8a0d3`, `a3a2519`); the §11 event-trigger edit in `jump.rs`
is parked mid-flight (uncommitted scratch: none — tree is clean at
`a3a2519`).
