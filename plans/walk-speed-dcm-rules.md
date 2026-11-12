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

## 2b. THE MECHANISM (code audit, 2026-11-12 — supersedes the suspect ranking)

Reading `stepping.rs:201-233` + `placement.rs:213-224` + the CSV closed the
case. Three facts chain into the attractor:

1. **The trigger is always true at touchdown.** `dcm_crossed` fires when
   `dir·(ξ − plant) ≥ toe − margin`, `margin = v·T_sw`. At v=150 the margin
   is 42 vs `toe = 8` — the gate reads `≥ −34`, i.e. "lift unless ξ is 34+
   px *behind* your plant." A brake plant is aimed ≈44 px ahead of the hip,
   ξ sits 45 ahead ⇒ `ξ − plant ≈ +1` ⇒ trivially crossed. The "event"
   trigger is dead code in practice: it fires the tick the stance begins.
2. **Only the anti-phase gate grants any stance at all.** `stance_over &&
   wants_step && !other_swinging` — a foot cannot lift while its partner is
   mid-swing, so stance duration = the partner's flight = the swing PACING
   `t_target = stride/v` (stepping.rs:285), capped at 0.28. The DCM event
   never schedules anything; the flight pacing does.
3. **The brake is granted a horizon but starved of authority.** The
   anti-phase gate floors stance at the partner's flight (≈0.3 s — close to
   the law's t_c, so the horizon is roughly honored; earlier "stance ≈ 0"
   reasoning corrected). The starvation is geometric: at v=150→v_des=60 the
   deadbeat demands a plant ≈46–52 px ahead of the hip while `x_max ≈ 54`
   — 85–96 % of reach. Swing delivery lag (the foot travels ~90 px in
   0.28 s while the hip runs at 150; the last px take a measured ~0.2 s to
   close) lands it 10–20 px short ⇒ real deceleration `ω₀²·(x−p)·T` is
   ~2× weaker than promised, and the shortfall grows with Δv ⇒ the
   attractor. `capture_saturated` fires intermittently (invisible — not in
   the CSV; hence F0). The law/scheduler fight is confirmed in a subtler
   form: the law's promise depends on a plant the scheduler's cadence
   cannot *deliver*, and the always-true trigger (1) makes the scheduler
   blind to that (it never waits for ξ the way the margin formula claims).

Fix, faithful to the user's package (its guard 3.1 is now the PRIMARY lever —
smaller Δv per step makes the demand comfortably reachable, killing the
delivery-lag shortfall): drive `v_des` toward `cmd.speed` at a *reach-budgeted*
rate so per-step demand ≤ ~70 % of `x_max` (the 120→80→40→0 staircase the user
predicted, as a command shaper — §4d-3.1's "automatic in the clamp" reading
holds only because the clamp was silently *failing* to decelerate; with the
shaper the clamp is reserved for emergencies). Plus the scheduler honesty: the
trigger evaluates against the NEXT support (the incoming plant), lifting when
ξ crosses the incoming foot's edge — at cruise ~immediate (as the margin
intended), under braking the stance holds while ξ is behind the incoming
plant. Implementation: `swing_dyn` flag; predictive `t_hold` + `t_cap = 2·T_sw`
fallback; `T_sw = clamp(L_lead/v, 0.15, 0.35)` driving swing pacing, margin,
and the deadbeat floor; 30 ms touchdown debounce. Re-solve already ships
per-frame (`placement.rs:196` horizon-stretch); swap fixed `t_c` for scheduled.
F0 (export `trigger_reason`, `capture_saturated`, plants) lands FIRST — the
delivery-lag hypothesis above is the thing to see.

## 2c. Why the attractor is ~150 (suspects, in audit order — historical)

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

### 4c. The fight, and the corrected math (2026-11-12 re-derivation)

The measured attractor is a **deadbeat ↔ event-scheduler conflict**: the law
promises velocity decay at deadline `t_c`; the scheduler grants stance for
whatever the swing clock/event produces; when they disagree the promise is
voided and neither Δv nor cadence responds to the command. The user's fix
package (predict the event deadline, re-plan through swing, guard the per-step
velocity change) is the resolution. Before adopting it, the feasibility math
was re-derived in the repo's own closed form — and it corrects BOTH the earlier
§4c note and the intuition that any horizon choice makes the brake reachable.

From `p − x = α·v − β·v_des`, `α = 1/(ω₀·tanh(ω₀t_c))`,
`β = 1/(ω₀·sinh(ω₀t_c))`, two identities (cosh−1)/sinh = tanh(·/2):

- **Cruise/accel** (`v_des = v`): `p − x = (v/ω₀)·tanh(ω₀t_c/2)` — grows
  with horizon, bounded by `v/ω₀`.
- **One-step stop** (`v_des = 0`): `p − x = (v/ω₀)·coth(ω₀t_c)` — shrinks
  toward `v/ω₀` as the horizon GROWS; explodes as it shrinks (a short
  horizon demands MORE, not less — the superseded note above, and the
  "−23 px in reach" table floated in chat, are wrong; this is the verified
  algebra).

At `v = 150, z₀ = 104`: `v/ω₀ = 49 px` vs `x_max ≈ 54`. **Every honest brake
demand sits at or past the reach edge regardless of `t_c`.** Feasibility is
therefore forced by geometry:

1. the reach clamp IS the per-step brake authority (plant at max lead ⇒
   `Δv` per step ≈ `v/ω₀·(coth−1)`-bounded bleed);
2. large command changes MUST span multiple steps (120→80→40→0) — the user's
   guard 3.1 is a necessity, not a preference;
3. what swing time then buys is **cadence**: more clamped-brake feet per
   second, each delivered *before the CoM crosses the old pivot* (§4a
   Phase-1-only braking). Short swing = many small feasible brakes.

### 4d. The three-point scheme (user package, corrected against 4c)

**Fix 1 — predict the event deadline for the law (not a constant).** The
deadline the deadbeat should promise is the stance the scheduler will
actually grant. Shipped trigger geometry (`stepping.rs:205`):
`dir·(ξ − plant) ≥ toe − margin`, `margin = v_com·T_swing`, so the edge sits
`margin − toe` BEHIND the pivot and the naive `T_step = (1/ω)ln(Δ/(ξ₀−p))`
(the user's formula, edge *ahead*) is negative/undefined exactly when braking
is hardest — a sign correction, not a rejection:

```
edge  := plant + dir·(toe_len − margin)         # same edge as the trigger
T_ev  := (1/ω₀)·ln( (edge − ξ₀)·dir / … )       # defined while ξ₀ < edge (cruise)
```

In cruise it is the true short window before `dcm_crossed` fires (~tens of ms
— the gait grants far less stance than the fixed `t_c = stance_ts` promises);
in deep brake it is undefined and the *measured* horizon takes over — the
shipped `ADAPT_TC` (`types.rs:215`) already folds in the previous stride's
measured wait; the upgrade is to predict from current `ξ₀` and to floor the
law's promise at the horizon the *swing schedule* guarantees (Fix 2), so the
planner and the state machine stop fighting.

**Fix 2 — re-solve the swing target every tick in flight (closed-loop
re-planning).** The form `p = ξ_t − (ξ_des − ξ_t)/(e^{ωT_rem}−1)` is right but
`ξ_des` is ill-defined for cruise (the law targets a velocity at a deadline,
not a point). The clean shipped-primitive equivalent: at the *current*
overspeed, the brake demand is `≥ v/ω₀ ≥ 49` and past reach — so
`t_c_planned` is never the lever; instead re-solve the deadbeat each swing
tick with `v_com(t)`, `ξ(t)` (they change with every physics bump) and the
**guaranteed** deadline `t_c_g = t_c_event + T_rem`, clamping the result to
reach (the clamp is the authority, per 4c — a saturated target is *correct*,
not an error). This replaces one-shot liftoff placement: friction/impact
changes mid-step are absorbed by the next plant, smoothly.

**Fix 3 — hybrid guard.** (3.1) per-step Δv capped by reach — automatic in
the corrected reading (the clamp does it); no separate slew limiter needed,
which also **supersedes the Point 0 setpoint-ramp proposal** from chat — the
law + clamp produces the 120→80→40→0 staircase *by physics*, without a
command shaper. (3.2) `T_swing ∝ 1/v` — keep as **fixed spatial lead**
`L = v·T_swing` (the margin is already built on it; the swing then always
covers one swing-time of travel, with the trigger unchanged). (3.3) timeout
fallback — already shipped as `hovered_in` (stuck swing lands anyway) +
`wants_step` freeze (stuck stance settles safely); the one real cliff is
`v_des → 0` mid-swing: floor the law's `v_des` at ~15 px/s and let the last
segment be handled by the existing capture-point stop path (`stopping` in
`walk.rs:184`).

**Point 3 (two-phase impedance) stays deferred to RL** as in the pre-
revision text: continuous stiffness over the stance cycle is exactly what a
residual should discover, not hand-build (R4; also the jump landing-absorp-
tion's mechanism).

### 4e. Per-tick update loop (v2, the three fixes combined)

```
every tick (walking mode):
  # reality (unchanged stamps)
  omega0 := sqrt(g / z0);  xi := com_x + v_com/omega0
  v_des  := cmd.speed * dir_sign            # raw intent — NO slew limiter (4d-3.1)

  # cadence by fixed spatial lead (4d-3.2)
  T_sw   := clamp(L_lead / max(|v_com|, v_floor), T_min, T_max)   # e.g. L=30 px, [0.15,0.35]

  # deadline the event/schedule guarantees (Fix 1) — measured off the STANCE plant:
  #   xi diverges from plant as (xi−p)e^(ωt), so edge crossing:
  edge   := plant_stance + dir·(toe_len − v_com·T_sw)
  T_ev   := (1/omega0)·ln( dir·(edge−plant) / dir·(xi−plant) )
            # defined iff both dir-distances share sign and ratio > 1. Branches:
            #  · margin > toe (any walking speed with lead L≥toe): edge behind ξ at
            #    touchdown → T_ev ≈ 0 → liftoff IMMEDIATE, stance = T_sw exactly.
            #    (This is why the fixed clock secretly scheduled the gait — B0 makes
            #    it open and controllable: T_sw IS the horizon.)
            #  · v < toe/T_sw (low speed): T_ev > 0 predicted — the event grants
            #    MORE stance than the swing asks (pendulum support time); use it.
            #  · ξ behind plant (deep brake): diverges away, never crosses → the
            #    shipped com_crossed emergency grants ≈ (plant−x)/v; fallback =
            #    ADAPT last-granted horizon.
  t_c    := max(clamp(T_ev, 0, T_max) + T_rem_of_swing, 0.15)  # floor: law stability

  # placement, re-solved EVERY tick including mid-swing (Fix 2)
  p      := capture_point_deadbeat(xi, v_com, v_des, omega0, t_c)
  p      := clamp(p, hip ± x_max)          # saturation = max feasible brake (4c)
  if leg.swing: retarget leg.swing_to = p   # was frozen at liftoff

  # liftoff: shipped dcm_crossed + margin + new 30 ms touchdown debounce
```

Build order (replaces the old "Scope for this implementation round"):
**B0** = the dynamic-`T_sw`-as-fixed-spatial-lead schedule + per-tick swing
retargeting + `t_c` = predicted/mixed horizon, all behind one env
(`SWING_DYN=1`, default off ⇒ bit-identical committed gait); the debounce
ships with it. Then run the 60-120-60 ladder (§3, now reinterpreted as
fixed-vs-dynamic cadence A/B). `td_solve`/`hip_v_mix` (F1/F2) stay as the
next suspects if the attractor outlives B0 — with B0, short *granted*
horizons may make the old double-correction benign; the CSV will say.

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
