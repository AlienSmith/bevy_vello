# The Prediction Math — DCM/MPC foot placement, parameter by parameter

Brief companion to `plans/swing-phase-timing.md`. Everything here is 1-D
(forward walking), units px and s, scale **100 px = 1 m**, so
`g = 980 px/s²`. This is the math the loop-1 gait runs on; each knob is listed
with what it *means physically*, not just its formula.

---

## 1. The model: an inverted pendulum that you can only steer by its feet

While one foot is planted, the whole character behaves (to first order) like a
mass `m` on a rigid leg pivoting at the foot: the **Linear Inverted Pendulum
(LIPM)**. The hip/CoM height `z` is held roughly constant by loop 2, so the
horizontal dynamics reduce to one law:

```
v_dot = ω₀² · (x − x_foot)          ω₀ = √(g / z)
```

- `x` — measured CoM x.
- `x_foot` — where the planted foot actually is (the support point).
- `ω₀` — the pendulum's natural fall rate [rad/s]. Bigger = the body
  diverges faster. At `z = 90 px`: `ω₀ = √(980/90) ≈ 3.3 s⁻¹`,
  so instability time `1/ω₀ ≈ 0.30 s` — roughly one human step.

**This is the whole controller's authority picture:** you never push the hip
directly. You choose `x_foot` (by where the *swing* foot lands), and physics
turns that choice into hip acceleration. Foot behind CoM ⇒ `v_dot > 0`
(gas). Foot ahead ⇒ brake. Foot under ⇒ coast.

## 1b. The exact foot-placement law (shipped 2026-10-27 as `DCM_TC`)

Ask the LIPM for the constant `x_foot` that puts the CoM velocity at `v_des`
after `t_c` seconds. With `x(0) = x`, `v(0) = v`, the LIPM solution is

```
x(t) = x_foot + (x − x_foot)·cosh(ω₀t) + (v/ω₀)·sinh(ω₀t)
```

and imposing `x'(t_c) = v_des` gives the closed form

```
p = ξ + (v·e^(−ω₀t_c) − v_des) / (ω₀·sinh(ω₀t_c))
  = x + α·v − β·v_des
      α = 1/(ω₀·tanh(ω₀t_c))         β = 1/(ω₀·sinh(ω₀t_c))
```

Two coefficients from **one** physical parameter `t_c` — a deadline, since a
foot can only act while planted, so `t_c ≈ T_stance`.

Why this matters: the horizon law in use (`p = ξ − v_des/(T_conv·ω₀)`) is
exactly this with `α → 1/ω₀` and `β → 1/(T_conv·ω₀)`. That is not an
approximation with a tuning knob — it is a **truncation**, and the two halves
are linked exactly by

```
β(t_c) = Kp   ⟺   sinh(ω₀·t_c) = T_conv   ⟺   t_c = asinh(T_conv)/ω₀
```

So every `Kp = 1/(T_conv·ω₀)` has an exact `t_c` that comes with a *free
correct `α`*; tuning `Kp` alone throws that away. Likewise the control form
`p = ξ − Kp(v_des − v)` has a single dof and cannot hit both coefficients.
**Do not tune `Kp` — pick `t_c`.**

Limits: `t_c → ∞` ⇒ `p = ξ` (plain capture point, asymptotic stop);
`t_c → 0` ⇒ `p = x + (v − v_des)/(ω₀²t_c)` (stiff velocity-error spring).
And the identity `α − β = tanh(ω₀t_c/2)/ω₀` makes the cruise lead
`v·tanh(ω₀t_c/2)/ω₀ → v·t_c/2`, i.e. this law **contains Raibert's neutral
point** as its small-`ω₀t_c` limit: at `t_c = T_s = 0.28` it gives a 15.7 px
lead against the neutral point's 16.8 px.

`t_c` at 120 px/s (`ω₀ = 3.2998`, `z = 90`):

| `t_c` | `α` | `β` | cruise lead `v(α−β)` | character |
|---|---|---|---|---|
| 0.12 | 0.805 | 0.746 | 7.1 px | too stiff (overshoots to 244 px/s) |
| 0.15 | 0.661 | 0.588 | 8.8 px | measured best (6.0 s steady) |
| 0.200 | 0.524 | 0.428 | 11.6 px | non-monotone |
| 0.28 | 0.416 | 0.286 | 15.7 px | calm brake, ≈ neutral point |
| 0.50 | 0.326 | 0.121 | 24.7 px | near-capture-point, slow |

---

## 2. The prediction: the capture point (DCM)

The law above is a runaway exponential, so "where the body **is**" is not the
thing to aim at — "where the body is **falling toward**" is. Solve the LIPM
for the unique point whose trajectory, if the foot is planted there, decays to
rest:

```
xi = x + v / ω₀          (the DCM / capture point)
```

Read it as *body state expressed as a landing commitment*: a point moving at
`v` only captures if planted at `xi` or ahead of it. The offset
`v/ω₀ ≈ 36 px` at 120 px/s: the faster you go, the further ahead of the body
its balance point sits.

**Rule of all MPC rules:** put `xi` inside the current support polygon and you
are guaranteed capturable; `xi` past the support edge means "committed to fall
forward — the next step must catch it."

## 3. Propagating the prediction one step

If the next stance plants at `p`, the LIPM says how the capture point evolves
during the flight time `T`:

```
xi(T) = p + (xi − p) · e^{ω₀·T}
```

The DCM diverges from the planted foot exponentially. At cruise
(`T = T_swing = 0.28 s`): `e^{3.3·0.28} ≈ 2.5` — a gap of 20 px at liftoff is
a gap of 50 px at touchdown. **This factor is why aiming at *now* fails:**
evaluated mid-flight the plan must aim at where `xi` will *be* at landing.
(The two candidate laws differ only in this term: the exponential (§4 of the
main plan) vs. the reactive `hip + gain·v·T` stand-in that approximates the
same lead with one tunable knob.)

## 4. The placement law: what target actually gets emitted

Demand that after touchdown the *new* stance converge the velocity to
`v_des` within a horizon `T_conv` — i.e. the required acceleration
`a_des = (v_des − v)/T_conv`, inverted through `v_dot = ω₀²(x − x_foot)`:

```
x_foot = x + (v − v_des)/(T_conv · ω₀²)
       = xi − v_des/(T_conv · ω₀)         (identical when T_conv = 1/ω₀)
```

- `T_conv` — **speed-tracking responsiveness**, in units of `1/ω₀`.
  `T_conv = 1.0` (default): converge within one pendulum time ≈ one step;
  human-like. `0.7`: harder brakes/accelerations, shorter steps at the same
  speed; more authority demanded of friction. `1.4+`: soft — measured to
  fall: the brake it plans is too weak to capture the body in time.
- `v_des` — **the command**. `TARGET_SPEED` px/s (signed). The only "intent"
  in the whole system; everything else is consequence.
- At `v = v_des` the law emits `x_foot = x`: land **under** the CoM. That is
  the zero-lead cruise bug: at liftoff-state it cancels the flight travel;
  evaluated at the *predicted touchdown state* the same formula keeps the
  brake lead. Same law, right evaluation time.

## 5. The trigger: when to launch the catch

The placement can only fulfill its promise if the swing *starts* early
enough. Liftoff condition (§3 of the main plan):

```
xi ≥ toe_edge − v · T_swing        (primary, DCM margin)
x   ≥ plant edge                   (emergency; what we have today)
```

`margin = v·T_swing` (~34 px at cruise) is the down-payment for flight time:
lift off a full swing-duration early, or the foot will plant behind a `xi`
already past the edge — no braking stance ever exists. Matched pair: the
margin trigger without touchdown-state placement is worse than no trigger
(measured: the foot chases a stale target it can't reach in the capped flight).

## 6. Feasibility clamps: where the promise is broken on purpose

- `x_max` (`±hip`) — geometric reach: `√((L₁+L₂)² − z²)·safety_margin ≈ 54 px`
  at the defaults. Every target is clamped into it; an un-clamped target beyond
  `x_max` raises `capture_saturated` — the "uncapturable, fall imminent"
  signal (plan §8). Clamps are where MPC says *I have no solution*, not where
  it fudges one.
- `toe_len` — support-edge proxy (`8 px`; no heel/toe particles yet): how far
  past the plant `xi` may travel before the step is forced. Larger = more
  ankle-like authority from geometry, longer late stance, riskier catch.
- `T_max` guard on `T = dx/v` in the exponential iteration: without it the
  fixed point diverges when `ω₀·dx/v > 1`. The implemented variant sidesteps
  iteration: evaluate once with `T = phase_timer` (the swing schedule itself),
  exponent capped at 2.0.

## 7. Fulfillment: how the emitted number becomes physics

The resolver commands **poses**, never forces. The chain per tick:

```
measure (x, v, z)  →  xi, ω₀           [prediction]
                   →  x_foot = xi − v_des/(T_conv·ω₀)   [placement]
                   →  swing profile → IK → angular constraints   [swing authority]
                   →  touchdown, foot at x_foot          [the physical pivot]
stance thereafter:  v_dot = ω₀²(x − x_foot)             [CoM does the rest]
```

Fulfilled iff: (a) target inside `x_max`; (b) liftoff early enough (the
margin); (c) the physical foot lands where commanded (friction + leg
stiffness + PD lag — the part our soft-body fights, hence plant re-anchoring,
the backward solve at touchdown, and the learned-budget idea in §4.1).
The backward solve closes (c) *kinematically*: it re-inverts the *achieved*
plant offset into the hip velocity for the next plan, so landing error is
absorbed as a velocity correction instead of poisoning the next prediction.

## 8. Parameter table (current defaults, px/s units)

| knob | env | default | meaning (one line) |
|---|---|---|---|
| `z` (hip height) | — | 90 | pendulum height; sets ω₀, reach `x_max`, `1/ω₀` clock |
| `g` | — | 980 | gravity at 100 px = 1 m |
| `ω₀` | derived | 3.3 | fall rate [1/s]; one step ≈ its time scale |
| `v_des` | TARGET_SPEED | 120 | the only command; signed cruise velocity |
| `T_conv` | T_CONV | 1.0 | how fast the plan promises to fix speed error (in 1/ω₀ units) |
| `T_swing` | SWING | 0.28 | flight time = horizon of the prediction = margin budget |
| `T_stance` | STANCE | 0.28 | the step clock; the DCM advances `e^{ω₀T}` per stance |
| `toe_len` | TOE_LEN | 8 | how far xi may go past plant before forced liftoff |
| `x_max` | derived | ≈54 | reach clamp; saturation = uncapturable flag |
| `kv` `beta` | — | 0.12, 1.0 | Raibert feedback + landing-error reversion (legacy cruise law) |
| `mpc_trigger` | MPC_TRIGGER | off | the §3 margin trigger + §4 prediction (matched pair) |

## 9. One worked step (cruise, forward, exact current numbers)

State: `x = 0`, `v = 120`, `z = 90`, stance foot at `p = −30`,
`ω₀ = 3.3`, `T_swing = 0.28`.

1. `xi = 0 + 120/3.3 = +36.4` → xi is 66 px past the current plant ⇒ the
   margin trigger fires when `xi ≥ p + toe − v·T = −30 + 8 − 33.6 →` already;
   liftoff now.
2. Predict at landing: `xi(T) = −30 + 66.4·e^{0.92} ≈ −30 + 166 = +136`.
3. Place: `x_foot = xi(T) − v_des/ω₀ = 136 − 36.4 ≈ +100`?? — beyond `x_max`;
   the honest signal: at cruise from *this* pivot the plan wants a step longer
   than the leg allows ⇒ clamp to reach, and the hip will indeed pass over it
   (the stance will be gas, exactly the tilt-walk you observed).
   The steady cycle instead sits where the clamped demand and reach agree:
   `dx ≈ x_max + v/ω₀` ⇒ stride ≈ 90 px, period ≈ 0.56 s ⇒ **v ≈ 160 px/s
   emergent cruise**, above `TARGET_SPEED` — the speed the *geometry* chose,
   not the command. (The command can only bind through (2)+(3) *inside* reach.)
4. Stance: with `x_foot = +100` clamped to hip+54, foot lands ahead ⇒ brake
   until next tick says otherwise.

(The naive stale-state version of step 2 skips the exponential: `x_foot =
36.4 − 36.4 = 0` = under the body = pure coast/gas — the measured ratchet.)

---

Related: main plan `plans/swing-phase-timing.md` (§2–§4 derivation),
session status `plans/event-gait-phaseA-handoff.md` (what is built, what was
measured, next candidates).
