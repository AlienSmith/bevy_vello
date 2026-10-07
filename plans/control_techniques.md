# Control & Feel Techniques Reference

> Status: **REFERENCE** (not a plan — a notebook of techniques worth knowing,
> ranked by how soon each one matters for this project).
>
> Companion documents:
> - [`arm_ik_tuning.md`](arm_ik_tuning.md) — how to tune the shipped arm stack
> - [`gravity_and_walking.md`](gravity_and_walking.md) — the walking plan
>
> Context for everything below: XPBD active ragdoll, physics at 90 Hz, render
> at 400+ FPS, aim for Stick Fight / TABS feel, CPU-bound, all joints driven
> by angular **rest targets** through compliance springs.

---

## How to read this document

**Tier 1** techniques fix problems you already have or will definitely hit.
**Tier 2** gives names to things you already built (useful for searching) plus
three ideas worth stealing. **Tier 3** is XPBD-solver-specific. **Tier 4** is
game feel, not physics — cheap and disproportionate.

**If you only take three:**

1. **One Euro Filter** — permanently fixes the noise-vs-lag tradeoff that
   forced `kd` down to 0.05.
2. **Raibert's velocity-error foot placement** — free shove recovery, one line.
3. **Hysteresis + dwell on every state transition** — prevents the relay-chatter
   bug class you already paid a session for.

---

# Tier 1 — techniques that fix real problems here

## 1.1 One Euro Filter (the direct upgrade to the kd noise ceiling)

**Problem it solves.** The aim ray shimmered at `kd = 0.1` and was clean at
`kd = 0.05`. That is the *fixed* low-pass dilemma: stiff enough filtering kills
jitter but adds lag; loose filtering tracks fast motion but passes noise. No
fixed cutoff wins both.

**The fix** (Casiez, Roussel, Vogel — CHI 2012): make the cutoff
**speed-adaptive**. Low cutoff at rest → jitter is filtered out. High cutoff in
motion → lag disappears. Used by essentially every VR hand tracker and every
drawing tablet.

```
// one per tracked signal, stateful across frames
dx_hat = lowpass(dx, alpha_d)          // derivative estimate, FIXED cutoff
fc     = fc_min + beta * |dx_hat|      // cutoff rises with speed
x_hat  = lowpass(x, alpha(fc))         // filtered value

lowpass(x, a) = a * x + (1 - a) * prev_hat
alpha(fc)     = 1 / (1 + 1 / (2 * PI * fc * dt))
alpha_d       = 1 / (1 + 1 / (2 * PI * fc_d * dt))   // fc_d ~ 1 Hz typically
```

**Two knobs:**

| Knob | Meaning | Tuning |
|---|---|---|
| `fc_min` | cutoff at rest (Hz) | raise until rest jitter is gone; 0.5–1.5 Hz typical |
| `beta` | speed sensitivity | raise until motion lag is acceptable; 0.001–0.05 typical |

**Where to apply:**
- the mouse target before IK (removes jitter at the source — best placement),
- and/or ω before `kd_offset` in [`calculate_arm_ik()`](../examples/game_lib/src/character/systems.rs)
  (lets you raise `kd` back up, possibly removing the need for a low ceiling),
- the slope EMA in the walking plan (§5/§6.3 — replaces the hand-rolled EMA).

**Cost:** two floats of state per signal. Trivial. This is the highest-value
item on the list.

---

## 1.2 Frame-rate-independent smoothing

**Problem.** The α0.5 chase in
[`interpolate_toward_angle()`](../examples/game_lib/src/character/systems.rs)
is a one-pole low-pass whose *effective time constant changes with dt*. At
90 Hz physics vs 400+ render — and if the physics rate ever changes — the feel
changes with it.

**Fix.** Replace the raw alpha with a time constant τ (seconds to close ~63%
of the gap):

```
alpha = 1 - exp(-dt / tau)
```

Now `tau` is the slider, and the behaviour is identical at any framerate.

- τ = 0.02 s → snappy, nearly instant
- τ = 0.05 s → the α0.5-at-90Hz equivalent (roughly)
- τ = 0.2 s → floaty / heavy

Applies to: arm rest chase, spine heading chase, slope EMA (walking plan),
`assist_decay` ramps, landing droop. **Anything that interpolates toward a
target should be τ-parameterized, not α-parameterized.**

Note the kd term is *already* correctly frame-rate independent — `kd·ω` has
units `[s]·[rad/s] = [rad]`, kd **is** the time constant, and it must never be
multiplied by dt. Only the α interpolators need this conversion.

---

## 1.3 Second-order spring smoothing ("SmoothDamp" / critical damping)

**Problem.** `interpolate_toward_angle` + `max_angle_rate` clamp gives
*velocity discontinuities*: constant slew speed, then a hard stop at the target.
It reads as robotic.

**Fix.** A critically-damped spring smoother preserves velocity continuity,
lands with exactly zero overshoot, and has one state variable (velocity).
This is Unity's `Mathf.SmoothDamp` / Unreal's `CriticalDamp`:

```
omega  = 2 / smooth_time
x      = omega * dt
exp    = 1 / (1 + x + 0.48*x*x + 0.235*x*x*x)
change = current - target
temp   = (vel + omega*change) * dt
current = target + (change + temp) * exp
vel     = (vel - omega*temp) * exp
```

**One knob:** `smooth_time` (seconds to settle) — a genuinely intuitive unit.

**Where worth it:** spine heading, hip, any joint a player watches closely.
**Where overkill:** elbows, fingers. Your existing `rotate_toward` is fine there.

Relationship to the shipped stack: this *replaces* `interpolate_toward_angle`
+ `max_angle_rate` with a single better-behaved primitive. The
ω-schedule / compliance / length-normalization layers all stay as-is.

---

## 1.4 Hysteresis + minimum dwell time (for the locomotion state machine)

**Problem.** Grounded ↔ Airborne will chatter exactly like the dead-zone relay
did on the arm aim — a noisy sensor near a single threshold flips state every
frame, and each flip re-triggers transitions (anchor re-record, compliance
ramps, landing squat) → visible vibration.

**Fix.** A **Schmitt trigger**, not a dead zone: two thresholds plus a timer.

```
Grounded -> Airborne:   no ground within h_far    AND  held for t_dwell
Airborne -> Grounded:   ground within h_near      (h_near < h_far)
t_dwell ~ 0.05-0.1 s
```

Apply the same pattern to:
- GettingUp (input must be held for N ms, plus minimum down-time),
- crest detection in the walking plan (don't soften the drive for a 1-frame
  probe miss),
- KnockedDown entry (impulse must exceed threshold for the full dwell window).

**Rule of thumb: any state transition driven by a noisy sensor needs BOTH a
band and a timer.** A band alone still chatters at the band edge; a timer
alone still latches onto a single spike.

---

## 1.5 1/f noise for "alive" idle motion

**Problem.** Procedural characters read as *dead* when they are too steady.
A perfect sine sway reads as a machine; perfect stillness reads as a corpse.

**Fix.** Pink-ish noise. The cheap fake version (two or three incommensurate
sines — no state, no library):

```
sway = sin(t*1.0)
     + 0.5 * sin(t*2.31 + 1.7)
     + 0.25 * sin(t*5.17 + 0.4)
```

Irrational frequency ratios mean it never visibly repeats. Feed a *tiny* amount
(±1–2°) into:
- `desired_heading_upper` while idle (breathing / weight shift),
- stride length and Bézier apex as gait irregularity (foot placement is never
  exactly the same twice — this alone kills most of the "marching robot" look),
- arm rest pose when not aiming.

Nobody notices it consciously. Everybody notices its absence.

---

## 1.6 Soft joint limits via compliance ramp

**Problem.** A hard clamp at a joint limit (e.g. the 0.95·(l₁+l₂) soft-knee
cap) prevents the snap but still produces an abrupt "hits a wall" look.

**Fix.** Ramp compliance up as the joint approaches its limit, so the
character *eases* into the limit:

```
margin  = (theta_limit - |theta|) / theta_soft_band      // 1 = free, 0 = at limit
alpha_eff = alpha / clamp(margin, 0.1, 1.0)
```

Same mechanism family as the ω-schedule already shipped — it's just scheduled
on *position* instead of *velocity*. Reuses
[`angular_compliance_at_length()`](../../study_vello/integrations/vello_physics/src/utility.rs)
as the base.

Applies to: knee extension, hip range, elbow hyperextension. The visual
difference is "muscular deceleration" vs "hit an invisible wall".

---

# Tier 2 — names for things you already built (search terms)

Recognizing the standard names makes it dramatically easier to find prior art.

| What you built | The textbook name | Search term |
|---|---|---|
| ω-scheduled compliance softening | **Gain scheduling** | "gain scheduling control" |
| `+kd·ω` on measured ω, not on error derivative | **Derivative on measurement** | "derivative kick", "D on measurement" |
| `kd_max_offset` clamp | **Reference governor** / setpoint clamping | "reference governor", "anti-windup clamp" |
| Compliance = "how hard does it push when blocked" | **Impedance control** | "impedance control robotics" |
| `desired_heading_upper → up` | **Virtual model control** | Pratt & Pratt, "virtual model control" |
| Touchdown vertical droop → stiffen | **SLIP** | "spring-loaded inverted pendulum", Raibert hoppers |
| Rest-angle springs as the animation blender | **Kinematic intent, physical execution** | "active ragdoll", "pose-driven physics" |
| interpolate-then-clamp | **Rate limiter + first-order lag** | "slew rate limiter" |
| Length-normalized compliance | **Scaling by inertia** | (no standard name — yours is reasonable) |
| No integral term | **PD-only / impedance control** | see §2.4 below |

## 2.1 Raibert's three-part hopper controller (validates the whole design)

The 1980s MIT Leg Lab (Marc Raibert, later Boston Dynamics founder) decomposed
hopping into **three independent controllers**. It maps 1:1 onto the walking
plan:

| Raibert | Ours |
|---|---|
| Hopping (foot placement) | `d_step` in §6.2 |
| Attitude (torso upright) | heading pin |
| Thrust (stance-leg spring) | `pin_vertical_weight` |

The hopping controller is:

```
x_foot = x_hip + v * T_stance/2  +  k_v * (v - v_desired)
         \___ our d_step ___/      \___ MISSING TERM ___/
```

**Add that velocity-error term.** It gives shove recovery for free: after a
push, `v > v_desired` briefly, so the next foot placement lands *further
forward*, which is exactly how humans catch themselves. Without it, the
character can only recover via the anchor-break rule (reactive); with it, it
recovers *proactively*, one step ahead.

Start `k_v` at ~0.1 s and tune up. This is a one-line change to walking plan
§6.2 and the single highest-value idea in this document after the One Euro
filter.

## 2.2 Capture point (the same idea, from humanoid literature)

```
x_capture = x_com + v_com * sqrt(2 * h_com / g)
```

Meaning: "where must I put my foot to come to a complete stop." Use it as the
**maximum** recovery step length so the character doesn't do the splits when
launched. Cheap to compute; it clamps the Raibert term sanely at high speeds.

Related search terms: "capture point", "divergent component of motion (DCM)",
"extrapolated center of mass (XCoM)". All the same family.

## 2.3 Why you should never add an I term

Two independent reasons, both decisive:

1. **Windup.** Integral action accumulates error while the arm is blocked by a
   wall. When the wall disappears, the accumulated integral releases
   explosively. Anti-windup clamping fixes it, but then you've built a worse
   version of the governor you already have.
2. **Steady-state error under load is a feature.** It is how the player *sees*
   the character straining. Compliance springs give you this for free and it is
   physically honest — a real arm holding a weight *does* droop.

PD-only is the **correct** choice for active ragdolls, not a simplification.

## 2.4 Also worth knowing (not immediately applicable)

- **LQR / pole placement** — computes optimal gains instead of tuning by hand.
  Needs a linear model; XPBD is too nonlinear for this to pay off. Skip.
- **ZMP (zero moment point)** — the classic humanoid balance criterion.
  Irrelevant when the root pin is the balance (our design). Would matter only
  if we ever went full TABS friction-driven.
- **Trajectory optimization / MPC** — what modern AAA physics characters and
  Boston Dynamics actually use. Requires a solver in the loop; far too
  expensive for CPU-bound 90 Hz. Skip permanently.
- **Phase-Functioned Neural Networks (PFNN)** — learned gait from a phase
  signal. Needs training data and a neural net in the hot path. Skip.
- **DeepMimic / RL locomotion** — beautiful results, enormous offline cost,
  and it produces exactly the *opposite* of the hand-tunable-sliders feel we
  want. Skip.

The honest summary: **everything we need is pre-2000 control theory plus game
feel tricks.** The modern ML stack solves authoring-cost problems we don't have.

---

# Tier 3 — XPBD-solver-specific things

## 3.1 Perceived stiffness ∝ iteration count

XPBD fixed PBD's dt-dependence via `α̃ = α/dt²` (already used in
[`angular_compliance_at_length()`](../../study_vello/integrations/vello_physics/src/utility.rs)),
but stiffness still **saturates with iterations**: a very low compliance simply
cannot be reached if there aren't enough solve passes to converge.

**Diagnostic:** if a joint feels "mushy" no matter how low you set compliance,
the fix is iterations, not compliance. Test by doubling iterations and
comparing.

## 3.2 Iterations vs substeps

They buy different things:

| | Buys | Costs |
|---|---|---|
| More iterations | stiffness / constraint accuracy | linear CPU |
| More substeps | fast-motion correctness (tunneling, high-velocity feet, thin colliders) | linear CPU *and* re-runs collision |

For a 90 Hz physics loop with a fast walker and feet that move quickly,
**one extra substep usually beats four extra iterations.** Substeps are also
the fix if feet ever punch through thin floor bars.

## 3.3 Global XPBD Damping constraint

Macklin et al. 2016 §4.6 defines a `Damping` constraint that scales all
velocities toward the center-of-mass motion:

```
v_i <- v_i - k * (v_i - v_com)
```

A cheap global stabilizer if the whole character ever slowly *gains* energy
from many small contacts (the classic soft-body jitter-accumulation failure).
Keep `k` small (0.001–0.01) or the character feels underwater.

## 3.4 Impulse-based knockback (never position teleports)

Apply `Δv = J/m` to particle **velocities** on a hit. Never move positions to
express a knockback.

Position teleports lie to the solver's velocity estimate (`v = (x - x_prev)/dt`),
which produces the "hit character jitters for 10 frames" artifact and can pump
energy into neighbouring constraints. Velocity impulses are honest: the solver
integrates them naturally and the collision response stays correct.

Same rule applies to getting-up (don't snap to upright — ramp the pin
authority, as the walking plan §5 already specifies) and to spawning /
respawning (teleport is fine there only because nothing is in contact yet).

---

# Tier 4 — game feel, not physics

Cheap, disproportionate payoff, and orthogonal to everything above.

## 4.1 Hit-stop / freeze frames

Pause the physics step for 2–4 frames on impact. The single highest
impact-per-line-of-code technique in existence. Works because the player's eye
needs the collision to *register*; without it, hits feel weak no matter how
much force you apply.

Implementation: a global `hitstop_timer`; while > 0, skip `step()` but keep
rendering. Slight slow-motion (dt × 0.2) instead of a hard freeze is the
softer variant.

## 4.2 Anticipation

Wind up *against* the strike first. For this project that is literally one
keyframed rest-angle waypoint before the punch — the compliance springs do the
rest. A punch with no wind-up reads as a teleport; a 60 ms wind-up reads as
intent.

## 4.3 Overshoot-and-settle

Free when `kd` is low. **Don't tune it away** in the name of responsiveness —
overshoot reads as weight and momentum. If the aim feels "lifeless," check
whether `kd` was raised too far.

## 4.4 Saccades for gaze

If head/eye look-at is ever added: biological gaze does **not** slew smoothly.
It moves in fast discrete jumps (saccades) with micro-pauses (fixations).

```
if |angle_to_target| > saccade_threshold and time_since_last > min_fixation:
    snap gaze toward target quickly (tau ~ 0.03 s)
else:
    hold
```

The difference is instantly readable as "alive" vs "turret".

## 4.5 Coyote time / input buffering

Allow the getting-up input to be pressed slightly *before* the conditions are
met (buffer ~0.2 s), and allow a brief grace period after leaving the ground.
Same trick as platformer ledge forgiveness. Pairs naturally with the dwell
timers in §1.4 — one is "require the condition to persist", the other is
"remember the input briefly".

## 4.6 Follow-through & secondary motion

Already emergent from the compliance stack: the arm lagging the torso *is*
Disney's principle #5 (follow-through) and #2 (secondary action). The
ω-schedule is what makes the lag speed-dependent, which is exactly the
"heavy limb" look. **Preserve it** — don't stiffen everything until the
character moves as one rigid block.

## 4.7 Squash, stretch, and non-uniform scale

Not applicable to a particle skeleton without extra work, but worth knowing:
the cheap version is scaling the *rendered* bone mesh along its axis based on
the constraint's current strain (`|length − rest_length| / rest_length`). Pure
presentation, zero physics cost, and it sells impacts hard.

---

# Appendix A — the tuning philosophy these all share

Every technique above follows the same three rules that the arm work proved:

1. **Passive mechanisms first.** The ω-schedule and compliance ramps can only
   *withhold* authority — they are identity at rest and therefore cannot create
   rest jitter. Always prefer a passive mechanism over an active one
   (kd, velocity injection) for the same symptom.
2. **Every regime boundary becomes a slider.** Don't hardcode thresholds; the
   correct value is not knowable in advance and the tuning UI is nearly free.
3. **Fix the measurement before the controller.** Most "controller is
   unstable" diagnoses are actually "the sensor is noisy." The One Euro filter
   is a measurement fix; that is why it outperforms any gain adjustment.

# Appendix B — symptom → technique quick index

| Symptom | Try | Section |
|---|---|---|
| Rest shimmer / jitter | One Euro filter, or `kd` ↓ | 1.1 |
| Laggy tracking of fast motion | One Euro filter (`beta` ↑), or SmoothDamp | 1.1, 1.3 |
| Robotic / mechanical motion | SmoothDamp, 1/f noise | 1.3, 1.5 |
| Feel changes when framerate changes | τ-parameterize all α interpolators | 1.2 |
| State flicker / transition chatter | Hysteresis + dwell | 1.4 |
| "Dead" character when idle | 1/f noise on heading + gait params | 1.5 |
| Joint slams into its limit | Compliance ramp near limit | 1.6 |
| Character can't recover from a shove | Raibert `k_v·(v − v_desired)` term | 2.1 |
| Recovery step is comically huge | Capture-point clamp | 2.2 |
| Joint feels mushy at any compliance | More iterations | 3.1 |
| Feet tunnel through thin floors | More substeps | 3.2 |
| Whole body slowly gains energy | Global Damping constraint | 3.3 |
| Post-hit jitter for ~10 frames | Impulse, not position teleport | 3.4 |
| Hits feel weak | Hit-stop | 4.1 |
| Attacks feel like teleports | Anticipation waypoint | 4.2 |
| Gaze/head looks like a turret | Saccades | 4.4 |
| Input feels unresponsive near transitions | Coyote time / buffering | 4.5 |

# Appendix C — references

- Casiez, Roussel, Vogel. *1€ Filter: A Simple Speed-based Low-pass Filter for
  Noisy Input in Interactive Systems.* CHI 2012.
- Macklin, Müller, Chentanez. *XPBD: Position-based Simulation of Compliant
  Constrained Dynamics.* MIG 2016. (§4.6 Damping; the `α̃ = α/dt²` derivation.)
- Jakobsen. *Advanced Character Physics.* GDC 2001.
- Raibert. *Legged Robots that Balance.* MIT Press, 1986. (three-part hopper
  controller)
- Pratt & Pratt. *Virtual Model Control: An Intuitive Approach to Bipedal
  Locomotion.* IEEE RA-M, 1998.
- Koolen et al. *Capturability-Based Analysis and Control of Legged
  Locomotion, Part 1.* IJRR, 2012. (capture point)
- Holden, Komura, Saito. *Phase-Functioned Neural Networks for Character
  Control.* SIGGRAPH 2017. (read it to know why we don't need it)
- Geyer & Herr. *The bipedal spring-mass model.* (SLIP)
- Thomas, Witting, Winkler. *Game Programming Gems 4* — "Filtering and
  smoothing" / SmoothDamp lineage; also Nystrom, *Game Programming Patterns*
  (state machines, and why hysteresis matters).
