# Raibert-Controlled 2D XPBD Character — Design Doc

Status: converged (2026-10-21). Scope: 2D sagittal-plane biped made of XPBD particles,
rigid distance-constraint bones, angular-constraint "motors", driven by a Raibert-style
three-loop controller. No animation data, no pinning, no direct external forces on the body.

---

## 1. Core principles

1. **The ground is the only external force.** The controller touches nothing but angular
   constraint parameters (targets, compliance, damping). Collision + friction at the feet
   is what moves the body. Walking = controlled falling, caught step by step.
2. **Template matching.** Raibert's loops were derived for the SLIP template: a point mass
   at CoM height on a massless springy leg. The body is designed to *match* that template
   so the loops' assumptions hold. Control difficulty scales with model–body mismatch;
   body design is the cheapest place to fix mismatch.
3. **Feedback absorbs actuator error.** Constraint compliance/kp cannot be computed
   analytically (coupled constraint graph, contacts, substep artifacts). It doesn't need
   to be: the loops are error-driven, so actuation only needs to be monotone and
   ballpark-correct. Params are hand-tuned feel parameters, seeded by heuristics and
   bounded by solver stability.
4. **Balance is dynamic.** The character is never statically balanced. Loop 1 steps to
   catch the falling CoM every stride. Loop 3 is *not* a balance controller — it keeps
   the plant (mass distribution / attitude) predictable so loop 1's template stays valid.

## 2. Body design (settled scope)

- **Stick-like upper body**: spine angular constraints stiff (low compliance) and
  critically damped (zeta ~= 1). A shove produces a small visible bend, then returns
  without ringing. The small give is the "physical feel"; the wobble under impact comes
  from legs and whole-body dynamics anyway.
- **Mass budget**: 80–90% of total mass in the core/pelvis particle(s). Upper torso and
  head share the remaining 10–20%. This keeps whole-body CoM ~= core at a near-fixed
  offset, bounds h_eff, and keeps hip-torque attitude authority high.
- **Weightless arms**: low-mass, NOT zero-mass (XPBD divides by mass; zero mass blows up
  or gets ignored). 1–2% of total mass each. Decorative + minor momentum trim only.
- **Rigid bones**: distance constraints at (near) zero compliance. Watch for:
  - impact harshness on landing (option: brief knee-extension gain reduction on touchdown),
  - jitter at full leg extension (option: tiny nonzero compliance = "stiff tendon").
- **Geometry check**: character must not be too tall/top-heavy for its step reach.
  Shove test (sec. 7) measures peak CoM displacement vs maximum reachable step
  correction (see shove test, sec. 8). If shoves routinely exceed reach, no tuning fixes
  geometry — redesign mass.

## 3. Measurement layer (per frame, cheap)

```
CoM    = sum(m_i * p_i) / sum(m_i)        # ALL particles incl. arms, welded props
v_CoM  = sum(m_i * v_i) / sum(m_i)        # EMA-filtered (2–4 frame time constant)
z_CoM  = CoM height above ground/foot
theta  = torso pitch (world-frame) — from hip->shoulder vector (2D: trivial, no SVD)
omega  = d(theta)/dt, filtered
```

**Rule: loops 1, 2 and all balance quantities use the whole-body computed CoM — never the
hip particle.** Raibert could use the hip because on his robot hip ~= CoM by construction.
On this character, torso lean shifts CoM relative to hip exactly when balance matters;
measuring at the hip makes loop 1 step for a mass distribution that no longer exists.
(With the stick body the error shrinks to a small constant — but true CoM always works,
costs ten lines, and stays correct if spine/mass/props change later.)

Torso fit angle and whole-body CoM are different objects with different jobs:
attitude (loop 3) uses theta; balance (loops 1–2) uses CoM.

## 4. The three loops

All three loops are the complete decomposition of 2D template dynamics:
loops 1–2 = translation of CoM (x, z); loop 3 = rotation about CoM (pitch).
The CoM is momentum bookkeeping (Koenig decomposition), not a joint — actuation
happens at the hip/knee/ankle; effects are referenced to the CoM.

### Loop 1 — foot placement (balance), capture-point form

Upgraded from Raibert's heuristic to the exact LIPM solution (Pratt et al. 2006):

```
omega_0 = sqrt(g / z_CoM)
x_capture = CoM_x + v_CoM_x / omega_0
```

- z_CoM is measured, so thrust/lean-induced height changes automatically rescale
  capture distance (couples loop 2 in for free).
- **Reachability clamp**: clamp x_capture to the leg's reachable workspace (from leg
  segment lengths / full-extension geometry). Clamping = "in trouble" signal
  (available later for lean/brace/N-step recovery; out of current scope).
- Acts **only during swing phase** (gated by the gait clock).
- Foot target (world) -> hip/knee angular constraint targets via analytic 2-bone IK.
- **Scripted swing profile**: lift (knee flex) -> advance (hip flex) -> extend (knee
  extend) -> plant, as a hip/knee angle profile over swing phase, with loop 1 setting
  only the touchdown x. Without the profile the toe drags and each touchdown injects
  a random kick the other loops chase.

### Loop 2 — thrust (height/energy), virtual-leg form

Bones are rigid, so there is no prismatic leg to modulate. Thrust = **knee/ankle
extension through angular constraints** during stance ("virtual leg"):

- Stance: extension targets/gains toward full extension; because the foot is pinned by
  friction, extension pushes the CoM up via ground reaction.
- Apex-height feedback: predicted apex from ballistic flight vs target height -> error
  scales the extension command for the next stance. Energy-based, hence naturally
  mass-adaptive (heavier load -> pushes harder, no retuning).
- Flight: knee flexes, leg retracts for swing clearance.
- Landing: controlled flexion (absorption) early stance.

### Loop 3 — attitude (torso pitch)

- PD on (theta_cmd - theta) and omega, with **absolute world-frame target** (upright +
  small scripted lean), computed from the measured torso angle — NOT relative to pelvis.
  Pelvis rocks with the gait; torso holds world-vertical. This structurally breaks the
  pump cycle between leg drive and torso correction.
- Delivery into XPBD constraints:
  - torque inversion: theta_target += (tau_des + kd*theta_dot) / k
  - damping baked into target: theta_target_used = theta_target + kd*theta_dot/k
    (XPBD has no damping parameter; this realizes the kd term). Alternative if noisy:
    velocity-augmented constraint C = theta - theta_target + beta*theta_dot*h.
- Gated primarily to **flight phase** (in stance, foot placement + leg already give
  attitude authority; hip torque in flight = momentum exchange leg <-> torso, exactly
  the original hopper mechanism).
- Damping-first tuning: zeta >= 0.9–1.0 on all spine joints. High damping still looks
  physical (energy removal = muscle tone); high stiffness with snap-to-target looks robotic.

## 5. Capture Point deep dive — the loop-1 upgrade

### 5.1 What Raibert's original loop 1 was

Raibert (1986) placed the foot at a heuristic "neutral point" plus corrections:

```
x_foot = x_hip + k_v * (v_measured - v_desired) + k_p * (x_hip - x_desired)
```

- Measured at the **hip** (valid only because his robot's hip ~= CoM by construction).
- The forward-offset term (foot ahead of hip proportional to speed) was **hand-derived
  from observation** of how far the hopper needed to step to hold speed — a tuned
  heuristic, not a solved quantity.
- Errors in the offset or gains show up as speed drift, which the feedback slowly
  corrects — but the controller has **no notion of falling**: it regulates speed, and
  balance is an emergent side effect of "steps land in roughly the right place."

### 5.2 What the capture point is

Derived from the **Linear Inverted Pendulum Model (LIPM)**: CoM at constant height z on
a massless leg, exactly the stick-body template this character is designed to match.
The LIPM dynamics are linear with natural frequency omega_0 = sqrt(g/z), and the exact
solution to "where must the foot go so the CoM comes to rest above it" is:

```
x_capture = x_CoM + v_CoM / omega_0
```

Formalized by Kajita (2001, "extrapolated CoM") and Pratt et al. (2006, "capture
point"). Placing the foot at x_capture brings the CoM to zero velocity directly above
the contact in exactly one step — it is the **boundary between capturability and
falling**, computed, not tuned.

### 5.3 Same formula, different status

The expressions look near-identical (`x + v*sqrt(h/g)` vs `x + v/omega_0`) — because
Raibert's heuristic was an approximation of this quantity. The upgrade is not the
formula; it is everything that comes *with* the derivation:

| | Raibert neutral point | Capture point (LIPM) |
|---|---|---|
| Origin | tuned heuristic from observation | exact solution of the template dynamics |
| Reference point | hip (assumes hip ~= CoM) | true whole-body CoM (measured) |
| Pendulum length | fixed assumed h | measured z_CoM — adapts to crouch, lean, thrust |
| Meaning of result | "step roughly here to hold speed" | "step exactly here to stop the fall" |
| Failure awareness | none — steps saturate silently | reachability of x_capture IS the balance verdict |
| Speed regulation | explicit (k_v on velocity error) | implicit (desired CoM velocity enters via target) |

### 5.4 What problems it solves for this character

1. **Silent impossible steps -> explicit signal.** With Raibert's formula, a big shove
   makes the neutral point land somewhere the leg can't reach; the controller commands
   it anyway and the character collapses with no warning — the classic "weird walking"
   after a disturbance. With the capture point + reachability clamp, saturation is a
   *measurable state*: the controller knows it is uncapturable before it falls, which
   enables (later, out of scope) bracing, leaning, or N-step recovery.
2. **Height/pose coupling handled for free.** Because omega_0 uses measured z_CoM,
   loop 2's thrust, landing compression, and any torso lean automatically rescale the
   capture distance. Raibert's fixed-h formula mis-scales the velocity term whenever
   the CoM height deviates from nominal — e.g., stepping long after every crouch.
3. **Consistency with the CoM reference rule.** Combined with sec. 3 (whole-body CoM,
   never hip), loops 1 and 2 measure the same object the template models. This removes
   the loop-1/loop-3 coupling pathology: torso lean shifts the real CoM, and the step
   target follows the *real* CoM instead of chasing a stale hip-based estimate.
4. **Less to tune.** Raibert's loop has k_v and k_p plus the neutral-point offset —
   three interacting gains, each with no ground truth. The capture point has **zero
   free parameters** in the formula itself; tuning moves entirely to where it belongs:
   the reachability clamp, gait timing, and the IK that realizes the placement.
5. **Model match is near-exact here.** LIPM assumes constant-height point mass on a
   massless leg. The stick-body design (80–90% mass at core, low-mass limbs) was chosen
   precisely to make that assumption true — so this formula is *more* accurate on this
   character than on most robots it was designed for. Residual mismatch (z varies
   during stance) is second-order and absorbed by per-frame recomputation.

### 5.5 What it does NOT solve

- It is still a **one-step, instantaneous** criterion: it assumes the step lands now,
  at the commanded point, with a pin-joint contact. Swing-leg dynamics, foot size, and
  contact compliance are outside it — the swing profile and IK (sec. 4, loop 1) must
  still deliver the placement physically.
- It says nothing about **gait rhythm** — when to step is still the gait clock's job
  (sec. 6); the capture point only says where.
- No attitude control (loop 3's job) and no energy control (loop 2's job). It replaces
  exactly one component: the placement rule inside loop 1.
- The 3D generalization (step width, lateral capture point, N-step capturability)
  exists but is unnecessary in 2D — the sagittal plane has no lateral fall mode.

### 5.6 Implementation notes

- Recompute every frame during swing (state changes; don't latch at swing start, or
  mid-swing shoves are ignored). Re-latching at a fixed commitment point late in swing
  is an option for footstep stability — tune by feel.
- v_CoM must be the EMA-filtered velocity (sec. 3): raw XPBD post-projection velocities
  are noisy, and the capture point multiplies that noise by 1/omega_0 (~0.3–0.5 s),
  turning measurement jitter directly into step-placement jitter.
- Reachability clamp from leg geometry: max hip-to-foot distance at full extension,
  minus a safety margin for the swing profile's planting posture. Log clamp events —
  frequency of clamping is the single best health metric for the whole balance system.

### 5.7 Angular-momentum extension (deferred upgrade, know the tripwire)

Plain LIPM/capture point assumes **zero body angular momentum** — a point mass that
can't spin. A shove that visibly rotates the torso breaks this: the character can
plant the foot at x_capture and still rotate right over it. The research lineage that
fixes this: ALIPM (Komura/Kajita, adds an L term to the pendulum dynamics), Pratt 2008
(capture point with angular momentum for push recovery), CMP — Centroidal Moment Pivot
(Koolen/Tedrake: the ground point where net contact force incl. spin torque has zero
moment; generalizes ZMP, collapses to it when L = 0), and DCM (Englsberger/Ott — the
capture-point formula reinterpreted as a state variable with principled gait timing).

**Why not now:** the stick-body design (core-heavy, stiff damped spine, weightless
arms) keeps L small and fast-decaying, so all of the above collapse back to the plain
capture point already implemented here. The upgrade buys nothing while the body holds
the template.

**Tripwire symptom:** the character survives non-spinning shoves but **topples over a
correctly-placed foot after shoves that visibly spin the torso** (feet in the right
place, body rotating anyway). That is unmodeled angular momentum.

**Graduated fix path when the tripwire fires:**
1. Cheapest first: strengthen loop 3 authority / spine damping to kill L at the source
   (body-design philosophy — keep the template valid).
2. The 3-line L-correction: measure centroidal angular momentum about the CoM — in 2D
   a scalar, one cross-product sum over particles:
   ```
   L = sum( m_i * cross(p_i - CoM, v_i) )          # scalar in 2D (z-component)
   x_capture += k_L * L / (m_total * z_CoM * omega_0)   # k_L starts at 1, tune by feel
   ```
   (Form follows ALIPM/Pratt 2008; sign convention: verify against a spinning-shove
   test — positive L in the fall direction should lengthen the step.) Still
   template-class math, no model inversion, fully compatible with the
   compliance-parameter actuation.
3. Only if principled step *timing* is ever needed (variable-speed gaits, multi-step
   push recovery): DCM-based scheduling replacing the contact-event gait clock — a
   scope expansion, not a bug fix.

## 6. Gait clock and phase gating

- Phases: stance / late-stance thrust / flight / swing, detected from foot-contact
  events on sole particles (2D, two feet: trivial).
- Loops act only in their phases: loop 1 in swing, loop 2 in stance/thrust, loop 3
  mainly in flight. **Ungated continuous loops interfere** (loop 3 hip torque fighting
  loop 2 thrust mid-stance; foot placement correcting while planted) — a primary
  source of weird-walking behavior.
- The same clock can phase-lock arm counter-swing (deferred; arms currently decorative).

## 7. Actuation & parameter tuning

- Constraint params are hand-tuned feel knobs: seed by heuristic (stiffness scaled
  with segment mass), binary-search the stability boundary (everything very stiff;
  back off toward feel), then tune two perceptual axes: **snappiness** and
  **authority ceiling** (where physics starts winning over the controller — the
  TABS<->Stick Fight dial).
- Analytic compliance calibration was explored and **rejected**: multiple constraints
  per particle, unilateral contacts, and substep/iteration artifacts make exact
  effective stiffness uncomputable without re-running the solver inside the gain
  calculator. Feedback makes exactness unnecessary; monotonicity suffices.
- Friction is load-bearing: verify no foot creep during stance push-off (slope test).
- Mass-carrying range: fixed tuned params + feedback cover moderate loads gracefully
  (sag -> struggle -> collapse, a monotone, funny, human progression). Design the load
  budget (weapons/props weightless in current scope) and only add the weld mass-ratio
  nudge if heavy carries ever enter scope.

## 8. Staged tuning procedure

Tune one loop at a time; coupled loops tuned simultaneously are near-impossible.

1. **Hop in place** — freeze loops 1 & 3, tune loop 2. Pass: 20 hops, apex within ±5%,
   no drift, no visible vibration.
2. **Walking, torso locked upright** — freeze loop 3 (high spine stiffness), tune loop 1
   at slow forward speed. Pass: regular steps, tracks speed, no wandering.
3. **Unlock loop 3 at low authority** — reduce spine stiffness to intended value, small
   loop-3 gains. Pass: turning it on does not reintroduce sway. If it does: suspect
   CoM-consistency first, gain magnitude second.
4. **Shove tests at every stage** — push mid-stance and mid-flight. Stage isn't done
   until it recovers from a firm shove. Also measures the authority envelope:
   peak CoM displacement vs max step reach.

### Symptom -> cause cheatsheet

| Symptom | Likely cause |
|---|---|
| Steps too long, lurching then hopping backward | capture-point velocity term too high, or CoM measurement lagging |
| Tiny shuffling steps, slow topple | loop-1 gain too low / placement saturating near current foot |
| Bounce height drifts | apex estimate contaminated by flight noise; low-pass it |
| Torso pumps at step frequency | loop 3 reacting to per-step kicks with too little damping (raise zeta to ~1) or noisy omega measurement |
| Oscillation near step frequency | resonance between gait and spine response; move spine natural frequency well below gait frequency (torso = slow damped pendulum) |
| Toe catches ground during swing | missing/weak swing retraction profile |
| Feet slide during push-off | insufficient contact friction |
| Jitter at full leg extension | rigid bones vs extension motor vs contact — stiffest solver combination; add tiny bone compliance |
| Works unloaded, sags/stumbles loaded | authority ceiling; expected. Check mass budget / weld nudge |

## 9. Upper body / arms — general framework (for reference; mostly deferred)

Three-layer stack (hierarchical control with task priority):

1. **Include, don't ignore** (free): whole-body CoM already contains arms/head, so
   balance auto-compensates for any internal motion. Done in current scope.
2. **Posture regulation** (cheap): weak, critically damped PD to rest pose per segment,
   plus phase-locked counter-swing feedforward (left arm forward with right leg) to
   partially cancel the legs' reaction kick. Deferred until walking is solid.
3. **Momentum allocation** (moderate, optional): in flight, L_total about CoM is
   conserved; allocate desired torso-rate correction across leg (primary, largest
   inertia), arms (secondary), head (trim) — the diver/cat mechanism. Build only when
   flight shoves expose the hip's authority limit.

The fully general solution (centroidal momentum / whole-body QP, as in Atlas/ANYmal)
requires an exact dynamics model and torque outputs — incompatible with an XPBD
black-box solver and compliance-parameterized actuators. Rejected for the same reasons
as ZMP tracking, MPC/trajectory optimization. RL handles model-free soft bodies but
replaces the whole architecture and needs a training pipeline — parked as a later
alternative only if template control plateaus.

## 10. Lineage rationale (why this design)

- Raibert's three loops are a **regulator, not a gait generator**: in the original work
  they trim a schedule/scripted nominal (spring-leg resonance for the hopper,
  hand-authored swing schedule for the biped). Feedback corrects deviations from a
  gait; it cannot invent the gait. Hence: phase gating + scripted swing profile are
  required components, not polish.
- Games (TABS, Stick Fight, Overgrowth) use FSM/pose-target PD motors, not Raibert —
  robustness from body design (core-heavy, wide stance), damping, and forgiving
  physics. Stick Fight's 2D stepping rule is a discretized capture point.
- This design = union: robotics control structure (capture point + apex feedback +
  attitude PD) on a game-style body with game-style tuning discipline.
- Field lineage for context: Raibert 1986 -> ZMP (Vukobratovic 1968+; ASIMO) ->
  LIPM/capture point (Kajita 2001, Pratt 2006) -> centroidal momentum / WBC / MPC
  (2009+) -> RL (2018+). The middle shelf all demands accurate models; capture point
  is the one advanced upgrade that stays inside template control. Adopted.

## 11. Scope summary (settled)

**In:**
- Stick-like upper body, core-heavy mass budget, low-mass (not zero) arms
- Whole-body computed CoM (+ filtered velocity, height) as the balance reference for
  loops 1–2; world-frame torso angle for loop 3
- Capture-point loop 1 with reachability clamp, 2-bone IK, scripted swing profile
- Loop 2 as virtual-leg extension with apex-height feedback
- Loop 3 as damped PD on absolute torso pitch, torque-inverted into constraint targets
- Foot-contact gait clock with per-phase loop gating
- Staged tuning procedure and verification checks

**Out / deferred:**
- Angular-momentum capture-point correction (ALIPM/CMP/DCM family) — deferred with a
  defined tripwire symptom and 3-line upgrade path (sec. 5.7)
- Carried heavy objects (weld pipeline + mass-ratio nudge)
- Slow one-scalar gain adapter
- Arm counter-swing keyframes, layer-3 momentum allocation
- CoM-aware torso lean when step saturates; N-step recovery
- Heavy weapon physics (fake weight visually if ever needed)
