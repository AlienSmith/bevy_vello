# Swing-Phase Timing Plan — Event-Triggered Stepping with DCM Targeting

Standalone plan for the gait system: an event/state-based stepping FSM (replacing any
fixed-phase clock), specifying the swing trajectory concretely — when to lift, where to
aim, how fast to move, how to adapt when the physics lags, and how to keep demand
inside what friction can actually deliver.

Lineage: this is the standard LIPM/DCM footstep-planning architecture (Kajita 2001
extrapolated CoM; Pratt 2006 capture point; Englsberger/Ott 2013-2015 DCM gait
generation; IHMC/Atlas step-timing adjustment), specialized to a 2D XPBD character.
Not an experiment — each component has a known-good published counterpart.

---

## 1. Design principles

1. **Events and states drive the gait, never timers.** Every phase transition fires on
   a measured condition. Timers exist only as pathological-case guards (min/max time
   per state), never as the primary trigger.
2. **The reference trajectory is a servo target, not a contract.** When the body lags
   the reference, re-time and re-aim (sec. 6) — never demand catch-up.
3. **Never ask for the infeasible.** The demand entering the placement law is clamped
   by a *learned* feasibility budget (sec. 4.1); event-triggered rebase (sec. 6) only
   handles what the budget could not foresee. Feedforward prevents the limit cycle;
   feedback cleans up the rest.
4. **Everything measurable is measured.** Trigger margins, swing duration, tracking
   error, achieved acceleration, slip — all derived from simulation state, all logged.
   Budgets are learned from measurement, never computed from assumed friction.

## 2. Core quantities (per frame)

```
omega_0 = sqrt(g / z_CoM)                      # z_CoM: measured CoM height
xi      = x_CoM + v_CoM / omega_0              # DCM / capture point (1D scalar)
v_des                                        # commanded speed (player input)
x_stance                                       # current stance ankle x
support region = [x_heel, x_toe] of stance foot
```

All from the measurement layer: whole-body mass-weighted CoM/vCoM computed every frame
over all particles, v_CoM EMA-filtered (2-4 frame time constant). Filtering matters
doubly here: the placement law multiplies velocity noise by ~1/omega_0.

## 3. Step trigger (stance -> swing)

**Primary trigger — DCM support margin:**

```
trigger when   xi >= x_toe - margin
```

The margin provides lead time for the laggy swing leg. Set it from measured swing
duration (standard push-recovery practice):

```
margin ~= (xi_toe - xi) traveled during one T_swing
        ~= v * T_swing_measured        (first estimate; refine empirically)
```

**Emergency trigger:** `x_CoM >= x_ankle` (CoM passed the stance ankle) — lift
immediately with the fastest profile; this is the late version of the trigger and
should be logged as a tuning failure if it fires during normal walking.

**Guards (all transitions):**
- min-stance-time (~80-120 ms): prevents chatter after landing impacts.
- swing leg must be unloaded: its foot contact force ~ 0 before lifting (never lift a
  loaded leg).
- Hysteresis on all contact/force conditions: N consecutive frames above a start
  threshold, lower release threshold. XPBD contact flags flicker as projections push
  particles in/out of penetration; without hysteresis the FSM chatters.

## 4. Step target — velocity schedule and acceleration-based placement

**Design assumption: CoM velocity reaches v_des by the end of the convergence horizon
T_conv** (nominally one step). From the LIPM stance dynamics
`v_dot = omega_0^2 * (x_CoM - x_foot)`, the required constant acceleration and the
placement that produces it:

```
a_des  = (v_des - v_now) / T_conv
x_foot = x_CoM - a_des / omega_0^2  =  x_CoM + (v_now - v_des) / (T_conv * omega_0^2)
```

**Identity with the DCM law:** setting T_conv = 1/omega_0 (~0.3 s, human scale) makes
this exactly `x_foot = xi - v_des/omega_0` (the velocity-tracking placement law;
capture point = v_des 0 case). T_conv is therefore a tunable responsiveness knob with
a principled default: shorter = aggressive speed tracking, longer steps; longer =
softer convergence.

The schedule provides an explicit velocity reference and tracking error:

```
v_ref(t) = v_now + a_des * t          (clamped at v_des)
e(t)     = v_ref(t) - v_actual(t)     (the rebase signal, sec. 6)
```

**Touchdown prediction (fixed-point solve).** x_foot above is evaluated at the
*predicted touchdown state*, and the horizon T depends on the step length
(`T = dx / v`, sec. 5) — circular; resolve by 2-3 fixed-point passes. Using the DCM
form (with foot planted at x_f, `xi(t0+T) = x_f + (xi(t0) - x_f)*exp(omega_0*T)`):

```
dx_0 = (v - v_des)/omega_0 + v * T_nominal
repeat 3x:
    T  = dx / v                       # v = current CoM speed, floor at v_min
    dx = (xi - x_stance) * exp(omega_0 * T) + (v - v_des)/omega_0
x_t = x_stance + dx
```

Notes:
- At cruise (v = v_des) this converges to dx ~= v * T — the classical step-length
  identity, with the exponential supplying exactly the lead the diverging CoM demands.
- Clamp dx to [dx_min, dx_reach_max] (reach from leg geometry at full extension minus
  planting-posture margin). **Clamp-saturation frequency is the primary health metric
  of the whole balance system — log every clamp event.**
- Non-convergence or sustained saturation at dx_reach_max = uncapturable -> fall path
  (sec. 8).
- **Where acceleration authority lives:** during stance, horizontal CoM acceleration
  is set by where the ground-reaction line passes the CoM — small authority via ankle
  push/ZMP shift inside the current foot, mostly committed by that foot's placement.
  The high-authority handle is the in-flight swing target: it decides the next
  stance's acceleration. Rebase (sec. 6) therefore redirects the swing target first,
  ankle modulation second.

### 4.1 Learned feasibility budget — demand clamp (the friction problem)

**The problem.** Physics grants no infinite friction. The schedule can demand
`a_des > a_max` (friction cone: `a_x <= mu*(g + a_z)`, mu unknown; plus ZMP-margin and
motor limits). This bites hardest at **gait initiation**: v_now = 0 with a step change
in v_des spikes a_des far past the budget. If infeasible demand reaches the placement
law, the failure is a **limit cycle, not a transient**: long step -> foot must deliver
F > mu*N -> slips -> tracking error e -> rebase shortens step -> speed undershoots ->
demand rises -> slips again. Persistent foot sliding and hip waggle are the signature.
Pure reactive rebase cannot fix this: seeing the wall after hitting it, every frame,
is not the same as knowing where the wall is. Feedback against a saturated actuator
rings unless the saturation is made explicit (classical anti-windup lesson).

**The fix: learn the budget from measurement, clamp the demand before placement.**
No analytic mu needed — the frame history tells us what was reachable:

```
# per stance frame, while demand was high (measured, not assumed):
a_achieved = EMA of measured CoM horizontal acceleration        (~10-frame EMA)
slip       = stance foot contact particle moves horizontally
             while its contact force is above threshold (with hysteresis)

budget update:
  on slip:                          budget <- a_achieved at slip onset * 0.8   # decrease fast
  no slip for ~1 s and demand high: budget <- budget + delta                   # increase slow
  initial seed (before any data):   budget <- conservative guess (~2 m/s^2)

demand clamp (every frame, before sec. 4 placement):
  a_cmd <- clamp(a_des, -budget, +budget)
  v_cmd <- v_cmd + a_cmd * dt   (converges toward v_des whenever headroom exists)
  # placement law and velocity schedule use v_cmd / a_cmd, never raw v_des / a_des
```

(Asymmetric decrease-fast / increase-slow, congestion-control style: slipping is
expensive and informative; not slipping only weakly suggests headroom.)

**Startup behavior with the clamp:** first steps are short and feasible; the character
accelerates at the maximum rate the floor actually permits, reaching cruise over 2-4
steps — no slide, no waggle, and no analytic formula anywhere. This mirrors real gait
initiation (progressively lengthening steps) and is the same mechanism class as
traction control: demand, detect the limit, back the demand off.

**Division of labor (both halves required):**

| Mechanism | Direction | Handles |
|---|---|---|
| Learned-budget clamp (this sec.) | feedforward: never ask the infeasible | startup spike, sustained low friction, added mass |
| Event-triggered rebase (sec. 6) | feedback: re-solve from current state on schedule error | shoves, unexpected slip, budget misestimate |

Governor alone cannot react to a shove; rebase alone limit-cycles at startup. In
steady cruise both are dormant (a_des within budget, e ~ 0) — behaviorally identical
to compute-once-per-step.

**Game-feel bonus:** on a low-friction surface the budget learns down within a step or
two and the character visibly shortens its stride — players read the adaptation as
physical intelligence. Rejected alternatives: stiffer motors/more iterations (raises
motor budget, not friction budget — leg just fights the slide harder); fake physics
(velocity writes / stance-foot no-slip welds — removes the feel the system exists
for; keep only in a cheat budget if pure physics proves unplayable); globally longer
T_conv (softens the spike but makes normal walking mushy — the clamp adapts the
effective horizon only when saturated).

## 5. Swing velocity spec — the mid-swing alignment rule

Geometric rule: **at the moment the swing foot has covered half its x-distance, it is
vertically under the hip.** Derivation: foot at mean speed u, hip at speed v, alignment
at t_mid gives u * t_mid = v * t_mid, hence:

```
u_mean  = v                     # swing foot mean horizontal speed = hip speed
T_swing = dx / v                # required swing duration
```

**Profiles** over normalized phase tau = t/T in [0,1]:

Horizontal — minimum jerk (zero endpoint velocities; plant-at-speed = skid):

```
x_f(tau) = x_s + dx * (6 tau^5 - 15 tau^4 + 10 tau^3)
u(tau)   = (dx/T) * (30 tau^4 - 60 tau^3 + 30 tau^2)
u_peak   = 1.875 * dx/T ~= 1.875 * v
```

(Alternative: cycloid, u_peak = 2*dx/T, gentler endpoint jerk. Either acceptable;
min-jerk is the standard pick.)

Vertical — sine arch with clearance h_c ~= 10-15% of leg length (more on rough
terrain):

```
y_f(tau) = y_ground + h_c * sin^2(pi * tau)
vy_peak  = pi * h_c / T
```

**Motor adequacy check (quantitative):** u_peak and the IK-derived joint angular rates
are the demand on the hip/knee angular constraints. If measured tracking error during
swing is systematically large, either the motors are too soft for this gait speed or
the trigger margin is too small (not enough lead time). Measure, don't guess: log peak
tracking error per swing.

## 6. Closed-loop execution — schedule tracking, event-triggered rebase, retiming

**Never open-loop play the trajectory.** Per frame during swing:

1. **Track the velocity schedule; rebase the target only on error** (event-triggered,
   not continuous):

   ```
   e = v_ref(t) - v_actual              (sec. 4 schedule, built on v_cmd per sec. 4.1)
   if |e| > e_thresh (~10-15% of |v_cmd - v_now|, absolute floor in cm/s):
       T_rem = remaining schedule time
       x_t  <- x_CoM_now + (v_now - v_cmd) / (T_rem * omega_0^2)   # rebase from current state
   ```

   Note: the rebase uses v_cmd (governor output), never raw v_des — otherwise a rebase
   during saturation would re-inject the infeasible demand the budget clamp removed.

   Steady state: e ~ 0, no rebase ever fires — behaviorally identical to computing
   the target once at step initiation. Disturbance or actuation weakness (thrust too
   low, load added, foot slip): e crosses the threshold, one clean rebase event,
   logged. The error signal diagnoses *cause class* (actuation deficit vs external
   shove) for tuning. Apply a small rate limit to the rebased target so the IK does
   not snap.
2. **Re-time to reality**: compute progress tau from *achieved* foot position, not
   elapsed time:

   ```
   tau_progress = clamp((x_f_now - x_s) / (x_t - x_s), 0, 1)
   tau_time     = t_elapsed / T
   tau          = blend(tau_time, tau_progress)      # e.g. max, or weighted
   ```

   Lag => tau advances slower => the reference resumes from where the foot actually
   is. The profile shape is preserved; only its speed adapts. Catch-up spikes are
   structurally impossible.
3. **Servo**: IK the reference point (x_f(tau), y_f(tau)) to hip/knee angular
   constraint targets each frame; the constraint PD delivers. Tracking error is
   handled by steps 1-2, not by pushing harder.

**The rebase policy (shrink the target when behind):** a rebase from current state
over the remaining horizon automatically yields a closer feasible x_t. Rules:
- Enforce dx_min so recovery steps don't degenerate into shuffles-in-place.
- If the rebased target cannot capture the DCM (placement law saturates at reach),
  accept the short step anyway — the *next* step's trigger fires earlier with larger
  xi, naturally producing multi-step recovery. Only N consecutive failed captures
  (e.g. 2-3) escalate to the fall path.
- This makes recovery emergent: short steps after disturbance, lengthening back to
  cruise as xi stabilizes. No special recovery mode needed for moderate shoves.
- Optional (looks): freeze the target late in swing (tau > ~0.7-0.8, matching human
  step-length commitment data) except for emergency plant — a foot visibly changing
  its mind on the way down reads robotic.

## 7. Plant and touchdown events (swing -> stance)

- **Normal plant**: foot within eps of x_t (eps ~= few % of leg length) AND |vy| small
  AND tau >= ~0.9. Sole contact + normal force above threshold confirms stance
  (contact alone can be a graze; use the hysteresis rule).
- **Early plant (emergency)**: DCM within critical distance of support edge -> plant
  immediately at current foot position, wherever it is. Better a late short step than
  a perfect step that arrives after the fall.
- **Force plant (guard)**: t_swing > T_max (~1.5x nominal) -> plant at current
  position. Catches snagged legs, geometry collisions.
- On plant: latch the new support region; loop 2 (thrust) takes over on the loaded
  leg; min-stance-time guard starts.

## 8. FSM summary

```
        xi crosses toe-margin (or emergency: CoM past ankle)
STANCE --------------------------> SWING
   ^                                 |
   | sole contact + force > thresh   | foot at target (or early/force plant)
   +---------------------------------+
   
any state --unrecoverable (N failed captures / torso angle limit)--> FALL
FALL --settled (low motion)--> LIE
LIE --get-up sequence complete (CoM above feet, height ok)--> STANCE
```

(FALL/LIE/get-up per the failure-state design: ground is a statically stable regime;
get-up is a quasi-static pose sequence reusing the extension actuators.)

## 9. Tuning knobs and diagnostics

| Knob | Meaning | Start |
|---|---|---|
| T_conv | velocity-schedule convergence horizon | 1/omega_0 (~0.3 s) |
| e_thresh | rebase trigger on schedule-tracking error | 10-15% of velocity delta |
| trigger margin | lead time before DCM reaches toe | v * T_swing_measured |
| h_c | foot clearance | 10-15% leg length |
| tau blend weight | time-vs-progress trust in retiming | 0.5, tune by feel |
| eps, vy thresholds | plant acceptance | few % leg length |
| T_max, min-stance | guards | 1.5x nominal, ~100 ms |
| dx_min / dx_max | step clamps | from leg geometry + looks |
| budget seed | initial accel budget before slip data | ~2 m/s^2 (conservative) |
| budget dec/inc rates | slip response speed | x0.8 on slip / small additive per ~1 s |

**Log every transition with its triggering condition, every rebase event with its
e value, and every slip event with budget before/after** (build day one; the
difference between a 5-minute and 5-hour debug of "why is it hopping on one foot").
Health metrics: clamp-event frequency, rebase frequency (should be ~zero at cruise),
slip frequency (should be ~zero once budget converges), emergency-trigger frequency,
mean swing tracking error, steps-to-recover after standardized shoves.

## 10. Verification checklist

- [ ] Steady cruise: steps regular, no emergency triggers, no clamp events, no rebases
- [ ] Startup from v=0 with abrupt v_des: accelerates over 2-4 short steps with NO
      foot sliding and NO hip waggle (budget clamp active; verify slip count = 0)
- [ ] Speed ramp: step length scales with v_cmd via the placement law, no retuning
- [ ] Stop command (v_des -> 0): converges to capture-point standing in 1-2 steps
- [ ] Low-friction surface: slip detected within 1-2 steps, budget learns down,
      stride shortens visibly; returns to normal surface, budget recovers
- [ ] Mid-swing shove: target rebases, foot lands usefully, recovers in <= 2 steps
- [ ] Mid-stance shove: early trigger, short recovery steps, back to cruise
- [ ] Large shove: N-failed-capture escalation -> clean fall -> get-up -> walk
- [ ] Lag stress test: artificially weaken swing motors — retiming stretches the
      swing, gait stays coherent (no catch-up spikes, no FSM chatter)
- [ ] Slope: no foot creep during stance (friction adequacy)

## 11. Context summary (this doc is standalone)

Architecture assumed by this plan (from the converged design discussions):

- **Body**: 2D XPBD particle character. Stick-like upper body — 80-90% of mass at the
  core/pelvis particles, stiff critically-damped spine, low-mass (never zero-mass)
  arms. Rigid bone distance constraints. Actuation = angular constraint parameters
  only (targets/compliance/damping); ground collision + friction is the sole external
  force. No pinning, no direct body forces.
- **Measurement**: per-frame whole-body mass-weighted CoM, v_CoM (EMA-filtered),
  z_CoM, torso pitch (world-frame, from hip->shoulder vector). Balance quantities
  always use the true CoM, never the hip particle.
- **Control loops** (Raibert three-loop structure on the SLIP/LIPM template):
  loop 1 = foot placement (this doc), loop 2 = thrust via knee/ankle extension with
  apex-height feedback (virtual-leg form; bones are rigid, no prismatic leg),
  loop 3 = torso pitch PD on absolute world-frame target, damping baked into
  constraint targets (theta_target += kd*theta_dot/k), flight-phase gated.
- **Failure states**: uncapturable (sustained clamp saturation / torso angle limit)
  -> FALL (go limp, physics) -> LIE (stable regime: support = whole grounded body)
  -> get-up (quasi-static pose sequence: legs under CoM, curl over feet, extend;
  reuses loop-2 extension actuators) -> handoff back to walking when CoM is above
  the feet and height is restored.
- **Tuning discipline**: staged — hop-in-place (loop 2) -> walking with locked-upright
  torso (loop 1) -> unlock loop 3 at low authority -> shove tests at every stage.
  Constraint params are hand-tuned feel knobs (seed by mass heuristic, bound by
  solver stability); feedback absorbs actuator-model error, so monotone ballpark
  compliance suffices — analytic calibration is impossible (coupled constraint
  graph, contacts, substep artifacts) and unnecessary.
- **Deferred upgrades with tripwires**: angular-momentum capture-point correction
  (tripwire: topples over correctly-placed foot after spinning shoves; fix: scalar
  L = sum m_i * cross(p_i - CoM, v_i), x_capture += k_L * L/(m * z * omega_0));
  carried-mass weld + mass-ratio compliance nudge; arm counter-swing keyframes.

Out of scope (mature-solution extras deliberately skipped, with reasons):
- Heel-toe ZMP rotation planning (3D/real-robot smoothness; N/A to 2D particle feet)
- Multi-step MPC footstep sequences (one-step-ahead + emergent multi-step recovery
  covers game needs)
- Enhanced DCM / virtual pivot (models leg mass; stick body makes it second-order)

## 12. Closing note — this architecture is closed-form MPC

The field's path was: Raibert heuristics (1986) -> LIPM/capture point, exact one-step
placement (2001-2006) -> DCM multi-step gait generation (2013-2015) -> receding-horizon
MPC (2010s-present, Atlas/ANYmal/Cassie). This design climbed the same ladder and
stopped at the right rung: **for a 1D LIPM with a one-step horizon, the MPC optimization
has a closed-form solution — and that solution is exactly the placement law in sec. 4.**
No optimizer is needed because the "solve" is arithmetic.

| MPC component | This design's counterpart |
|---|---|
| Predictive model | LIPM exponential prediction (sec. 4 fixed-point solve) |
| Horizon | one step (T_conv / T_swing) |
| Constraints (friction cone, torque limits) | learned acceleration budget (sec. 4.1) |
| Receding-horizon re-solve | event-triggered rebase from current state (sec. 6) |
| Terminal cost (reach v_des, stay capturable) | the velocity schedule: v -> v_des by horizon end |
| Reference tracking | v_ref(t), e(t) per frame (sec. 4) |

**Boundary conditions where a real optimizer (QP/MPC solver) becomes worthwhile:**
1. Horizons beyond ~2 steps matter: multi-step recovery over stepping-stone/rough
   terrain where each placement constrains the next.
2. Constraints become coupled and numerous: friction cones + torque limits + joint
   limits + obstacles interacting, beyond what a scalar learned budget expresses.
3. The closed forms stop existing: varying CoM height, turning, 3D lateral dynamics.
4. Compute and tooling are available: QP per control cycle, cost-weight tuning,
   validation harness.

None describe a 2D stick-body game character on mostly-flat ground. The recurring theme
applies: each rung of the ladder buys generality by demanding more model accuracy, and
an XPBD solver with flickering contacts and unknown friction is the least accurately
modelable plant of all. The learned budget absorbs exactly the model error a QP with a
wrong mu would confidently plan through. If the layered design (schedule + clamp +
rebase) ever feels ad hoc in practice, the principled refactor is a budget-constrained
re-solve of the whole step at trigger time (constant-acceleration segments at
+/-budget toward v_cmd) — the same mechanism unified, still closed-form.
