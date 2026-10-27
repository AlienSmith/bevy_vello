# Walk Improvement Plan — from the 9-second failure narrative (2026-10-27)

Goal stated by the user: with the current settings the character does
**start slow → walk → pause → walk backwards → fall, all within ~9 s**
(it does walk cleanly for 2–5 s — a usable core). **Success = walks the
whole 10 s script window under these settings.** The belief (shared): if it
survives this adversarial point, it survives most viable settings.

Companion docs: `dcm-prediction-math.md` (law + parameters),
`continues_prediction.md` (per-frame steering + safety filters),
`event-gait-phaseA-handoff.md` (build state + A/B history). Pipeline:
`tools/physics-qa/run_walk.sh` now drives `visual_arm_pd` (`WIN_HIDDEN=1`,
GPU collision chain; Linux twin of `run_visual.ps1`).

---

## 1. The reproduced trace (tag `diag`, defaults 1e-8/70, script 0.5s wait +
## 10s forward move + 1s wait)

| t | hip x | hip vx | reading |
|---|---|---|---|
| 0.5–2.0 | 172→195 | 0→90 | **slow start**: creep, some backward steps |
| 2.5–5.5 | 248→567 | ~120 | **clean walk** at command speed |
| 6.0–6.5 | 586→590 | 55→−5 | **pause** |
| 7.0–11.5 | 565→173 | −33…+73 oscillating | **walks backwards** toward the start point, legs still alternating, gates FAIL (NO_ADVANCE on late swings) |
| end | — | — | settles/falls at origin — hip caught between lagging plants |

Full-window gates: swing FAIL ~t9 (`NO_ADVANCE, NO_OVERTAKE, NO_LEAD_GAIN`),
slide 45.7 px. Late-swing failures sit right at the t≈10.5 s non-determinism
edge (handoff §0b) — treat verdicts there as noisy, use repeats.

## 2. The causal chain — why a *recomputing* controller walks backwards

The recompute (measurement + `plan_target_x` every tick) only fixes
*tracking* lag. The committed law's **plan bias** at cruise is zero lead
(`x_foot = xi − v_des/(T_conv·ω₀)` evaluated at liftoff ⇒ lands under the CoM;
the lead cancels because `v_des/ω₀ ≈ v·T_fly` at `ω₀T ≈ 1`). Chain:

1. **No designed lead** ⇒ every stance is gas/coast; hip never braked
   (tilt-forward while walking; `xi` always ahead of the plant).
2. **Fulfillment lag**: soft physics lands the foot *behind* even that
   (slide 46–86 px; reach clamp saturates late). Feet systematically lag.
3. **The deadbeat backward solve converts lag into reversed velocity.**
   At touchdown, `v_support = (delta_x + kv·v_des)/(Ts/2 + kv)` with
   `delta_x = plant_x − hip_x` (the *achieved* offset). A lagged plant is
   behind the hip: `delta_x < −kv·v_des ≈ −14 px` ⇒ **`v_support < 0`
   outright**, and `beta = 1` writes it into the hip velocity unblended.
   One lagged landing ≈ tens of px/s of commanded reversal; the next tick
   the *plan itself* follows the negative `v` (signed targets) ⇒ the
   resolver walks the hip back toward the lagging foot while the input
   still says forward. That is the pause → walk-backward oscillation
   (`vx` swinging −33…+73 at the ~step rate).
4. **Fall** = attitude/pendulum finally losing the lagged-feet, head-forward
   geometry (plus the late-swing `NO_ADVANCE` collapses: the foot lands flat
   because the moving endpoint is still being chased when the timer expires).

Key line: the backward solve is *correct* control only while the plant error
is noise. Under the zero-lead law the error is **systematic**, and a deadbeat
inversion of a systematic error is an oscillator. Fixes 1 and 3 are a matched
pair: give the plan a real lead, *and* stop letting unfulfilled plans rewrite
the velocity sign.

### 2b. The startup bootstrap gap (user derivation, 2026-10-27) — promotes Stage 4

At `v = 0` the capture point is blind: `xi = x + 0/ω₀ = x`, so the placement
law emits `x_foot = x − v_des/ω₀ ≈ x − 37 px` — **behind the hip**. For the
rear foot that is off the reach clamp; for the lead foot it is a *backward
step*. This is the true root of the residual `TD_FLOOR=0.2` startup drift
(see Stage 1 RESULTS): the body tips back (θ→−0.47 rad) because the plan keeps
commanding steps behind it while it stands still. Three facts resolve it:

1. **A step cannot create velocity — only push-off can.** The CoM accelerates
   only via friction at *planted* feet (`v_dot = ω₀²(x − x_foot)`); swinging a
   light leg moves nothing. Foot placement only sets the *sign* of the stance
   acceleration.
2. **The standing straddle already contains the gas stance.** Rear foot behind
   hip ⇒ `x − x_foot > 0` ⇒ any forward velocity the hip has gets accelerated.
   The DCM just can't *see* this at `v = 0` (zero lead = zero information).
3. **Therefore startup = freeze the placement math, drive the hip, let the DCM
   boot itself.** While `|v| < v_boot`: (a) never target a swing foot behind
   its pivot — floor it at `pivot + toe_len` (the gas guard); (b) suppress
   timer liftoff (step only on contact events). The hip velocity ramp under
   `accel` + planted-foot friction raises `v`, which grows `xi = x + v/ω₀`,
   and within a step or two the ordinary margin trigger fires and the DCM
   machinery resumes in its valid regime. No new math — a **phase gate**.

Symmetric stop note: braking is legitimate because `v > 0` gives `xi` real
information (the `stopping` capture branch). The bootstrap guard is the
`v → 0` mirror of exactly the failure Stage 1's floor patched from the other
side. **Implement this before Stage 2/3.**

## 3. The plan — stages, each small, flagged, measurable

Every stage: implement behind env flag → 3 repeat runs of `diag`-settings →
worst-of-3 on the metrics → keep only if strictly better. Never stack two
behavior changes in one commit.

**Baseline-to-beat (current settings, worst-of-3):**
- travel through move window positive and monotonically-ish increasing;
- **no interval ≥ 0.5 s with median vx < 0** (kills walk-backward);
- ≥ 8 consecutive swings with touchdown lead `(foot − hip) > 0` at landing
  (kills "never ahead");
- swing gate NO_ADVANCE count = 0 over t ∈ [2, 10.5];
- stays up for the full 11.5 s; stop settle within ~1.5 s of script end.

### Stage 1 RESULTS (2026-10-27, shipped as `TD_FLOOR`, default-off, kept)

Implemented as `td_solve_floor` (config field, env `TD_FLOOR`, default −1 = off).
A plan-vs-achieved *tolerance* gate was coded then **removed**: at the landing
tick the kinematic plant equals the same-tick plan by construction, so that
check is vacuous; the physical lag instead enters through `delta` directly,
because `hip_pos.x` is anchored to the real P3. So the floor is the only
non-vacuous half of Stage 1.

Measured on the `diag` settings with **physical Δx velocity** (NOT sampled
`tP3vx`, which reads each step's surge spike as forward and under-counts
reversal — that metric misled the first read):

| run | phys-forward | phys-BACKWARD | net Δx | cruise hold |
|---|---|---|---|---|
| `TD_FLOOR` off | 4.9 s | 4.2 s | **0** (ends where it started) | collapses |
| `TD_FLOOR=0.2` | 5.4 s | 3.8 s | **+558** | pinned 120 after recovery |

**Verdict: a real but PARTIAL win — keep the knob.** Net travel 0→558 px and the
hip holds `v_des=120` from t≈6 s to the end (the walk-backwards-forever
oscillation is gone). But ~3.8 s of backward motion REMAINS, confined to the
startup transient t≈0.8–5.5 s: the body tips back (θ reaches −0.47 rad ≈ 27° at
t≈4.1 — the visible "fall"), the physical P3 hip is dragged behind while the
kinematic plan marches on, then it recovers and walks forward cleanly.

**Attribution:** the residual backward phase is NOT the velocity sign inversion
(the floor fixed that) — it is the **startup zero-lead placement bug**: the law
commands a foot ≈ `−v_des/ω₀ ≈ −37 px` behind at `v≈0` ⇒ backward first steps ⇒
the body sails back. That is **Stage 4's gas guard**, promoted to the next
priority; the floor can't reach it because it gates *velocity*, not *placement*.
122/122 tests green; flag-off bit-identical.

### Stage 1 — stop the bleeding: touchdown-solve fulfillment gate
*(attacks pause/walk-backward + fall; one function, one flag)*
The inversion trusts `delta_x` even when physics badly missed the plan.
Gate it: only apply the full support-velocity rewrite when the achieved plant
is *near the planned target* (|achieved − planned| ≤ tol, tol a config knob,
e.g. 0.25·x_max); otherwise blend with beta→0 (or skip, leaving `v` to the
cruise dynamics). Optionally also clamp: `v_support` may brake but never flip
sign against `v_des` — the plan should reverse direction through *intent*,
not through an error inversion.
- Flag: `TD_SOLVE_TOL` (px; large = old behavior).
- Expectation: pause/oscillation shrinks immediately; character may still
  lag but won't walk backwards — `v_des`-seeking instead. This also protects
  every later stage from the ratchet→reversal→oscillation loop, so it goes
  **first**.
- Measure: oscillation metric — count of sign flips of `vx` in [2,10];
  median `vx` in [6,10] should rise toward v_des.

### Stage 2 — terminal freeze + liftoff-time sanity
*(attacks NO_ADVANCE late swings; from `continues_prediction.md` filter A)*
Latch the swing endpoint when `phase_timer < 0.15·T_swing`: stop re-planning
in the last 15 % of flight so the leg executes a fixed, reachable target
instead of chasing a laser pointer at the moment it cannot still move.
- Flags: `SWING_FREEZE=0.15` (fraction; 0 = off).
- Expectation: swing gates NO_ADVANCE/NO_OVERTAKE flip on the ~9 s swings;
  realized landings stop overshooting into the slide.
- Measure: swing gate verdicts; touchdown lead variance drops.

### Stage 3 — the missing lead: landing-state placement law
*(attacks the root plan bias; the matched pair, re-landed on top of 1–2)*
Now that fulfillment is gated and targets are frozen, evaluate the placement
law at the **predicted touchdown state** so the plan *requests* a leading
foot. Two variants, one flag each, A/B against each other:
- `MPC_EXPON`: `x_foot = p + (xi − p)·e^{ω₀·t_flight} − v_des/(T_conv·ω₀)`
  (the reverted exponential; single evaluation per tick, exponent cap 2.0 —
  already proven to pin vx ≈ 120 in the old A/B; unit-tested once).
- `MPC_LEAD` (user's reactive model): `x_foot = hip + clamp(gain·v·T_stance,
  ±x_max)` with `gain ≈ 1` ⇒ lead ≈ one stride; zero at standstill by
  construction (no backward startup steps); no ω₀ trust.
Plus the §3 margin trigger (`MPC_TRIGGER`) enabled *with* whichever wins —
it is the matched half (trigger early enough to plant the longer lead).
- Expectation: touchdown lead `(foot − hip) > 0` on most steps; tilt-angle
  at stop small enough that the stop-capture (stage 4) can save it; and with
  stage 1 in place the ratchet is braked instead of inverted.
- Measure: the ≥8-consecutive-positive-lead criterion; speed pinned near 120
  without clamp saturation (`capture_saturated` count → rare).

### Stage 4 — start and stop transitions
*(attacks slow start + final fall; `continues_prediction.md` filter C + §4.1)*
- **Gas guard**: while `|v| < |v_des|`, floor the plant target at
  `pivot + toe_len` (never *behind* the support foot); the start may only
  gas, never brake. Kills the backward first steps and the 2 s crawl.
- **Stop capture**: on input→0, the current capture-point braking path
  (`stopping` branch) plus a velocity deadband (<5 px/s ⇒ plant exactly
  under hip, DCM term zeroed) so the settle doesn't oscillate; straddle-pose
  normalization is deferred (handoff Phase A3) but the *fall* must be gone
  before it matters.
- Measure: first sustained forward motion < 1.2 s; no fall in the post-move
  wait; settle in ≤ 1.5 s.

### Stage 5 — hold the line: slide + parameter robustness
With 1–4 in, the character walks the full script; then attack the standing
failure sources: stance drift 46 px (friction/compliance interplay, plant
re-anchor tolerance), then re-run the **original adversarial sweep** (a few
TARGET_SPEED/STEP_HEIGHT/STANCE points) proving the walk is now settings-
insensitive. Log clamp-saturation count as the health metric (plan §10).

## 4. Standing caveats

- **Cross-platform nondeterminism**: visual twin gives 3 distinct md5s over
  5 Linux runs, diverging at t≈10.5 s (GPU narrow phase). All verdicts are
  worst-of-3; do not read tail differences as signal.
- The **backward solve is load-bearing** for `test_hip_stays_between_feet…`
  and related tests — stage 1 changes need the invariant tests re-derived
  against the gated version (a landed-with-error plant is *allowed* now, so
  "hip stays between feet" tests may need tolerance updates).
- Keep `MPC_*` flags default-off; committed baseline must stay bit-identical
  (`mpc_off` md5 check) until a winner graduates.
- Every stage lands with: unit test for the new pure function + telemetry
  counter + one-line entry in the handoff doc.

## 5. Immediate next action

Stage 1 first (`TD_SOLVE_TOL` gate on `compute_support_velocity` application
in `step_lower_body` §6 + sign-flip clamp): it is ~15 lines, it converts the
worst symptom (walk-backward oscillation) into a benign one (lag), and it
makes every later stage measurable without the oscillator corrupting A/Bs.
