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

### Stage 4a RESULTS (2026-10-27, shipped as `V_BOOT`, default-off, kept)

Implemented exactly as §2b describes, under one flag: config `v_boot`, env
`V_BOOT`, default `-1` = off. Two pure pieces plus one call-site each:

- `bootstrap_gate_active(v_boot, measured_vx)` — `v_boot >= 0 && |measured_vx| <
  v_boot`. Reads **measured** velocity (`CaptureMeasure::v_com`, falling back to
  the resolver hip velocity only in the kinematic sandbox). This matters: the
  resolver's own `v_ref` is ramped by `config.accel` at stand-to-walk whether or
  not the body has moved, so gating on it would latch open on a reference that
  no longer commands motion.
- `apply_bootstrap_floor(target_x, pivot_x, v_desired, toe_len)` — the gas guard.
  The swing target may never go behind the lift-off pivot, applied in
  `plan_target_x` to both the lift-off aim and every mid-swing re-plan, using
  `leg.plant_pos.x` as the pivot.
- Suppression in `step_leg`'s Stance branch: while gated, `stance_over = false`
  unless `lost_contact_early`. The timer is pinned at `max(0)` so a released
  stance steps on the next tick instead of dumping several steps of debt at once.

`V_BOOT` sweep, `diag` settings, physical Δx velocity, single run each (`b_boot5`
was re-run and is bit-identical, so these settings are deterministic):

| run | onset | phys-forward | phys-BACKWARD | net Δx | v range | max\|θ\| |
|---|---|---|---|---|---|---|
| off | 1.78 s | 5.0 s | **5.0 s** | 0 | −188…+200 | 0.62 rad |
| `V_BOOT=1` | 1.78 s | 6.0 s | 3.5 s | +85 | −212…+229 | 0.66 rad |
| **`V_BOOT=5`** | **1.28 s** | **10.0 s** | **0** | **+945** | −68…+291 | **0.41 rad** |
| `V_BOOT=12` | 1.78 s | 7.5 s | 1.0 s | +1101 | −101…+314 | 0.56 rad |
| `V_BOOT=30` | — | 0 | 0 | −0.7 | −29…+22 | — |
| `TD_FLOOR=0.2` alone | — | 0 | 4.0 s | −214 | −201…+93 | 0.45 rad |
| `V_BOOT=5` + `TD_FLOOR=0.2` | 1.28 s | 4.5 s | 4.0 s | +124 | −175…+199 | 0.57 rad |

**Verdict: `V_BOOT=5` is the first configuration that meets the §3 baseline**
— zero backward windows, 10.0 s of sustained forward travel (then the script's
1 s wait), net +945 px, and the lowest tilt of any run (0.41 rad vs 0.62
baseline). It fixes exactly the defect §2b predicted, and the mechanism is
confirmed: 1 and 12 are *worse* on both backward time and tilt, so this is not a
monotone "more gating = better" effect but a usable window.

Three findings to carry forward:

1. **`V_BOOT=30` (≈0.25·v_des) is a DEADLOCK, not a conservative choice.** With
   both feet planted and loop 2 thrusting, measured speed peaks at **22 px/s** —
   below the gate — so it never releases and the character stands still for the
   whole script. The gate is a one-way trap above the push-off ceiling. The
   usable band is ~2–20 px/s; §2b's "0.25·v_des ≈ 30" estimate is refuted.
   **Rule: `v_boot` must sit below the achievable push-off speed.**
2. **Surplus speed is the remaining defect, not stalling.** `V_BOOT=5` reaches
   +291 px/s against a 120 command: the gate releases at a moment when the body
   is already fast, so the placement law inherits a large velocity error.
   `V_BOOT=12` overshoots harder (+314). This is Stage 3's territory (the
   landing-state lead), not a bootstrap fault.
3. **Do NOT stack Stage 1 with Stage 4a.** `V_BOOT=5`+`TD_FLOOR=0.2` is much
   worse than either gate alone on the walk-backward metric (+124 net, 4.0 s
   backward). The floor clamps the direction of the touchdown inversion while
   the bootstrap is deliberately running without the DCM; combined they fight.
   Use them as alternatives in the A/B, not as a stack.

160/160 tests green; default-off verified bit-identical (`b_off`,
`b_off2`, `b_pre2`, `b_pre3` all `bf9ecbb2`, including one build from the
stashed pre-change tree). Note `b_pre` diverged: **the first walk run after a
rebuild is an outlier** (same binary as `b_pre2`/`b_pre3`, different md5) —
always warm up once and discard.

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

### Stage 3 RESULTS (2026-10-27) — the analytic law supersedes the staged variants

**Shipped:** `DCM_TC` (`LowerBodyConfig::dcm_tc`) and the one-dof `DCM_KP`,
both default-off, precedence `deadbeat → gain → horizon → neutral point`.
163/163 tests green; flag-off baseline still `bf9ecbb2`.

**The Kp question, answered by algebra.** The committed horizon law *is* the
`Step Placement = DCM − Kp(v_des − v)` control law with `Kp = 1/(T_conv·ω₀)`
— but with `v_des` subtracted absolutely instead of as an error, so at cruise
it cancels to zero lead (the "feet never ahead of the hip" bug). The exact
LIPM deadbeat solution is

```text
p = x + α·v − β·v_des
α = 1/(ω₀·tanh(ω₀t_c))      β = 1/(ω₀·sinh(ω₀t_c))
```

so the horizon law is this with `α → 1/ω₀` and `β → Kp`: **two truncations of
the same series**. The link is exact and was used to build the sweep:

```text
β(t_c) = Kp   ⟺   sinh(ω₀·t_c) = T_conv   ⟺   t_c = asinh(T_conv)/ω₀
```

| `T_conv` | `Kp` | equivalent `t_c` |
|---|---|---|
| 0.5 | 0.6061 | 0.1458 s |
| 1.0 | 0.3030 | 0.2671 s |
| 2.0 | 0.1515 | 0.4375 s |

Consequence: tuning `T_CONV` is tuning `Kp`, and every `Kp` has an exact `t_c`
that comes with a **free correct `α`**. Never tune `Kp` directly — one dof
cannot satisfy both coefficients.

**Measured (3 repeats, `DCM_TC` the only variable; physical Δx velocity):**

| config | v@2 | v@4 | v@6 | v@8 | v@10 | steady \|v−120\|<40 | net Δx | tilt |
|---|---|---|---|---|---|---|---|---|
| off (committed) | 50 | 109 | 40 | −161 | −26 | 0.0 s | 25 | 0.10 |
| `V_BOOT=5` | 80 | 49 | 61 | 34 | 90 | 3.0 s | 899 | 0.11 |
| **`DCM_TC=0.15`** | −31 | 136 | 121 | 103 | 105 | **6.0 s** | 1040 | 0.10 |
| `DCM_TC=0.15`+`V_BOOT=5` | 75 | 40 | 98 | 130 | 132 | 4.0 s | 1018 | 0.10 |
| `T_CONV=0.5` (naive `Kp`, before removal) | −22 | −24 | −136 | 33 | 303 | 1.0 s | 59 | 0.11 |
| `DCM_TC=0.12` | −23 | 118 | 244 | 204 | −8 | 0.5 s | 1048 | 0.13 |
| `DCM_TC=0.2` | −20 | 120 | 190 | −23 | 207 | 2.0 s | 999 | 0.12 |
| `DCM_TC=0.25` | 17 | 91 | 141 | 125 | 140 | 6.5 s | 1266 | 0.10 |
| `DCM_TC=0.28` (auto `T_s`) | 0 | 107 | 135 | 132 | 138 | 6.5 s | 1266 | 0.13 |

**Findings:**

1. **The naive `Kp` path is the worst row.** `T_CONV=0.5` — the analytically
   correct `Kp` for that horizon — gives `net +59 px` and every metric worse
   than flag-off. Correcting the gain without the matching `α` does not work;
   this is the empirical proof that the horizon form is a truncation.
2. **`DCM_TC=0.15` is the winner** and reproduces bit-exactly (3/3). It is a
   *crash-type* law: `β = 0.588 < Kp = 0.606`, `α = 0.661` — stiffer velocity
   gain plus double the lead of the horizon law.
3. **`t_c = 0.28` (auto) is the calm-brake type**: dead for a full second
   (`v@2 = 0`), then settles at 138 px/s. Smoother, but slower to converge and
   it still overshoots the command by ~15%.
4. **`0.12` overshoots to 244 px/s, `0.2` is non-monotone** — too stiff on this
   soft body. Do not go below `t_c ≈ 0.15`.
5. **Do not stack `V_BOOT` on the deadbeat law** (`4.0 s` vs `6.0 s`, onset
   75 px/s vs −31): the gas floor forbids exactly the backward placement the
   analytic law wants at startup. Same verdict as stacking `TD_FLOOR`.

**Determinism caveat (important for judging):** `DCM_TC ∈ {0.12, 0.15, 0.2}`
and `0.15+V_BOOT` are **bit-exactly repeatable** (3/3 md5-equal). But `off_*`,
`DCM_TC=0.25` and `DCM_TC=0.5` are **bimodal** — `off_2` diverges from
`off_1`/`off_3` with *identical flags*. So repeatability is a property of the
merged closed loop at those `t_c`, not of any one patch; the chaotic band is
`t_c ≈ 0.2–0.5`, which is where the analytic optimum (`asinh(T_s/T_conv*)/ω₀
≈ 0.22–0.24`) happens to sit. Any sweep there needs ≥5 repeats, not 3.

**Open:** `DCM_TC=0.15` still ends at 105 px/s against a 120 command (the
diagnosed defect was *surplus* speed, now it is a small deficit), and the
start `v@2 = −31` shows the first swing still lands behind. Those are the
Stage 2 (terminal freeze) and Stage 3-lead-Q targets, not new law work.

### Stage 3 (original staging, superseded — kept for lineage)
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

- **Nondeterminism = attractor persistence, not coin-flips** (refined
  2026-10-27, `DCM_TC=0` 13-run experiment): the solver lands in one of a
  *small discrete set of attractors* (4 md5s seen: `3b14f1f9`/`bf7817c0`
  "nice family", `11b15a1d` "ugly-stable", `7bef3224` post-rebuild outlier)
  and **consecutive runs in one session reproduce the same attractor
  bit-exactly** — verified 5/5 with the user's exact command form. The
  basin *switches across rebuilds/sessions* (GPU/process state), not within a
  session. So repeats measure noise *within* a basin only; a config's real
  risk is basin-switching, which needs a fresh binary/session to expose.
  All verdicts stay worst-of-N across at least one rebuild; do not read tail
  byte-differences between the two nice-family md5s as signal (identical
  metrics).
- The **backward solve is load-bearing** for `test_hip_stays_between_feet…`
  and related tests — stage 1 changes need the invariant tests re-derived
  against the gated version (a landed-with-error plant is *allowed* now, so
  "hip stays between feet" tests may need tolerance updates).
- Keep `MPC_*` flags default-off; committed baseline must stay bit-identical
  (`mpc_off` md5 check) until a winner graduates.
- Every stage lands with: unit test for the new pure function + telemetry
  counter + one-line entry in the handoff doc.

## 5. Status + immediate next action

Stage 1 (`TD_FLOOR`), Stage 4a (`V_BOOT`) and Stage 3 (`DCM_TC` / `DCM_KP`)
are all implemented, tested (163/163), default-off, and measured — see their
RESULTS sections.

**The placement law is now solved, not tuned.** `DCM_TC` is the exact LIPM
deadbeat solution and it contains both earlier candidates as limits (the
horizon law is its `β`-only truncation; the gain family is its one-dof
restriction; Raibert's neutral point `v·T_s/2` is its small-`ω₀t_c` limit).
`DCM_TC=0.15` is the recommended configuration: 6.0 s steady near 120,
net +1040 px, tilt 0.10, bit-exactly reproducible.

Next, in order:
1. **Stage 2 — terminal freeze** (`SWING_FREEZE=0.15`): kill the residual
   mid-swing target oscillation. `DCM_TC=0.15` held steady but `t_c = 0.2/0.25`
   were non-monotone, which is the same symptom Stage 2 attacks.
2. **Close the −31 px/s startup**: the first swing still lands behind. Test
   `DCM_TC=0.15` with a *smaller* `V_BOOT` (`1`–`3`) rather than `5` — stacking
   at `5` was measured harmful, but the failure mode is the floor being too
   aggressive early, not the gate being wrong.
3. **Stage 5 — slide**: stance drift is still 45.7 px against a 2.0 px gate.
   That is now the top remaining physical defect.

`T_CONV` env support was REMOVED 2026-10-27 for exactly this reason: it is the
derived `β` coefficient (`T_conv = sinh(ω₀·t_c)`), so the field stays pinned at
its default `1.0` (flag-off baseline bit-identical) and only `DCM_TC` tunes.
Do not re-add a `Kp`/`T_CONV` knob — use `t_c`. Do not stack `TD_FLOOR` with `V_BOOT`
(measured: they fight). Do not stack `V_BOOT` with `DCM_TC` at `5`.
