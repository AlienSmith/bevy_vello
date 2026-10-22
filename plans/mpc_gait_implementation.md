# MPC Gait — Implementation Doc (branch `mpc_gait`)

Working plan for implementing `plans/swing-phase-timing.md` (event-triggered stepping
with DCM targeting) on top of the current resolver. Phases are ordered so each one is
independently buildable, unit-testable, and verifiable with the existing headless
harness (`sweep_step_height.py` / `analyze_gesture.py`). The design doc is the spec;
this doc is the work breakdown: what to touch, what to add, what to delete, how to
verify.

Ground rules (from the design + our §20–§24 findings):

- **No timer may schedule a phase transition.** Timers exist only as min/max guards.
- **Never demand the infeasible**: demand clamps (budget) before the placement law.
- **Everything measurable is measured and logged**: triggers, rebases, slips, clamps.
- Physics (collision, friction, compliance) stays exactly as settled. All changes are
  gait/resolver/measurement-layer code.
- Legacy timer-based gait logic (`GaitAuthority` ladder, `max_swing_wait`, fixed
  stance/swing scheduling, the §17–§19 interim patches) is **removed**, not extended.

Key existing code (anchors):

| Piece | Location |
|---|---|
| DCM / capture point | `resolver.rs::capture_point` (omega0 = sqrt(g/z_com)) |
| Placement / target | `resolver.rs::plan_target_x` (+ `compute_forward_target`) |
| Gait FSM per leg | `resolver.rs::step_leg` (Stance/Swing arms, timers, contact gates) |
| Swing profile | `resolver.rs::swing_profile_foot` (3-phase Hermite) |
| Backward solve | `resolver.rs::compute_support_velocity` / `compute_v_max` |
| Velocity shaping + stop-settle | `resolver.rs::step_lower_body` (§3–§4 blocks) |
| Measurement | `measure.rs` (`CaptureMeasure`, CoM/vCoM EMA, apex), `hip_pin.rs::measure_body` |
| Contact truth | `hip_pin.rs::update_ground_support` / `FootContact` (debounced) |
| Config/env | `LowerBodyConfig` (resolver.rs), `HipPinParams::from_env` (hip_pin.rs) |
| Telemetry | `headless/channels.rs` (particle cols), `headless/plugin.rs`/`sample.rs` (scalars) |
| Verification | `tools/physics-qa/analyze_gesture.py`, `sweep_step_height.py` |

---

## Phase A — Placement law: T_conv schedule + fixed-point touchdown (design §4)

**Goal.** Replace `plan_target_x`'s kinematic neutral-point branch with the
acceleration-based placement law, and add the fixed-point touchdown solve. Everything
else (FSM, profile) unchanged, so A is a pure-math swap that is easy to bisect.

**Work items**

1. `LowerBodyConfig`: add `t_conv: f32` (default `1.0/omega0` at standing height,
   ~0.3 s) and `v_min_plan: f32` (floor speed for `T = dx/v`, e.g. 10 px/s).
   Env override `T_CONV` in `HipPinParams::from_env`.
2. New free function `resolver.rs::placement_target(x_com, v_com, v_cmd, omega0, t_conv)`
   implementing `x_foot = x_com + (v_com − v_cmd)/(t_conv·omega0²)`; unit-test against
   the identity `= xi − v_cmd/omega0` when `t_conv = 1/omega0`.
3. New `resolver.rs::solve_touchdown_x(...)` — the fixed-point loop (design §4):
   `dx0 = (v−v_des)/omega0 + v·T_nominal`, then 3 passes of `T = dx/max(v, v_min)`,
   `dx = (xi − x_stance)·exp(omega0·T) + (v − v_cmd)/omega0`. Clamp to
   `[dx_min, dx_reach_max]`; return `(x_t, saturated: bool)`.
4. `plan_target_x`: route the measured branch through 2+3 (stopping case keeps the
   existing brake capture offset). Keep the `hip ± x_max` clamp but **count saturation
   events** (see logging below). The unmeasured fallback (Raibert neutral point) stays
   for the kinematic sandbox tests.
5. ~~Remove the interim §19.2.1 deadbeat `T_remain` pacing from `step_leg`'s Swing arm~~
   **DEFERRED to Phase B (measured):** removing it collapsed the walk (sh50 body
   107.2 → 0.1) because Phase B's progress-based retiming is its replacement and is
   not implemented yet. Kept in capped form `t_target = min(dist/v, swing_duration_ts)`.
6. Keep the `v_ref` ramp for now (it becomes `v_cmd` proper in Phase C).
7. **ADDED (measured):** the fixed-point `T = dx/v` feedback is positive — no fixed
   point exists when `(ξ₀−x_s)·ω₀/v > 1/e`, and the iteration ran away to the reach
   clamp every step. `solve_touchdown_x` and the `plan_target_x` caller cap the
   predicted horizon at `t_nominal` (`swing_duration_ts`). Cruise identity `dx = v·T`
   is unaffected when a consistent fixed point exists.

**Phase A status (2026-10-22).** Done and committed. 125/125 unit tests. Sweep
regressed vs §20 baseline (sh28 −105.2 vs 69.6; sh50 25.6 vs 107.2) — accepted:
the placement law is MPC but the FSM is still timer-driven; the mismatch is what
Phase B removes (see progress.md §25).

**Logging.** `DebugInfo`: add `clamp_saturated: bool` per tick + a cumulative
`clamp_sat_count`; export as scalar telemetry columns (`clmp`, reuse `capture_saturated`
slot if present). Design §4: clamp-saturation frequency is the primary health metric.

**Tests.** Unit tests for `placement_target` identity/saturation, `solve_touchdown_x`
convergence at cruise (`dx → v·T`), degenerate `v=0`, non-finite guards. Update
`test_compute_forward_target_neutral_point` and capture-point tests as needed. The
invariant tests (IK length, velocity continuity, stop-settle) must stay green.

**Verify.** `cargo test -p game_lib lower_body`; rebuild `headless_arm_pd`;
`sweep_step_height.py --heights 28,50`: body speed ≥ §20 baseline (69.6/107.2),
no regression in pass/n, `clmp` events logged (expect a few at startup, ~zero cruise).

---

## Phase B — Event FSM: DCM-margin trigger + progress-retimed swing (design §3, §5–§7)

**Goal.** Replace both timer-scheduled transitions with state triggers; make swing
timing progress-based (retiming), which delivers the timer-free landing (§23).

**Work items**

1. **Trigger (stance→swing).** Replace `wants_step`/`com_crossed`/timer logic with:
   primary `xi >= x_toe − margin`, emergency `x_com >= x_ankle` (logged as tuning
   failure if it fires in normal walk). `margin = v·T_swing_measured` (EMA of actual
   swing durations, seed `swing_duration_ts`).
   - Toe/ankle: 2D particle feet have no heel/toe split — use
     `x_toe = plant_x + toe_half` with `toe_half` a small config knob (default ~6 px),
     documented as an approximation of the support region.
   - Guards: min-stance-time (`min_stance_s`, default 0.1 s) after touchdown; swing leg
     must be unloaded (contact force ≈ 0 / not touching) before lifting; anti-phase
     (`!other_swinging`) kept.
2. **Swing profile → min-jerk + sine arch (design §5).** New
   `swing_foot_minjerk(tau, x_s, dx, y_ground, h_c)`:
   `x = x_s + dx·(6τ⁵−15τ⁴+10τ³)`, `y = y_ground + h_c·sin²(πτ)`,
   `h_c` = 10–15% leg length (config `swing_clearance_frac`, default 0.12). Delete the
   3-phase Hermite (`swing_profile_foot`) and its two unit tests (replaced by
   min-jerk endpoint/jerk tests).
3. **Retiming (design §6.2).** `tau = blend(tau_time, tau_progress)` where
   `tau_progress = clamp((x_f_now − x_s)/(x_t − x_s), 0, 1)` from the *achieved*
   (resolved pose) foot x — available via `prev_positions`/leg `foot_pos` — and
   `tau_time = t_elapsed/T` with `T = dx/max(v, v_min)` from Phase A. Blend weight
   config `tau_blend` (default 0.5 = max-like; tune by feel). No catch-up spikes:
   lag ⇒ tau advances slower.
4. **Touchdown (swing→stance, design §7).** Normal plant: `|x_f − x_t| < eps`
   (eps ≈ 3% leg length) AND `|vy_foot| < vy_max` AND `tau >= 0.9` AND sole contact +
   normal force above threshold (hysteresis: 2 frames arm / 1 frame release — reuse
   `debounce_contact`). Force plant: `t_swing > T_max` (1.5× nominal) — logged.
   Early/emergency plant: DCM inside critical distance of support edge — logged.
5. **Delete legacy scheduling.** Remove: stance/swing timer as primary trigger
   (`phase_timer <= 0.0` transitions), `GaitAuthority` ladder + `max_swing_wait`
   (subsumed: contact truth is now always required), `lost_contact_early` special case,
   `hovered_in`, and the §17 `com_crossed`/§19 deadbeat code. Keep `plant_anchored`
   re-anchor semantics (once per stance) — still correct and still needed.
   Keep `stance_duration_ts`/`swing_duration_ts` ONLY as nominal seeds for
   `T_nominal`, margin, `T_max` and the min-stance guard.
6. **Hysteresis everywhere** contact/force is read (design §3 guard): XPBD contact
   flickers; every condition gets start/release thresholds.

**Tests.** Rewrite the gait-phase tests for the FSM: trigger fires on DCM margin (not
timer); emergency trigger logged; min-stance guard blocks chatter; unloaded-leg
requirement; retiming stretches (not spikes) when the foot lags (simulate by scaling
achieved x); touchdown acceptance window; force plant at `T_max`; anti-phase
alternation; walk→stop converges via capture-point placement. Keep invariant tests.

**Verify.** Unit suite; headless sweep 28/50 — expect: body speed ≥ Phase A,
`midair` failures drop sharply (feet now hold until real touchdown *without* a wait
cap forcing a fake stance), emergency-trigger count ≈ 0.

---

## Phase C — Learned feasibility budget + slip measurement (design §4.1)

**Goal.** Kill the startup demand spike at the root (replaces the `v_ref` ramp hack);
never ask friction for more than it delivered recently.

**Work items**

1. **Measurement layer** (`measure.rs` + `hip_pin.rs`):
   - `a_com_x`: EMA (~10-frame) of finite-differenced CoM horizontal acceleration.
   - `slip`: stance-foot contact particle horizontal motion while its contact force is
     above threshold (with hysteresis). Needs per-foot contact force magnitude in
     `FootContact` (extend `update_ground_support`; it already sees collision events).
   - Export both as telemetry scalars (`acomx`, `slip_l`, `slip_r`).
2. **Budget governor** (new `resolver.rs::AccelBudget` struct, owned by `ResolverState`):
   seed ~2 m/s² scaled to px (design says conservative guess; pick px equivalent via
   gravity=980 px/s² ⇒ ~200 px/s², tunable `BUDGET_SEED`);
   on slip: `budget ← a_achieved_at_slip_onset · 0.8` (fast decrease);
   no slip ~1 s while demand high: `budget ← budget + delta` (slow increase,
   `BUDGET_INC`).
3. **Demand clamp** in `step_lower_body` §3 (velocity shaping): replace the
   `v_ref` ramp with `a_cmd = clamp(a_des, ±budget)`, `v_cmd += a_cmd·dt`, and thread
   `v_cmd` (not raw `v_des`) into the placement law, rebase, and schedule. Remove
   `ResolverState::v_ref` (superseded).
4. **Velocity schedule** (design §4): `v_ref(t) = v_now + a_des·t` clamped at `v_cmd`,
   and per-frame tracking error `e(t)` — feeds Phase B's rebase threshold
   (`e_thresh` = 10–15% of `|v_cmd − v_now|`, floor ~5 px/s). Wire the event-triggered
   rebase (design §6.1): on `|e| > e_thresh`, re-solve `x_t` from current state over
   remaining horizon, using `v_cmd` (never raw `v_des`), rate-limited so IK doesn't
   snap. Optional late-swing freeze (`tau > 0.75`) behind a config flag.
5. **Logging.** Slip events with budget before/after; rebase events with `e`;
   budget value as telemetry scalar (`budg`).

**Tests.** Budget decreases fast/increases slow (unit); clamp keeps `a_cmd` within
budget; schedule identity at cruise (`v_ref → v_des`); rebase fires on injected error
and is silent at cruise (`e≈0` ⇒ zero rebase events); startup from `v=0` with abrupt
`v_des` produces short feasible steps (kinematic-level test with a fake slip injector).

**Verify.** The §10 startup check: abrupt `v_des=120` from rest → 2–4 lengthening
steps, **slip count = 0**, no hip waggle; gesture tool shows both legs landing
(the trailing-leg midair should disappear — demand is now feasible by construction).

---

## Phase D — Plant guards + failure states (design §7–§8)

**Goal.** Robustness: early/force plant paths exercised, FALL/LIE/get-up FSM so an
uncapturable state degrades gracefully instead of jittering.

**Work items**

1. Early plant (DCM critical distance) and force plant (`T_max`) — mostly landed in
   Phase B; here: tune thresholds, ensure logging, unit-test both paths.
2. **FALL detection**: N consecutive failed captures (2–3) or torso angle past
   `theta_max` ⇒ `Fall` state: all gait authority off, legs go limp (compliance to
   soft values), physics takes over.
3. **LIE**: settled detection (CoM speed < thresh for ~0.5 s, torso near horizontal or
   CoM low) ⇒ stable regime, no actuation targets beyond damping.
4. **Get-up**: quasi-static pose sequence reusing loop-2 extension actuators:
   legs-under-CoM → curl over feet → extend; success = CoM above feet + height
   restored ⇒ hand back to STANCE. Sequence as a small keyframed sub-FSM in the
   resolver (state enum + progress), targets via existing IK.
5. Telemetry: `gait_state` scalar (STANCE/SWING/FALL/LIE/GETUP) so scripts can segment.

**Tests.** Phase-order tests (walk→FALL→LIE→GETUP→STANCE); get-up determinism;
FALL releases authority (no phantom steps while limp); large-shove simulation
(velocity injection) escalates only after N failed captures.

**Verify.** §10 large-shove checklist item in headless (inject `v_com` kick via a test
script step): clean fall, get-up, resume walking.

---

## Phase E — Tuning, hysteresis hardening, checklist (design §9–§10)

**Goal.** Turn the mechanism into a tuned gait; make the diagnostics permanent.

**Work items**

1. Knob table → env vars in `HipPinParams::from_env`: `T_CONV`, `E_THRESH`,
   `TRIGGER_MARGIN_SCALE`, `SWING_CLEARANCE`, `TAU_BLEND`, `PLANT_EPS`, `T_MAX_SCALE`,
   `MIN_STANCE`, `DX_MIN`, `DX_MAX`, `BUDGET_SEED`, `BUDGET_DEC`, `BUDGET_INC`.
2. Sweep harness: extend `sweep_step_height.py` (or new `sweep_gait.py`) to A/B the
   knobs via the telemetry scalars (rebase/slip/clamp/emergency counts + body speed +
   per-leg pass/n).
3. Run the full §10 verification checklist; record results as §25 in
   `physics_tuning_progress.md`.
4. Log every transition with its triggering condition (design §9 — the 5-minute vs
   5-hour debug rule). Keep counters in `DebugInfo`, export scalars.
5. Cleanup: delete dead legacy config fields/env (`STANCE`/`SWING` semantics change to
   nominal seeds — document), stale `GaitAuthority` references in `hip_pin.rs` UI and
   docs, old §17–§19 interim code comments.

---

## Sequencing & commit policy

- One commit per phase (code-only messages; findings go to
  `physics_tuning_progress.md` as §25+ per user convention).
- Each phase must leave `cargo test -p game_lib lower_body` green and the headless
  walk no worse than the previous phase on body speed + pass/n (record numbers in the
  progress doc).
- Phase A+B together deliver the §23 state-based gait; Phase C delivers the root-cause
  startup fix; D+E are robustness/tuning.

## Out of scope (per design §11)

Heel-toe ZMP rotation, multi-step MPC sequences, enhanced DCM/virtual pivot, angular
momentum capture correction (tripwire documented in the design), 3D anything.
