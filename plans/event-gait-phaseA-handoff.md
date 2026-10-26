# Event-Gait Phase A — Handoff (2026-10-26)

Status doc for resuming the loop-1 rewrite (time-based gait → event/DCM gait,
`plans/swing-phase-timing.md`). Records what is committed, what was measured,
the physics that explains it, and the candidate directions — including two the
user raised in discussion that have NOT been coded yet.

Branch `world_scale_t`. HEAD = `34c4eb2 update`.
**`tmp/` is untracked** (run artifacts live in `tmp/headless/walks/`).

---

## 1. What is committed and working (the baseline to build on)

### Committed control (smooth_start lineage)
- Asset fix: foot softbodies LLL/RLL `60 / 14.5 / 14.5` (`v10.character.json`).
- `tools/physics-qa/run_walk.sh` — one-command headless walk + plot + gates.
- `tools/physics-qa/plot_p3.py` — hip + feet x/y over time.

### Phase A infra (in `resolver.rs`, behind a default-OFF flag)
- `omega0(z_com, gravity) -> Option<f32>` — `√(g / z_com)`. With `g=980`,
  `z≈90` ⇒ `ω₀ ≈ 3.30`, `1/ω₀ ≈ 0.30 s`. (100 px = 1 m.)
- `capture_point_with_horizon(xi, ω₀, v_des, cfg) -> Option<f32>` —
  `x_foot = xi − v_des/(t_conv·ω₀)`, the §4 velocity-tracking placement law
  evaluated at the **current** (stale) DCM. `t_conv` default `1.0`
  (⇒ `T_conv = 1/ω₀`).
- `StepPlan` carries `xi: Option<f32>`, `omega0: f32`, `mpc_trigger: bool`.
  `xi = com_x + v_com/ω₀` is populated once per tick in `step_lower_body`.
- §3 DCM-margin liftoff trigger `dcm_crossed` in `step_leg` (Stance branch),
  gated behind `plan.mpc_trigger`. Fires when the DCM crosses the support
  edge minus one swing's travel (`margin = v·T_swing`). Emergency trigger
  `com_crossed` (CoM past plant) remains unconditional.
- `trigger_reason` telemetry on `LegState` (`1`=timer, `2`=dcm, `3`=emergency),
  surfaced in `DebugInfo.left/right_trigger_reason`.
- Env knobs in `hip_pin.rs from_env`: `T_CONV` (min 0.25), `TOE_LEN`,
  `MPC_TRIGGER` (bool). **Verified the env path works** — `MPC_TRIGGER=1`
  changes the output CSV md5. (The earlier "no measurable effect" reading was
  a stale binary, not a wiring bug.)

### Flag-off == committed baseline (bit-for-bit)
`run_walk.sh mpc_off` md5 = `3457fb05…` reproduces the pre-Phase-A walk.
121/121 lower_body tests green.

---

## 2. The failure physics (why the stale-state law ratchets)

LIPM stance dynamics: `v_dot = ω₀²·(x_CoM − x_foot)`.
**Foot placement is the throttle/brake:** foot behind CoM = gas, ahead =
brake, under = coast.

The stale law at `t_conv=1`: cruise (`v = v_des`) ⇒ `x_foot = xi − v_des/ω₀
= com_x` ⇒ foot lands **directly under** the CoM = **zero designed lead =
coast**. Because the law is evaluated at *now* but the foot lands ~`T_fly`
later, the braking lead `(v−v_des)/(T_conv·ω₀)` is cancelled ~1:1 by hip
travel `v·T_fly` (`ω₀·T_fly ≈ 0.92 ≈ 1`). Net effect: only a one-sided gas
floor, no brake ceiling until the reach clamp ⇒ the hip **ratchets** upward
and the feet lag ⇒ forward fall onto the knees.

Confirmed by telemetry on the stale law: hip 181→265+ px/s (cmd 120), feet
lead +20→−80 px, peak 357 px/s, height collapse.

Scale facts used repeatedly: `v/ω₀ ≈ 36–37 px` at 120 px/s;
`e^(ω₀·T_fly) ≈ 2.4–2.5` at `T_swing=0.28`.

---

## 3. What was tried: the §4 exponential (user has since discarded the code)

A touchdown-state prediction `predicted_touchdown_x` was implemented and then
**intentionally reverted by the user** (kept the simpler stale-state infra;
"easier to continue from previous position"). The math and results are
preserved here so the direction can be re-opened without re-deriving.

Law (pivot `p` = current stance foot; no iteration — the swing timer *is* the
touchdown-time estimate, re-evaluated every frame):

```
xi(T) = p + (xi − p)·exp(ω₀·T)              (DCM diverges from pivot)
x_t   = xi(T) − v_des/(t_conv·ω₀)           (velocity-tracking offset at T)
exponent clamped to [0, 2.0]                (plan §4 naive iteration diverges
                                             when ω₀·dx/v > 1)
```

### A/B result (MPC_TRIGGER=1 + exponential), cmd 120 px/s, 11.5 s run
| run | start moving | travel | hip vₓ median last 3 s | notes |
|---|---|---|---|---|
| baseline (stale, flag off) | 1.73 s | 638 px | **0** (fell/stopped) | swing gate FAIL, slide 84 px |
| exponential t_conv=1.0 | 3.97 s | 912 px | **112, p90=120** | **ratchet DEAD — limit cycle back on v_des**; swing FAIL, slide 73 px |
| exponential t_conv=0.7 | 1.92 s | 1088 px | 41 (decel) | faster start, but over-brakes |
| exponential t_conv=1.4 | 3.58 s | 22 px | 0 | **falls** — soft law can't capture |

**Wins:** the exponential kills the runaway and pins `v ≈ v_des`;
lower `t_conv` ⇒ sharper brake, higher ⇒ soft/uncapturable.
**Problems (all real, none solved):**
1. **Slow start (3.97 s):** at standstill `xi − p ≈ 0`, so the whole target is
   `−v_des/(t_conv·ω₀) ≈ −37 px` ⇒ commands the foot to land *behind* the
   support foot for the first ~15 steps (the "both feet move backward then
   forward" startup). `t_conv=0.7` shrinks it but then over-brakes while walking.
2. **Period-2 asymmetry:** touchdown offsets alternate per leg (left ~{+22,−26},
   right ~{+19,−28}) — the lead sign cycles between legs instead of settling.
   Hypothesis: the pivot choice (single stance foot vs. double-support midpoint)
   makes effective step time alternate between legs.
3. **§3/§4 constant mismatch:** the trigger fires `xi` past `toe − v·T` but the
   placement aims `xi − v_des/ω₀`; the two use different constants, so the
   matched pair isn't truly matched (the steady-gait identity wants the lead
   scaled by `e^(−ω₀·T_stance)`, not a bare `v_des/ω₀`). Likely the source of
   problem 2.

---

## 4. Candidate directions (NOT yet coded — user's own instincts)

The user pushed back on trusting the model precisely and proposed two simpler
ideas. Both are legitimate and share the key insight: **we already re-plan
every frame** (`step_leg` Swing branch refreshes `swing_to.x = plan_target_x`
each tick), so a model error never accumulates — it's corrected before the
foot lands. High control rate is what makes a crude law robust.

### Direction A — reactive lead, scaled to speed (no ω₀, no exp)
```
foot_target = hip + lead(v),   lead(v) ≈ gain · v · T_stance   (clamped to ±x_max)
```
- lead **∝ v** is essential: a *fixed* pixel lead brakes-to-stop at low speed
  and does nothing at high speed. `lead → 0` at standstill ⇒ no backward start.
- At cruise the foot must land ahead of the CoM by ≈ one stride (`v·T_stance`);
  the exponential reproduces this as a `≈2.4×` multiplier on the stale lead, so
  Direction A is the exponential with a hand-tunable `gain` replacing `exp(ω₀·T)`.
- Re-aim at high rate; never trust the exact number.

### Direction B — hip-RELATIVE target instead of world-absolute (user raised)
The current DCM rewrite is a **world** anchor (`xi` is built from `com_x`), so
`plan_target_x` returns an absolute ground coordinate independent of the hip.
That switch is the most likely cause of the floating-foot / stance-lag failures:
when the hip is too fast the world target exits `hip ± x_max`, the reach clamp
fires, and the foot can't keep up.

A hip-relative target `foot = hip + lead` is **reachable by construction** (no
clamp starvation, no floating foot) and tracks the hip so the foot can't lag by
more than `lead`. Caveat it MUST satisfy: the relative lead still has to scale
with speed (§4A), or a lurching hip just carries the foot along at the same
lead = ratchet again. Precedent already in the code: the jump/airborne branch
deliberately uses a hip-relative offset (`step_lower_body`, "world-space floor
target is unreachable while the hip is high"), and the original Raibert law was
offset-based (`compute_forward_target` → `hip_x + v·t + offset`).

### A+B combined (probable landing spot)
```
foot_target = hip + clamp(gain · v · T_stance, −x_max, +x_max)
```
re-planned every frame, `mpc_trigger`-gated. Startup-safe (v→0 ⇒ lead→0),
reachable by construction (bounded to `±x_max`), self-scaling with speed.

---

## 5. The pipeline (how to run / measure / decide)

All from repo root. `run_walk.sh [TAG] [KEY=VALUE ...]` — any `KEY=VALUE` is
forwarded as env to the harness (`MPC_TRIGGER`, `T_CONV`, `TOE_LEN`,
`TARGET_SPEED`, `STEP_HEIGHT`, `STANCE`, `SWING`, `GAIT_AUTHORITY`,
`LEG_COMPLIANCE`, …). Defaults `LEG_COMPLIANCE=5e-9 STEP_HEIGHT=50`.
Rust change ⇒ rebuild first:
`cargo build --release -p collision_detection --example headless_arm_pd`.

Artifacts land in `tmp/headless/walks/<TAG>.{csv,png,character.json}` (the
asset is snapshotted per run so each artifact is self-identifying; determinism
verified — same inputs ⇒ byte-identical CSV md5).

Gates printed by `run_walk.sh`:
- `analyze_swing.py --per-swing` — did each swing lift / advance / overtake /
  gain lead (the swing gate). FAIL if a foot never lifts or lands behind hip.
- `analyze_foot_slide.py --skip-start 1.6 --skip-end 10.5` — worst stance drift
  (gate 2.0 px). Current best walk still SLIDING at ~73–84 px.
- rigidity / leg verdict lines.

Telemetry CSV: 104+ cols. Key ones for this work: `t`, `tP3x/y/vx` (hip=P3),
`tPLLLx`/`tPRLLx` (left/right foot), `gait_l`/`gait_r` (0=Stance,1=Swing).
Quick speed/lead probe (used to produce §3): mean/median/p90/max of `tP3vx`;
per-leg touchdown lead `tP3x − foot_x` at `gait` 1→0 transitions.

`plot_p3.py` renders hip + both feet x/y for visual confirmation.

**Determinism check pattern:** run the same config twice, `md5sum` the two CSVs
— must match. Flag-off `mpc_off` must equal the committed baseline md5.

---

## 5b. Knee bend-side fix (2026-10-26, post-Phase-A handoff)

A regression surfaced during the Windows port: while walking right the leg
**sometimes bent left, sometimes right**. Root-caused and fixed; unrelated to
the §3/§4 gait law but blocks clean reading of the walk, so done first.

### Diagnosis (telemetry, not speculation)
- `bend_sign` is a **constant +1** on the walk path (only the GUI `raibert_sandbox`
  ever flips it). The flip is kinematic, not a wrong branch.
- The 2-bone IK puts the knee at a **chord-relative** side: `rotate(dir, a·bend_sign)`
  in `solve_leg_ik`. The *world* side of that offset flips whenever the hip→foot
  chord swings past vertical, and the knee collapses onto the chord (`a→0`) at
  saturated reach (`hip→foot ≈ L1+L2 = 110px`, measured up to 111px). Stance chord
  is always down (one side); the swing chord passes up — 70 world-side flips, all
  in swing. Confirmed in the resolver's **commanded targets** (`tP40/tP41`), not
  just live particles ⇒ the bug is the solver convention, not the hip-pin wiring.
- **y-handedness gotcha (the user's call):** in vello's **y-down** frame
  `rotate(+angle)` winds the opposite visual way than the math convention
  `bend_sign`'s doc assumes, so `+1` produced a **goat leg** (knee behind the
  ankle: 556/594 stance frames). A first fix using a cross-product *winding vs.
  the chord* inherited that mirrored side and was itself y-ambiguous (an
  enforce-`cross≥0` rule is even geometrically impossible at full-extension
  heel-strike — the test proved it).

### Fix — `select_anterior_knee` (resolver.rs)
State the anatomical rule on the **x axis only** (identical in y-down and y-up,
so it cannot mis-map). The IK has exactly two valid knees, mirrors across the
hip→foot chord; **pick the one with maximal `knee.x·facing`** (facing right ⇒
the more anterior one). Selection (not enforcement) holds at every chord angle
and reach; mirroring preserves both bone lengths exactly.
- New `LowerBodyConfig.facing` (+1 right default, −1 left); `FACING=left|right`
  env in `hip_pin::from_env`.
- Applied at BOTH flip sources: `step_lower_body` IK output and the `hip_pin`
  composed knee targets (after the thrust/retract biases, before assignment).
- `enforce_knee_winding` kept as the chord-relative primitive; `mirror_knee_across_chord`
  factored out; `select_anterior_knee` is the shipped anatomical rule.

### Verification
- 155/155 `game_lib` tests green; new `test_knee_anterior_selection_body_frame`
  asserts anteriority on trailing-stance poses + length preservation + facing mirror.
- Headless walk `anterior_v2`: knee leads ankle **568/579 stance frames** (was
  12/0 goat), **0 side flips** on targets, first swing segments now PASS.
- Windows port `tools/physics-qa/run_walk.ps1` is the PowerShell twin of
  `run_walk.sh` (same pipeline; env-var scoping + `Select-String`/`Select-Object`
  substitutes; needs `pip install numpy matplotlib pillow`).

### New open items (visual, this run)
The fixed walk looks good in `visual.rs` but shows two behaviours to chase next:
1. **Forward torso tilt** while walking (attitude loop-3 lean; tune `ATT_LEAN_KP/KD`
   or the gravity-coupling term).
2. **Mid-walk pause** — the gait stalls, stands taller, then resumes (likely a
   stance/swing handoff or DCM-margin trigger timing issue, ties into the §3/§4
   matched-pair work in §6).

## 6. Concrete next steps (ordered)

1. Decide law: implement **§4 A+B** (`foot = hip + clamp(gain·v·T_stance,±x_max)`)
   behind a new flag (e.g. `MPC_RELATIVE`), reusing Phase A infra; leave the
   exponential out (user prefers the simpler starting point).
2. Fix the matched-pair constant: pick `gain` from the steady-gait identity
   (lead = one stride) rather than sweeping blindly; alternatively derive the
   single consistent §3/§4 constant so trigger and placement agree.
3. Resolve the per-leg 2-cycle: likely the `pivot`/reference choice; with a
   hip-relative law the pivot disappears, which may fix it for free — verify.
4. Add a **gas-phase guard** so startup never commands a step behind the
   support foot (lead floor at `lead ≥ 0` when `|v| < |v_des|`, or ramp the
   lead in with measured speed).
5. A/B: baseline / relative-lead (a few `gain`s). **Success signature:** swing
   gate PASS, foot-slide toward the 2 px gate, hip vₓ pinned near `v_des`
   without the backward start, single steady stride (no alternating lead sign).
6. Then, deferred from earlier: Phase A3 **STAND normalization** (the walk must
   stop on the symmetric "one foot ahead, one behind" rest pose, not a straddle
   that tips over); §4.1 learned feasibility budget (Phase B); §7 touchdown
   events (Phase C).
7. Update `tools/physics-qa/physics_tuning_progress.md` with a short pointer to
   this doc once the relative-lead experiment has run.

---

## 7. Gotchas / traps already discovered

- **Re-timing the swing timer is self-referential** in a pure-kinematic resolver
  (the profile's own x defines progress). A `phase_timer = max(timer, span_now)`
  floor let a runaway target re-extend the schedule every frame ⇒ infinite swing
  + anti-phase froze the other leg. Do NOT gate landing on a timer floor; land
  on `phase_timer <= 0` (the §19.2.1 deadbeat block, restored and committed).
- **Trigger-only (margin without a lead-generating placement) is WORSE than the
  baseline** (measured lead −18/−32 vs −7/+21). §3 and §4 are a matched pair;
  don't enable `MPC_TRIGGER` alone.
- **Exponent/iteration divergence:** the plan §4 fixed-point iteration diverges
  when `ω₀·dx/v > 1`. If the exponential is ever revived, clamp the exponent
  (2.0 was used) and evaluate once, not iterate.
- **`prediction_margin` config field** must stay (a diff once deleted it and it
  broke GaitLimits/tests). It is unrelated to the `t_conv`/`toe_len` additions.
- Headless build needs the package: `cargo … -p collision_detection --example
  headless_arm_pd` (fails as a bare `--example` otherwise).
- Plan doc `swing-phase-timing.md` line ~106 has an identity slip (`+v/ω₀`
  where the LIPM derivation gives `−v_des/ω₀`); and a §24-style note that the
  naive iteration diverges is still to be added. Fix when next in that file.
