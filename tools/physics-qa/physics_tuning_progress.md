# Physics Tuning Progress — Walking Foot-Slide (2026-10-20)

Running log for the walking physics tuning loop. Each entry records the config
under test, the exact run, the numbers, and the verdict. Append chronologically.

Status legend: `[x]` done, `[-]` in progress, `[ ]` planned.

## 0. TL;DR (current state)

- The telemetry CSV now exports the **per-leg resolver gait phase** (`gait_l` /
  `gait_r`, 0 = Stance, 1 = Swing), so `analyze_foot_slide.py` segments stance
  from swing with the **event signal**, not a geometric floor tolerance.
- Baseline walk (`walk_gait.csv`, friction_compliance_scaler=0.1 on all
  colliders) is **SLIDING**: worst stance drift 53.6 px on the right foot
  (t=0.03–0.83 s, startup transient), 29.7 px on the left. In steady state
  (after t≈1.1 s) stance drift drops to ~2–5 px per stance.
- Friction semantics confirmed: floor contact uses `StaticCollisionConstraint`,
  where **lower `friction_compliance_scaler` = more friction**.
- **Friction sweep done** (§6, session 2). The feet (LLL/RLL) are now at
  `friction_compliance_scaler = 0.35` (was 0.1): steady-state worst stance drift
  **5.15 → 3.14 px**, full-run (startup) worst **54.2 → 31.1 px**. The curve is a
  U: *more* friction makes the steady drag **worse** (0.0 → 8.97 px), so the
  optimum is a plateau at 0.30–0.40, not at the asset's 0.1.
- **Next:** the residual ~3 px is **foot-vs-target tracking error**, not grip
  (commanded target moves only ~0.7 px per stance). Friction cannot reach the
  2 px gate; see §6.5 for the non-friction levers.
- **Swing gate added** (§7). The walk does **not step**: the realized feet stay
  ~195 px apart while their commanded targets are 2–7 px apart, each swing gains
  only 1.3–2.7 px relative to the body, and the body travels **9.9 px/s against
  `cmd_vx = 104 px/s` (10%)**. The rendered twin reproduces it to within ~0.3 px,
  so this is the shared controller/physics, not the renderer.
- **Best walk so far** (§9): `LEG_COMPLIANCE=1e-9` **plus** `STEP_HEIGHT=70`.
  Stiff legs alone made it walk backwards (§8) because the default 28 px swing arc
  was absorbed by the compliant joint — at `1e-6` a 70 px arc still realizes only
  6.9 px of lift. With both, the swing gate **passes 29/29** (lift 55 px, advance
  +29…+49 px, lead −37 → +42 px), foot separation 38 px vs 36 px commanded, body
  at 84–101% of `cmd_vx`. Remaining: stance slide 32–52 px and rig `max_rel_dev`
  0.10–0.16 (vs 0.05).
- **Loop 3 is now leg-delivered** (§10, Phase E). The waist `P1_P2_P3` is a pure
  posture hold (`theta = 0.0`, straight stick); the measured-attitude PD is turned
  into a **lean of the leg PD's world-upright reference** (`upright_reference_lean`)
  and applied by the knee joints. Defaults `ATT_LEAN_KP=0.5`, `ATT_LEAN_KD=0.03`.
  Same best-walk config now gives swing **28/29**, body **96%** of `cmd_vx`, rig
  `max_rel_dev` **0.1502**. A/B with the lean off is strictly worse
  (26/29 swings, 71% speed, `max_rel_dev` 0.2254), so the delivery change is kept.
- **Contact-gated gait clock (§6 / Phase F) was built and REJECTED** (§11). The
  per-foot contact layer is real and correct — at `GAIT_AUTHORITY=observe` the
  telemetry is byte-identical to the timer-only control (`md5 d6c97f8a…`) — but
  every mechanism that *acts* on it degrades the walk monotonically: swing
  **28/29 → 17/29 → 14/27**, rig `max_rel_dev` **0.1502 → 0.1874 → 0.3144**,
  stance drift **61 → 157 → 236 px** for `Contact` / `Full`. Default is therefore
  `GAIT_AUTHORITY=timer` (pre-§6, unchanged). See §11.3 for the three causes; the
  key finding is that the residual ~61 px slide is a **foot contact/friction**
  problem, not a gait-clock timing problem.

## 1. What changed this session

### 1.1 Gait phase export (`gait_l` / `gait_r`)

- [`report.rs`](../examples/collision_detection/examples/headless_arm_pd/report.rs)
  — new [`gait_code()`](../examples/collision_detection/examples/headless_arm_pd/report.rs:34):
  0 = Stance, 1 = Swing (matches `GaitPhase` declaration order in
  [`resolver.rs`](../examples/game_lib/src/lower_body/resolver.rs:143)).
- [`sample.rs`](../examples/collision_detection/examples/headless_arm_pd/sample.rs:129)
  — reads `pin.state.left.phase` / `pin.state.right.phase` and writes
  `gait_l` / `gait_r` right after `vphase`. Header comment updated
  "102-column" → "104-column".
- Schema (relevant body-mode scalar columns, in order):
  `tau,t,act,kind,kp,kd,theta_deg,omega,cmd_vx,vphase,gait_l,gait_r,body_theta,...`.

### 1.2 Analyzer rewrite (event segmentation)

- [`analyze_foot_slide.py`](analyze_foot_slide.py) now prefers the `gait_*`
  event columns (`in_stance = gait == 0`) and only falls back to the geometric
  floor test when the columns are absent (old CSVs). It prints a
  `segmentation:` line so you can tell which path ran.
- Reason: the realized foot lift is only ~5 px. The old geometric tolerance
  (8 px) swallowed the whole run as one stance phase and reported drift equal
  to the foot's forward travel (a false SLIDING verdict).

### 1.3 Telemetry schema notes

- Coordinate frame: resolver targets are bevy **y-up**; particle positions are
  vello **y-down** (`bevy_to_vello(p) = (p.x, -p.y)`), so **smaller y = higher
  up** and the floor is the *largest* y (~277.5 left / ~276.4 right).
- Feet: `PLLL` (left foot) and `PRLL` (right foot) particles; commanded targets
  `tPLLL` / `tPRLL`.
- Sidecars: `<csv>.events.csv` = input timeline; `<csv>.leg.csv` =
  `t,left_err_px,right_err_px,left_dx_px,right_dx_px` (dx = foot.x − target.x,
  negative = lagging).

## 2. Baseline measurement (friction_compliance_scaler = 0.1 everywhere)

Config under test:

- `v10.character.json`, all 12 colliders:
  `push_compliance_penetration_scaler: 0.3`,
  `friction_compliance_scaler: 0.1`,
  `pull_compliance_scaler: 0.7`.
- Feet colliders: **LLL** (`v10.character.json:125`, left lower leg, contains
  PLLL) and **RLL** (`v10.character.json:169`, right lower leg, contains PRLL).
- `LowerBodyConfig` defaults: thigh=calf=55 px, hip_height=90, target_speed=120,
  stance=swing=0.28 s (alternating ~1.8 gait cycles/s).

Run:

```bash
cd /home/ubuntu/bender/bevy_vello
cargo build --release -p collision_detection   # 2.51s
mkdir -p /tmp/friction_qa
SCRIPT=$PWD/plans/walk_script.json \
OUT_CSV=/tmp/friction_qa/walk_gait.csv \
./target/release/examples/headless_arm_pd
python3 tools/physics-qa/analyze_foot_slide.py /tmp/friction_qa/walk_gait.csv --gate 2.0
```

Rig gate + sidecar summary:

| metric | value |
| --- | --- |
| rows / duration | 1036 / 11.5 s (0 NaN) |
| rig (max relative deviation) | FAIL, 0.0494 |
| leg gate | peak left 145.1 px @t1.23, right 109.5 px @t11.50; combined 145.1 px @t1.23; tail WAGGLE (rms 6.164 px, rev 2) |
| cell hash | `de561ce4919bb0ba` (WAGGLE verdict, kp=0.5 kd=0.065) |

Foot-slide verdict (analyze_foot_slide.py, gate 2.0 px):

| foot | stances | total dur | drift mean | drift max | total drift | peak speed | worst window |
| --- | --- | --- | --- | --- | --- | --- | --- |
| left | 19 | 6.1 s | 4.857 px | **29.740 px** | 92.280 px | 102.6 px/s | t=0.03–0.54 s |
| right | 18 | 6.4 s | 8.006 px | **53.570 px** | 144.100 px | 117.9 px/s | t=0.03–0.83 s |

**Result: SLIDING** (worst 53.570 px > 2 px gate).

Breakdown of the drift:
- **Startup transient** (t < ~1.1 s): first stance on each foot slides hard —
  the worst windows are both the very first stance. This is the body accelerating
  into the walk / feet not yet gripping.
- **Steady state** (t ≈ 1.1–10.5 s): left ~2–3.5 px per stance, right ~4–5 px per
  stance. Right is consistently worse than left.
- **Stop transient** (t > ~10.5 s): modest.

## 3. Friction semantics (established)

The floor is a **static collider** → `StaticCollisionConstraint`
([`constraints.rs`](../../../../study_vello/integrations/vello_physics/src/constraints.rs:236)):

```rust
firction_strength = (1.0 - self.friction_compliance).clamp(0.0, 1.0);
max_friction_disp = firction_strength * penetration.abs();
```

**So LOWER `friction_compliance_scaler` = MORE friction (stronger grip) for floor
contact.** Friction force is also proportional to penetration — stiffer push
compliance (lower `push_compliance_penetration_scaler`) raises the grip too.
(Contrast: `FineCollisionConstraint` uses the scaler directly.)

The `v10.character.json` comment warns: "use 0.0 as no friction and avoid using
super small friction compliance which cause jittering." Current value 0.1 is
already low; the floor should already grip hard. The sliding is therefore likely
not a "not enough friction" case — see §4 hypotheses.

## 4. Open questions / next steps

- [ ] Sweep `friction_compliance_scaler` on **LLL/RLL only**: 0.1 (baseline) →
  0.05 → 0.02 → 0.0. Record worst steady-state stance drift per config. Expect
  jitter at the low end.
- [ ] If drift does not move with friction, test whether the slide is actually
  **penetration-driven**: sweep `push_compliance_penetration_scaler` (0.3 →
  0.2 → 0.1) on the feet.
- [ ] Characterize the startup transient separately — the first-stance 29–53 px
  is a different phenomenon from steady-state slide (acceleration / feet not yet
  settled) and may need the wobble/straightness gate instead of a friction knob.
- [ ] Record steady-state per-stance drift only (exclude t < 1.1 s and t > 10.5 s)
  as the friction comparison metric.

## 5. Reproduce (commands)

```bash
cd /home/ubuntu/bender/bevy_vello
cargo build --release -p collision_detection
mkdir -p /tmp/friction_qa
SCRIPT=$PWD/plans/walk_script.json OUT_CSV=/tmp/friction_qa/<name>.csv \
    ./target/release/examples/headless_arm_pd
python3 tools/physics-qa/analyze_foot_slide.py /tmp/friction_qa/<name>.csv --gate 2.0
python3 tools/physics-qa/analyze_leg_divergence.py /tmp/friction_qa/<name>.leg.csv
```

Edit the character asset at
`examples/collision_detection/assets/character/v10.character.json` (no rebuild
needed for asset-only changes — the game reads it at startup).

## 6. Friction sweep — feet only (session 2)

### 6.1 Tooling

- [`sweep_foot_friction.py`](sweep_foot_friction.py) — the sweep loop. For each
  candidate it patches `friction_compliance_scaler` **only** inside the named
  collider blocks (textual, so the diff is the two lines it changed), runs the
  headless walk, re-measures the CSV with `analyze_foot_slide`, and restores the
  asset byte-exactly (also on failure/interrupt). No rebuild: the value is read
  from the runtime asset at startup.
- [`analyze_foot_slide.py`](analyze_foot_slide.py) gained `--skip-start` /
  `--skip-end`: a stance segment is reported only when it lies fully inside the
  steady window. Defaults are no trimming, so earlier verdicts are unchanged.

```bash
# from the repository root (works on Windows and Linux)
py bevy_vello\tools\physics-qa\sweep_foot_friction.py ^
    --frictions 0.0,0.02,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.5 --repeat 2
```

One run is 5.3–5.6 s wall, 1036 rows, and repeats differ by ≤ ~0.2 px.

### 6.2 Window choice (why 1.6 s, not 1.1 s)

With `--skip-start 1.1` the "worst stance" for several candidates was the stance
at **t = 1.13–1.72 s**, i.e. the tail of the startup transient. That made
0.32/0.35 look like a cliff (8–11 px) when the same runs measure 2.9–3.4 px at a
1.6 s window. The table below uses **t ∈ [1.6, 10.5] s** for every candidate.

### 6.3 Result (LLL/RLL, walk_script, 2 repeats, worst of the repeats)

| friction | steady worst px | steady L max | steady R max | steady mean L/R | full-run worst px |
| --- | --- | --- | --- | --- | --- |
| 0.0 | 8.970 | 6.470 | 8.970 | 4.79 / 6.52 | 59.960 |
| 0.02 | 7.790 | 5.810 | 7.790 | 4.04 / 5.86 | 59.880 |
| 0.05 | 6.250 | 4.740 | 6.250 | 3.29 / 4.96 | 57.590 |
| 0.1 (old default) | 5.150 | 3.470 | 5.150 | 2.51 / 4.07 | 54.240 |
| 0.15 | 4.380 | 3.340 | 4.380 | 2.19 / 3.67 | 48.630 |
| 0.2 | 3.720 | 3.040 | 3.720 | 1.92 / 3.00 | 43.940 |
| 0.25 | 3.600 | 3.440 | 3.600 | 1.89 / 2.69 | 40.300 |
| 0.3 | 3.250 | 2.860 | 3.250 | 1.93 / 2.55 | 34.640 |
| **0.35 (picked)** | **3.140** | 2.740 | 3.140 | 1.57 / 2.27 | 31.090 |
| 0.4 | 3.720 | 3.720 | 3.110 | 1.82 / 2.38 | 27.670 |
| 0.5 | 6.800 | 3.180 | 6.800 | 1.63 / 2.39 | 23.880 |

Every candidate still fails the 2 px gate on the worst stance.

### 6.4 Findings

1. **More friction is worse, not better.** `friction_compliance_scaler` 0.0
   (maximum grip) gives the *largest* steady drag, 8.97 px; 0.35 gives 3.14 px.
   The static-contact correction is `min(strength · |penetration|, tangential
   displacement)`, so a stronger grip holds the planted foot against the body's
   forward motion for longer and the tangential correction ends up *dragging* it
   forward. §3's reading of "0.1 is already low, so the floor grips hard" was
   right about the grip and wrong about the metric: hard grip here is what
   produces the measured drift.
2. **Optimum is a plateau at 0.30–0.40**, minimum at **0.35**
   (`friction_strength = 0.65`). Above 0.4 (weaker grip) a stance starts to
   genuinely slip: 0.5 → 6.80 px. The startup transient, by contrast, improves
   monotonically with weaker grip (59.96 → 23.88 px), so 0.35 is a compromise,
   not a joint optimum.
3. **Applied:** LLL (`v10.character.json:143`) and RLL (`:187`) are now at
   `friction_compliance_scaler: 0.35`; every other collider is unchanged at 0.1.
4. **The residual is tracking, not grip.** Per stance at 0.35 the commanded
   target moves only 0.6–0.8 px, while the realized foot moves 1.6–2.3 px mean
   (3.14 px worst) and the foot-vs-target error grows ~2.5–3 px, signed +x —
   the foot is dragged forward inside its own stance, in the travel direction.
   No friction value reaches 2 px, because friction is not what sets this number.

### 6.5 Next levers (non-friction)

- [ ] `push_compliance_penetration_scaler` on the feet: 0.3 → 0.2 → 0.15. Friction
  is proportional to penetration, so a stiffer normal reaction changes the grip
  curve that §6.4 found the optimum of.
- [ ] `inner_constraints_scaler` / `frame_constraints_scaler` on LLL/RLL — is the
  foot particle being dragged *inside* the collider (soft-body shear) rather than
  the collider sliding on the floor?
- [ ] Per-foot split: at 0.35 the split is L 2.74 / R 3.14; at 0.4 it inverts to
  L 3.72 / R 3.11, so `LLL=0.35, RLL=0.4` would score 3.11 px — a 0.03 px gain,
  not worth the asymmetry. The right (support) foot stays the limiter.
- [ ] The startup transient (31 px at 0.35, first ~1.7 s) is a separate
  phenomenon and needs the wobble/straightness gate, not a friction knob.

## 7. Rendered twin + swing gate (session 3)

### 7.1 The rendered twin reproduces the headless walk

`visual_arm_pd` (the rendered twin, `examples/headless_arm_pd/visual.rs`) was run
on the same asset and script and its CSV compared row-by-row with the headless
one (same 90 Hz cadence ⇒ index-aligned; 1036 rows each, 802 ticks in the
`t ∈ [1.6, 10.5] s` window):

| column | what | mean abs diff | max abs diff | mean magnitude |
| --- | --- | --- | --- | --- |
| `t` | time | 0.000 | 0.000 | 6.05 s |
| `cmd_vx` | commanded hip vx | 0.000 | 0.000 | 120.0 |
| `tPLLLx` | left foot target x | 0.950 | 3.310 | 240.4 |
| `tPRLLx` | right foot target x | 0.941 | 3.330 | 242.8 |
| `PLLLx` | left foot x | 0.719 | 2.120 | 121.3 |
| `PRLLx` | right foot x | 0.631 | 1.540 | 316.4 |
| `P3x` | spine end x | 0.574 | 1.290 | 215.1 |
| `PHx` | head x | 0.772 | 2.640 | 185.0 |

Gait phase: **3.7% of ticks disagree** and both runs have the **same 30 phase
transitions per leg** (the disagreement is one-tick edge positioning). Body
travel over the window: 109.4 px headless vs 108.6 px visual (0.7%). Foot-slide
gate: 3.14 px headless vs 3.20 px visual.

So **the renderer does not change the physics** — the twins agree to sub-pixel
mean / ~3 px worst. Any walking bug seen in the window is real, and the headless
CSV is a faithful proxy for what is on screen.

### 7.2 The walk does not step (swing gate)

[`analyze_swing.py`](analyze_swing.py) segments `gait_* == 1` and measures, per
swing, whether the foot lifts, advances *relative to the body*, and **overtakes**
the opposite foot (lead crosses zero). Result on both twins (identical):

| metric | left | right | what a walk needs |
| --- | --- | --- | --- |
| swings (window) | 15 | 14 | — |
| lift mean / min / max | 4.40 / 3.81 / 4.90 px | 2.06 / 1.60 / 2.67 px | a real clearance |
| advance rel. body mean / min | +2.70 / +1.42 px | +1.33 / +0.83 px | ≈ a step length |
| lead at liftoff → touchdown | −197.3 → −192.9 px | +192.8 → +196.1 px | crosses 0 |
| lead gain mean / min | +4.40 / +2.65 px | +3.38 / +1.36 px | ≈ a step length |
| tracking error (foot − target) | −113 / −115 px | +80 / +76 px | small |
| realized foot separation | 197.3 px | 192.8 px | ≈ target separation |
| **commanded** target separation | **6.6 px** | **2.0 px** | — |
| body speed vs `cmd_vx` | **9.9 px/s vs 104 px/s (10%)** | same run | ≈ 100% |

Verdict: **FAIL** — every left swing is `NO_OVERTAKE`; 6 of 14 right swings are
`NO_LIFT` (they never clear 2 px).

What the numbers say, in order of importance:

1. **The legs are splayed, not stepping.** The feet sit ~195 px apart (left ~115
   px behind its own IK target, right ~77 px ahead) while the commanded targets
   are 2–7 px apart. The swing foot only gains 1–3 px relative to the body and
   the lead never crosses zero: it lifts, comes back down in front of where it
   started *in body space*, and the split stance persists.
2. **The body does not travel at the commanded speed.** 9.9 px/s realized vs
   104 px/s commanded — the hip pin commands walking, the contact/friction
   response and the splayed legs absorb it. This is why the walk reads as
   "wobbling forward" rather than walking.
3. **Friction cannot fix this** (§6): it is a foot-placement / leg-reach failure,
   not a grip failure. The stance-drift number (3 px) was always a second-order
   symptom of it.

### 7.3 Next (walking, not friction)

- [ ] Why does the leg not reach its own IK target? The feet end up ~100 px
  outside the commanded span, close to the 55+55 px thigh+calf reach from the
  hip. Check whether the leg IK is saturating (target outside the reachable
  disc) before blaming the PD gains — dump the hip→foot distance against the
  thigh+calf sum.
- [ ] With the feet splayed, the body is being held back by contact. Verify
  whether the pin's `cmd_vx` is being cancelled by the collision response
  (`apply_explicit_impulse_on_*`) or by the leg constraint pulling the hip.
- [ ] Gate the fix on both tools: `analyze_swing.py --gate-gain 20` (lead must
  gain a real step) plus `analyze_foot_slide.py --skip-start 1.6` (stance must
  not slide). A gait that passes only one of them is still broken.

## 8. Leg stiffness: "just make the legs stiffer" does not work (session 4)

The hypothesis was that the leg angular constraints are too compliant to hold the
body, let alone swing the trailing leg forward. Three things had to be
established before testing it.

### 8.1 The asset cannot set the leg angular stiffness

`drive_hip_pin` **owns** the leg angular compliance and rewrites it every frame
([`hip_pin.rs:772`](../examples/game_lib/src/lower_body/hip_pin.rs:772)):

```rust
right_leg.config.angular_compliance = params.leg_compliance.max(1e-12);
left_leg.config.angular_compliance  = params.leg_compliance.max(1e-12);
```

So the `Angular` compliance of `P2_P3_P40/P41` and `P3_P40/P41_PLLL/PRLL` in
`v10.character.json` is **dead at runtime**. The live value is
`HipPinParams::leg_compliance` (default `1e-6`), overridable with
`LEG_COMPLIANCE`. The `Distance` (bone) compliance is *not* rewritten, so
`set_joint_compliance.py` does reach it.

### 8.2 Sweep (`sweep_leg_compliance.py`, bones at their asset values)

| LEG_COMPLIANCE | body px/s (cmd 104) | lift mean | swing advance | lead gain | stance drift max | rig `max_rel_dev` |
| --- | --- | --- | --- | --- | --- | --- |
| **1e-6 (default)** | +10.3 (10%) | 3.1 px | +1.9 px | +3.6 px | 3.13 px | 0.0497 |
| 5e-7 | −0.6 (−1%) | 5.5 px | +1.4 px | −3.6 px | 34.71 px | 0.2929 |
| 1e-7 | −1.1 (−1%) | 10.8 px | −1.9 px | −5.3 px | 10.40 px | 0.2741 |
| 5e-8 | −1.8 (−2%) | 17.4 px | −3.4 px | −6.1 px | 15.77 px | 0.3443 |
| 1e-8 | −25.7 (−25%) | 47.1 px | −1.0 px | −2.1 px | 36.33 px | 0.4075 |
| 5e-9 | +85.9 (83%) | 25.7 px | +32.0 px | +71.0 px | 52.43 px | 0.2577 |
| 1e-9 | **−179.1 (−172%)** | 57.6 px | −19.1 px | −36.1 px | 66.15 px | 0.4598 |

(The 5e-9 row is the earlier run with the bone compliance also at 1e-9; with the
bones back at 5e-7/1e-6 the same point reads −39 px/s, adv +8.6, gain +15.7 —
i.e. it is a knife edge, not a reproducible operating point.)

### 8.3 What 1e-9 actually does

Run in the renderer (`visual_arm_pd`, legs at 1e-9): the character walks
**backwards** at 172% of the commanded speed — it is funny to watch and it is a
real measurement: `body: travelled -179.4 px/s against cmd_vx=104.4 px/s`.

At 1e-9 the legs overpower the hip pin: the pinned-frame rigidity collapses
(`max_rel_dev` 0.0497 → 0.38–0.62, versus 0.05 baseline) and the per-joint
solves overshoot into a limit cycle. Taming it does **not** work either:

| LEG_COMPLIANCE | extra lever | body px/s | stance drift | rig dev |
| --- | --- | --- | --- | --- |
| 1e-9 | `LEG_MAX_ANG_RATE=20` | −109.8 | 64.50 px | 0.2396 |
| 1e-9 | `LEG_ALPHA=0.5` | −86.0 | 64.77 px | 0.4470 |
| 1e-9 | `ATT_COMPLIANCE=1e-9` (stiffer pin) | −182.4 | 82.62 px | 0.6184 |
| 5e-9 | `ATT_COMPLIANCE=1e-8` | −62.2 | 54.25 px | 0.4378 |

### 8.4 Conclusion

- Stiffening the legs past **1e-6** does not buy a walk: the body stops or runs
  backwards, the stance foot slides 10–80 px, and the pinned frame tears. `1e-6`
  is the edge of stability, not a soft default — which is presumably why it was
  chosen.
- The one regime that looks like walking is ~5e-9 (`body +86 px/s`, swing advance
  +32 px, lead gain +71 px), but it is unstable run-to-run and slides 52 px while
  planted. Do not hard-code it.
- The real defect to fix is the **per-frame rest-angle jump** that the stiff
  spring then amplifies: `leg_alpha = 1.0` and `leg_max_angle_rate = 1e6` mean the
  rest angle teleports to the IK target every tick, and a stiff constraint turns
  that into an impulse. A rate-limited / critically-damped rest chase (or solving
  the IK target saturation from §7.3 first) is the way to make stiffness usable.
- Code state: `HipPinParams::default().leg_compliance` stays **1e-6** with the
  sweep documented at the field; the asset keeps its authored `5e-07` / `1e-06`
  joint values (dead for the legs' `Angular`, live for `Distance`).

Reproduce:

```bat
py bevy_vello\tools\physics-qa\sweep_leg_compliance.py --values 1e-6,5e-7,1e-7,5e-8,1e-8,5e-9,1e-9
py bevy_vello\tools\physics-qa\sweep_leg_compliance.py --values 1e-9 --extra-env ATT_COMPLIANCE=1e-9
py bevy_vello\tools\physics-qa\set_joint_compliance.py --compliance 1e-9 --dry-run
```

## 9. The gait was arc-starved: stiff legs + a taller step arc step properly

§8's conclusion ("stiff legs do not work") was **only half right**: it is true for
the *default step arc*. The user's read of the animation was better than the
table — the swing wasn't reaching ahead of the hip before touchdown, so the leg
pushed the body backwards. Two independent axes were being conflated:

- **stiffness** (`LEG_COMPLIANCE`) — can the leg execute the arc at all?
- **arc shape** (`STEP_HEIGHT`, `SWING`) — what arc is it asked to execute?

Both are now env-overridable (`STANCE`, `SWING`, `STEP_HEIGHT` in
[`HipPinParams::from_env`](../examples/game_lib/src/lower_body/hip_pin.rs:86)),
so they can be separated. `1e-6` × `1e-9` × `SWING` 0.28/0.14 at
`STEP_HEIGHT=70`:

| LEG_COMPLIANCE | STEP_HEIGHT | SWING | body px/s (cmd 104) | lift mean | swing adv | lead gain | stance drift | rig dev |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1e-6 | 28 (default) | 0.28 | +10.3 (10%) | 3.1 px | +1.9 px | +3.6 px | 3.13 px | 0.0497 |
| 1e-6 | 70 | 0.28 | +5.1 (5%) | **6.9 px** | −2.8 px | −11.4 px | 56.83 px | 0.3008 |
| 1e-6 | 70 | 0.14 | +4.5 (4%) | **6.4 px** | −0.7 px | −4.1 px | 53.64 px | 0.3055 |
| 1e-9 | 70 | 0.14 | +94.8 (91%) | 48.1 px | +49.2 px | +74.6 px | 78.54 px | 0.0858 |
| **1e-9** | **70** | **0.28** | **+104.5 (101%)** | **46.3 px** | **+42.4 px** | **+86.6 px** | 52.17 px | 0.1586 |

Reading the rows:

1. **At `1e-6` the leg cannot execute any arc.** Command `STEP_HEIGHT=70` and the
   realized lift is **6.9 px** — the compliant joint absorbs 90% of the swing
   amplitude, so the foot never clears the floor, never overtakes, and the body
   does not travel. Raising the arc or halving the swing time changes nothing.
2. **At `1e-9` the same 70 px arc is executed** (46 px realized) and the gait
   steps: advance +42 px, lead −42.8 → +43.7 px, body at 101% of command.
3. **Swing speed is not the driver** (the user's point): 0.28 s and 0.14 s give
   nearly the same result at `1e-9` — 0.28 s is actually *better* on body speed
   and stance slide. What mattered was stiffness *plus* a big enough arc.

### 9.1 The swing gate now passes (rendered twin, `LEG_COMPLIANCE=1e-9 STEP_HEIGHT=70`)

`analyze_swing.py` on `visual_step70.csv`:

| leg | swings | verdict | lift mean/min/max | advance | lead liftoff → touchdown | lead gain |
| --- | --- | --- | --- | --- | --- | --- |
| left | 15 | **PASS 15/15** | 55.6 / 43.6 / 89.3 px | +49.4 px | −38.1 → **+37.5** | +75.5 |
| right | 14 | **PASS 14/14** | 55.1 / 40.5 / 74.1 px | +28.8 px | −36.9 → **+42.4** | +79.4 |

Foot separation is now 38.1 px against a commanded 36.4 px (it was 197 vs 6.6),
and the body travels 87.6 px/s against 103.9 px/s (84%). This is a walk.

### 9.2 What is left

- **Stance slide 32–38 px** (rendered) / 52 px (headless): the planted foot is
  still dragged. Now that the gait is real, the grip axis of §6 is worth
  revisiting — it was tuned against a non-stepping gait.
- **Frame rigidity 0.10–0.16** vs 0.05 baseline: the stiff legs pull harder on
  the pin. `COMPLIANCE` / `ATT_COMPLIANCE` are the levers.
- Contact duty is now 0–1 contacts at the 1 s sampling points instead of 4–6:
  the big arc means long air time, so `stance`/`swing` timing deserves a sweep
  of its own.

### 9.3 Best known walk (reproduce)

```bat
cd /d D:\bender\bevy_vello
set "ASSET_ROOT=D:\bender\bevy_vello\examples\collision_detection\assets"
set "SCRIPT=D:\bender\bevy_vello\plans\walk_script.json"
set "LEG_COMPLIANCE=1e-9"
set "STEP_HEIGHT=70"
set "OUT_CSV=D:\bender\bevy_vello\tmp\headless\visual_step70.csv"
target\release\examples\visual_arm_pd.exe
```

The startup line now prints what the process actually got
(`walk params: leg_compliance=1.0e-9 ...`), so the window is self-identifying.

### 9.4 The startup "lean left" is an attitude-rest transient, not a body tilt

Watching the run, the upper body leans left for the first ~1.5 s (the body barely
moves), then leans right and the walk settles. Measured on `visual_step70.csv`
(loop-3 attitude columns):

| window | min `att_rest_cos` | equivalent rest re-aim | `meas_theta` RMS | `body_theta` range |
| --- | --- | --- | --- | --- |
| t < 2 s | 0.434 (@ t=1.59 s) | **64.3°** | 0.37 | −0.11 … 0.00 |
| t ≥ 2 s | 0.505 | 59.7° | 0.24 | 0.00 … 0.11 |

So the *reference* swings by up to 64° while the *measured* body angle stays
within ±0.11 (≈ ±6°): the lean is the attitude loop re-aiming its rest during
startup, and the body follows it back. `att_target` (the commanded bias) stays
within ±0.07 rad (≈ 4°), so it is the rest re-aim, not the commanded target.

Upper-body tuning is untouched, as noted — the levers are already there and can
be swept the same way: `ATT_COMPLIANCE`, `ATT_ZETA`, `ATT_TARGET_DEG`,
`ATT_MAX_DEG`, plus `COMPLIANCE`/`DAMPING` for the position pin. The natural first
question is whether the startup re-aim should be rate-limited the way the leg rest
angle should be (§8.4) — both are rest targets teleporting under a stiff spring.

## 10. Phase E — loop 3 is delivered by the legs, not the waist (session 5)

### 10.1 What changed

The plan §5 loop-3 actuator moved from the waist joint `P1_P2_P3` to the **knee
joints** `P2_P3_P40` / `P2_P3_P41`, matching the physical argument: the body
above P3 is a stiff stick, the only external forces are gravity, foot contact and
ground friction, so a vertical stick is decided by the *legs*. The two legs
apply opposite torques, so the stick swings upright instead of folding.

Implementation:

- `P1_P2_P3` is now a **posture hold only** — `tick_loop3_angular` emits
  `theta = 0.0` unconditionally, i.e. keep the spine straight. The old
  measured-attitude override at the waist is gone (the resolver's internal model,
  `tick_attitude`, still runs and still drives the external-pin airborne path).
- `resolver::attitude_lean_pd(theta_meas, omega_meas, bias, kp, kd, max_angle)`
  is the new pure loop-3 law: `lean = bias + attitude_pd(...)`, clamped to
  `±max_angle`. It returns an **angle**, not a torque.
- The angle is delivered through `LimbConfig::upright_reference_lean`, which
  tilts the world-upright reference axis `a_t` used by `calculate_leg_pd`
  (`a_t = hip + (L·sin(lean), −L·cos(lean))`). Both legs get the same lean.
- Gating in `drive_hip_pin`: `attitude_enabled && !external_pin && grounded &&
  measured.n > 0`, else `lean = 0`. So flight is ballistic (no contact ⇒ no leg
  authority) and the mid-air assist pin keeps the frame it owns.
- New knobs: `ATT_LEAN_KP` (default 0.5), `ATT_LEAN_KD` (default 0.03), also
  exposed as sliders in `hip_pin_ui`.

Sign chain (test-verified): `measured_attitude` is `+` for a forward lean; the
loop returns `−`, which tilts `a_t` backwards, which yields a **negative** hip
rest joint angle, which makes the stance legs rotate the trunk back upright.
Negative feedback, not positive.

Because rest == `a_t` angle == `lean`, the joint error under a lean of `θ`
becomes `θ_live − lean = θ + Kp·θ = (1+Kp)·θ`: the knob is effectively extra
proportional gain on the leg PD's upright reference, with `Kd` supplying the
damping. That is physically coherent — a stronger upright pull, not a new
mechanism.

### 10.2 Run (lean on, the new default)

```
LEG_COMPLIANCE=1e-9 STEP_HEIGHT=70 ./target/release/examples/headless_arm_pd
```

- `body: rows=1036 nan_rows=0 rig=FAIL max_rel_dev=0.1502`
- rigidity: `P1-P2=0.0419 P2-P3=0.0076 P1-P3=0.1502 P3-P40=0.0047 P40-PLLL=0.0043
  P3-P41=0.0083 P41-PRLL=0.0041`
- `leg: peak left=78.1px right=57.9px combined=78.1px tail=WAGGLE(rms=19.958 rev=7)`
- swing: **left 15/15 PASS, right 13/14 (NO_ADVANCE ×1) → 28/29**; body
  **99.5 px/s vs cmd 103.8 px/s (96%)**; lift mean ~50 px; lead −38 → +38 px;
  foot sep 38.9 vs 38.8 commanded
- slide: **SLIDING**, worst stance drift 61.14 px (left mean 19.0/max 61.1, right
  mean 23.7/max 52.1)

### 10.3 A/B control (lean off)

Same command, but `ATT_LEAN_KP=0 ATT_LEAN_KD=0` — the waist is still straight,
so this isolates the leg-delivered lean:

| metric | lean off | lean on |
| --- | --- | --- |
| rig `max_rel_dev` (`P1-P3`) | 0.2254 | **0.1502** |
| swing pass | 26/29 (L 14/15, R 12/14) | **28/29** (L 15/15, R 13/14) |
| body travel | 74.4 px/s (71% of cmd) | **99.5 px/s (96%)** |
| foot slide worst / mean | 57.0 / 15.3–23.2 px | 61.1 / 19.0–23.7 px |

The lean wins on rigidity, on stepping, and on forward progress; the only cost is
a marginally worse worst-stance slide (+4 px on one stance), which is the known
friction/tracking regression, not a loop-3 effect.

### 10.4 Verdict against the relaxed bar

Accepted. The walk clearly steps (feet lift ~50 px and land in front of their
liftoff x), the hip advances at 96% of the commanded speed, the physics is
finite (`nan_rows=0`), and the tuck (`P1-P2`, `P2-P3`) is *tighter* than before.
Known residuals, unchanged by this phase: stance slide (~60 px worst) and the
rigidity gate (`P1-P3` 0.15 vs the 0.05 baseline) — both are §6/§8 levers
(contact gating, friction), not loop-3 delivery.

Reproduce (Linux, from `bevy_vello/`):

```bash
ASSET_ROOT=$PWD/examples/collision_detection/assets \
SCRIPT=$PWD/plans/walk_script.json OUT_CSV=/tmp/phase_e_qa/loop3_leg.csv \
LEG_COMPLIANCE=1e-9 STEP_HEIGHT=70 ./target/release/examples/headless_arm_pd
python3 tools/physics-qa/analyze_swing.py /tmp/phase_e_qa/loop3_leg.csv
python3 tools/physics-qa/analyze_foot_slide.py /tmp/phase_e_qa/loop3_leg.csv
```

Control run: add `ATT_LEAN_KP=0 ATT_LEAN_KD=0`.

## 11. Phase F — contact-gated gait clock: measured, and **rejected** (session 6)

Plan §6 asked for the gait clock to take its authority from real contact rather
than from timers. The contact layer already existed (`update_ground_support`
reads the previous tick's `CollisionEventBatch`); what was missing was *per-foot
resolution*. That was built, measured, and it makes the walk worse at every
level. The mechanism stays behind a default-off switch.

### 11.1 What was built

Per-foot ground truth in [`GroundSupport`]: each foot collider (`LLL`/`RLL`,
matched via `Connectivity.name`, which is the asset's `path_id`) gets its own
`FootContact { touching, on_ticks, off_ticks, normal, point, normal_speed }`,
debounced with `CONTACT_ON_TICKS = 2` / `CONTACT_OFF_TICKS = 1`. The resolver
receives raw `touching` (not the debounced flag) plus the measured foot x mapped
into resolver space; the hysteretic flag is used only for the loop-2 thrust gate,
where a stable signal matters more than a one-tick-late one.

Three mechanisms sit behind `GaitAuthority`, **additive** so each can be A/B'd:

| level | plant re-anchor | early lift-off | swing extension | thrust gate |
|---|---|---|---|---|
| `Timer` (default) | — | — | — | phase |
| `Observe` | — | — | — | phase |
| `Contact` | once per stance | — | — | per-foot contact |
| `Full` | once per stance | yes | yes | per-foot contact |

`GAIT_AUTHORITY=timer|observe|contact|full` switches it at runtime; 154 unit
tests cover the ladder, including one asserting `Timer` is bit-identical to the
pre-§6 clock.

### 11.2 Result — every level is worse than the control

All runs `LEG_COMPLIANCE=1e-9 STEP_HEIGHT=70`, `walk_script`, 1036 rows,
`nan_rows=0`.

| metric | `Timer` / `Observe` | `Contact` | `Full` |
|---|---|---|---|
| swing pass | **28/29** | 17/29 | 14/27 |
| rig `max_rel_dev` | **0.1502** | 0.1874 | 0.3144 |
| `P1-P2` tuck | **0.0419** | 0.0599 | 0.1824 |
| `P2-P3` tuck | **0.0076** | 0.0128 | 0.0689 |
| stance drift worst | **61.1 px** | 157.3 px | 235.6 px |
| leg peak | **78.1 / 57.9 px** | 105.5 / 79.6 px | 108.1 / 104.4 px |
| tail | WAGGLE rms 19.958 rev 7 | WAGGLE rms 34.829 rev 7 | WAGGLE rms 13.567 rev 12 |

`Timer` and `Observe` are **byte-identical** (`md5 d6c97f8a…`), which is the
proof that the contact layer is correctly wired and non-perturbing. Every
measurement-based correction then *degrades* the walk monotonically with
authority.

### 11.3 Why — three distinct causes, all confirmed

1. **The plant re-anchor as originally written was a positive feedback loop.**
   The first version re-anchored every tick contact held: `plant_pos` sets the
   foot target → leg PD drives the foot there → next tick measures the drifting
   foot → moves the plant to it. That drags the gait plan along with the slip it
   was supposed to resist. Measured cost: 61 px → 130 px of drift. Restricting
   it to **once per stance** (`plant_anchored`) fixed the loop but still landed
   at 157 px, which leads to cause 2.
2. **Fixing the plant to the *measured* foot removes the only absolute
   reference in the loop.** The stride was implicitly anchored by
   `plant_pos` being a *plan* value that the leg PD chased and lost against by a
   consistent, bounded amount. Following the measurement instead makes each step
   inherit the previous step's slip, so error accumulates: 61 → 157 px.
3. **The two timing mechanisms fight the anti-phase interlock.** `step_leg` only
   lets a leg lift off when the *other* foot is planted. Early lift-off fires on
   a one-tick-stale contact report (the batch is written at the end of
   `FixedUpdate` and read at the start of the next tick), so it can launch a
   swing against a foot that is one tick from landing. The still-airborne
   opposite leg then blocks the swap, the swing extends waiting for contact that
   arrives late, and the stance geometry collapses: `P1-P2` 0.042 → 0.182,
   `P2-P3` 0.008 → 0.069, swing pass 28/29 → 14/27.

### 11.4 Verdict

Do **not** enable §6 for this character as it stands. `GaitAuthority::Timer`
remains the default and reproduces the Phase E numbers exactly.

The negative result is worth more than the feature: it shows the ~61 px residual
slide is **not** a gait-clock authority problem. The clock and the physics agree
about *when* things happen; they disagree about *where the foot is*. That is the
foot's own contact/friction response (plan §8), not the resolver's timing. §6
would only pay off after the foot stops slipping, at which point the measured
plant and the planned plant coincide and the correction becomes a no-op.

Reproduce (from `bevy_vello/`):

```bash
for LVL in timer observe contact full; do
  ASSET_ROOT=$PWD/examples/collision_detection/assets \
  SCRIPT=$PWD/plans/walk_script.json OUT_CSV=/tmp/phase_f_ab/$LVL.csv \
  LEG_COMPLIANCE=1e-9 STEP_HEIGHT=70 GAIT_AUTHORITY=$LVL \
  ./target/release/examples/headless_arm_pd
  python3 tools/physics-qa/analyze_swing.py /tmp/phase_f_ab/$LVL.csv
  python3 tools/physics-qa/analyze_foot_slide.py /tmp/phase_f_ab/$LVL.csv
done
```

## 12. Floor fix + gesture tool → the asymmetry is a left-side advance defect (session 7)

### 12.1 Floor/standing-height fix landed

The resolver previously assumed a virtual floor at `floor_y = 0` with `hip_height
= 90` px, while the collision response places the real floor a live stance
(~106 px) below the hip — a ~16 px divergence. `hip_pin` now latches the realized
standing height from the live leg seed each time it is (re)seeded
(`standing_height = hip.y - foot.y`) and overrides the resolver config with it,
so the resolver's ground-referenced geometry (stand height, swing plant, reach
`X_max`, LIPM, stand regulator) computes against the real floor. `measure_body`
also anchors its floor at `anchor_p3.y + standing_height` instead of the
hardcoded `y = 0`, un-starving CoM-height / apex feedback.

### 12.2 New gesture tool: `analyze_gesture.py`

Reads the whole-body walk CSV and describes the walking gesture per gait cycle
from hip `P3`, knees `P40`/`P41`, feet `PLLL`/`PRLL`, head `PH`. Per swing it scores
the three acts that make a real step — **lift** (max foot clearance), **advance**
(gain of foot − body x), **overtake** (swing-over-stance foot lead gain) — plus
knee flexion, head bob, hip drop, and body speed. Prints a left/right summary, an
optional `--per-cycle` table, and an overlaid 3-panel `--plot` (forward x /
vertical lift+bob / knee flexion).

Usage:

```bash
python3 tools/physics-qa/analyze_gesture.py tmp/headless/walk3/w_5e-9_sh70.csv --per-cycle
python3 tools/physics-qa/analyze_gesture.py tmp/headless/walk3/w_8e-9_sh50.csv --plot /tmp/g.png
```

### 12.3 Finding — the walk is now real, but one-leg asymmetric

With the floor fixed, the body travels 28–85 px/s (real forward motion, up from
"hardly scratches"). The swing gate (analyze_swing) already showed right-leg
steps cleanly while the left is flagged `NO_ADVANCE`. The gesture tool pinpoints
*why*: the two legs step with near-identical knee flexion (~45–63°), but the
**left leg fails specifically on the advance metric** while the right is a
consistent stepper.

| run | body px/s | left (pass/total) | right (pass/total) |
|-----|-----------|-------------------|--------------------|
| w_5e-9_sh70  | 60 | 8/15  | 13/16 |
| w_5e-9_sh110 | 84 | 9/15  | 11/16 |
| w_8e-9_sh90  | 85 | 9/15  | 12/16 |
| w_5e-9_sh50  | 67 | 6/15  | 14/16 |
| w_8e-9_sh50  | 41 | 3/15  | 14/16 |
| w_1e-8_sh70  |  1 | 4/15  | 10/16 |
| w_5e-9_sh28 |  <1 | 0/15  |  3/16 |

Per-cycle detail (w_8e-9_sh50): left `adv` swings −26.5 … +38.8 (mean −5.7) so
most left swings end NO; right `adv` stays +13…+46 (mean +25.6) so nearly all
right swings STEP. Two spatial clues corroborate a left-side placement defect:
the left floor (max planted foot y) sits consistently higher than the right
(e.g. 292.3 vs 288.7, 284.8 vs 283.8 — the left foot does not plant as low), and
the left stance holds slightly higher.

### 12.4 Verdict / next lever

This is **not** a compliance or lift problem — the left leg flexes the same and
lifts fine; it repeatedly finishes its swing **without having gained ground on
the body** (the body overlaps/outruns the short left stride), so the left foot is
"creeping forward with the body" instead of being placed ahead of it. The lever
that now matters is **per-leg stride length / foot-placement x** (loop-2 foot
placement and how far the swing leg leads before planting), lengthened on the
left to match the right. Best walk on the fixed floor so far: `LEG_COMPLIANCE=5e-9
STEP_HEIGHT=70..110`.

## 13. Foot-physics diagnosis — the missed cycles are airborne, not stuck (session 8)

### 13.1 Why this matters

Before changing stride/step-code we must know *what the foot physically did* on
the swings that failed (→ NO). "Stuck on the floor" and "can't reach the floor"
demand opposite fixes. The gesture tool now classifies every swing by the foot's
actual vertical state, independently of the pass gate:

- **stuck** — the foot never left the floor: `max_lift < gate`. (floor too sticky
  / leg too weak to lift)
- **midair** — the foot left the floor but never came back to it: `min_clear`
  (deepest the foot reaches) is still above tolerance. The body is riding high
  enough that the foot simply cannot touch down in that window.
- **lifted** — the foot rose *and* re-contacted the floor; it only failed on
  advance (stride/timing), not on reaching the floor.

`airFr` (fraction of the swing with clearance > tolerance) complements this.

### 13.2 Result (per-cycle, only failed → NO swings listed)

**w_5e-9_sh70** (body 60 px/s) — all 11 failed swings are `phys=midair`, `air=1.00`:</br>
`left: 1.73,2.32,3.50,4.68,5.27,7.03,8.21` (adv −40..−10, gains 1..54) — left NOs are airborne-whole
`right: 6.14,9.09,9.68` — same, midair.

**w_8e-9_sh50** (body 41 px/s) — the slow walk is the one where failures are mostly `lifted`:</br>
`left` NO cycles: `1.73 midair`, then `2.32,2.91,3.50,4.09,5.27,6.44,7.62,8.21,9.39,9.98` all `lifted`
(air=0.88–0.96, i.e. the foot *does* touch the floor), `4.68 midair`. For the left, 11/12 misses are
lifted — foot reaches the floor but gains no ground.

**w_5e-9_sh110** (body 84 px/s) — the high-arc walk: `lift=143`(`!`), `maxFlex` 66–98°, all 16 NOs `midair`,
air=1.00 with bob 51–63 px and hipDrop 38–58 px. The body `bounces` up and the foot never lands.

**w_8e-9_sh90** (body 85 px/s) — 9/10 NOs midair (air=1.00, bob 21–37 px); only 1 lifted.

### 13.3 What this proves

Across the band **stuck = 0 for every run** — no foot was gluey enough to fail
the lift in this compliance range. The dominant cause of a missed cycle depends
on it: at high step height / high speed (sh70+, sh90, sh110) the failures become
**midair** — the body is *bouncing* (hipDrop 38–66 px, head bob up to 71 px) so the
swinging foot is airborne for the entire swing and never reaches the floor; those
are the "right leg too strong ⇒ body hops ⇒ left foot can't reach the floor"
swings the user suspected. At the slow sh50 the failures are mostly **lifted** —
the foot does touch down but still fails on advance.

So there are **two** distinct physical failures, not one: (a) *vertical hop /
body bounce* starving the foot of contact (dominant at high step height), and
(b) *insufficient stride advance* even when the foot lands (dominant at low step
height). The gait is oscillating between "too bouncy" and "not advancing."

### 13.4 Targeted next lever

too hot or the swing arc apex being too high for the plant. The first thing to
test is **lowering STEP_HEIGHT toward ~50 while keeping compliance 5e-9** (which
removes the midair hops), then **lengthening the left stride** to fix the
remaining lifted-but-no-advance. If confirming the hop hypothesis: log per-tick
`hip clearance = floor − hip.y` across the walk and diff STEP_HEIGHT 50 vs 90 at
the same compliance — hop amplitude should scale with step height, and the
midair:pixel count should fall correspondingly.

## 14. Root cause — the "walker" is running a hopping engine, not walking (session 9)

### 14.1 The user's question changed everything

While deciding whether to tune physics-world knobs (friction, collision scale)
vs character/gait knobs, the user asked the decisive question: "**why are we
running a jumping loop for walking?**" The answer is the root cause of *all* the
weird behaviour we chased in §12/§13.

Physics knobs are already settled and were never the problem:
- collision scale is fine — the character stands with <1 px wobble (§ stand sweep);
- friction is fine — the `SLIDING` verdict from `analyze_foot_slide.py` (55–67 px
  stance, peak 415–450 px/s) is an **artifact of the hop**: the feet are airborne
  ~100% of the run (only 3 of 1036 ticks within 3 px of the "floor"), so the
  "slide" is the feet being *dragged forward mid-air* by the hopping body, not a
  loss of ground grip.

### 14.2 Correcting the mechanism — the walk is NOT a hopping loop

The vertical phase is exported as `vphase` in every telemetry row. Across the
whole 11.5 s walk it is **`0` (Stand) for all 1036 ticks** — the ballistic
`Load → Launch → Flight → Land` path **never runs on a pure walk**. The user's
instinct ("don't over-rotate to a jump loop") was right; the resolver's
[`tick_vertical`](examples/game_lib/src/lower_body/resolver.rs:686) correctly
holds the hip on the floor line via `regulate_height` the whole time.

What the telemetry *does* show is a real, large vertical bounce of the heavy hip:

| hip height above floor (stand=106 px) | value |
|---|---|
| range | 84.4 … 168.4 px |
| mean | 122.0 px |
| % above stand+2px | **92.9%** |
| peak overshoot over stand | ~62 px |

So the hip is pushed up to ~1.6× its standing height and oscillates 84→168 px
every cycle, driven by **loop-2 / `step_leg` ground reaction**. At each
stance→swing swap the stance leg's thrust delivers an upward impulse into the
heavy hip (`P3vy` peaks ≈ +23 px/s), the hip rises, the leg's foot leaves the
floor, and the vertical regulator pulls it back down. That is the *bounce*
behind §13's `midair` (foot airborne ~100%) and the left `NO_ADVANCE`. It is a
**leg-driven vertical impulse into a hip too heavy / a vertical regulator too
stiff to absorb it** — not a hop engine.

### 14.3 STEP_HEIGHT sweep (physics fixed, only gait knob swept)

| STEP_HEIGHT | body px/s | left stck/air/lift | right stck/air/lift |
|---|---|---|---|
| 28 | **-60.8** (backward!) | 0/15: 0/6/9  | 3/16: 0/11/5 |
| 50 | 67 | 6/15: 0/8/7  | 14/16: 0/14/2 |
| 70 | 60 | 8/15: 0/13/2 | 13/16: 0/14/2 |
| 90 | 35 | 8/15: 0/11/4 | 10/16: 0/13/3 |
| 110 | 84 | 9/15: 0/15/0 | 11/16: 0/14/2 |

`stuck=0` everywhere (friction is fine). At low STEP_HEIGHT (28) the thrust surplus
is so small the walk collapses to backward; raising it inflates the bounce until
the left foot is midair 15/15 (sh110). No STEP_HEIGHT fixes the bounce because the
surplus vertical impulse is intrinsic to the stance-push.

### 14.4 Verdict and the gait-code change

**The forward step's stance thrust over-shoots vertically** — it lifts the heavy
hip off the floor instead of keeping the feet planted, so the foot can't stay
down to grip/advance. The fix is to keep the legs' step **flat on the floor**:
- soften / clamp the vertical component of the stance thrust so the hip does not
  rise above the stand line (or lower `VERTICAL_K_S/K_D`, or add hip vertical
  damping that resists the upward push),
- keep the swing as the only vertical lift, and only for the swing foot.

This is the change that will keep the stance foot planted (no midair/drag), let
the swing foot reach the floor, and give a symmetric, ground-contact walk.
`sweep_step_height.py` + the `vphase`/hip-height checks are kept as the regression
harness (hip stays within ±2 px of stand, `midair` → 0, both legs pass).

## 15. Startup/velocity mismatch + loop-2 coupling → the one-leg "hop" (session 10)

### 15.1 The user's mechanism (confirmed by telemetry + code)

The persistent "one leg hopping all the time" is not a deliberate jump. Three
combined causes:

1. **Startup velocity mismatch.** At the walk start `cmd_vx` jumps toward 120 px/s
   almost instantly (I measured it ramping 3.5 → 14 → 28 → 42… within ~0.1 s)
   while the body's realized `hip_vx` is ~0–35 px/s and even slips **negative**
   at the first push (t≈0.57–0.60 s `hip_vx ≈ −24…−33`). Loop 1/loop 2 read that
   gap as a huge balancing error.

2. **Fixed stance/swing timers.** `stance_ts` / `swing_ts` are constant, so the
   gait advances to the next step on schedule **whether or not the previous foot
   planted** and **whether the hip is mid-bounce**. A bad (airborne, early) state
   is not waited out; it is stepped forward from.

3. **Loop-2 foot placement runs ahead (Raibert coupling).** With the measured
   loop active, [`plan_target_x`](examples/game_lib/src/lower_body/resolver.rs:1251)
   targets the LIPM capture point `x_CoM + (v_CoM − v_des)/ω₀`. While `v_CoM`
   lags `v_des`, the demanded plant is far ahead of the hip (clamped to
   `hip ± x_max`). Then
   [`compute_support_velocity`](examples/game_lib/src/lower_body/resolver.rs:1009)
   (the generalized backward solve) converts that plant offset into a **hip
   velocity** `(delta_x + kv·v_des)/(Ts/2+kv)` that keeps *demanding* the body
   leap forward. Loop 2 therefore converges toward a one-leg-hop: the leg plants
   far ahead and the feedback demands the hip bound forward to reach it.

Measured signature that matches: the hip rides above `stand+2px` for 92.9% of
the walk (peak ~62 px over stand), peaking in the ~0.1 s after each step; and at
sh50 the left leg is airborne-but-lifted while the right is the consistent
stepper → the "leading right" leg is the one doing the leaping.

### 15.2 What to change (all gait code, physics settled)

Working on top of the settled physics, the fix addresses the startup/timer/coupling:

- **A. Cap / ramp `v_desired` to the body's grasp** — don't command 120 while the
  realized speed is ~0. Ramp `cmd_vx → v_des` over a few cycles (or clamp the
  capture-point `v_des` contribution) so the plant target does not leap ahead of
  reach at startup.
- **B. Let the gait wait for contact** — gate the stance→swing advance on the
  planted foot actually holding (contact / vertical settled), i.e. a **contact-
  or ground-gated timer**, instead of a pure fixed timer, so an airborne/bouncing
  state is not stepped forward from.
- **C. Re-tune loop-2's `kv` and/or how `compute_support_velocity` feeds the hip**
  so a single overly-forward plant is not converted into an ever-escalating hip
  velocity demand (break the "plant far ahead → demand hop" loop).

These are the levers to discuss next. The regression harness from §14
(hip within ±2 px of stand, `midair` → 0, both legs pass) verifies whichever
combination we pick.

## 16. How the feedback loop is wired, and how this class of problem is usually fixed

### 16.1 The architecture (measured-state feedback, not open-loop)

The resolver closes on **measured** state, never on its own commanded pose:

1. **Measurement** — [`measure.rs`](examples/game_lib/src/lower_body/measure.rs)
   produces `CaptureMeasure { com_x, v_com, z_com }` + a measured CoM apex, fed
   into the resolver as `state.measured` / `state.measured_apex`
   ([`resolver.rs`](examples/game_lib/src/lower_body/resolver.rs:393)).
2. **Loop 1 (height)** — `tick_vertical` closes on the measured resulting state;
   `regulate_height` integrates the hip against the measured gap; the apex latch
   prefers the measured CoM apex over the internal ballistic model.
3. **Loop 2 (foot placement)** — [`plan_target_x`](examples/game_lib/src/lower_body/resolver.rs:1251)
   computes the swing plant from the measured capture point `x_CoM + (v_CoM − v_des)/ω₀`
   (falls back to the Raibert neutral point only when no measurement). This is the
   live feedback: **where the foot plants is a function of the measured velocity
   error.**
4. **Plant re-anchor (`Contact`)** — `step_leg` re-anchors a stance plant to the
   measured foot x, but only **once per stance** (`plant_anchored`). The comment
   at [`resolver.rs:248`](examples/game_lib/src/lower_body/resolver.rs:248) states
   the trap explicitly: *"Re-anchoring every tick is a positive feedback loop:
   target → PD → measured → target."* They already guard against self-feeding.

### 16.2 Why loop 2 turns into a one-leg hop

Loop 2 is a **velocity-error feedback**: `plant_target = x_CoM + (v_CoM − v_des)/ω₀`.
At startup `v_CoM≈0` while `v_des=120`, so the plant demands land far ahead; the
backward solve [`compute_support_velocity`](examples/game_lib/src/lower_body/resolver.rs:1009)
`(delta_x + kv·v_des)/(Ts/2+kv)` then feeds that same over-forward error back into
a hip-velocity demand every stride. With fixed stance/swing timers (§15.1-2) the
loop cannot "wait it out" — it compounds until it converges to the hop.

### 16.3 The usual ways this is resolved (all control-structure, physics untouched)

- **Slew / rate-limit the reference.** Don't feed the loop a step change it can't
  track 1:1. Ramp/`approach()` `cmd_vx → v_des` (the code already has
  [`approach()`](examples/game_lib/src/lower_body/resolver.rs:604)) so the
  bootstrap transient does not feed a huge plant error. *This is the canonical
  fix for a loop that oscillates/steps wildly at startup.*
- **Gate on "good" feedback only.** Only act on a plant the physics actually
  delivered — the `plant_anchored` once-per-stance guard and the
  `GaitAuthority::Observe/Contact` ladder are the shells of this. Extending it to
  *wait for contact before advancing the gait* (§15.2 B) is the same principle.
- **Bound the loop gain.** Cap the speed ramp and the support-velocity feed so one
  bad plant cannot escalate into an ever-larger hip demand; the capture point is
  already clamped to `hip ± x_max`, the missing piece is bounding the *rate* and
  the *velocity feed*.

**Bottom line:** the textbook resolution is slew the velocity reference (A) so the
bootstrap doesn't overreach, **and** gate the gait on measured contact (B) so an
airborne/bouncing state is never stepped forward from. Together they break the
self-amplifying hop loop without touching the settled physics.

## 17. Design pivot — event-driven gait (user direction)

> "maybe it is wrong to set an exact swing time and step time... maybe it should be
> about if the COM_x is moved across the ahead feet we should start the swing of the
> behind feet. swing and stance is only for character to fall over. we can have a
> fixed time for the sand body cause it is kinematic and responds instantly. physics
> world is lagging, we should have the walking cycle adapt to it instead of having
> the physics world adapt to a fixed walk cycle." — user

The user's proposal: advance the gait **off measured state**, not off a fixed clock.
The timer becomes a *fallover bound* (how long a leg can stay planted/swinging before
the soft body would tip), not the scheduler itself.

### 17.1 Current gating (what actually triggers phase changes)

- **Swing → Stance: already contact-gated (`Full`).** [`resolver.rs:1480`](examples/game_lib/src/lower_body/resolver.rs:1480)
  holds the final swing point and waits for a measured touchdown before declaring
  Stance (bounded by `max_swing_wait`). The foot does not plant on the timer alone.
- **Stance → Swing: pure timer.** [`resolver.rs:1399`](examples/game_lib/src/lower_body/resolver.rs:1399)
  counts down `stance_duration_ts`; [`resolver.rs:1429`](examples/game_lib/src/lower_body/resolver.rs:1429)
  ends stance on `phase_timer <= 0.0` (or early lost-contact). The advance condition
  at [`resolver.rs:1430`](examples/game_lib/src/lower_body/resolver.rs:1430) only
  requires move intent (`v_desired > SWING_TRIGGER_SPEED`) + not-other-swinging. It
  **never checks whether the CoM has actually advanced past the planted foot.**

So the exact gap is **liftoff is timer-scheduled, not state-triggered** — the leg is
picked up because a clock expired, even while the CoM is still over/behind the foot.
That is the mechanical source of the one-leg hop: the cycle is forced forward into a
state the physics has not delivered.

### 17.2 Proposed change

- Make the stance→swing event **"CoM has advanced past the stance foot."** Direction
  aware (v_desired sign). Roughly the Raibert neutral-point test:
  `sign(v_des)*((coM_x - plant_x) - lead_fraction*stride) > 0` → want_step.
  If v_des ≈ 0, keep the stop-settle behavior (no advance).
- Demote `stance_duration_ts` / `swing_duration_ts` to **safety clocks only**: a max
  stance (don't let the leg stay planted forever / soft body tips) and a max swing
  (don't let the foot dangle). They cap runaway, they do not schedule.
- Keep the existing contact gates (`lost_contact_early`, touchdown wait) — they are
  already the correct side of this design; the missing half is the liftoff trigger.

Net: the walk circle adapts to the physics (steps when the body has really crossed the
plant), the kinematic resolver stops commanding a cycle faster than the soft body can
respond, and the timers remain purely as fallover safety. This is the implementation
direction we take next.

## 18. "How fast to swing" and "when to land" — what the algorithm already gives us

The algorithm already determines **where** to land and **what** the foot does at every
instant; what it does **not** derive is **how fast** to traverse the swing. The gap:

- **Land *where* — live, already closed on measured state.**
  [`capture_point`](examples/game_lib/src/lower_body/resolver.rs:1191)
  `x_land = com_x + (v_com − v_desired)/ω₀`, `ω₀ = √(gravity/z_com)` (LIPM capture
  point). [`plan_target_x`](examples/game_lib/src/lower_body/resolver.rs:1251) clamps
  to `hip ± x_max` and falls back to the Raibert neutral point only when unmeasured.
  So the landing spot is a function of `v_com` + `v_desired` — not a timer.
- **Foot *path* — fully specified.**
  [`swing_profile_foot`](examples/game_lib/src/lower_body/resolver.rs:1290) is a C¹
  lift→advance→extend Hermite spline from lift-off `from` to the continuously
  re-planned target `to` ([`resolver.rs:1455`](examples/game_lib/src/lower_body/resolver.rs:1455)).
  Clearance, forward advance, and a shallow touchdown approach are baked into the
  control points.
- **Land *when* / swing *speed* — fixed clock, NOT state-derived.**
  The spline parameter `s` is advanced linearly `1→0` over `swing_duration_ts`
  ([`resolver.rs:1458`](examples/game_lib/src/lower_body/resolver.rs:1458)). So today
  swing speed ≡ `distance / swing_duration_ts`, a constant, even though the *target* is
  re-planned live each tick. The missing piece is a state-derived rate.

### 18.1 The event-driven rule that completes the design

Traverse the spline so the foot reaches the capture point exactly when the CoM does:

```
T  = |x_land − current_hip_x| / v_desired   (with a floor ≈ 0 + small epsilon)
ṡ  = 1 / T
```

Slow body → slower swing to match; fast body → quicker swing. The `Full` contact gate
([`resolver.rs:1480`](examples/game_lib/src/lower_body/resolver.rs:1480)) already holds
the touchdown point until measured contact. Combined with §17 (lift on CoM crossing),
this closes the loop:

- lift when the CoM has passed the stance foot (§17, event-driven) — **when to lift**;
- swing at `ṡ = 1/T` so the foot arrives as the CoM does — **how fast**;
- land at the capture point + contact-gated touchdown — **where + when to land**.

Timer demoted to a fallover safety cap only. Physics untouched.

### 18.1 Refinement (user) — anchor the swing midpoint to the measured CoM

> "we know how fast coM is moving. we just make sure the feet is right beneath the
> com for half of the swing time." — user

Simpler and more robust than the `ṡ = 1/T` arrival rule in §18. Keep the swing **timer
only as the timebase for the midpoint** ("half the swing time") and anchor the foot's
horizontal position so it sits directly under the CoM at `s = 0.5`:

```
com_mid = com_x0 + v_com · (swing_time / 2)   // v_com measured live
foot_x(s = 0.5) = com_mid
```

Split the swing at the midpoint: the first half carries the foot from the old plant to
`com_x0 + v_com·(swing/2)`, the second half from under the CoM to the capture point.
Maps onto [`swing_profile_foot`](examples/game_lib/src/lower_body/resolver.rs:1290):
today the horizontal control points are shape constants (`0→1.0dx`, midpoint ≈ `0.49dx`)
independent of the body ([`resolver.rs:1301`](examples/game_lib/src/lower_body/resolver.rs:1301));
we make the midpoint `= com_mid`, a measured anchor.

Same pacing benefit as §18 (slow body → the foot lingers near the plant mid-swing and
does not overreach; fast body → it crosses under the CoM toward the capture point), but
no `T`-floor and no divide — just a measured clamp. The timer stays as the midpoint
timebase (the user's "half the swing time"); horizontal position becomes state-driven.
Physics untouched.

## 19. How established legged locomotion actually solves this (canonical precedents)

The user asked what established practice does for this exact problem — it is old and
well-solved. Our design maps onto it well, with two upgrades to borrow.

### 19.1 What our design already matches (confirming §17/§18)

- **Lift on "CoM passes the stance foot" (§17)** is the *neutral point* / *leg-crossing*
  footstep trigger from Raibert (1986) through modern machines (MIT Cheetah, Cassie,
  ANYmal). Deployed systems don't step on a wall clock; they output an online footstep
  *plan* and spline the swing leg to it. → standard, keep.
- **Timer as a fallover safety cap** is the *stance deadline / max-swing timeout* that
  every legged controller keeps as a failsafe. → standard, keep.

### 19.2 Two things mainstream practice adds that we should adopt

1. **Deadbeat timing — use *time-to-touchdown*, not half the (constant) timer.**
   The capture-point camp (Pratt 2006) and modern quadrupeds explicitly compute the
   remaining swing time from `v_com`: `T_remain = (x_land − x_com)/v_com`, then re-derive
   the swing spline over `T_remain` each step. This is strictly more correct than
   "half the swing timer" (§18.1) because it re-schedules instantly when `v_com` changes
   mid-swing. → upgrade §18.1's midpoint rule into a deadbeat time-to-touchdown schedule.
2. **Rate-limit the reference `v_des` (reference generation).** Every real controller
   ramps desired velocity instead of stepping it 0→120. This is the single most canonical
   fix in the field, and it removes the overreach feedback (§15.1/§16.3 option A) at the
   source. We already have [`approach()`](examples/game_lib/src/lower_body/resolver.rs:604).

### 19.3 Net recommendation (all gait/resolver code, physics untouched)

- Lift: CoM-passes-foot (§17) — keep.
- Swing: schedule over remaining time to the capture point: `T_remain = (x_land − x_com)/v_com`
  (§18.1 upgraded to deadbeat).
- Reference: rate-limit `cmd_vx → v_des` via `approach()` (§19.2.2, the canonical hop fix).
- Timer: fallover cap only — keep.

## 20. Implementation + verification (event-driven gait landed)

Implemented all three changes in `resolver.rs` (pure gait code, physics untouched):

1. **Reference ramp** (§19.2.2): new `ResolverState::v_ref`, ramped each tick via
   `approach(v_ref, v_cmd, accel·dt)` — the capture point & backward solve now see a
   smoothly rising reference instead of a `0 → 120` step.
2. **Event-driven lift** (§17): `stance_over` also fires when the measured CoM has
   advanced past the stance foot in the direction of `v_desired`. The stance timer is
   demoted to a fallover cap.
3. **Deadbeat swing pacing** (§19.2.1): the swing spline advances by
   `T_remain = |x_land − x_com| / v_com` (capped by `swing_duration_ts`, floored), so
   the foot arrives when the body does instead of on the fixed swing clock.

Verification (headless walk, `LEG_COMPLIANCE=5e-9`, window 1.6–10.5 s, gesture
classifier pass/`n` = swings passing lift+advance+overtake):

| stepHt | body px/s | left pass/n (`stck/air/lift`) | right pass/n (`stck/air/lift`) |
|-------:|----------:|------------------------------:|-------------------------------:|
| 28     | 69.6      | 6/15 (`0/13/2`)              | 7/16 (`0/14/2`)               |
| 50     | 107.2     | 9/15 (`0/9/6`)               | 15/16 (`0/11/5`)              |

**Reading.** Compared with §14.3 (pre-change: sh28 → **backward −60.8**, sh50 → 67):
the walk now moves **forward** at every height, including the previously-backward 28.
`stuck = 0` everywhere (friction/contact fine). The right leg is now a consistently
stepping leg (15/16 at sh50, 0 midair), confirming the right-leg hard-wire (§H1) no
longer fanatics; the asymmetry that remains is **left-leg midair/lifted**, so the
direction of the residual work is now unambiguous: **the left leg leaves the floor but
does not re-contact / does not complete its advance.** The one-leg broad-hop is gone —
the hop's signature (high midair on the non-stepping leg) is gone from the right side.

## 21. Next: symmetric two-foot landing (how to finish)

The walk now makes ~107 px/s (first >100). What's left is making **both** legs land
properly, not just the right. The residual failure is left-leg `midair`/`lifted`.

### 21.1 Why the left leg fails (corrected)

Correction: the legs do **not** both converge on one forward capture point. Only one
leg swings at a time — the anti-phase gate `!other_swinging` in `step_leg` plus the
half-stance phase offset at `ResolverState::new` ([`resolver.rs:458`]) guarantee the
two legs alternate. Each swing targets its **own** forward plant in `plan_target_x`.
(If both legs did plant to the identical point it would be a hop — that is not what
happens.)

The left leg is the **trailing** leg each cycle because the right is *hard-wired* as
the leading leg via that half-stance offset (§H1). The real asymmetry driver is
**phasing, not geometry**: the right leg sets the stride pattern and the left follows
it, so whatever phasing / reach / height bias exists is expressed on the trailing leg.
Concretely the left swings from a *retracted, partially compressed* stance and has to
reach farther to complete its advance — so it shows `midair` (over-reach past its
reachable hinge) or `lifted` (touches but does not advance) where the lead leg doesn't.
The fix is to *balance the two legs' phase and reach*, not to make them share a target.

### 21.2 Candidates (in order of relevance)

1. **Balance the phasing** — the root bias is the hard-wired right-leg-leading offset
   (§H1: `ResolverState::new` gives the right leg a half-stance head start). Balance it
   (take the phase offset out, or mirror the two legs so neither is permanently the
   trailing one). This is the most direct fix for a phasing-driven asymmetry.
2. **Per-leg swing reach clamp** — the trailing (left) leg swings from a compressed,
   retracted stance; clamp its `swing_to` to what it can actually reach from its real
   hinge, not the ground-reference `X_max`, so the 2-bone IK ray-clamp doesn't lift it
   into `midair`.
3. **Per-leg `T_remain`** — the trailing leg has a shorter remaining stride; base
   deadbeat timing on its actual plant distance rather than one global
   `swing_duration_ts` timebase.

## 22. The skipped landing is because contact-gated touchdown was off (user insight)

> "if we make sure the leg has to land to start another circle, there's no way for
> the leg to skip a land problem, right?" — user

**The user is right — and the verification never had the rule on.** The headless walk
defaults to `GaitAuthority::Timer` (`HipPinParams::default` at
[`hip_pin.rs:464`](examples/game_lib/src/lower_body/hip_pin.rs:464), and
`ResolverState::new` at [`resolver.rs:480`]). The `run_walk` sweep env never sets
`GAIT_AUTHORITY`. So at `Timer` authority:

- The contact-gated touchdown (§6 `waiting` arm at
  [`resolver.rs`](examples/game_lib/src/lower_body/resolver.rs:1489)) **is off** — it
  only acts at `GaitAuthority::Full`.
- **swing→stance fires on the timer alone**: [`resolver.rs:1505`](examples/game_lib/src/lower_body/resolver.rs:1505)
  declares Stance when `phase_timer <= 0.0` whether or not the foot touched.

This is exactly the hole that yields the left leg's `midair`: the swing ends on the
clock, the leg is parked at its target while still airborne, and the next stance begins
with no real support. My §17 only made **lift** event-driven (CoM-crossing), not
**landing** — so the "must land before the next circle" rule is not enforced.

### 22.1 Proposed fix

Gate **swing→stance on real touchdown** — the `Full`-authority contact gate — so a leg
cannot "skip a land." If touchdown is not observed, hold the swing at the plant point
(no advance) rather than declaring stance on the timer. Caveat: §11 rejected
contact-gated timing earlier, but that was the *pre*-event-driven resolver; with the
CoM-crossing lift (§17) + deadbeat pacing now active, re-test with
`GAIT_AUTHORITY=Full` is warranted.

### 22.2 Result — Full authority makes it WORSE (the gate is not the missing piece)

Ran the current event-driven walk at `GAIT_AUTHORITY=Full`

(`LEG_COMPLIANCE=5e-9`, window 1.6–10.5 s). `Full` adds early-lost-contact stance cut +
touchdown-hold swing→stance. Compare against the §20 `Timer` baseline:

| stepHt | Timer body | Full body | Timer L/R pass | Full L/R pass |
|-------:|-----------:|----------:|---------------:|--------------:|
| 28     | 69.6       | **−48.0** | 6/15 / 7/16   | 2/15 / 1/16   |
| 50     | 107.2      | 49.3      | 9/15 / 15/16  | 6/15 / 6/16   |

At `Full` the walk **goes backward at 28 (−48.0)** and both legs' pass rates collapse
(sh50: 9/15 and 15/16 → 6/15 and 6/16; `midair` dominates both). So making touchdown
mandatory does **not** close the hole — the contact gate, on top of the event-driven
gait, *degrades* the walk.

Interpretation: the left leg's `midair` is **not** caused by "swing ends on the clock
before touching." Under `Full` the leg still can't touch — it holds in the air waiting
for a touchdown that never arrives until `max_swing_wait` forces it. So the contact
gate is not the missing piece. The body is not *over*-forward (twice as much as needed,
the user's first guess also being tested); it is instead failing to **bring the trailing
leg back down to the floor** — a reach/height problem (the §21.2 #2 per-leg reach clamp
is now the leading candidate), not a gating/timing problem. §22.1's proposed gate is
**rejected by measurement**; revert to `Timer` for the haul.

## 23. Pivot: status-based (state) gait, not time-based (user direction)

> "lets not commit yet. time based logic won't fit physics well. I think we need to
> improve on the status based solution." — user

The user is right, and §22.2's measurement *proves* it: both `Timer` and `Full` are
time-based under the hood, and neither closes the skipped-landing hole. `Full` holds
the swing only until `max_swing_wait`, then forces a stance — still on a clock. That is
exactly why a timing-gate cannot fix a physics-lagging walk: the soft body lags and
drifts, so any wall-clock schedule (stance timer, swing timer, wait cap) will end the
swing while the foot is airborne.

### 23.1 The state-based gait (fully timer-free transitions)

Make both gait transitions pure **state machines** on measured contact + position,
with no wall-clock dependency:

- **Stance → Swing** — already state-based (§17): fire on measured CoM passing the
  stance foot in the direction of motion. *Timer removed, fallover-only.*
- **Swing → Stance** — the missing half: fire on **measured contact (touchdown) only**.
  The foot parks at the plant point and the swing holds until the physics reports the
  foot is touching. **No `max_swing_wait` timeout** — without a planted contact signal
  the swing simply does not close (this is what distinguishes a true status-based
  landing from the §22 `Full` gate, which still timed out).
- The only remaining uses of time are **safety** (a "stuck/no-contact" watch so a
  genuinely fallen/unanchored leg can't hang forever), never scheduling. This matches
  §17/§19's "timer as fallover cap" philosophy, now applied to landing too.

The anti-phase gate (`!other_swinging`) + the half-stance phase offset stay; they decide
*which* leg may lift, not *when* — timing is removed from the circle.

### 21.3 Localized to one file, physics untouched

All three are in `resolver.rs` (`step_leg` / `plan_target_x` / `swing_profile_foot`),
gait-only, no physics model change. Cheap to A/B via `analyze_gesture` `pass/n` per
leg.

## 24. Feasibility: full DCM/MPC gait replacement (`plans/swing-phase-timing.md`)

The user authored an event/state-based DCM-tracking gait plan (Kajita→Pratt→
Englsberger lineage, specialized to the 2D XPBD character). Key claim (§12): for a 1D
LIPM with a one-step horizon the MPC solve is **closed-form arithmetic** — no QP needed.

### 24.1 Scaffold we already have

- `omega_0 = sqrt(g/z_CoM)` + DCM `xi = x_CoM + v_CoM/omega_0` — [`capture_point`]
  (resolver.rs:1191) already is the DCM.
- Placement law `x_foot = x_CoM + (v_now − v_des)/(T_conv·omega_0²)` = our
  `plan_target_x`, with `T_conv` (default 1/omega_0) as the new responsiveness knob.
- DCM-support-margin trigger (§3) generalizes our §17 CoM-crossing lift.
- Min-jerk + sine-arch swing (§5) generalizes `swing_profile_foot`; §5's mid-swing
  alignment rule is the user's earlier §18.1 idea.
- Touchdown acceptance (§7: contact + force + hysteresis) = §23/§22 status-based
  landing, plus early/force plant guards we lack.
- Learned acceleration budget (§4.1) directly cures the startup `a_des` spike that is
  the one-leg-hop root cause — a *principled* replacement for our `approach()` ramp.
- Verification checklist (§10) maps onto our `analyze_gesture`/sweep harness.

### 24.2 Realistic risk

1. **Gait-core rewrite.** Replaces the FSM/timing/placement/swing core of resolver.rs
   (~1500 lines + ~2000 of legacy-gait tests). The 100 passing unit tests encode the
   old timer-based phases; a full replacement means re-writing most of `tests`
   (invariant tests — IK length, capture-point math, velocity continuity — survive).
2. **Needs measurement we don't have yet.** The learned budget needs per-frame CoM
   `a_achieved` and foot-slip detection (loaded-foot horizontal motion). We measure
   CoM/vCoM/apex, not slip/a_achieved — a small measurement-layer extension.
3. **Hysteresis/guard tuning** for XPBD contact flicker (~10 new knobs).
4. **Loop-2 thrust/apex-height feedback re-wiring** for jump support inside the FSM.

### 24.3 Recommended phasing (not big-bang)

- **A** placement law with `T_conv` + fixed-point touch-down prediction (swap
  `plan_target_x`). Low risk, pure math.
- **B** DCM-support-margin trigger + progress-based (`tau`) retimed swing — delivers
  §23's timer-free landing properly. Low-medium.
- **C** learned acceleration budget + slip/a_achieved measurement (replaces `approach()`
  ramp). Medium.
- **D** early/force plant + FALL/LIE/get-up FSM. Medium-high.
- **E** hysteresis/guard tuning + full §10 checklist. Iterative.

**Verdict: implementable, right architecture, no optimizer dependency; but a
multi-session gait-core rewrite — strongly recommend A→E phasing. §23 is subsumed by
phase B.**
