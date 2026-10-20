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
