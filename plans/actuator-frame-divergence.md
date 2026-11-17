# Actuator frame divergence — why the feet always lag the pose (2026-11-17)

## The question

The walker is constantly pitched forward and the physical feet sit **behind**
the pose the controller emitted. Is the tilted spine being used to compute the
feet, and is that the divergence?

**Answer: yes — but the frame is inverted from the naive reading.**

- The **controller** authors the pose in a world-upright frame
  (`a_t` is built from `pose.upper_angle`).
- The **physics actuator** executes the hip joint against the **real**
  `P2→P3` bone, which is pitched forward.

So the realized leg is the commanded leg **rotated by the torso-pitch error**,
and the foot lags by `L · (θ_meas − θ_cmd)`.

Reproduce on `tmp/walker_qa/sd5.csv`
(`SCRIPT=tmp/walker_qa/walkv60-boost90-40-70-40.json WIN_HIDDEN=1`):

```
python3 tools/physics-qa/three_level_trace.py tmp/walker_qa/sd5.csv --range 4 6
python3 tools/physics-qa/analyze_delivery_gap.py tmp/walker_qa/sd5.csv
```

## The chain

```
particles ─► observe (Fiction) ─► control (Pose, hip-relative) ─► actuate
                                                                    │
        physics ◄── emit (2 angular + 1 distance per leg) ◄─────────┘
```

Relevant code:

| step | code |
|---|---|
| reality → fiction | [`observe_tick()`](../walker_control/src/observe.rs:436) |
| fiction → pose | [`WalkingCtrl::step()`](../walker_control/src/control.rs:511), hip-relative map at [`control.rs:635`](../walker_control/src/control.rs:635) |
| pose → joints | [`leg_commands()`](../walker_control/src/actuate.rs:241) |
| joints → solver | [`emit()`](../walker_control/src/emit.rs:106), [`angular_config()`](../walker_control/src/emit.rs:53) |

## The mechanism, verified against the asset and the solver

Joint topology from `examples/collision_detection/assets/character/v10.character.json`:

```
Angular/P2_P3_P40:   ["P2", "P3", "P40"]    hip  : p0=P2, p1=P3, p2=P40
Angular/P3_P40_PLLL: ["P3", "P40", "PLLL"]  knee : p0=P3, p1=P40, p2=PLLL
Distance/P2_P3: 1e-6                        torso is a DISTANCE only
```

Solver measurement, `AngleConstraint::angle_basis` (constraints.rs:740):

```
v1 = p1 − p0 ; v2 = p2 − p1
sin = u1.y·u2.x − u1.x·u2.y = −cross(u1,u2)
θ  = atan2(sin, cos) = φ(v1) − φ(v2)
```

The actuator mirrors it exactly:

```rust
let theta_hip  = angle_at(live.waist, live.hip, live.knee); // v1 = hip−waist  (REAL torso)
let a_t        = pose.hip + Vec2::new(len*sin_lean, -len*cos_lean);
let target_hip = angle_at(a_t, pose.hip, knee);             // v1 = hip−a_t    (COMMANDED torso)
```

Equilibrium is `θ_measured = θ_target`:

```
angle(real torso, real thigh) = angle(commanded torso, commanded thigh)
```

If the real torso is `δ` more forward than `a_t`, the real thigh must be `δ`
more forward **in the same atan2 sense**. The knee target carries no torso term,
so it only preserves the *thigh↔calf* angle — the whole leg rotates rigidly by
`δ` about the hip. A downward chute (foot below hip) rotated by a forward `δ`
lands **backward**. Exactly the observed lag.

Because both hip joints reference the *same* `P2→P3` bone, the error is
**common-mode**: both feet move together. The CSV shows L stance +10.8 px and
R stance +11.9 px — a shared frame error, not per-leg tracking.

## Evidence (sd5.csv, 1135 walking + 361 standing rows)

Rotation test — rotate the commanded hip→foot vector by
`δ = meas_theta − att_target` about the hip and compare with the physical foot:

| mode | δ | error **without** rotation | error **with** rotation |
|---|---|---|---|
| standing L | +2.75° | mean 7.37, median 6.77 px | **mean 1.62, median 1.28 px** |
| standing R | +2.75° | mean 6.71, median 5.45 px | **mean 2.05, median 1.31 px** |
| walking L | +9.16° | mean 16.04, median 16.39 px | **mean 9.07, median 7.86 px** |
| walking R | +9.16° | mean 15.96, median 17.07 px | **mean 9.14, median 8.55 px** |

Standing x-only: raw **+5.72 / +4.92 px** → after rotation **−0.73 / +0.12 px**.
`L·δ = 104 × 0.048 ≈ 5.0 px` matches the raw bias.

Standing is the decisive control: `StandingCtrl` emits constant rest offsets —
no gait law, no swing, no placement — and the rotation explains ~100 % of the
divergence there.

Controls that rule out the usual suspects:

| check | value |
|---|---|
| chord **length** delivered (stance) | 99.1 commanded vs 98.4 actual (**<1 px**) |
| chord **direction** error | −8.6° walking, −3.0° standing |
| corr(chord direction, `meas_theta − att_target`) | −0.50 … −0.89 |
| knee-joint angle actual vs commanded (walking) | median **−0.9° / −0.8°** |
| thigh / calf lengths | 55.6 / 55.6 both commanded and actual |

So the *shape* is delivered and the *knee* tracks; the **hip joint**, which is
the torso-referenced one, is where the rotation enters.

## What is left after removing the frame

Walking residual after the rotation, decomposed into radial (chord) and
tangential (further rotation):

| | raw ex | rotated x | radial | tangential |
|---|---|---|---|---|
| L stance | +10.8 | +3.8 | −1.0 | −3.8 |
| R stance | +11.9 | +4.2 | −1.1 | −4.2 |
| L swing | +17.3 | −4.0 | +2.6 | +1.1 |
| R swing | +16.9 | −4.6 | +2.6 | +2.2 |

- **stance ~4 px tangential** — the plant slips backward (`slide` median
  −4.5 px in `analyze_delivery_gap.py`). Contact/friction, not the frame.
- **swing radial +2.6 px, sd ~10 px** — the chord is not exactly delivered while
  the foot is fast; genuine actuator tracking lag.
- **aim** — the placement lead uses the fiction `v`
  (`v_law`, `walker_math/src/walk.rs:320`; only the brake path reads measured),
  so with `v_filt` 70–155 against `v_des` 40–70 the swing target is placed for a
  slower body: touchdown `ERR` median −17 px.

## Why it never self-corrects

1. The pose is hip-relative and re-expressed around the **measured** hip every
   tick, so the frame error is re-baselined, never seen by control. There is no
   outer loop on the foot's world position.
2. Torso pitch is an **unactuated absolute DOF**. `P2_P3` is a distance joint and
   `P1–P2–P3` is deliberately not an actuator (`actuate.rs:26`). The attitude
   command is routed through the same hip joints that place the legs, so it
   cannot null the pitch without moving the feet.
3. The loop-3 correction is applied *in the same frame* as the error:
   `u = att_target ≈ −kp·δ`, so the realized world rotation is
   `δ − u = δ(1+kp)`. **Positive `kp` increases the foot lag.** This is
   consistent with the documented tuning failures (0.4/0.2 fell at 5.5 s,
   0.2/0.3 at 4.9 s, 0.1/0.15 barely survived).

## Fix A — pure world reference (committed)

Use the **measured** torso axis as the hip joint's reference instead of the
virtual one:

```rust
// walker_control/src/actuate.rs, leg_commands()
- let target_hip = angle_at(a_t, pose.hip, knee);
+ let target_hip = angle_at(live.waist, live.hip, knee);
```

Equilibrium becomes `angle(real torso, real thigh) = angle(real torso, IK
thigh)`, so the **world** thigh direction equals the IK direction and the foot
target is delivered regardless of torso pitch.

Observed consequence: the hip joints were also the only attitude authority, so
with both legs world-referenced the **upper body rotates freely about `P3`**.
That is exactly the DOF argument above — torso pitch is the global rotation and
no internal constraint can hold it.

## Fix B — two-channel blend (falsified)

The hip target is now a per-leg blend, carried by
`Pose::stance_load: [f32; 2]`:

```rust
let world = angle_at(live.waist, live.hip, ik_knee); // delivery, world frame
let held  = angle_at(a_t, pose.hip, ik_knee);        // attitude, commanded lean
let target_hip = world + load * wrap_angle(held - world);
```

`load` is scheduled in `control_stage` from the **gait phase**
(`out.pose.*_foot.stance`), with a symmetric EMA crossfade
(`alpha = dt / ControlConfig::stance_load_ts`, default **0.04 s**):

- the stance leg targets `1`, the swing leg targets `0`;
- with complementary targets the crossfade preserves `load_l + load_r = 1`, so
  a phase flip hands authority over without a gap and without a step;
- `BoostStart` pins both feet to `1` (the launch is the leaned stance).

**Why not contact:** measured contact in this rig is true only ~24 % of walking
ticks (`sd5`: `contact_l` 0.23, `contact_r` 0.25), so a contact gate produced
`load_l/r ≈ 0.12` and `meas_theta` ran to 2.46 rad — the torso flailed. The FSM
phase always names exactly one stance leg.

Why a blend: the earlier hard `if stance { a_t } else { live }` switch stepped
the hip rest basis by the torso-pitch error (~9° on sd5) at every touchdown, on
the leg that had just become load-bearing — a velocity kick. The crossfade makes
the handover bumpless while preserving the sum of authority.

**Standing and Boost are two-foot support by definition.** Their load targets are
`[1, 1]`, not the raw contact flag; `StandingCtrl` also emits `stance: true` for
both feet, so both struts stay at active compliance. Gating the final stand on
`touching` dropped both struts to idle (contact is true ~24 % of ticks) and the
character fell after the walk.

**Known transition caveat:** the mode machine switches to Standing the moment
the command ends, so the Walking stop-settle never runs. If the feet are
staggered at that instant, the hardcoded rest offsets are a step command. The
fix above keeps support, but a smoother stop needs the machine to stay in
Walking until both feet are planted and the speed is low.

### Status: falsified (2026-11-17)

Both gating variants were run and made the walk worse:

- the contact-gated blend starved the attitude channel (contact true ~24 % of
  ticks → `load ≈ 0.12` → `meas_theta` up to 2.46 rad);
- the phase-gated crossfade (one stance leg at `load = 1`, swing at `0`) makes
  the character **fall backward during walking**.

The second result is the informative one: the swing hip joint is not idle in
attitude terms. The swing leg's forward acceleration reacts on the torso
backward, and the pre-fix `a_t` swing hip joint cancels that reaction. With the
swing leg world-referenced, the cancellation is gone and the stance leg alone
cannot hold the pitch.

**Conclusion:** the two hip joints are *jointly* the attitude actuator. Any
split that frees one of them for delivery removes half the authority — there is
no blend that keeps both. The functional code was reverted to the pre-fix `a_t`
behavior (HEAD), the only stable configuration. The remaining options are the
independent actuators, not another blend:

1. `Angular(PW, P3, P2)` against a pinned world-reference particle (`PW` above
   the hip, `free_mask = 0b110`), or
2. an ankle joint (a real foot-orientation DOF+actuator), or
3. accept the foot lag and keep the stable walk.

**Separate real bug found:** measured contact is true only ~24 % of walking
ticks (`sd5`: `contact_l` 0.23, `contact_r` 0.25). Any future gate (strut,
attitude, phase authority) must not trust it until the contact reporting is
fixed.

Telemetry appends `contact_l`, `contact_r`, `hit_l`, `hit_r`, `load_l`,
`load_r`: `load_*` is the weight the actuator actually consumed, so the gate is
measurable (`plans/stance-delivery-is-the-brake.md` §6b asked for exactly this).

## Validation plan

1. `cargo test -p walker_control` — 88 tests, including blend
   monotonicity/continuity, the load ramp/instant drop, the boost pin, and the
   two frame tests.
2. Re-run the sd5 script to a new CSV; keep `sd5.csv` for A/B.
3. Gates:
   - `meas_theta` bounded like sd5 (sd ~0.03–0.04 rad) — the stance channel held;
   - swing touchdown `ERR` / `end_gap` shrink toward the aim/slip floor;
   - `load_l/r` ≈ 1 through stance and 0 through swing, no chatter;
   - `load_l + load_r` ≈ 1 across each handover (no authority gap);
   - final stand: `load_l = load_r = 1` and `meas_theta` bounded.
4. If `meas_theta` runs away, the stance channel lacks support (read
   `contact_*`), not a gain; if swing `ERR` does not improve, the frame was not
   the landing problem.

## Side notes (not the cause)

- `target_knee = angle_at(pose.hip, knee, foot_pose.pos)` (`actuate.rs:287`)
  mixes the **clamped** IK knee with the **unclamped** foot. At the v10 rest
  offsets the chord (110.3–111.0 px) exceeds `thigh+calf = 110`, so the
  commanded triangle is slightly inconsistent with the bone lengths — a 1–2°
  effect.
- The stance foot is commanded ~2.6 px below the live particle centre; the
  collider radius accounts for most of it. Not a control error.
