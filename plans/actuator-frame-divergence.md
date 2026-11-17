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

## The second term: velocity-proportional servo lag (2026-11-17)

The rotation test's residual is not noise. Fitting the swing-phase foot error
against the static frame error `δ = meas_theta − att_target` and the commanded
foot velocity `v_cmd = tPLLLvx` gives

```
ex ≈ b_δ·δ + τ·v_cmd + c·ω_body
```

| run | phase | mean ex | b_δ | τ | `τ·v` at swing p95 | c |
|---|---|---|---|---|---|---|
| sd7 (60/90-40-70-40) | stance | 10–11.5 px | 72–87 px/rad | ~7 ms | ~0 | ~0 |
| sd7 | swing | 17.2–17.7 px | 41–45 px/rad | **15–19 ms** | 7.5–9.6 px | 3–6 |
| sd8 (60/120/60) | stance | 7.7–8.5 px | 46–67 px/rad | ~6–8 ms | ~0 | 1–2 |
| sd8 | swing | 13.7 px | 25–31 px/rad | **15–20 ms** | 7.7–9.8 px | 3–10 |

Stance is static frame only (`v ≈ 0`). Swing carries a **~15–20 ms
velocity-proportional lag** on top of the static frame — about 1.5 ticks at
90 Hz, i.e. the observe→control→actuate→solve transport plus the joint
spring-damper lag. `ω_body` contributes the smaller `c` term.

Why: the actuator is a pure **position** servo. It emits `rest_cos/rest_sin`
and `rest_length`, never a rate. The joint's required rate is
`θ̇_plan + ω_body` (the hip must counter-rotate to hold a world thigh under a
rotating torso), and neither is fed forward. The DCM is not at fault — it plans
CoM position/velocity and does not need the angular rate — the *servo* needs it.

**Lead attempt rejected (2026-11-17).** A `τ·v` lead on the commanded foot was
implemented and run (`sd10`, `FOOT_LEAD_TS=0.02`). It overshoots at landing:
the vertical swing velocity is downward near touchdown, so the linear lead pushes
the target below the plan/floor (`tPRLLy` 316–318 vs `planRy` 314 at
t≈5.50–5.52), and `LowerBodyPose` velocities are finite differences of the pose,
which spike at phase transitions (`tPRLLvx` −4369 / +2767 px/s at t≈5.55), so
the lead can be tens of px. The lead was removed.

The measured τ is still real. The structural fix is the solver's velocity
projection (`AngularConstraintConfig::angular_velocity = Some(θ̇_rest + ω_meas)`,
`sigma > 0`), which uses rates rather than a tuned delay.

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

**Architecture (implemented 2026-11-17): stance controls the spine; the swing
foot is controlled from the stance feet and the spine.**

- **Stance leg** → hip target against the **commanded lean** (`a_t`): the
  planted foot + leg hold the torso at the desired attitude.
- **Swing leg** → hip target against the **live `P3→P2`**: the swing foot is
  placed in the spine frame the stance feet have established.

The split is **walking-only**, gated by `Pose::stance_split` (set by
`WalkingCtrl`). Standing, Boost, and every other controller set it `false` and
keep the commanded-lean reference on **both** legs — the committed behaviour —
so the final stand and the launch are unchanged.

`ATT_BLEND` was removed; this is the split exactly as specified. The known
problem to improve on is that the swing hip no longer contributes a spine
spring, so the model can pitch backward under the swing reaction. That is the
next thing to fix — not by re-blending the references, but by improving how the
spine is held (stance authority, cadence/crouch, or an additional actuator).

**Measured context:** the stance foot is planted (`slide` −0.6 px with
`STANCE_DEPTH=1`), so the stance leg is a good anchor. At `hip_height = 104`
the stance leg is near full extension (chord 99–110 of 110, `x_max 30.5 px`),
which is the likely authority limit; `HIP_HEIGHT = 96–100` is the physical
margin lever (`x_max 45.6 / 39.0 px`).

**Separate real bug found:** measured contact is true only ~24 % of walking
ticks. Any future gate must not trust it until the contact reporting is fixed.

## Boost: the right foot is the stance foot (2026-11-17)

The launch now uses the same stance machinery as the gait:

- **Right foot = stance** (`stance: true`, strut active). It is the v10 FRONT
  foot for an eastward boost and the one the gather pulls under the hip, so it
  carries the launch.
- **Left foot = swing** (`stance: false`). It holds its rest offset behind the
  hip and rides the angular joints only.
- **Stance press (`STANCE_DEPTH`) applies to the right foot**, via the same
  `stance_press` helper the gait uses. The launch has no stance phase timer, so
  the envelope is pinned at its peak: `age = min(since, T_gather/2)`, i.e. the
  press ramps in over the first half of the gather window and then holds at full
  depth for the rest of the cell.

This replaces the earlier "both feet stance" choice (which existed because a
swing-flagged foot rode the idle 1e-2 tether and the lean crept into a forward
rock). The back foot is still behind the hip, which is what keeps the handover
committing the fall forward. `stance_split` stays `false` in Boost, so both legs
still target the commanded lean `a_t`.

**`HIP_HEIGHT` reaches the launch too.** `BoostStartCtrl` now takes the
`LowerBodyConfig`, and the support (right) plant depth is the configured
standing hip height instead of the blueprint's hardcoded 103 px. The whole rest
pose shifts by `hip_height − 103`, so the v10 x offsets and the 3 px left/right
depth skew survive; `hip_height <= 0` (unseeded) falls back to 103. In the QA
harness the attach already does `config.lower.hip_height = 104.0` then
`apply_lower_env`, so `HIP_HEIGHT=96` now crouches the boost by the same 8 px it
crouches the gait — one knob, both phases.

## Why a squashed boost falls BACKWARD (2026-11-17, `tmp/walker_qa/1.csv` vs `0.csv`)

`0.csv` = committed code, `HIP_HEIGHT=104`, `STANCE_DEPTH=0`: the boost releases
at t=1.39, `v_filt` −4.8 → +4.6, the walk advances 179 → 1875 px over 20 s and
ends standing. `1.csv` = right-foot stance + `HIP_HEIGHT=98` +
`STANCE_DEPTH≈2`: the boost runs its FULL cell (no release at 0.4 s), the hip
moves 175.5 → 167.0 (**backward 8.5 px**), and the walk then travels backward
(mean `v_filt` −8.1, hip 167 → 119).

Three measured steps, all on the entry tick:

1. **The squash was a step, not a ramp.** The plants were holding the whole
   body at depth 103/106 and the boost's first tick commanded 98/101 — both
   chords yanked 5 px toward the hip in one 11 ms tick (≈450 px/s closure).
   `P3vy` 6.6 → 31.1 px/s (down), `P3vx` −11.3 → −61.5 px/s (backward), and the
   hip lost 6 px of height over the next 0.3 s. A planted chord cannot be
   stepped: the strut reacts *on the hip* one-for-one, and folding a loaded leg
   with an anterior knee translates the hip backward (squat mechanics).
2. **The release condition is forward-only.** `boost_handover_due` needs
   ξ = com + v·τ to lead past the frontmost foot. With `v_com` negative, ξ moves
   *behind* the plants, so the launch never fires and the boost holds the
   leaned pose for the whole cell.
3. **The handover is then poisoned.** The gait starts with the CoM behind both
   plants; its DCM event lifts the wrong foot and the body keeps going
   backward. The `boost_lean` of 0.05 rad only bought ~1.8 px of CoM offset —
   an order of magnitude less than the 9–15 px the squat hands it. (That lean
   has since been removed: the launch is upright and the kick is the drive.)

Fixes now in place:

- the crouch is **eased** (`boost_crouch_ts`, smoothstep, default 0.25 s), so
  the entry tick is the blueprint rest pose and the depth arrives after the
  body has had time to fold under load;
- the **push-off** (`boost_push`, default 0) extends the BACK leg's chord along
  its own hip→foot direction, ramped by `boost_push_ts` and clamped to
  `thigh + calf − chord`;
- the launch is **upright**: `upper_angle = 0` for the whole cell. The old 0.05
  rad pre-lean only bought ~1.8 px of CoM offset, and because the torso target
  is referenced to the live thigh it also had to be paid for in pitch — now the
  kick is the drive and the attitude reference stays vertical;
- the **kick-off** (`boost_kick`, default **15 px**, ramp
  `boost_kick_ts` = **0.04 s**) drives the BACK foot *backwards along travel*:
  the leg extends behind the body, so the reaction is **100 % forward** hip
  travel (a chord extension is only ~30 % — the back chord is 18° off vertical).
  It is clamped by the horizontal reach left, `sqrt(reach² − y²) − |x|`, so the
  clamp — not the knob — decides: ~15 px at `hip_height = 96`, ~10 px at 98.
  The 0.04 s ramp is deliberate: at 15 px it is ~375 px/s of foot drive, fast
  enough to beat the slip that turned the first (0.12 s, 8 px) attempt into a
  drag. No env override: the value is baked.

**Headroom, and why the two push-offs differ.** At the v10 rest pose the back
chord is |(−33, 106)| = 111 px of a 110 px leg, so `reach − chord` is *negative*
and both are no-ops — that is why a real launch crouches first, not a bug. The
crouch then buys very different room:

| `hip_height` | back chord | chord extension room | **backward kick room** |
|---|---|---|---|
| 104 | 110.9 | 0 | **0** |
| 98 | 106.3 | 3.7 | **10.4** |
| 96 | 104.4 | 5.6 | **15.0** |

The backward kick is the better lever on paper: it uses the leg's *horizontal*
reach, and every pixel of it should be forward hip travel instead of ~3 px of
lift. `boost_kick` defaults to 8 px — inert at `HIP_HEIGHT=104`, active at 96–98.

**Measured 2026-11-17 (first kick run): the kicking foot SLID, and the body did
not move.** Over t = 1.21 → 1.30 the back-foot target went 141.0 → 131.4 px and
the *actual* foot 136.6 → 130.2 px, while the hip went 174.4 → 175.0 px — i.e.
the foot dragged ~6 px backwards across the floor and only 0.6 px of that became
forward hip travel. The pose can only *command* a foot position; whether that
becomes a push is decided by grip. With a sliding foot there is only kinetic
friction, so the launch still ends at zero forward speed: it released at t=1.34
with `v_com ≈ −0.7` (the release test is positional — ξ just past the front
foot), the gait then started at 0 while the plan asked 90 px/s, threw the first
swing foot 24 px forward (tL 134 → 158) while the hip recoiled, and the body
walked backwards from then on.

Root cause of the slide: **the back foot was not a support foot at all.** When
the right foot became the only stance leg, the back leg was flagged `swing` for
the first 0.2 s of the launch — an idle 1e-2 tether, no authority — and the
crouch ramp then *lifts* a swing foot (it moves both plant targets toward the
hip). So the kick started on a foot that was in the air and merely dragged it.
That is exactly the "back foot lifts before the push" in the picture.

Fixes now in place:

- **both feet are support for the whole launch** (`stance: true` on both), so the
  crouch pulls the *hip* down onto a planted back foot instead of lifting the
  foot into the air;
- the **kick waits for the crouch** (`since − boost_crouch_ts`) so the back foot
  has landed and loaded before it is driven;
- every emitted chord is **clamped to `thigh + calf`** (`clamp_len`), so neither
  the press nor the kick can command a leg past its reach (the back chord is
  already 111 px of a 110 px leg at the v10 rest depth).

If the foot still slides with a loaded, crouched leg, then it really is a grip
limit (`friction_compliance_scaler`, and `BOOST_KICK_TS = 0.03–0.05` to make the
drive an impulse rather than a drag), and the pose layer cannot deliver a
push-off at all — the answer is then the strut/actuator route.

## The walking hop: the feet were planted on the MEASURED HIP (diagnosed 2026-11-17)

The walk that follows the fixed landing hopped: the hip swung **20.2 px**
vertically, the "stance" foot left the floor, and the contact-only slip was
**22–29 px per stance** (max 48). `STANCE_DEPTH` made it worse, and the telemetry
says why.

`pose.foot - pose.hip` is a **solver-frame offset**, and the walking pose emitted
`hip + to_vello(offset)`. So the measured hip's vertical excursion was copied
1:1 into the commanded foot:

- `(target_y − hip_y)` during stance is **98.00 px, constant**, on both feet;
- the *world* y of that stance target therefore spans **303 … 323** against a
  floor line at **307.4** — 4 px of air at the top of the vault, **16 px of
  floor press** at the bottom;
- **42 %** of stance-flagged ticks had the stance foot >2 px off the floor
  (both feet airborne on 41 % of walking ticks), and `corr(hip_y, foot_y)` on
  the stance leg is 0.76.

That press-and-release is a spring: the foot is shoved 16 px into the collider at
the bottom of the vault and it pushes the body back out. A constant
`STANCE_DEPTH` offset cannot cancel an oscillation around the floor line — it
just adds to the press at the bottom — which is exactly why a larger depth made
the hop worse.

### Status: REVERTED (2026-11-17)

The fix was implemented, unit-tested and run; the walk got worse end to end, so
the walking pose is back to `hip + to_vello(solver offset)` bit-for-bit. The
diagnosis above stands and is worth keeping, because it says exactly what a
future attempt must do — as a **package**:

1. **Feet in the floor frame** — `floor_vello_y − (pose.foot.y − floor_y)`
   instead of the measured hip. This does remove the hop mechanism, and the
   emitted stance target then holds 306.1 instead of spanning 303..323.
2. **The emitted hip must be a command too**, or the character KNEELS:
   `leg_commands` derives each strut's rest length from `|pose.hip − foot|`, so
   with the foot pinned to the floor that rest length became the body's *current
   sag*, re-measured every tick — a spring with zero restoring force whose set
   point walks downhill with the body. Measured: hip 208 → 237 in 0.45 s while
   both foot targets sat on 306.1.
3. **`leg_commands` must pivot the stance hip target at the LIVE hip (`P3`)**
   once (2) is in place: the target only depends on the torso reference's
   *direction*, and pivoting at a commanded height skews it by
   `atan(height error / thigh)` (≈20° at a 20 px sag). The swing branch already
   worked this way.

With all three in place the kneel was gone but the run was still worse than the
legacy map, so it was reverted rather than tuned on top of a frame change.

**Before trying again:** once the foot is genuinely on the floor, the stance leg
carries the body at the commanded chord. At `hip_height = 98` with 110 px bones
the knee is ~54° off straight — a deep squat — so the achieved height is set by
the leg stiffness (`KNEE_COMPLIANCE`, `STRUT_ACTIVE`), not by the command; the
v10 rig stands at ~104 and needs far less knee torque there. Also
`floor_vello_y = anchor.y + hip_height` inherits the seed's rig-vs-config height
gap (`plans/floor_anchor_live_stance.md`, the 90-vs-106 note).

## One-sided floor clamp on the stance foot — tried, REVERTED (2026-11-17)

With `STANCE_DEPTH = 0` the stance foot *is* commanded below the ground: the pose
rides the measured hip, so a body standing lower than `hip_height` buries the
plant by that much (measured: the target spanned 303 … 323 against a floor line
at 307.4, i.e. up to 16 px under).

The clamp tried was one-sided and minimal — `pos.y = pos.y.min(floor_vello_y)`
for a stance foot only, with `STANCE_DEPTH` applied *after* it so the press kept
its range. Unit-tested and run: **it breaks the walk.** The leg loses the
over-extension the body was standing on (at `hip_height = 98` and 110 px bones
that over-extension is what makes the stance a column rather than a 54° squat),
so the body no longer has support and the walk degrades.

That is the same lesson as the floor-frame attempt, in a smaller dose: **the
"penetration" is load-bearing.** The stance foot is buried precisely because the
leg is over-extended and the floor is what stops it; remove the burial and the
leg must carry the body at the commanded chord instead. Any real fix has to make
the leg load-bearing first — stiffness (`KNEE_COMPLIANCE`, `STRUT_ACTIVE`), the
commanded height (`HIP_HEIGHT` near the rig's own ~104), or a damped strut — and
only then can the foot be kept on the surface.

## Side notes (not the cause)

- `target_knee = angle_at(pose.hip, knee, foot_pose.pos)` (`actuate.rs:287`)
  mixes the **clamped** IK knee with the **unclamped** foot. At the v10 rest
  offsets the chord (110.3–111.0 px) exceeds `thigh+calf = 110`, so the
  commanded triangle is slightly inconsistent with the bone lengths — a 1–2°
  effect.
- The stance foot is commanded ~2.6 px below the live particle centre; the
  collider radius accounts for most of it. Not a control error.
