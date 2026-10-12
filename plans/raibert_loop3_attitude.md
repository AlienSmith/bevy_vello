# Raibert Loop 3 — Body Attitude (Orientation) PD Controller

Standalone reference for the third Raibert loop. Kept separate so it can be
picked up later (XPBD frame integration, sandbox extension, tuning) without
re-deriving anything.

> **One-line summary:** a rotational PD at the hip that keeps the torso upright.
> It reads the *body's own* angle and angular rate and writes a torque. It does
> **not** touch the position or velocity of the hip or leg particles.

---

## 1. The formula

```
τ = Kp · (θ_desired − θ_actual) + Kd · (θ̇_desired − θ̇_actual)
```

- **P term** `Kp · θ_error` — virtual rotational spring at the hip.
- **D term** `Kd · (θ̇_desired − θ̇_actual)` — virtual rotational damper.
- `θ_error` is a *derived* quantity: `θ_error = θ_desired − θ_actual`. It is the
  P branch of the formula, not an independent input.

In the very common walking case `θ_desired = 0` and `θ̇_desired = 0`, so the law
collapses to the textbook form:

```
τ = Kp · (−θ_actual) + Kd · (−θ̇_actual)
        └─ P: push back on tilt ─┘  └─ D: resist tumble ─┘
```

That is a *critically-dampable* rotational spring anchored to world vertical,
exactly one DOF. Nothing else.

---

## 2. Variables

| Symbol | Kind | Units | Meaning |
|---|---|---|---|
| `τ` | **Output** | N·m | Target rotational torque at the hip. **+** twists the torso **forward**, **−** twists it **backward**. |
| `θ_actual` | Input | rad | Current torso-group orientation relative to **absolute world vertical**. `0.0` = perfectly upright; **+** = leaning forward; **−** = leaning backward. |
| `θ̇_actual` | Input | rad/s | Angular velocity of the torso — how fast it is currently tumbling/spinning. |
| `θ_desired` | Input | rad | Target angle for the upper body. `0.0` for standard walking. Modulate for a sprint lean or a braking lean. |
| `θ̇_desired` | Input | rad/s | Target angular rate. Usually `0.0`; used to command a steady rotation (e.g. a deliberate spin). |
| `θ_error` | Calculated | rad | `θ_desired − θ_actual`. How far off-balance the torso currently is. |
| `Kp` | Tuning | N·m/rad | **Proportional gain** — virtual rotational **spring stiffness**. Larger → hip pushes back harder when the body tilts off target. |
| `Kd` | Tuning | N·m·s/rad | **Derivative gain** — virtual rotational **damper**. Counteracts angular speed. Without it the torso oscillates like a bobblehead. |

---

## 3. What it couples to (and what it must not)

This is the defining property and the reason loop 3 is worth having:

- **Reads:** the torso orientation group (`θ_actual`) and its angular rate
  (`θ̇_actual`) — both derived from the *upper-body / frame* particles.
- **Writes:** a torque `τ` at the hip that rotates the **torso only**.
- **Does NOT read or write** the position or velocity of the hip or leg
  particles. Loops 1 and 2 own hip/leg *translation* (height + foot placement);
  loop 3 owns body *rotation*. Different DOF, different layer — no contention.

Sign convention (must be consistent between input measurement and output):

- `θ_actual` positive = forward lean. A forward lean therefore yields
  `θ_error = −θ_actual < 0` and `τ < 0` = backward twist → the hip corrects the
  lean. Self-consistent as long as the measurement frame and the torque sign
  share the same world-vertical reference.

---

## 4. Where it lives in this codebase (XPBD integration)

The character has two layers:

1. **Driven ragdoll** — joint particles + XPBD constraints (legs, spine, arms).
2. **Rigid frame** — defined by the three spine particles `P1 (upper spine /
   shoulders)`, `P2 (waist — shared pivot / rotation center)`, `P3 (lower spine
   / hips)` ([observers.rs:844–848](bevy_vello/examples/game_lib/src/character_factory/observers.rs:844)),
   exposed as `BalancedCoreFrame` / `SpineFrames`.

Loop 3 is a **frame-layer** controller. The actuator already exists:

- [`apply_rigid_offset_to_frame()`](study_vello/integrations/vello_physics/src/utility.rs:924)
  pivots at the **waist** (`frame_positions[3]`), decomposes offsets into
  `linear = Σoffset / N` and `angular = Σ r × offset`, with `inertia = Σ r·r`,
  then moves each frame particle by `Δ_i = resolved_linear + (ω × r_i)`.
- It already accepts a `rotation_resistance` scalar (0 = full rotation,
  1 = pure translation) — the **passive** counterpart of loop 3.

Mapping a `τ` command onto that actuator:

```
Δω = τ · dt / I          // I = Σ r·r about the waist pivot (frame inertia proxy)
ω_target = ω_current + Δω
// distribute exactly as apply_rigid_offset_to_frame does:
Δ_i = (ω_target × r_i)   // r_i = particle_i − pivot
```

Loop 3's ideal pivot is the body CoM ≈ **P2**, which is already the frame's
rotation center, so no pivot change is needed. But note the frame includes
**P3 — which *is* the hip particle** (see §9): the hip joints `P2_P3_P40` /
`P2_P3_P41` pivot at P3. Rotating the frame therefore rotates P3 (P2 is the
pivot, P1 and P3 swing about it), carrying the leg roots with the body attitude.
That does not violate §3 — loop 3 has no hip–position or hip–velocity *term* —
but it does mean the torque must be **clamped** and `rotation_resistance` tuned
so it never overpowers the planted-foot stance constraints (see §9).

### Relationship to the arm's existing PD

The arm has **no torque actuator** — only an angular *rest* constraint — so it
fakes the D term in *target space* ([`kd_offset = (+kd·ω).clamp(±kd_max_offset)`](bevy_vello/examples/game_lib/src/character/systems.rs:637),
sign derivation at [systems.rs:616–627](bevy_vello/examples/game_lib/src/character/systems.rs:616)).
Because the torso *does* have a torque path, loop 3 can be the **direct**
force-domain PD (`Kd·(θ̇_d − θ̇_a)`), which is simpler and more honest than the
arm's target-space construction. They are not the same mechanism.

### Relationship to the spine drive

[`calculate_spine_drive()`](bevy_vello/examples/game_lib/src/character/systems.rs:392)
is a *position/heading* servo: it interpolates bone headings toward a **latched
desired heading** and pulls via external position constraints. It is **not**
loop 3. Loop 3 is the *rotational force* controller for the whole body attitude.

---

## 5. Discrete integration & stability notes

- Integrate at the physics tick rate. `Δω = τ·dt/I` is explicit Euler; keep `dt`
  in the production band (30–240 Hz) as with loop 1.
- Choose `Kd` from a target damping ratio. For inertia `I` and spring `Kp`, the
  natural frequency is `ω_n = sqrt(Kp/I)`; critical damping needs
  `Kd = 2·sqrt(Kp·I)`. Start slightly under- or over-critical and tune:
  - **Too little `Kd`** → bobblehead oscillation (the classic symptom).
  - **Too much `Kd`** → sluggish, "sticky" torso that lags commanded leans.
- `Kp` alone (no D) will always ring. `Kd` alone can't hold a static lean offset
  (no restoring force). Both are required; they are a spring and a shock
  absorber, not alternatives.
- Bound the output: `τ = clamp(τ, ±τ_max)` so a knock/teleport spike cannot flip
  the torso in one tick. (Analogous role to `kd_max_offset` on the arm.)
- Stateless: recompute `θ_error`, `θ̇_actual` from the live particles every tick;
  no accumulator, no windup.

---

## 6. Units & scale

Inherit the sandbox conventions ([`raibert_vertical_loop.md` §7](bevy_vello/plans/raibert_vertical_loop.md:1101)):

- Pixel space, scene scale `0.5`.
- Gravity `g̃ = 980 px/s²`.
- If `τ` and `I` are kept in **pixel** units (px² for `I`, px²/s² for `Δω`
  consistency), the gains are pure numbers tied to the pixel scene — no N·m
  conversion needed. If real N·m is required, convert via the scene's px→m
  scale and per-particle mass. **Decide this once** and keep `Kp`, `Kd`, `I`,
  and `τ_max` in the same unit system.

---

## 7. What still needs a decision (open)

1. **Source of `θ_desired`.** Options, in increasing complexity:
   - Constant `0.0` upright stabilizer (start here).
   - Raibert's original velocity-error law:
     `θ_desired = θ₀ + k_v · (v − v_desired)` — lean back under acceleration,
     forward under braking. This is the classic loop-3 coupling and is the
     physically-motivated choice for dynamic locomotion.
   - Externally commanded (aim/recoil/landing pose), PD only tracks it.
2. **`θ_actual` measurement basis.** Which particle group defines "the torso"?
   See **§9** — resolved to a torso-scoped attitude body (P1/P2/P3 about P2),
   with `θ` taken on that set's axis.
3. **Verification path.** The sandbox
   ([`resolver.rs`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1))
   is a pure position/velocity model with **no attitude state or torque output**,
   so loop 3 isn't testable there as-is. Cheapest headless route: add a tiny
   `BodyAttitude { θ, θ̇, τ }` state + integrator to the sandbox (pure math, no
   solver), sweep `Kp/Kd`, then port verified gains into the torso layer.
4. **Interaction with landing/flight.** During flight there is no ground
   reaction, so this is the *only* attitude authority (air reorientation). On
   `Land`, a temporary `θ_desired` bias toward vertical may be desirable.

---

## 8. Why this shape fits a driven ragdoll

Classic loop 3 is "torque the *body* against the leg's ground reaction to hold a
commanded pitch." In our multi-particle ragdoll the "rigid body" is the frame
`[P1, P2, P3]` rotating about its P2 pivot (the CoM), and the leg reacts at the
hip P3 — the same physical arrangement as Raibert's, distributed across the
torso chain rather than expressed as a single hip motor. It is one DOF, one
equation, and it composes cleanly with loops 1 (height) and 2 (foot placement)
because it adds only rotation about the CoM — no CoM translation — so it does
not duplicate loop 2's velocity job.

---

## 9. Topology: root, CoM, pivot, and where the hip actually is (P2 ↔ P3)

Raibert's loop 3 is written for a **single rigid body**: a torso with one CoM,
one orientation `θ`, and the leg attached at the hip. In the classic derivation
the CoM sits essentially at the hip, so "torque about the hip" and "torque about
the CoM" are the same point.

Our character is not that shape. From the blueprint and the controllers:

- **Spine particle chain** (`SpineController.particles`): `[PH, P0, P1, P2, P3]`
  top→bottom — head, neck, then the torso chain
  ([mod.rs:202](bevy_vello/examples/game_lib/src/character/mod.rs:202)).
- **Torso masses:** `P1`, `P2`, `P3` each carry `inv_mass = 0.1` → mass 10; head
  and limbs are mass 1.
- **The hip particle is P3.** The hip joints are `P2_P3_P40` and `P2_P3_P41`
  (pivot at **P3**, ±60°) and the knees are `P3_P40_PLLL` / `P3_P41_PRLL`
  (pivot at **P40/P41**, ±30%). `P3` connects to `P40`/`P41` by distance
  constraints, and `P40`/`P41` down to the feet `PLLL`/`PRLL`. The frame maps
  `spine_end → P3 (lower spine / hips)`
  ([observers.rs:847](bevy_vello/examples/game_lib/src/character_factory/observers.rs:847)).
- **P2 is the waist / rotation centre.** The frame maps `spine_mid → P2`
  ([observers.rs:846](bevy_vello/examples/game_lib/src/character_factory/observers.rs:846)),
  and `calculate_spine_drive` uses `particles[3]` (P2) as its centre
  ([systems.rs:418](bevy_vello/examples/game_lib/src/character/systems.rs:418)).

So the roles are:

| Role | Our particle |
|---|---|
| Head | PH |
| Neck | P0 |
| Upper spine / shoulders | P1 |
| Waist / rotation centre / spine-drive root | **P2** |
| **Hip** (leg attach; bottom of the torso chain) | **P3** |
| Knees | P40 / P41 |
| Feet | PLLL / PRLL |

### The CoM ↔ hip offset, and why it is the correct configuration

The upper-body CoM lands **near P2** (P1/P2/P3 are heavy and roughly centred on
P2), i.e. **~70 px above the hip (P3)**. That is physically right — a human's CoM
sits above the hip — and it differs from Raibert's single rigid body only because
his model collapses CoM and hip into one point.

The consequence is the clean part: **applying loop-3 torque about P2 is pure
rotation that does not translate the CoM**, so it does not duplicate loop 2's
velocity job. Loop 2 owns foot placement / CoM translation; loop 3 owns the
rotation about the CoM. Orthogonal.

### The two proposed fixes, re-assessed

**"Make P1–P2–P3 rigid."** — *Right instinct, and the frame already is that
body.* The frame `[P1, P2, P3]` is the rigid attitude body loop 3 needs, rotating
about its `spine_mid` pivot (P2). What remains open is only **how much rotation
authority to expose during ground stance**:

- During **flight / airborne** rotation is free (no ground reaction), so a rigid
  `[P1, P2, P3]` body is exactly right and loop 3 has full authority.
- During **ground stance** the leg is planted, so driving the frame rigidly
  injects velocity into the (planted) leg chain and can fight the stance IK and
  the loop-2 reach constraints.

So no *new* rigid constraint is needed — the frame is the body. The design
question is the stance-vs-flight authority split, which is why loop 3 commands a
**torque** rather than directly moving the particles.

**"Move the mass centre to P3."** — *No.* P3 **is** the hip, so moving the CoM
there would locate the CoM *at the hip* — closer to Raibert's single-body
simplification but *less* physical for an articulated character, where the mass
belongs above the hip. It would also perturb the body's inertia and lever arms.

### What the pivot choice means for the actuator

Loop 3's torque is conceptually **about the body CoM (≈ P2)**, and P2 is already
the frame's rotation center — so no pivot change is needed. But note the frame
includes **P3, the hip**: rotating the frame therefore rotates P3 (P2 stays put
as the pivot; P1 and P3 swing about it). That is correct and expected — the legs
are carried by the torso attitude and react at the hip — and it does **not**
violate §3, which forbids a hip-position or hip-velocity *term*, not the physical
coupling of the hip to the body.

The only real constraint is authority:

- Clamp `τ` (`±τ_max`) and keep `rotation_resistance` meaningful so a fast
  attitude swing never overpowers the planted-foot stance constraints.
- Measure `θ_actual` on the **upper-body axis** (e.g. the P2→P1 heading vs. world
  vertical). P3 is the *hip*, so the P2→P3 segment is below the torso's mass
  centre — it is the hip linkage, not the torso attitude axis.

### Net

- Rigidify? **No new constraint — the frame `[P1, P2, P3]` already is the rigid
  attitude body.** Rotate it about its P2 pivot; decide stance vs. flight
  authority.
- Move CoM to P3? **No** — P3 is the hip; the CoM correctly sits above it near P2.
- Pivot? **Keep P2 = CoM.** Rotation about it is CoM-pure and does not duplicate
  loop 2. Our existing frame already pivots here.
- Only real work: clamp/tune `τ` so attitude authority never fights the stance
  constraints, and define `θ` on the upper-body axis (not the hip segment).

---

## 10. Sandbox ↔ character: propagating the P3 change to P2

The discussion above settles **where loop 3 acts in the character**: about the
CoM, which is ≈ P2. But the *validated control model* — the kinematic sandbox in
[`resolver.rs`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1) — is
**hip-centric**. Its `LowerBodyPose.hip`
([resolver.rs:264](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:264))
is a single control point, and that point maps to the character's **P3 (the hip)**,
not to P2.

So the sandbox and loop 3 disagree about their reference point by one rigid
offset: the body-local lever arm from the CoM/rotation-centre down to the hip.

$$
r_{hip} = P_3^{\,\text{local}} - P_2^{\,\text{local}} \approx (0,\ -h_{com}),\qquad h_{com}\approx 70\,\text{px}
$$

The two frames are related by the attitude itself:

$$
P_3^{\,\text{world}} = P_2^{\,\text{world}} + R(\theta)\, r_{hip}
$$

where `θ` is exactly the loop-3 attitude (the upper-body axis vs. world vertical).
This is the *same* `θ` loop 3 regulates — so the coupling is not a new mechanism,
it is the definition of the two reference points.

### Why this matters for loop 3

Loop 3 computes `τ` about P2, producing `Δω = τ·dt / I` about P2. The sandbox,
however, only knows how to place and move its `hip` (= P3). To feed the sandbox
(or to lift its output back onto the character) the P2-frame rotation and the
P3-frame position must be converted through `r_hip` above. Left unconverted, a
loop-3 rotation about P2 would be mis-applied as an **extra hip translation** —
the exact failure mode §3 forbids (loop 3 must contribute no hip position or
velocity term).

### Two ways to propagate it

**(a) Bridge-side conversion (keep the sandbox pure).**
Leave [`resolver.rs`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1)
exactly as committed at `5797fec` — hip-centric, single control point. In the game
layer, convert at the seam:

- forward: `hip_target = P2_world + R(θ_desired)·r_hip`, feed `hip_target` to the
  resolver;
- backward: take the resolver's hip pose, back out `P2_world = hip_world − R(θ)·r_hip`
  and feed P2 to the frame actuator.

Cheap, no test churn. Cost: the P3↔P2 conversion and the attitude `θ` live *inside
the game glue*, so loop 3 can never be validated headlessly in the sandbox — the
§7 verification gap stays open.

**(b) Rebase the sandbox control point to the CoM (P2).**
Make the resolver's control point the **CoM (P2)** and attach the legs at the
rigid body-local offset `r_hip`, so the sandbox reproduces the character 1:1: the
control point *is* the loop-3 pivot. Benefits:

- loop 3 becomes **testable in the pure sandbox** (`cargo test -p game_lib`), the
  same way the vertical loop was — closing §7's "verification path" question;
- `apply_rigid_offset_to_frame` can drive the CoM point directly, no seam math;
- the sandbox's leg geometry already models hip-attach, so only the *reference
  point* moves up by `h_com`; the reach envelope (`compute_x_max`) is expressed in
  hip-drop terms and can stay, with `hip_height` measured from the CoM.

Cost: it re-derives the control point and shifts many existing tests’ expected
quantities (currently hip-referenced), so it deserves its own plan, not a drive-by
edit.

### Recommendation

Prefer **(b)**, but stage it. Loop 3 is being added now, and (b) is what makes
"loop 3 works" *provable* before the XPBD port — the same reason the vertical loop
was prototyped in the sandbox first. Concretely: keep `5797fec` as-is during the
immediate loop-3 authoring, and raise (b) as a follow-on plan
("rebase resolver control point to CoM / attach legs at `r_hip`"). Until then use
(a) as the interim seam, with the single rule that `r_hip` and `θ` are the *only*
coupling between the two frames.

### Net

- Mismatch confirmed: sandbox `hip` = character **P3**; loop 3 / frame pivot =
  **P2** (CoM), one lever arm `r_hip ≈ (0, −70 px)` away.
- Map with `P3_world = P2_world + R(θ)·r_hip`; `θ` is the loop-3 attitude itself.
- Interim: (a) convert at the game seam. Durable: (b) rebase the sandbox control
  point to the CoM so loop 3 is headlessly testable. Recommend (b) as its own plan.
