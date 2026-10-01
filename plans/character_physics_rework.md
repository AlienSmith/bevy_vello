# Character Physics Rework — Root Isolation + Active FK + Two-Segment Spine

> Status: **Planning / reminder doc**. We will walk through each of the four
> points below and discuss **before** implementing. This file captures the
> problem statement, the reasoning we have so far, and known open questions.
> It is intentionally a memory aid, not a final spec.

---

## 0. Current State (baseline)

### 0.1 Skeleton (from `v8.character.json`)

16 particles. Frame / spine triple is `[P1, P2, P3]`:

| stage | particles | note |
|-------|-----------|------|
| head / neck | `PH`, `P0` | `PH` top of head, `P0` neck top |
| **spine_start** | `P1` | `inv_mass = 0.01` (heavy) |
| **spine_mid** | `P2` | shared pivot / controller root, `inv_mass = 0.01` |
| **spine_end** | `P3` | `inv_mass = 0.01` |
| shoulders | `P11` (L inner), `P12` (R inner) | `inv_mass = 1.0` |
| upper arm outer | `P10` (L), `P13` (R) | |
| hands | `PLLA` (L), `PRLA` (R) | |
| pelvis | `P30` (L), `P31` (R) | |
| hips | `P40` (L), `P41` (R) | |
| feet | `PLLL` (L), `PRLL` (R) | |

Frame config: `rotation_resistance = 0.0` (full rotation allowed),
`collision_damping = 1.0`, `fk_target_damping = 0.1`.

Limb chains:
- Arms: `P1 -> P11/P12 -> P10/P13 -> PLLA/PRLA`
- Legs: `P3 -> P30/P31 -> P40/P41 -> PLLL/PRLL`

### 0.2 Transmission graph (the FK damping target tree)

[`transmission_graph.rs`](study_vello/integrations/vello_physics/src/transmission_graph.rs)
- A **rigid forward-kinematics tree**, built once from the `DistanceConstraint`
  + `AngularConstraint` topology.
- Batty: rooted at **`spine_mid` P2** with `spine_start` P1 and `spine_end` P3
  as depth-1 children (see
  [`ensure_transmission_graph()`](study_vello/integrations/vello_physics/src/soft_body_connection.rs:287)).
- Each particle has exactly one tree parent (loops/cross-links skipped).
- [`compute_targets()`](study_vello/integrations/vello_physics/src/transmission_graph.rs:230):
  anchors the 3 frame particles at their current positions, then places every
  descendant rigidly in depth order (chain step = rotate `dir_in` by the rest
  turn angle, then `target = joint + dir_out * rest_len`).
- [`apply_rigid_fk()`](study_vello/integrations/vello_physics/src/soft_body_connection.rs:236):
  uses these targets today as a **damping** target — eases each particle toward
  `target` by `fk_target_damping`, and rewrites `previous_pos` so the
  implied velocity is preserved.

### 0.3 Collision correction (mutually exclusive path)

[`apply_collision_correction()`](study_vello/integrations/vello_physics/src/soft_body_connection.rs:149)
is chosen in
[`post_step()`](study_vello/integrations/vello_physics/src/soft_body_connection.rs:305)
when the `transmission-graph` cargo feature is **off**:
- Reads per-body coarse collision offsets, applies to joint particles via
  [`apply_kinematic_delta`](study_vello/integrations/vello_physics/src/collision_response.rs:116).
- Propagates to the spine as a **single rigid motion about `spine_mid` P2** via
  [`resolve_rigid_spine()`](study_vello/integrations/vello_physics/src/utility.rs:975):
  solves translation `V` + rotation `w` about P2, gated by `rotation_resistance`,
  and returns pure **position** deltas for P1/P2/P3 (shifted on both `pos` and
  `previous_pos` to preserve velocity — no energy injection).

### 0.4 Constraint machinery (relevant primitives)

- [`ExternalPositionConstraint`](study_vello/integrations/vello_physics/src/connection_constraint.rs:295):
  pulls a particle toward a world target with `compliance` and `damping`.
  `solve()` does the XPBD position pull;
  `damp_particle_velocity()` adjusts `previous_pos`.
- [`BilinearConnectionConstraint`](study_vello/integrations/vello_physics/src/connection_constraint.rs:99):
  the skeleton↔body frame coupling (joint particle ↔ 4 softbody frame corners
  via UV). `solve_external_force` aggregates coarse offsets for the joint.
- Two-way XPBD coupling is the default (`skeleton_drives_body` / legacy path).

---

## 1. The core problem (root is being dragged)

`P1/P2/P3` is the **root** — the controller moves the whole body through it. But
the spine particles are connected to the limbs through XPBD constraint springs,
which are **two-way**: the limbs can drag the root around. That makes the body
hard to control consistently in a game. Current mitigation = make `P1/P2/P3`
very heavy (`inv_mass = 0.01`).

---

## 2. The four rework directions (work items)

We want a **full rework** across these 4 points, each discussed before
implementation.

### Point 1 — Make root → limb coupling one-directional

- Make the connection between `P1/P2/P3` and the other (limb) particles
  **one-directional** so the limbs can no longer push the root around.
- Fallback that is already present: super-heavy `P1/P2/P3`.
- **Consequence:** the limbs no longer transmit collisions to the root on their
  own, so **collision correction must account for walls / other colliders** on
  the spine itself.

### Point 2 — Promote the FK target to an active external position target

- Today the FK target (transmission graph output) is used only for **damping**.
- Proposal: use it **actively** as the target of an external position
  constraint (the [`ExternalPositionConstraint`](study_vello/integrations/vello_physics/src/connection_constraint.rs:295)
  pattern).
- Reason it is safe: because non-root particles can't drag the root around, the
  "FK-from-root → non-root drags root back toward target" feedback loop that
  previously motivated keeping FK as pure damping is no longer a concern.

### Point 3 — Make the FK result persistent

- The FK target positions should be computed and retained so they can be ported
  to the **game side** and rendered for **debug**.
- Today they are transient per-frame easing values. We need a stable storage /
  exposure shape.
- **DECIDED DIRECTION (discussed):** store the FK result as a map keyed by
  **particle arena `Index`** on `SoftBodyConnections`, because:
  - `TransmissionGraph::compute_targets` already emits `HashMap<Index, Vec2>`
    keyed by thunderdome arena `Index`, so this is zero-copy / lossless at the
    physics layer and is one-to-one with every `VelloParticle`.
  - It is the natural "source of truth" before any Bevy translation.
- **Follow the existing proxy routine — no extra Bevy layer translation.** The
  wrapper group `SoftBodyConnectionGroup<T>` already owns
  `index_connection_particle: HashMap<T, Index>` (game key `T` → arena `Index`)
  and mirrors every physics op through it (e.g.
  `get_connect_particle`, `queue_connect_particle_velocity`,
  `add_one_time_external_position_constraint`). The FK-target map follows the
  same shape: physics stores `HashMap<Index, Vec2>`, and the group gets a proxy
  accessor `get_connect_fk_target(&T) -> Option<Vec2>` that converts `T → Index`
  via `index_connection_particle`, exactly like the existing methods. The arena
  `Index` map is the storage; the group is the keyed mirror to the game side.
- **Game-side retrieve + store on `VelloParticle` (mirrors particle sync):**
  1. `update_connection_particles` iterates `Query<(Entity, &mut VelloParticle)>`
     and for each entity pulls `group.get_connect_particle(&e)` → writes
     `joint.particle`. The FK target does the **same**: pull
     `group.get_connect_fk_target(&e) -> Option<Vec2>` and store it on
     `VelloParticle` (a new `fk_target: Vec2` field alongside `particle`).
  2. Visualize it in the debug pass like the existing pivot visualizer
     (`create_update_pivot_visualizer`): for each particle draw a small point at
     `fk_target` using a **different color** from the particle-position points,
     so the FK target trail is immediately visible against the real particle.
- The whole chain is: FK solve writes arena-index map on `SoftBodyConnections`
  → group proxy `get_connect_fk_target` (T→index) → Bevy sync writes
  `VelloParticle.fk_target` → debug system draws colored points.

### Point 4 — Split the collision transform into upper + lower body

- Today [`resolve_rigid_spine()`](study_vello/integrations/vello_physics/src/utility.rs:975)
  treats `P1-P2-P3` as **one rigid body** rotating about `P2`.
- Proposal: solve the **upper body `P1-P2`** and the **lower body `P2-P3`** as
  **two separate rigid transforms**, both pivoting about the shared `spine_mid`
  `P2`.
- This mirrors the existing two-frame split
  [`build_spine_frames()`](study_vello/integrations/vello_physics/src/utility.rs:852)
  (`frame_start_mid` = P1→P2, `frame_mid_end` = P2→P3).

### Point 5 — Depth-based strength for collision damping / external constraints

- The damping strength (and future external-position-constraint strength)
  should scale with the **tree depth** of the particle.
- Example: shoulder = 1.0, elbow = 0.5, lower-arm end = 0.25.
- The [`TransmissionGraph`](study_vello/integrations/vello_physics/src/transmission_graph.rs:54)
  already tracks `depth` per particle.

### Point 6 — Humanoid *possible-shape* limits on the angular constraints

Status: **early / vague**. Added as a reminder to develop.

- Motivation: prevent the spine and limbs from **snapping into impossible
  shapes** for a humanoid character.
- Current [`AngularConstraint::solve()`](study_vello/integrations/vello_physics/src/constraints.rs:444)
  only enforces the *rest* angle (a single signed turn) with a soft XPBD
  compliance. It does **not** bound the achievable range of motion — so a
  joint can fold past a human-plausible limit.
- Idea: give joints (shoulder / elbow / hip / knee / spine, via the same angular
  constraints) a **valid angular range** around the rest basis, so the solver
  clamps / limits the pose instead of snapping.
- Unknowns (to flesh out before implementation):
  - Where the per-joint limits come from (JSON / character tool).
  - Whether to clamp the **FK target** (Point 2) and/or enforce in the
    **solver** (or both).
  - Interaction with active FK external-position targets (Point 2) and the
    depth-based strengths (Point 5).
  - Links to Point 7: beta-damping helps stability when range-clamps engage.

### Point 7 — XPBD beta-damping term

Status: **early / vague**. Added as a reminder to develop.

- Motivation: add a **velocity-level damping** to constraint iterations, in
  addition to the stiffness provided by the existing compliance term.
- Current formulation uses only `alpha = compliance / (dt * dt)` in the XPBD
  denominator (e.g. [`AngularConstraint::solve()`](study_vello/integrations/vello_physics/src/constraints.rs:481),
  [`DistanceConstraint`](study_vello/integrations/vello_physics/src/constraints.rs:30)).
- Idea: add a **`beta / dt`** term alongside the `compliance / dt²` term — the
  classic extended-XPBD damping coefficient (`beta`) that dissipates constraint
  relative velocity and reduces jitter / oscillation / snapping.
- Where it belongs: at minimum the **angular constraints** (Point 6 bites back
  into this — damping helps when range-clamping kicks in); possibly reusable
  across other XPBD constraint types.
- Unknowns (to flesh out before implementation):
  - Per-constraint `beta` value(s) and config/plumbing (`AngularConstraintConfig`,
    JSON, tool).
  - Exact extended-XPBD form (denominator term vs. `gamma` lambda reuse), and
    how it composes with depth-based strengths (Point 5).

---

## 3. Design considerations / open questions

### Q1 — Two-segment transform math (Point 4)

Upper (`P1-P2`) and lower (`P2-P3`) solved separately about P2 means:
- The same collision offset will produce a **different rotation `w`** for each
  segment.
- The torso gains an independent bend at the waist (P2) vs the hip (P3) — more
  natural, but we **lose the fully-rigid chain**.
- Needs decisions:
  - Does `rotation_resistance` apply per-segment?
  - How does the **shared-pivot translation `V`** reconcile between the two
    solves? (both write P2; last-write or blended?)

### Q2 — Depth → strength mapping (Point 5)

- Suggested ladder: shoulder=1.0, elbow=0.5, hand=0.25 suggests
  `strength = 2^(ref_depth - depth)` (or `2^-depth`).
- Must pin the exact reference depth. In the tree: `P2`=root (depth 0),
  `P1`/`P3` (depth 1), shoulders (depth 2), elbows (depth 3), hands (depth 4).
- Confirm whether to store strength per particle (baked map) or compute on the
  fly from depth.

### Q3 — Persistent FK storage (Point 3)

- **RESOLVED:** physics stores `HashMap<Index, Vec2>` on `SoftBodyConnections`
  (arena-index keyed, one-to-one with `VelloParticle`), written every
  `apply_rigid_fk` solve and exposed via a getter (`get_fk_targets` returning the
  `HashMap<Index, Vec2>`).
- **Game-side shape — follow the existing proxy routine:** the wrapper group gets a
  proxy accessor `get_connect_fk_target(&T) -> Option<Vec2>` that looks up
  `index_connection_particle: HashMap<T, Index>` (`T → Index`) then reads the
  arena-index FK map — mirroring `get_connect_particle` / `queue_connect_particle_velocity`.
  No Bevy-layer `Entity → Vec2` rebuild; the group is the keyed mirror.
- **RESOLVED — retrieve like the particle and store on `VelloParticle`:** extend
  `VelloParticle` with an `fk_target: Vec2` field; `update_connection_particles`
  pulls `group.get_connect_fk_target(&e)` per entity alongside
  `get_connect_particle` and writes it. Visualization draws a colored point at
  `fk_target` (distinct color from particle-position points), in the pivot/particle
  debug pass.
- **Open sub-question:** bulk debug needs all targets at once. Decide whether to add
  a group-level iterator (yield `(T, Vec2)` for every mapped particle) or rely on
  the per-particle `VelloParticle.fk_target` (iterate the particle query) — since
  `VelloParticle.fk_target` already carries every target per particle, the game can
  iterate `Query<&VelloParticle>` directly and a group iterator is likely unnecessary.

### Q4 — One-directional coupling implementation (Point 1)

- "One-directional root→limb" — which mechanism?
  - Option A: repurpose the bilinear coupling so the joint writes the softbody
    frame but the frame is not allowed to write the joint (asymmetric weights /
    frozen frame side).
  - Option B: a dedicated directed constraint type.
  - Option C: keep heavy mass and just increase it / make it truly infinite.
- This affects how collision correction for the spine itself is wired
  (the spine is now "free" except for the external FK target + collision).

### Q5 — Relationship between Point 2 (active FK) and Point 4 (collision)

- If FK is an active external position target AND collision correction
  transforms P1/P2/P3, they can fight. Need an ordering / strength rule so
  collision wins for the collision-involved segments while FK shapes the limbs.

### Q6 — Humanoid range limits source & enforcement (Point 6)

- Where do per-joint angular min/max limits come from (new JSON fields /
  character tool)?
- Enforce in the FK **target** calculation, in the **constraint solve**, or both?
  (Clamping only FK could still let the solver overshoot; limiting only solve
  could fight the FK target.)
- How to express the valid range — min/max absolute angles relative to the
  rest basis, per `rest_sin` orientation?

### Q7 — Beta-damping form & scope (Point 7)

- Exact extended-XPBD form: is `beta` a pure denominator term
  (`alpha + beta/dt`), or the lambda-reuse `gamma` formulation?
- Per-constraint `beta` values: baked into config (`AngularConstraintConfig`)
  or computed from depth (Point 5)?
- Should beta apply to **all** XPBD constraint types (distance, angular,
  bilinear, collision) or only angular (and maybe collision)?

---

## 4. Notes / next steps

- We will **discuss each point before implementing it**.
- This is a running reminder; update it as decisions are made.
- Related existing plans:
  - `bevy_vello/plans/balanced_core_frame_two_segments.md`
  - `bevy_vello/plans/rigid_collision_resolve_two_segments.md`