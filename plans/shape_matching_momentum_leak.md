# Shape-Matching Direct-Drag & the Collision Fast-Path

## Background

We observed that the character "steps on its own feet / can fly" — a spurious
propulsion that comes from the internal soft-body correction, not from external
forces. This note analyzes the mechanism and proposes physically-grounded fixes.

## The collision fast-path (existing)

`SoftBodyConnections::apply_collision_correction`
(`study_vello/integrations/vello_physics/src/soft_body_connection.rs:139`)

- Collects collision offsets from `bilinear_constraint.solve_external_force(...)`.
- Pushes them onto the 4 **frame particles** via
  `apply_rigid_offset_to_frame` (`utility.rs:807`).
- Those 4 frame particles feed `reconstruct_frame` (`soft_body_connection.rs:465`)
  → `BalancedCoreFrame::new(...)`, which seeds `frame.local_to_world(local_target)`
  used by the shape-matching constraint.

So collisions propagate **forward**: external → frame → shape-matching coordinate.

## The shape-matching direct-drag (the bug)

`PositionConstraint::solve` (`connection_constraint.rs:310`):

```rust
p.pos += delta_lambda * p.inv_mass * dir;
p.previous_pos += delta_lambda * p.inv_mass * dir;
```

- In `add_shape_matching_constraint` (`soft_body_connection.rs:264`) the frame
  particles get `compliance = 1000.0` (stiff reference), so **only the body
  particles move**.
- Net result: a correction is applied to body particles with **no
  equal-and-opposite reaction on the frame** → momentum leak.
- `damp_particle_velocity` (`connection_constraint.rs:335`) is just a velocity
  **sink**, not a momentum-conserving reaction, so it fights the angular leader
  instead of eliminating spurious propulsion.

This is exactly the "you can fly / step on your own feet" feel.

## The concern

The collision fast-path already relies on hand-tuned scalars:
- `collision_damping` (`add_shape_matching_constraint`:265-267) and
- `rotation_resistance` (used in `apply_rigid_offset_to_frame`).

These are **ad-hoc approximations** of the energy split — we don't actually
know the exact ratio, and the simulation is supposed to derive energy transfer
through the constraints. The existing fast-path is a "non-physical" shortcut
because the real propagation is too slow to simulate directly.

Adding a second non-physical fast-path on top would compound the vagueness and
make the whole system "act funny." So the goal is to fix the root cause with a
physically-honest mechanism, not to stack more shortcuts.

## Proposed physically-grounded fixes

### Option 1 — make `PositionConstraint` two-sided (preferred)
Split the correction between the body particle and the 4 frame particles by
their **actual inverse masses** (standard XPBD equality constraint).
- No vague `collision_damping` ratio.
- Net momentum is conserved → internal shape-matching cannot propel the body.
- Frame still moves from the exact reaction (correct physics).

### Option 2 — reuse `apply_rigid_offset_to_frame` with exact negated delta
Keep the existing vehicle but feed the *negated actual delta* of each body
particle into the frame with `ratio = 1.0` (no damping scalar).
- Removes the fudge; reaction is the exact Newton's-third-law counterpart.
- Smaller change, reuses collision vehicle.

### Option 3 — derive both ratios from real inverse masses
Keep the collision fast-path but replace hand-tuned scalars with
inverse-mass-weighted ratios so both collision and shape-matching reactions are
physically exact.

## Key files

- `study_vello/integrations/vello_physics/src/soft_body_connection.rs`
  - `apply_collision_correction` :139
  - `post_step` :202 (calls `shape_matching_damping` then `apply_collision_correction`)
  - `reconstruct_frame` :465
  - `solve_constraints` :474
  - `add_shape_matching_constraint` :255 (stiff frame compliance = 1000.0)
- `study_vello/integrations/vello_physics/src/connection_constraint.rs`
  - `PositionConstraint::solve` :310
  - `damp_particle_velocity` :335
- `study_vello/integrations/vello_physics/src/utility.rs`
  - `apply_rigid_offset_to_frame` :807

## Open question (not yet resolved)
Decide which option; Option 1 is the most physically principled (true
per-constraint momentum conservation) and removes the need for ad-hoc scalars.
