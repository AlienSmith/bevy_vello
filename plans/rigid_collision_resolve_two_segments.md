# Rigid Collision Resolve for the Two-Segment Spine (P1–P2–P3)

## Goal

Resolve collision corrections on the spine as a rigid affine transform over the
two segments sharing the pivot P2 (spine_mid). Both segments must rotate by the
**same** angle so the P1–P2–P3 chain behaves as one rigid body, with P2 resolved
exactly once.

## Design: rewrite the P1–P2–P3 resolve (3-particle)

The existing [`apply_rigid_offset_to_frame`](study_vello/integrations/vello_physics/src/utility.rs:886)
operates on a legacy 4-slot layout `[hips…, spine_mid]` and stores P2 twice
(`frame_particles = [P1, P2, P3, P2]`), forcing a fragile dedupe. Instead, add a
NEW explicit 3-particle function that solves the rigid motion (translation `V` +
rotation ω about the P2 pivot) and returns deltas for exactly `[P1, P2, P3]`.
No duplicate index ⇒ **no double-apply by construction**.

### New function in `utility.rs`

```
pub struct SpineRigidDelta {
    pub spine_start: Vec2, // P1 delta
    pub spine_mid:   Vec2, // P2 delta (the pivot)
    pub spine_end:   Vec2, // P3 delta
}

pub fn resolve_rigid_spine(
    spine_start: Vec2,                 // P1 world pos
    spine_mid:   Vec2,                 // P2 pivot world pos
    spine_end:   Vec2,                 // P3 world pos
    offsets: &[(Vec2, Vec2, f32)],     // [(world_pos, offset, damping)]
    rotation_resistance: f32,          // 0..1 0=full rotation 1=pure translation
) -> SpineRigidDelta
```

Algorithm (deltas are the small-angle rigid motion about P2):

```
pivot = spine_mid

accumulate linear = Σ offset*damping / n
           angular = Σ cross(r_j, eff_j)   where r_j = world_pos_j − pivot
           inertia = Σ r_j·r_j
           avg_r   = Σ r_j / n

w = (angular / inertia) * (1 − rotation_resistance)      // guard inertia ≈ 0
V = linear − w × avg_r                                    // re-solve translation

d1 (P1) = V + w × (P1 − pivot)
d2 (P2) = V                                      // pivot, no lever arm
d3 (P3) = V + w × (P3 − pivot)          // SAME w as P1 ⇒ both rotate about P2 by ω
```

Empty offsets → return all-zero deltas. Degenerate (inertia ≈ 0) → `w = 0`.

### Rework `apply_collision_correction` in `soft_body_connection.rs`
- Read live positions for `spine_start`/`spine_mid`/`spine_end` from
  `frame_particles[0..=2]` (indices 1 and 3 are both P2, so use only 0..2 for the
  unique P1/P2/P3).
- Call `resolve_rigid_spine` ONCE on `pos_offsets` to get the spine position deltas.
  Do **not** run a separate `vel_offsets` solve (that path injected a `vel_delta`
  through `apply_kinematic_delta`, adding energy).
- Apply the returned deltas to the **distinct** particle indices `[0, 1, 2]` only
  (P2 applied exactly once), as a **pure position correction**: shift **both**
  `particle.pos` and `particle.previous_pos` by the same delta. Because
  `velocity = pos − previous_pos` is unchanged, no kinetic energy is added.

### Pure position correction (no energy injection)
Because we shift `pos` and `previous_pos` together, the implied velocity is
preserved. This replaces the legacy `apply_kinematic_delta(pos_delta, vel_delta)`
call where a non-zero `vel_delta` perturbed velocity and added energy.

### Keep `apply_rigid_offset_to_frame`
It stays for the `transmission-graph` path and any legacy callers. The new
`resolve_rigid_spine` is the primary path for `apply_collision_correction`.

## Tests in `utility.rs`

- **Same-direction collision** → pure translation: all three deltas equal `V`,
  `w ≈ 0`; P3 moves with the body, no relative rotation.
- **Transverse collision** → P1 & P3 rotate about P2 by the identical ω:
  - `angle(P1 + d1 − P2) − angle(P1 − P2) == ω`
  - `angle(P3 + d3 − P2) − angle(P3 − P2) == ω`
  - `|(P3 + d3) − P2| == |P3 − P2|` (rigid length preserved to first order)
- **`rotation_resistance = 1`** → `w = 0` (pure translation), P3 keeps its
  orientation offset.
- **Empty offsets / degenerate pivot** → all-zero deltas, no NaN.
- **Pure position correction**: applying `d_i` to `pos` AND `previous_pos` leaves
  the implied velocity `pos − previous_pos` unchanged (asserted in the test).

## Out of scope (this pass)
- The `transmission-graph` path (`apply_collision_correction_transmission`) still
  uses the legacy `apply_rigid_offset_to_frame`; wiring it to `resolve_rigid_spine`
  is a follow-up.
- No character JSON / schema changes (positions only; frame is already 3-particle).

## Acceptance criteria
- `resolve_rigid_spine` yields P1 and P3 rotating about P2 by the identical ω.
- Segment length `|P2→P3|` is preserved under rotation.
- P2 is resolved exactly once (no duplicate index, no double-apply).
- All tests pass: `cargo test -p vello_physics` (incl. `--features transmission-graph`).