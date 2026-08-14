# Spine Control Fold Analysis

## Symptom

Pressing a spin key in one direction works, but pressing the **opposite** direction
folds the character into a weird shape instead of smoothly reversing its turn.

This document lists the potential sources of the problem, identified by tracing the
code and geometry. They are ordered from root cause to amplifying factor.

## The geometry

Spine particles (vertical line, x ≈ 347):

```text
PH  (347, 94)     ← head
P0  (347, 143)    ← upper chest
P1  (347, 202)    ← mid chest, CENTER OF ROTATION (t=0)
P2  (347, 275)    ← spine_mid, FRAME PARTICLE
P3  (347, 345)    ← spine_base, FRAME PARTICLE
```

Frame particles:

```text
P30 (309, 346)    ← left_hip
P31 (383, 346)    ← right_hip
P3  (347, 345)    ← spine_base (shared with spine)
P2  (347, 275)    ← spine_mid (shared with spine)
```

## Problem sources

### 1. Rotation center outside the frame (geometric mismatch)

The spine rotates around `P1` (y=202), but the frame lives entirely **below** it
(y=275 to y=346). `P1` is not a frame particle. The controller's "rotation" is
therefore geometrically a **shear** on the frame, not a rigid rotation.

### 2. Controller moves only 2 of the 4 frame particles

The spine controller writes velocity to 5 particles (`PH, P0, P1, P2, P3`). Of the
4 frame particles, only `P2` and `P3` receive spine velocity. `P30` and `P31`
(hips) receive none. The frame is sheared every substep.

### 3. Frame becomes non-rigid from controller input

The shear makes the 4 frame particles non-rigid relative to each other. The
distance constraints (`P2-P30`, `P2-P31`, `P3-P30`, `P3-P31`) and angular
constraints (`P2-P3-P30`, `P2-P3-P31`) resist this, creating internal stress.

### 4. `reconstruct_frame()` runs after `predict_positions()` (ordering)

The frame is rebuilt from already-sheared positions.
`PositionConstraint::solve()` then computes `world_target` from this distorted
coordinate system, dragging limbs toward wrong targets.

### 5. `cache_delta` conflates control motion with feedback

`PositionConstraint::solve()` records how much each limb was dragged. When the
frame is sheared, this drag includes both legitimate limb-following-frame motion
and the artifact of the distorted frame. The two are indistinguishable in
`cache_delta`.

### 6. Transmission graph propagates the conflated signal

The graph feeds `cache_delta` through the skeleton and applies `-drag` as a rigid
counter-offset to the frame. A rigid transform cannot undo a shear, so the
correction introduces additional distortion.

### 7. `-drag` directly sets positions on frame particles

The correction does `p.pos += deltas[i]` — a direct position write that bypasses
the distance/angular constraints between frame particles, potentially violating
the rigidity those constraints are trying to maintain.

### 8. The `dot_val < -0.99` amplifier (reversal-specific)

During 180° reversals, the controller injects a constant large spin
(`rotation_gain * 2.0`) instead of the decaying cross product. This makes the
shear very large, violently amplifying all of the above problems.

## Summary

The combined effect: controller shears frame → distorted `world_target` → limbs
dragged to wrong places → `cache_delta` conflated → `-drag` applies a rigid
counter-transform to a sheared frame → distance/angular constraints fight both →
the character collapses into a folded shape during reversal.

## Fix applied (controller-side)

The causal root is the shear the controller injects into the frame. The frame
basis itself is already stable (it rests purely on P3→P2, orthonormal by
construction in [`BalancedCoreFrame::new`](../study_vello/integrations/vello_physics/src/utility.rs:731)),
so the target *direction* is always consistent. The fix removes the shear at its
source in [`claculate_velocity_spine`](examples/game_lib/src/character/systems.rs:50):

1. **Rotation center moved from P1 → P2.** P2 (`spine_mid`) is a frame particle,
   so the controller's rotation is now a rigid frame rotation instead of a shear.
   The old code rotated about P1 (index 2, `t = 0`), which lives *above* the frame
   (y=202 vs frame y=275–346) and applied a tangential field that sheared the frame.

2. **All 4 frame particles are now driven rigidly.** The controller emits velocity
   events for P2, P3, **and the hips P30/P31** using the same rigid velocity field
   `ω × (pos − pivot) + translational`. Previously only P2/P3 were driven, so the
   hips were left behind and the frame sheared every substep. With a rigid frame,
   the shape-matching `-drag` correction becomes a clean rigid counter-transform
   instead of a distortion.

3. **Reversal amplifier tamed.** The old `dot_val < -0.99 → rotation_gain * 2.0`
   constant kick (unbounded, always the same rotational direction) is replaced by
   a smooth sign-correct ramp: `signum(cross_val) * sqrt(1 - dot_val)` clamped to
   [-1, 1]. This removes the violent shear kick at the exact moment of a 180° turn.

### Remaining latent fragility (not the fold, but robustness)

- `reconstruct_frame` still runs after `predict_positions` inside each substep
  ([`SoftBodyConnections::step`](../study_vello/integrations/vello_physics/src/soft_body_connection.rs:117)),
  so the frame origin chases just-moved P3. Benign with a rigid controller.
- `-drag` still writes positions directly (`p.pos += deltas[i]`), bypassing frame
  rigidity constraints. Only bites under external stress (collisions, hard pushes).
- Collision/external-force shears can still non-rigidize the frame independently.
