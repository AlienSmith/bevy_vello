# Plan: Spine Controller — Add Kd (target-space PD) and Track Three Linear Velocities

Status: **Draft for review** · Area: character control (spine drive)

## 1. Motivation

Two changes to the spine drive, both raised from watching the character after it
lands:

1. **It has no `kd` term.** The arm stack already implements the D half of a PD
   controller in *target space* (`rest_offset = +kd·ω`, see
   [`ArmConfig.kd`](bevy_vello/examples/game_lib/src/character/mod.rs:136) and the
   derivation in `calculate_arm_ik`). The spine has no equivalent — it relies only
   on the external-position spring (stiffness = `1/compliance`) plus the
   constraint's built-in `damping`. So it is effectively P-only with a crude
   velocity damper, and rings/overshoots the way the arm used to.
2. **The tracked state is the wrong basis.** Today the controller predicts
   **one linear velocity** (`vel2`, the P2 centre) and **two angular velocities**
   (`ω_upper`, `ω_lower`) — see the idle predictor at
   [`systems.rs:104–128`](bevy_vello/examples/game_lib/src/character/systems.rs:104).
   That is an *orientation-first* model. But once the character is **on the
   ground**, the spine's job is to keep the upper body **translated upright**
   (not fall over), which is a *translational* job, not a rotational one. The
   right basis is **three linear velocities** — one per driven particle
   (P1, P2, P3).

Both are small, local edits to one function plus one config struct. This plan
keeps the existing external-position-constraint machinery, the indicator visual,
and the angular-joint re-rest intact.

## 2. Current control (as-is, verified)

Per fixed tick, [`tick_spine_drive`](bevy_vello/examples/game_lib/src/character/systems.rs:17)
latches a *goal* onto [`SpineIndicator`](bevy_vello/examples/game_lib/src/character/mod.rs:231)
as **one centre + two bone headings**:

| Field | Meaning |
|---|---|
| `desired_center` | P2 goal (world, Vello y-down) |
| `desired_heading_upper` | P2→P1 goal |
| `desired_heading_lower` | P2→P3 goal |

- **Command branch** (`command_active`): `desired_center = P2 + dir·reach`,
  `desired_heading_upper = heading(dir)`, `desired_heading_lower = heading + live_bend`.
- **Idle branch** (lines 84–129): predict `desired_center = pos2 + vel2·2dt + g·2dt²`
  (**one** linear velocity) and each heading from `ω = cross(arm, Δvel)/|arm|²`
  (**two** angular velocities).

[`calculate_spine_drive`](bevy_vello/examples/game_lib/src/character/systems.rs:392)
then interpolates each of the three DOFs toward its goal
(`interpolate_toward_vec/angle(.., 0.5, max_step)`), builds three world targets

```text
p1_target = virtual_center + rotate_local(local_points[0], heading_upper − rest_upper)
p2_target = virtual_center
p3_target = virtual_center + rotate_local(local_points[2], heading_lower − rest_lower)
```

and queues one `ExternalPositionConstraintConfig { target, compliance, damping }`
per particle, plus a re-rest of the `P1_P2_P3` angular joint from the *same*
targets (so position and angular constraints never fight).

**The interpolated "virtual pose" IS the target** — so the force is
`K·(virtual_pose − pos)` with `K = 1/compliance`, and the only velocity term is
the constraint's own `damping: 0.01` (a raw velocity damper, not a tunable PD D).

## 3. Target control

### 3.1 Three linear velocities (replace 1 linear + 2 angular)

The idle predictor becomes a per-particle linear prediction, one velocity each:

```text
lead = 2·dt
for i in {P1, P2, P3}:
    pred_i = pos_i + vel_i · lead + g · lead²
desired_center        = pred_2
desired_heading_upper = atan2(pred_1 − pred_2)
desired_heading_lower = atan2(pred_3 − pred_2)
```

This uses `{vel1, vel2, vel3}` and no longer forms `ω` at all. The headings are
just the *readout* of the rigid frame from the three predicted points, so the
loop is genuinely translation-driven while the rest of the pipeline (visual,
angular re-rest) keeps working unchanged.

- **On the ground:** all three particles predict with the same `g·lead²` and
  their own `vel_i`, so the target pose is "where the body would drift next" —
  the constraint just holds it upright and stops the topple. Pure translation.
- **In the air:** the same law reproduces the ballistic carry (every particle
  gets the same gravity increment), so no *relative* rotation is injected — the
  spine holds its shape without needing a separate orientation model.

### 3.2 PD in target space (add Kd)

Add the D term exactly the way the arm does — by shifting the *target*, leaving
the solver untouched:

```text
target_i = pose_i − clamp_len( kd · vel_i, kd_max_offset )
```

where `pose_i` is the position-only virtual pose from §3.1. The constraint then
sees

```text
F_i ∝ (target_i − pos_i) = (pose_i − pos_i) − kd · vel_i
```

which is a **classical PD law**: `Kp = 1/compliance`, `Kd = kd/compliance`.

Sign derivation (mirror of the arm's `rest_offset = +kd·ω`): the constraint pulls
toward `target`, so `target = pose − kd·vel` produces a force term `−K·kd·vel`,
i.e. a genuine brake. (The arm writes `+kd·ω` only because its constraint angle is
`θ = φ_in − φ_out` with `θ̇ ≈ −ω`; here the state is a raw linear velocity, so the
sign is direct.)

- `kd` is a **time constant**, so like the arm's it is **not** multiplied by `dt`.
- Clamp the shift by `kd_max_offset` (a length) so a velocity spike can never
  fling the target away from the pose — the same safety bound as
  `ArmConfig.kd_max_offset` (≈0.12 rad) but in world units.
- **Order matters:** apply the D shift to the *external position* targets only.
  The `P1_P2_P3` angular rest must be computed from the **undamped** pose, so
  `rest == commanded` still holds and the position/angular constraints keep
  agreeing by construction (the invariant documented at
  [`systems.rs:479–496`](bevy_vello/examples/game_lib/src/character/systems.rs:479)).

### 3.3 Config additions

[`SpineConfig`](bevy_vello/examples/game_lib/src/character/mod.rs:64)
gains two fields, documented like the arm's:

```rust
/// Target-space D term (time constant, seconds). Shifts each particle's
/// external-position target *against* its velocity: `target = pose − kd·vel`,
/// giving `F = Kp·(pose − pos) − Kd·vel` with Kd = kd/compliance. Stateless,
/// re-derived every tick; NOT multiplied by dt (kd IS the time constant).
/// 0 disables (current behaviour).
pub kd: f32,
/// Hard clamp (px) on the `kd·vel` target shift, mirroring ArmConfig.kd_max_offset.
pub kd_max_offset: f32,
```

Defaults: `kd: 0.0` (off, so the change is behaviour-neutral until tuned),
`kd_max_offset: 30.0` (≈ one spine-particle spacing; scale to the character).
`damping` likely wants to drop toward `0.0` once `kd > 0` — see §6.

## 4. Code touch list

| File | Change |
|---|---|
| [`character/mod.rs:64`](bevy_vello/examples/game_lib/src/character/mod.rs:64) | Add `SpineConfig.kd`, `SpineConfig.kd_max_offset` + docs + defaults. |
| [`character/systems.rs:84–129`](bevy_vello/examples/game_lib/src/character/systems.rs:84) | Replace the 1-linear/2-angular idle predictor with the 3-linear `pred_i` predictor (§3.1). Command branch unchanged. |
| [`character/systems.rs:441–465`](bevy_vello/examples/game_lib/src/character/systems.rs:441) | Compute `pose_i` as today, then emit `target_i = pose_i − clamp_len(kd·vel_i)`. Read `vel_i` from the already-passed `particles`. |
| [`character/systems.rs:485–496`](bevy_vello/examples/game_lib/src/character/systems.rs:485) | Keep the angular re-rest on the **undamped** `pose_i` (do not feed the D-shifted targets). |
| [`spine_indicator.rs`](bevy_vello/examples/collision_detection/src/spine_indicator.rs) | Optional: add `kd` / `kd_max_offset` sliders to the tuning UI. Visual/draw code unchanged. |
| `character_tuning_main.rs` | Wire the new sliders if added. |

`calculate_spine_drive` already receives `particles: &Vec<VelloParticle>`, so
`vel_i` is available without a signature change.

## 5. Tests (`cargo test -p game_lib`)

- `kd = 0` ⇒ byte-identical targets to the current implementation (pure
  regression guard: the refactor is behaviour-neutral at the default).
- **Overdamped convergence:** a particle with `pose` ahead of it and forward
  velocity settles without overshoot when `kd > 0`; with `kd = 0` it overshoots.
- **Clamp:** `kd_max_offset` bounds the target shift for an extreme velocity.
- **Neutral predictor:** identical `(pos_i, vel_i)` ⇒ identical `(center, headings)`
  (the purity invariant from [`procedural_spine_targeting.md`](plans/procedural_spine_targeting.md:16)).
- **Angular agreement:** the `P1_P2_P3` rest is unchanged by the D term (rest ==
  undamped pose), so the constraint-agreement invariant holds for any `kd`.
- Existing spine/IK suites stay green.

## 6. Risks & open questions

1. **D double-counting.** The constraint's `damping: 0.01` also removes
   velocity. With `kd > 0` the user likely wants `damping → 0`. Decide: keep both
   (compose) or zero `damping` when `kd > 0`? Recommend documenting `damping` as
   "legacy velocity damper; prefer `kd`" and defaulting it to 0.
2. **Stability bound.** For the linear spring, a critically-damped `kd` is
   `2·sqrt(m·compliance)`-relative. Units here are px and `inv_mass`; the tuning
   rule should be stated in the config docs (start small, raise until the settle
   stops overshooting).
3. **Representation surface.** This plan keeps the `[center, two headings]`
   indicator representation (minimal diff) and derives it from the three linear
   velocities. A **fuller alternative** is to make the indicator carry
   `pose: [Vec2; 3]` / `desired_pose: [Vec2; 3]` directly. That is more literal to
   "track three linear velocities" but touches the whole visual pipeline
   (`draw_spine_indicator`, `build_bent_points`, the desired-outline scene).
   Recommend the minimal variant first; revisit if the heading plumbing becomes a
   limitation.
4. **Airborne rotation authority.** With the 3-linear predictor there is no
   explicit orientation term, so a deliberate mid-air re-aim relies on the
   command branch (which still sets `desired_heading_upper`). Confirm that is
   sufficient before removing the angular readout (which this plan does not do).

## 7. Out of scope

- Raibert loop 3 (body-attitude torque PD) — parked; see
  [`raibert_loop3_attitude.md`](bevy_vello/plans/raibert_loop3_attitude.md).
- The impulse-vs-velocity transport change — separate, see
  [`spine_impulse_interface.md`](plans/spine_impulse_interface.md).
