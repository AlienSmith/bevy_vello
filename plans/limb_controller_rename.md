# Plan (draft): Generalize the Arm IK Controller into a Limb Controller + Expose its Responsiveness Gain

Status: **Draft for discussion** · Area: character/limb control

## 1. Intent

Two changes to the arm IK controller
([`calculate_arm_ik`](bevy_vello/examples/game_lib/src/character/systems.rs:538)):

1. **Make its responsiveness tunable.** The controller currently hardcodes an
   interpolation weight of `0.5` — the same "magic 0.5" also hardcoded in the
   spine drive. That weight is the closest thing the arm has to a `Kp`
   (the *spring stiffness* is actually `1/angular_compliance`; the `0.5` is the
   **rest-angle chase gain** — how fast the constraint rest angle follows the
   IK-desired angle each tick). It should be a config field, like every other
   gain in `ArmConfig`.
2. **Rename/abstract it as a "limb" controller.** The same two-bone
   angular-constraint IK machinery is what the **legs** will need
   (see [`lower_body_pd_bridge.md`](bevy_vello/plans/lower_body_pd_bridge.md)),
   so the types and function should be named for a *limb*, not an *arm*.

## 2. The hardcoded `0.5`s (verified) — confirm which one

There are **two** distinct `0.5`s in the arm path. They are not the same thing:

| # | Site | Meaning | Is it a control gain? |
|---|---|---|---|
| A | [`rotate_toward`](bevy_vello/examples/game_lib/src/character/systems.rs:807): `interpolate_toward_angle(angle_a, angle_b, 0.5, max_delta)` | rest-angle **chase** weight (α) | **Yes** — the likely "Kp" |
| B | [`solve_aim_ik`](bevy_vello/examples/game_lib/src/character/ik.rs:136): `theta1 -= angle_diff * 0.5; theta2 -= angle_diff * 0.5;` | 4-iteration solver refinement split | No — a convergence weight |

The spine drive has the *same* α = `0.5` at
[`systems.rs:441/447/453`](bevy_vello/examples/game_lib/src/character/systems.rs:441),
and its doc comment already calls it "alpha 0.5 + the angular cap" natural
damping. **Assumption: the change targets A** (expose the chase α as a tunable
gain, shared with the spine). If it is B instead, the scope is smaller and
different.

## 3. Proposed rename map

The reusable unit is the **config**; the per-limb *instance* components
(`RightArmController`, `LeftArmController`, later a leg controller) stay
limb-specific.

| Old | New | Notes |
|---|---|---|
| `ArmConfig` | `LimbConfig` | the shared tuning + `ik_mode` |
| `calculate_arm_ik` | `calculate_limb_ik` | the shared solver |
| `WhichArm` | `WhichLimb` | `Left` / `Right` already generic |
| `ResetArmControlConstraintsEvent` | `ResetLimbControlConstraintsEvent` | |
| `ArmController` (legacy) | *(leave, or `LegacyLimbController`)* | only used via `CharacterController` |
| `RightArmController` / `LeftArmController` | *(keep)* | arm *instances* of a limb; a future `LeftLegController` fits the pattern |

Local variables (`arm`, `arm_q`, `arm.particles`) are cosmetic and can be
renamed opportunistically (`limb`), but are not part of the public contract.

## 4. The tunable gain

Add to `ArmConfig` (→ `LimbConfig`):

```rust
/// Rest-angle chase weight per tick (0..1): how fast each angular
/// constraint's rest angle follows the IK-desired angle. 1.0 = snap, low =
/// sluggish. This is the arm's practical "Kp": the true spring stiffness is
/// `1/angular_compliance`; this weight governs how hard the *target* moves
/// toward the solution. Previously hardcoded 0.5 in `rotate_toward`.
pub rest_chase_alpha: f32,
```

Default `0.5` (behaviour-preserving). Thread it through `rotate_toward` (add a
parameter) and its two call sites. Optionally share the same field with the
spine drive's `interpolate_toward*` calls so there is **one** "responsiveness"
knob across the body — decide whether that lives in `LimbConfig`, `SpineConfig`,
or a small shared `ControlTuning`.

## 5. Touch list (workspace)

`ArmConfig` / controller names are re-exported from
[`lib.rs`](bevy_vello/examples/game_lib/src/lib.rs:32) and used by:

- [`character/mod.rs`](bevy_vello/examples/game_lib/src/character/mod.rs:99) — definitions
- [`character/systems.rs`](bevy_vello/examples/game_lib/src/character/systems.rs:538) — the solver
- [`character/ik.rs`](bevy_vello/examples/game_lib/src/character/ik.rs:3) — `config: &ArmConfig`
- [`character_factory/observers.rs`](bevy_vello/examples/game_lib/src/character_factory/observers.rs:813) — constructs both configs
- [`input_management/systems.rs`](bevy_vello/examples/game_lib/src/input_management/systems.rs:5)
- [`weapons/system.rs`](bevy_vello/examples/game_lib/src/weapons/system.rs:34), [`weapons/mod.rs`](bevy_vello/examples/game_lib/src/weapons/mod.rs:7)
- [`health_bar/mod.rs`](bevy_vello/examples/game_lib/src/health_bar/mod.rs:18) — doc reference
- [`character/plugin.rs`](bevy_vello/examples/game_lib/src/character/plugin.rs:11) — event registration
- [`collision_detection/character_tuning_main.rs`](bevy_vello/examples/collision_detection/src/character_tuning_main.rs:682) — tuning UI

Mechanical `use`/type renames; `cargo check` surfaces every miss.

## 6. Tests

- Rename is behaviour-neutral: existing `game_lib` suite (76) + `ik.rs` tests
  stay green with **no assertion changes** (pure rename).
- `rest_chase_alpha = 0.5` reproduces current trajectories exactly.
- Sweep `rest_chase_alpha` ∈ {0.2, 0.5, 1.0}: monotonic convergence (higher = faster
  settle), no divergence at 1.0.

## 7. Relation to other work

- The **leg** use-case is exactly
  [`lower_body_pd_bridge.md`](bevy_vello/plans/lower_body_pd_bridge.md): the limb
  controller's two-bone angular IK is the natural fit for the sandbox-driven
  legs, so this rename is the precondition for that integration, not a cosmetic
  detour.
- Confirm the α is *not* duplicated a third time when the PD bridge lands
  (the PD law `a = Kp·Δx + Kd·Δv` would carry its own `Kp`).

## 8. Open questions

1. Which `0.5` — the `rotate_toward` chase α (A, assumed) or the `solve_aim_ik`
   split (B)?
2. Rename depth: config + function + enum only (recommended), or also the
   instance components (`RightArmController` → …)?
3. One shared responsiveness knob across spine + limbs, or one per config?

## 9. Resolved (user decision, 2026-10-12)

- **Q1 → A.** Target the `rotate_toward` rest-angle chase α. `solve_aim_ik`'s
  refinement split (B) is left alone — it is a convergence weight, not a gain.
- **Q2 → config + function + enum + event only.** `RightArmController` /
  `LeftArmController` (per-side *instances*) keep their names; the legacy
  `ArmController` struct is left for migration.
- **Q3 → one shared responsiveness knob.** `rest_chase_alpha` is added to **both**
  `LimbConfig` and `SpineConfig` (same value threaded to both sites), so the
  whole rig has a single responsiveness dial. Each config carries its own field
  (no third shared struct) to keep the ECS/config surface simple.

## 10. Implementation record (landed)

Rename applied across the workspace (mechanical, no assertion changes):

| Old → New | Sites |
|---|---|
| `ArmConfig` → `LimbConfig` | `character/mod.rs` (struct + `Default` + 2 fields), `character/ik.rs` (import, `compute_ik_positions` param, 3 test ctors), `character/systems.rs` (import, `calculate_limb_ik` param), `lib.rs` (re-export), `character_factory/observers.rs` (import + 2 ctors), `character_tuning_main.rs` (doc) |
| `calculate_arm_ik` → `calculate_limb_ik` | def (`systems.rs`) + 2 call sites (`update_character_movement`) |
| `WhichArm` → `WhichLimb` | `character/mod.rs`, `systems.rs`, `lib.rs`, `input_management/systems.rs` |
| `ResetArmControlConstraintsEvent` → `ResetLimbControlConstraintsEvent` (field `arm` → `limb`) | `character/mod.rs`, `character/plugin.rs`, `systems.rs`, `lib.rs`, `input_management/systems.rs` |
| `reset_arm_constraint_event` → `reset_limb_constraint_event` | def (`systems.rs`) + `character/plugin.rs` (import + schedule) |

New field:

- `LimbConfig::rest_chase_alpha: f32` (default `0.5`), threaded through
  `rotate_toward(.., alpha, ..)` → `interpolate_toward_angle(angle_a, angle_b,
  alpha, max_delta)`; the shoulder/elbow call sites in `calculate_limb_ik` pass
  `config.rest_chase_alpha`.
- `SpineConfig::rest_chase_alpha: f32` (default `0.5`), replacing the three
  hardcoded `0.5`s in `calculate_spine_drive`
  ([`systems.rs:441/447/453`](bevy_vello/examples/game_lib/src/character/systems.rs:441)).

Kept (unchanged): `RightArmController`, `LeftArmController`, legacy
`ArmController`, all `ArmControllerConfig` blueprint plumbing.

Tuning UI: added a `rest_chase_alpha (Kp)` slider to both the spine window
(`indicator_ui`) and the arm window (`arm_tuning_ui`) in
[`character_tuning_main.rs`](bevy_vello/examples/collision_detection/src/character_tuning_main.rs:679).

Verification:

- `cargo test -p game_lib --lib` → **76 passed; 0 failed; 1 ignored** (the
  pre-existing KNOWN DEVIATION ignore), no assertion changes.
- `cargo check -p collision_detection --all-targets` → **passes** (warnings only,
  all pre-existing).
- Behaviour-preserving: defaults reproduce the historical hardcoded `0.5` exactly.
