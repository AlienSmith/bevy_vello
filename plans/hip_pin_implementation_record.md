# Hip-Pin Integration — Implementation Record

Companion to [`hip_pin_integration.md`](bevy_vello/plans/hip_pin_integration.md:1). This file is
the durable record of *what landed* and *what is still to implement*, kept out of the commit
message so the commit history stays one line per change.

## What landed

New [`lower_body/hip_pin.rs`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1):

- `HipPin` / `HipPinParams` (`Resource`; `config: LowerBodyConfig`, `compliance ≈ 1e-7`,
  `damping ≈ 0.01`).
- [`rigid_frame_targets()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:103) — θ=0 rigid
  frame (`P3* = anchor_p3 + world_delta`; `P1*`/`P2* = anchor + world_delta + R(θ)·rest_offset`);
  pure translation at θ=0, so bone lengths are preserved exactly.
- [`try_seed()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:119) — anchor+delta mapping
  (Bevy y-up → Vello y-down: `Vec2::new(dx, -dy)`), seeded from `rest_offsets = [P1−P3, P2−P3, 0]`;
  guards on `root.initial_frame_coordinates.is_some()`.
- [`drive_hip_pin()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:153) — reads
  WASD → `MoveRight/Left/Up/Down` into `input_dir`, steps the resolver, computes `world_delta`,
  derives the three targets, and emits exactly **3** `CharacterExternalPositionConstraintEvent`
  for particles `[2,3,4]` (P1/P2/P3); nothing angular.
- [`reset_hip_pin()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:223) — `KeyR`.
- 3 unit tests: [`rigid_frame_preserves_bone_lengths`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:242),
  [`rigid_frame_theta_zero_is_pure_translation`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:259),
  [`rigid_frame_tracks_resolver_across_dts`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:277)
  (bounded walking episode over the resolver's **documented 30–240 Hz dt envelope**).

Supporting changes:

- `SpineDriveEnabled(pub bool)` (`Resource`, default `true`) in
  [`character/mod.rs`](bevy_vello/examples/game_lib/src/character/mod.rs:116); gates
  [`tick_spine_drive`](bevy_vello/examples/game_lib/src/character/systems.rs:17) via an
  `Option<Res<SpineDriveEnabled>>` early-return, so other scenes are unaffected.
- Exports in [`lib.rs`](bevy_vello/examples/game_lib/src/lib.rs:1);
  `mod hip_pin;` + re-exports in [`lower_body/mod.rs`](bevy_vello/examples/game_lib/src/lower_body/mod.rs:1).
- Tuning app [`character_tuning_main.rs`](bevy_vello/examples/collision_detection/src/character_tuning_main.rs:206):
  inserts `HipPinParams::default()` + `SpineDriveEnabled(false)`;
  `drive_hip_pin.before(update_constraint_world)` in `FixedUpdate`; `hip_pin_ui` (enable checkbox,
  θ slider, compliance/damping sliders); spawns `HipPin::new(...)` in `setup_entity`; and drops the
  static ground wall (Vello y=540) **scene-locally** to disable foot↔floor collision without
  touching the shared character collision group.

Topology reused: `P3 = hip`, `P2 = CoM` (see
[`raibert_loop3_attitude.md`](bevy_vello/plans/raibert_loop3_attitude.md:197) §9–§10).

## Verification

- `cargo test -p game_lib` → **79 passed, 0 failed, 1 ignored** (the ignored test is a pre-existing
  `KNOWN DEVIATION`, unrelated to this work).
- `cargo check -p collision_detection` → compiles (pre-existing warnings only).
- Git-level: `resolver.rs` is **not** touched by this work.

## Yet to implement (deferred, not abandoned)

1. **End-to-end measured-tracking gate** — assert `|P3_live − P3*| < tol` in the *running* tuning
   app. The headless portion is covered by the 3 unit tests; the in-app observation is user-run.
2. **Loop-3 attitude PD** — drive `θ` from the attitude controller instead of the fixed θ=0
   ([`raibert_loop3_attitude.md`](bevy_vello/plans/raibert_loop3_attitude.md:1)). Planned second step.
3. **Soft phase** — raise `compliance` so the pin becomes a compliant tracker, re-introducing the
   PD / velocity-target portion of [`lower_body_pd_bridge.md`](bevy_vello/plans/lower_body_pd_bridge.md:1).
4. **Spine remodel** — Kd + three-linear-velocity basis
   ([`spine_pd_three_linear.md`](bevy_vello/plans/spine_pd_three_linear.md:1)).
5. **Resolver coarse-`dt` hardening** — **pre-existing** limitation of
   [`regulate_height()`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:391) (symplectic
   Euler; only conditionally stable — see
   [`hip_pin_integration.md`](bevy_vello/plans/hip_pin_integration.md:167) §8). Not touched by this
   work; moot at the fixed 90 Hz physics tick (≈3.7× inside the stable band). Add substepping or a
   clamped `dt` in [`step_lower_body()`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:853)
   only if a deliberately coarse debug/slow-motion path is ever added.
