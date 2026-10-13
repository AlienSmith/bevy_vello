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

## Headless judge layer (2026-10-13)

The two headless harnesses now share a **judge** — pure, log-only post-processing over captured
telemetry in [`headless/judge.rs`](bevy_vello/examples/game_lib/src/headless/judge.rs:1) (re-exported
from [`headless/mod.rs`](bevy_vello/examples/game_lib/src/headless/mod.rs:1) and
[`lib.rs`](bevy_vello/examples/game_lib/src/lib.rs:1)). It answers "did the physics behave?" rather
than only logging it:

- [`tail_verdict()`](bevy_vello/examples/game_lib/src/headless/judge.rs:89) — settled-tail RMS about
  the mean + deadbanded angular-rate **sign reversals** (a monotonic catch-up is not a wobble);
  slices the last `tail_s`.
- [`cell_verdict()`](bevy_vello/examples/game_lib/src/headless/judge.rs:168) — `WAGGLE` if the tail
  fails, else `SLOW`/`PASS`.
- [`check_invariants()`](bevy_vello/examples/game_lib/src/headless/judge.rs:193) — finite + rectangular
  + magnitude ceilings, **scoped to numeric channels only** (a column is numeric when its name ends in
  `x|y|vx|vy|ax|ay|fx|fy`). The string `phase` column is deliberately skipped — full-column finite
  scanning false-flagged it as `non_finite`.
- [`check_bone_rigidity()`](bevy_vello/examples/game_lib/src/headless/judge.rs:294) — per-bone length
  hold (rest learned from row 0), with `worst_row` naming the offending tick.
- [`rows_hash()`](bevy_vello/examples/game_lib/src/headless/judge.rs:369) / `diff_rows` — FNV-1a
  fingerprint for determinism replay. 8 unit tests pass.

Wired into `headless_arm_pd.rs` (verdict + invariants + hash) and `headless_pin.rs` (invariants +
`P1–P2`/`P2–P3`/`P1–P3` bone rigidity + hash).

### Finding 1 — the pin's `rig=FAIL` is the jump impulse, not the tracking

With `PIN_RIGID_TOL = 0.02`, the pin gate reports `rig=FAIL max_rel_dev≈0.035`, but per-bone
timestamps land at `t≈5.4–5.9 s`, i.e. **inside the scripted 5.0 s jump**, not during the walk.
Pre-jump (t<5.0) the frame holds to ≈0.05–0.12 % on the edges and ≤1.14 % on the `P1–P3` diagonal;
the diagonal is the sensitive shear indicator and only stretches under the jump impulse. This is an
honest, actionable result: the pin holds the frame rigidly while walking; the jump is where to look
next.

### Finding 2 — the harness drove the physics non-deterministically (flaky hash)

Two `headless_pin` runs produced **different** `rows_hash` values (`9ebb0ee3…`, `24ba5958…`,
`23de2238…`, `a66905a5…`, `d9439af2…` …), while the semantics/metrics stayed stable. Rows 1–226 were
byte-identical and the event ledgers matched, with divergence appearing at the first scripted
release. Root cause: the run loop used wall-clock **`run_loop(1/60)`** against a **90 Hz** fixed sim,
so the number of fixed steps batched per frame varied run-to-run; the pin injects keys in `PreUpdate`
(render rate), so the batch boundary perturbed the physics from the first event onward. The arm was
never affected (its drive is fixed-time and its CSV is byte-identical) — which is exactly the
control that isolated the cause. **Fix:** `TimeUpdateStrategy::ManualDuration(1/FIXED_HZ)` in both
harnesses — one fixed step per frame, no wall-clock batching. After the fix the pin hash is stable
across 5 runs (`efdc55efddd35fb7`); the arm stays `c213e1989f0ab16b` and `PASS`.

**Conclusion:** the XPBD physics *solver* is deterministic; the earlier flakiness was the
harness's wall-clock frame clock, not the engine. Deterministic replay is now a first-class,
assertable property (`rows_hash`).
