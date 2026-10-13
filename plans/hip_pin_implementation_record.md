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

## PD command + speed-softened compliance + light framing (2026-10-13)

The pin emitter is upgraded from "pose-only stiff pin" to the user's PD law
`a = Kp(x_target − x) + Kd(v_target − v)`. Because the external-position constraint is a
**position projector** (no velocity slot — see
[`hip_pin_integration.md`](bevy_vello/plans/hip_pin_integration.md:32) §3), the PD is realized
in **position-target form**: the command acceleration is integrated one step and added to the
pose target:

```text
target_i = pose_i + (Kp·(pose_i − pos_i) + Kd·(v_hip − vel_i))·dt²
```

so the constraint's force is `Kp(pose − x) + Kd(v_hip − v)` — the limb's target-space D
convention ([`spine_pd_three_linear.md`](bevy_vello/plans/spine_pd_three_linear.md:94) §3.2).
`v_hip` is [`ResolverState::hip_vel`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:237)
mapped to the Vello frame — the feed-forward target the limb does **not** have (plan §3).

What landed:

- [`pd_target()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:121) — pure PD command,
  unit-tested (`pd_target_is_pose_when_gains_are_zero`, `pd_target_leads_toward_pose_and_commanded_velocity`).
- [`softened_compliance()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:137) —
  `base·(1 + s·(soft_scale−1))`, `s = min(|v|/v_ref, 1)`: the limb's ω-scheduled compliance
  ([`scheduled_compliance`](bevy_vello/examples/game_lib/src/character/systems.rs:616)) ported to
  **velocity space** (unit-tested: monotone + clamped + no-op at `soft_scale = 1`).
- `HipPinParams` gains `kp`, `kd`, `v_ref`, `soft_scale` with **neutral defaults**
  (`kp = kd = 0`, `soft_scale = 1`) so behaviour is unchanged until tuned; `hip_pin_ui` gains
  the four sliders.
- **P1/P2/P3 lightened**: `v10.character.json` particle `inv_mass` `1e-5 → 1.0` (they were
  near-infinite-mass anchors; the pin now does the work, so they are normal-weight) — the user's
  "make P1 P2 P3 much lighter".
- The spine drive stays **disabled but retained** (`SpineDriveEnabled`) — the user's explicit
  "disable, don't remove the code".

Verification: `cargo test -p game_lib hip_pin` → **6 passed** (3 new); `cargo build -p collision_detection`
clean (pre-existing warnings only).

Not yet tuned: the actual `kp`/`kd`/`v_ref`/`soft_scale` values (todo 64).

## Gravity opened — the pin holds the light frame (2026-10-13, todo 67/68)

The user asked to open gravity and confirm the pin still holds the (now light) `P1/P2/P3`.
Done, and clean.

**Two latent bugs found and fixed while "opening" gravity:**

1. **The tuning app's gravity toggle was a no-op at its default.** `GravityToggle::default()`
   already returned `enabled: true`, but [`VelloCollisionResponsePlugin`](bevy_vello/src/integrations/physics/plugin.rs:32)
   boots the world at `Vec2::ZERO` and [`gravity_toggle_ui`](bevy_vello/examples/collision_detection/src/character_tuning_main.rs:758)
   only writes gravity on a *change*. So the box read "checked" while the character floated.
   Fixed with a `Startup` system [`apply_default_gravity`](bevy_vello/examples/collision_detection/src/character_tuning_main.rs:204)
   that pushes the default into the world, plus a live `g` magnitude slider (so the app can match
   the harness).
2. **The harness hardcoded zero gravity.** [`headless_pin.rs`](bevy_vello/examples/collision_detection/examples/headless_pin.rs:234)
   now reads `GRAVITY` (`off` | `on` | `x,y`; `on` = `GRAVITY_ON` = `(0, 980)` px/s², matching the app).

**Gravity ownership — the load-bearing decision (todo 68).** The XPBD step applies `g·dt²` to
`P1/P2/P3` before the constraint solves, and the resolver **also** integrates its own `g` into
`pose`. The pin now feeds the same `g·dt²` forward into the target
([`drive_hip_pin`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:284)) so the constraint
cancels the physics gravity exactly and the sandbox solely owns the vertical dynamics. This is
**inert at `g = 0`** (the constraint's compliance is `1e-7`; the extra `g·dt² ≈ 0.12 px` term is
absorbed to sub-`0.35 px`), so the zero-g gate is unchanged.

**Verification (`GRAVITY` gate, deterministic, FIXED_HZ = 90, 6 s):**

| mode | rows | nan_rows | max_track_err | inv | rig | max_rel_dev | hash |
|---|---|---|---|---|---|---|---|
| `off` | 538 | 0 | 0.277 px | ok | ok | 0.0009 | `9a5b8808…` |
| `on`  | 538 | 0 | 0.315 px | ok | ok | 0.0006 | `4554d161…` |

- **Deterministic**: two `GRAVITY=on` runs produce the identical hash (`4554d16175554816`).
- **No `-g` sag**: keyed per-row join of the two runs gives `max|P3y_on − P3y_off| = 0.34 px`
  and **zero rows above 0.5 px** — the pinned frame tracks as tightly as the gravity-free case.
- **No residual vertical bias**: mean `P3vy` in the idle window (`t∈[3.0,3.4]`) is `-0.02 px/s`
  with gravity on (`-0.18` off); the jump arc apex is `-202.15` → `-202.43 px` (0.3 px).

Gate is **PASS**: the pin is strong enough to hold `P1/P2/P3` under gravity. Still open: the
`kp`/`kd`/`v_ref`/`soft_scale` tuning sweep in the gravity-on regime (todo 64).

## Gravity is real -- soften the pin and the frame falls (2026-10-13)

The user was rightly suspicious of the sub-pixel `0.34 px` error: *"wow this error is very small I
am a bit concerned if the gravity works. can you like use a large compliance for the pin. we should
be able to see the body fall down under gravity."* A stiff pin (`1e-7`) draws the frame back to the
resolver pose every substep, so it would mask gravity **whether or not** the physics were actually
integrating `g`. The way to tell the two apart is to **loosen the pin** -- then any real `g` shows
up as sag.

To sweep without rebuilding, [`params_from_env`](bevy_vello/examples/collision_detection/examples/headless_pin.rs:197)
now reads `COMPLIANCE` / `DAMPING` (defaulting to [`HipPinParams`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:47)).

**Idle-window `P3y` (world, y-up; more negative = deeper), `t` in `[4.6, 5.0] s`, 6 s run:**

| `compliance` | `GRAVITY=on` `P3y` | `max_track_err` | `GRAVITY=off` control `P3y` |
|---|---|---|---|
| `1e-7` | -173.16 | 0.32 px | -- |
| `1e-4` | -174.45 | 7.1 px | -- |
| `1e-3` | -185.06 | 47 px | -- |
| `1e-2` | -247.05 | 255 px | -167.06 |
| `5e-2` | -719.62 | 947 px | -- |
| `1e-1` | -1664.58 | 1621 px | -176.23 |

- **Sag grows monotonically with compliance under gravity** (-173 to -1665 px), so `g` is genuinely
  acting on `P1/P2/P3` and the stiff pin is what hides it.
- **The gravity-off control at the *same* compliance holds** (`1e-2` off: -167 px; `1e-1` off:
  -176 px), so the sag is unambiguously the gravity term, not the softness alone.
- **Time series (`c=1e-1`)**: `on` sinks -173 to -1713 px around `t` about `4.0 s` then rebounds as
  the constraint pulls back once the transient settles; `off` stays flat around -173..-181 px.

**Conclusion:** gravity was never missing -- with a stiff pin the sag is sub-pixel and invisible; a
soft pin makes the pinned frame visibly fall. The Stage-1a gate stands (stiff pin) *because* the pin
is deliberately stiff. The `kp`/`kd`/`v_ref`/`soft_scale` tuning sweep (todo 64) must pick a
compliance loose enough to absorb impulses yet stiff enough to hold the frame -- that is the real
tradeoff, now quantifiable.

Reproduce: `GRAVITY=on COMPLIANCE=1e-1 cargo run --release -p collision_detection --example headless_pin`
(vs. `GRAVITY=off COMPLIANCE=1e-1 ...` for the control).
