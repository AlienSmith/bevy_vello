# Fix: Anchor the Resolver Floor to the Live Stance Height

Status: implementation plan · Area: lower-body hip-pin ↔ Raibert resolver floor reference

## 1. The root cause (32-character version)

The resolver plans all ground reference geometry (stand height, swing plant, LIPM,
reach envelope) against `LowerBodyConfig.hip_height = 90`, but the real character's
live stance (captured by the leg bridge seed) is **106 px**. The physics floor is
spawned at the *live* stance (`hip − stance_h`, see `spawn_collision_floor`), so the
resolver's virtual floor sits **16 px above the real collision floor**. Every
grounded loop is therefore anchored 16 px off the plane the collision response
enforces.

## 2. Why simply touching `floor_y` is NOT the fix

The resolver is **translation-invariant** (test `test_design_nonzero_floor_offset`):
all heights are measured above `floor_y`, and it is re-anchored to the live hip via
`world_delta = pose.hip − seed_hip`. So moving `floor_y` does **not** reconcile the
90-vs-106 standing-height mismatch. The mismatch is between `hip_height` (the
stand/reach/heights knob) and the realized stance, i.e. the leg is physically
**106 px** long but the resolver models it as standing **90 px**.

The 16 px cancels only when the resolver's standing height equals the realized
stance, because that value drives:
- `regulate_height` stand target (`floor + stand`),
- the swing plant (`swing_to.y = config.floor_y`),
- the reach envelope `compute_x_max(thigh, calf, hip_height, margin)` and therefore
  `X_max`, `a_max`, `v_abs` (velocity shaping) in `GaitLimits::from_config`,
- the LIPM `z` in the capture point,
- jump apex / flight exit heights.

## 3. Reused design: per-pin standing height override

Keep the shared `LowerBodyConfig` untouched. Add a per-pin
`standing_height: f32` to `HipPin`, captured from the **live leg bridge seed** once
the rig is ready. Feed it to the resolver by passing a per-tick config clone with
`hip_height` overridden.

### Why `X_max` shrinking is correct, not a regression

At seed the reach is `√(110² − 90²)·0.85 ≈ 53.8`; at the real 106 px stance it is
`√(110² − 106²)·0.85 ≈ 26.5`. The wider reach was the resolver believing the leg
could spread farther than it physically can at the taller stance — the leading
hypothesis behind the observed "feet splayed ~197 px apart". Making the resolver
see the true stance shortens the reach to what the leg can actually do.

## 4. Edit sites

### 4.1 `HipPin` struct + `new` — [`hip_pin.rs`](../examples/game_lib/src/lower_body/hip_pin.rs:515)
- Add field `standing_height: f32`.
- `new(config)`: initialize `standing_height = config.hip_height` (fallback before
  the rig seeds).

### 4.2 Leg-bridge seed capture — [`hip_pin.rs`](../examples/game_lib/src/lower_body/hip_pin.rs:1229)
Where `pin.leg_seed` is first set, also set
`pin.standing_height = stance_height live` (`seed.live[0].hip.y − seed.live[0].foot.y`),
guarded so it is only latched once (in the `get_or_insert` seed block).

### 4.3 Per-tick config override — [`hip_pin.rs`](../examples/game_lib/src/lower_body/hip_pin.rs:987)
At the `step_lower_body` call, build
`let mut cfg = params.config.clone(); cfg.hip_height = pin.standing_height;`
and pass `&cfg` (also used by `attitude_lean_pd`/leg writes that read hip height).

### 4.4 `measure_body` floor — [`hip_pin.rs`](../examples/game_lib/src/lower_body/hip_pin.rs:804)
Replace the hardcoded `floor_y = 0.0` with the synced floor line. In the Vello
y-down world the floor is `anchor_p3.y + standing_height` (the live foot line). This
fixes the CoM `z_com/measured_apex` channel that currently `.max(0.0)`-clamps to
zero.

### 4.5 `reset_hip_pin` — [`hip_pin.rs`](../examples/game_lib/src/lower_body/hip_pin.rs:1424)
Relatch `standing_height` on reset (the seed is re-armed).

## 5. What I will NOT touch (keep behaviour otherwise identical)

- The shared `LowerBodyConfig.hip_height` default (90) — kept for the pure
  kinematic sandbox and the unit tests that assert against it.
- `tick_loop3_angular`, the waist posture hold, and the leg-gait phase logic.
- `GaitAuthority` and the §6 contact layer (rejected feature, left off).

## 6. Verification

1. `cargo test -p game_lib` — resolver unit tests still pass (they use the default
   config, untouched).
2. Headless walk (`headless_arm_pd`): the `floor...` info line should now report a
   live `stance_h ≈ 106` matching `resolver_hip_height ≈ 106` (16 px gap gone).
3. `analyze_foot_slide.py` / `analyze_swing.py`: watch stance drift drop and swing
   advance hold with a *default* (not `1e-9`) `LEG_COMPLIANCE` — the test of whether
   correcting the floor removes the need for extreme stiffness.
4. Confirm `measure_body` `z_com` is no longer pinned at 0.

## 7. Risk

- `X_max` shrinking changes the velocity envelope; a clean walk may need the
  `hip_height`-coupled levers re-tuned, but that is the *point* — tuning against the
  real stance instead of a phantom one.
- This is the first time `step_lower_body` gets a per-character config; extras must
  flow through the same `cfg` so nothing reads the shared default mid-tick.