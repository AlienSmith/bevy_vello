# Hip↔stance-foot distance strut — interface + usage plan (2026-11-02)

Two phases. **Phase 1** adds per-frame dynamic update of a **distance**
constraint's rest length + compliance, mirroring the **angular** constraint's
existing update path exactly. It is interface-only: nothing about the walk
changes until Phase 2 uses it. **Phase 2** spawns the hip↔foot strut joint and
drives its rest length from the IK each tick, stance-gated.

Motivation (from `plans/vertical-stabilizer.md`): the shipped loop-2 PD holds
hip height by **command authority** — it reacts a tick late and its gain sits
near the gait resonance (~11.3 rad/s at Kp=144), so the body still wobbles/jumps.
A hip↔stance-foot distance whose rest length equals the **IK leg reach** adds
**physical authority**: with the foot friction-pinned, "hip stays at leg-reach
from the foot" geometrically forbids the hip rising off the floor — the jump,
stopped in the solver as it happens.

## 0. Key facts (traced)

- The leg's hip→foot distance is **never set directly**: the leg is angular-only
  (joints `[P2_P3_knee, P3_knee_foot]`), thigh/calf lengths held by softbody
  distance constraints, so hip↔foot is the emergent third side.
- `DistanceConstraint { particle1, particle2, rest_length, compliance }`
  (`constraints.rs:12`). Its `solve` reads `self.rest_length` **live** every
  iteration (`constraints.rs:24`) → a `set_config` update takes effect with no
  rebuild, same as angular.
- The **angular** dynamic-update path is the template; **distance has none of it**:

| layer | angular (exists) | distance (to add) |
|---|---|---|
| struct | `AngularConstraintConfig{rest_cos,rest_sin,compliance}` + `get_config`/`set_config` | `DistanceConstraintConfig{rest_length,compliance}` + `get_config`/`set_config` |
| arena | `set/get_angular_config(Index)` | `set/get_distance_config(Index)` |
| group proxy | `set/get_connect_angular_config(&T)` | `set/get_connect_distance_config(&T)` |
| world | `ConstraintWorld::set_connect_angular_config(g,c,cfg)` | `…::set_connect_distance_config` |
| bevy wrapper | `VelloConstraintWorld::queue_character_angular_constraints` | `queue_character_distance_constraints` |
| event | `CharacterAngularConstraintEvent{character,joint,config}` | `CharacterDistanceConstraintEvent{…}` |
| register | `.add_event::<CharacterAngularConstraintEvent>()` | `.add_event::<CharacterDistanceConstraintEvent>()` |
| drain | `EventReader<…>` in the pivot/force system (`systems.rs:596`) | same system, extra reader |

The proxy key `T` (joint `Entity`) → `ConnectionConstraintIndex::Distance(Index)`
already exists (`collision_response.rs:368`), and `set_config` on angular matches
`ConnectionConstraintIndex::Angular(_)` then ignores others — the distance twin
matches `Distance(_)` only. This is why the mirror is clean: the storage
(`self.distance: Arena<DistanceConstraint>`, `index_connection_constraint`) is
already there; only the get/set + plumbing is absent.

## 1. Phase 1 — distance dynamic-update interface (both crates)

### study_vello/integrations/vello_physics/src/constraints.rs
- Add `pub struct DistanceConstraintConfig { pub rest_length: f32, pub compliance: f32 }`,
  `#[derive(Clone,Copy,Debug,Serialize,Deserialize,JsonSchema)]` (mirror the
  `AngularConstraintConfig` derive at `:456`).
- On `impl DistanceConstraint`: `pub fn get_config(&self) -> DistanceConstraintConfig`
  and `pub fn set_config(&mut self, &DistanceConstraintConfig)` (write
  `rest_length`, `compliance`). No walls/velocity — distance is scalar.

### …/src/soft_body_connection.rs
- `pub fn set_distance_config(&mut self, index: Index, cfg: &DistanceConstraintConfig) -> bool`
  (`self.distance.get_mut(index)`), mirroring `set_angular_config` (`:559`).
- `pub fn get_distance_config(&self, index: Index) -> Option<DistanceConstraintConfig>`.

### …/src/collision_response.rs
- `SoftBodyConnectionGroup::set_connect_distance_config(&T, &cfg) -> bool` /
  `get_connect_distance_config(&T) -> Option<cfg>` — match
  `ConnectionConstraintIndex::Distance(ix)` only, `_ => false/None`
  (mirror `:424`/`:438`).
- `ConstraintWorld::set_connect_distance_config(&mut self, g_index, c_index, cfg)`
  (mirror the world-level `set_connect_angular_config` at `:1070`).

### …/src/lib.rs
- `pub use` `DistanceConstraintConfig` next to `AngularConstraintConfig`.

### bevy_vello/src/integrations/physics/mod.rs
- `#[derive(Event)] pub struct CharacterDistanceConstraintEvent { character_entity, joint_entity, config }`
  (mirror `CharacterAngularConstraintEvent` `:169`).
- `VelloConstraintWorld::queue_character_distance_constraints(&mut self, &event)`
  → `get_group_mut(..).set_connect_distance_config(&joint, &config)`
  (mirror `queue_character_angular_constraints` `:79`).
- Re-export `DistanceConstraintConfig`.

### bevy_vello/src/integrations/physics/plugin.rs
- `.add_event::<CharacterDistanceConstraintEvent>()` (`:41` neighborhood).

### bevy_vello/src/integrations/physics/systems.rs
- In the existing character-events drain system (`:596`), add
  `mut dist_events: EventReader<CharacterDistanceConstraintEvent>` and
  forward each to `queue_character_distance_constraints`. No new system ordering
  needed — it already runs before the step.

**Accept (Phase 1):** both crates `cargo build`; `cargo test -p game_lib` still
172 green; a headless run with **no** `CharacterDistanceConstraintEvent` emitted
is byte-identical to the current baseline (the interface is dormant until Phase
2). Optionally a Rust unit test that inserts a distance config and reads it back
through the proxy key.

## 2. Phase 2 — spawn + drive the strut (game side)

Deferred until Phase 1 is green, listed so the interface is designed to fit:

- **Blueprint**: add two `Distance` joints (left/right), one per leg, rest
  = initial hip→foot, soft compliance (`1e-4`-ish scale, tunable). Their
  `Entity` keys are captured like the leg particle/joint handles in
  `character_factory/observers.rs` (the `Distance` branch already resolves here).
- **Drive** (`drive_hip_pin`, after `step_lower_body`): each tick, for a leg
  whose **foot has contact** (`foot_contact[..].touching`), emit a
  `CharacterDistanceConstraintEvent` with `rest_length = ‖pose.hip − pose.foot‖`
  from the just-solved IK (the same tick as the knee-angle target — no lag),
  `compliance = HIP_STRUT_COMPLIANCE`. For a leg **without** contact, either
  skip (constraint idles) or set `compliance` very soft (rest → free). Default
  OFF behind `HIP_STRUT=0`.
- **Stance gate is essential** (design `vertical-stabilizer.md` §3): a swing
  foot is mid-arc; tethering the hip to it fights the gait + DCM.

## 3. Validation (Phase 2) — MEASURED 2026-11-02 (below)

### 3a. Results (aligned command: SCRIPT=long_walk20_s90 TARGET_SPEED=120 FLOOR_HALF_LEN=3500; arms differ only by V_STABILIZE/V_STAB_KP/HIP_STRUT)

| arm | airborne 2-21s | hip band p-p | net_x | stall | stance drift |
|---|---|---|---|---|---|
| s1: PD-stiff(144)/strut-off | 46.8 % | 163 px | **+1316** | 4.1 s | 145 px |
| s2: PD-stiff/strut-ON | 55.5 % | 236 px | −155 | 7.0 s | 70 px |
| s3: PD-soft(25)/strut-ON | 61.1 % | 294 px | +471 | 7.0 s | 94 px |

Per-window hip band p-p (px, live P3):

| window | s1 off | s2 stiff/on | s3 soft/on |
|---|---|---|---|
| 2-3 s | 83 | 56 | 63 |
| 3-5 s | 109 | 94 | **16** |
| 5-8 s | 114 | 67 | 123 |
| 8-11 s | 162 | 95 | 286 |

**Findings**
- Strut-on HOLDS HEIGHT while cruise is healthy (s3 16 px band in 3-5 s — the
  only arm that ever meets the <±15 target) and DELAYS the stall (4.1→7.0 s);
  stance drift improves (145→70).
- But fiction-vs-reality y-gap stays ≤3 px in the good windows: the early
  step-2/3 wobble is COMMAND-SIDE — the resolver itself commands a −65 px sink
  over t≈1.3-1.9 (boot release→first steps) and reality faithfully follows.
- Late blow-up (s3 5-8 s onward) tracks the stall/brake sawtooth: v_filt
  collapses 172→47 px/s at t≈2-2.3 in s3 vs staying 130+ in s1; once the x-loop
  thrashes, ω₀(z_com) + reach x_max couple it back into y (design doc §"height→x
  couplings" in vertical-stabilizer.md, here inverted).
- Chord-drag cost: rest = ‖hip−foot‖ pulls along the hip→foot axis, so it
  reduces forward progress (s2 net −155). Candidates: (a) touchdown-LATCHED rest
  from the LIVE distance (true third side, no per-tick re-imposition of the
  commanded pose), (b) measured-chord sourcing (physics authority only).

## 3b. Original validation targets (kept for reference)

A/B on the aligned baseline (`tools/physics-qa/run_walk.sh`,
`long_walk20_s90`, TARGET_SPEED=120):

| config | airborne % | hip band p-p | net_x | stall onset | jump test |
|---|---|---|---|---|---|
| PD-stiff / strut-off (`vstab_ship`) | 48.2 | ~48 px | +1170 | 11.4 s | green |
| **strut-on / PD-soft (Kp=25)** | **<2 target** | **<±15 target** | **≥1170** | none | green |

If the strut lets `V_STAB_KP` drop off the resonant 144 **and** airborne drops
toward 0, it confirms physical authority replaced command authority. The five
`test_v_stab_*` unit tests stay green (Phase 1 does not touch the resolver).

## 3c. Follow-up sweep (2026-11-02, resolver @ 559a1b6 fiction-owned height)

| arm | air % | net_x | stall | bands 1.5-3 / 3-5 / 5-8 / 8-11 / 11-15 / 15-21 |
|---|---|---|---|---|
| stiff/off | 49.4 | +519 | 5.9 s | 71 38 67 59 28 45 |
| stiff/on | 54.1 | −375 | 6.9 s | 66 63 147 118 66 222 |
| **FLOOR_SNAP + strut** | 46.7 | **+502** | **14.1 s** | **26 43 24 17** 45 127 |
| FLOOR_SNAP alone (no strut) | 53.2 | +381 | 4.7 s | 102 83 55 77 78 190 |

**Dip mechanism (measured, deep dip t=1.54–1.67):** the stance CHORD collapses
106→84 px — the leg folds while `contact=0` flickers off mid-stance (thrust
authority revoked exactly when the loaded leg needs extension). Pendulum arc
(L−√(L²−d²)) explains only 0.3–4 px of a 5–24 px drop. Fiction tracks the
collapse 1:1 → command-side, as suspected.

**FLOOR_SNAP alone hurts** (bands up): snapping the plant without the strut
makes the leg fight itself. **FLOOR_SNAP + strut is the best vertical arm to
date** (bands ≤43 px through 11 s, stall 5.9→14.1 s, net_x restored positive):
the snap holds the plant, the spring holds the hip at the IK chord — physical
authority does what the PD could not. Late blow-up (45/127 after 11 s) tracks
the brake sawtooth (v_min −139, stall 14.1 s), i.e. the x-loop again.

## 4. Risks / guards

- **Over-constraining the knee**: three fixed sides freeze it — avoided because
  the rest length is re-sourced from IK every tick (dynamic, not constant). The
  knee-angle constraint and the strut must read the **same** solve.
- **Fight with friction/slide**: keep compliance soft; sweep before shipping.
- **Two writers to `hip_pos.y`** (HIP_Y_SYNC vs stabilizer) already noted in
  `vertical-stabilizer.md`; Phase 2 (physics strut) is orthogonal to that fiction
  bookkeeping, but the final config should pick one fiction owner once strut works.
- Phase 1 is additive/dormant: zero behavior change until an event is emitted.
