# Plan (draft): Hip-Pin Integration — Retire the Spine Drive, Drive P1/P2/P3 with Stiff External Position Constraints

Status: **decision recorded, not yet implemented.** This supersedes the *ownership* half of
[`lower_body_pd_bridge.md`](bevy_vello/plans/lower_body_pd_bridge.md) (see §11).

## 1. The pivot (user decision, 2026-10-12)

> "Pretend the spine controller doesn't exist; we will start new and won't use it. The only
> thing we would use for P1/P2/P3 control is the external position constraint, since we don't
> plan on actively bending it."

Consequences accepted:

1. The **spine drive is retired** — no active two-heading bend, no rest-pose chase, no
   `SpineIndicator` as the command surface.
2. **P1/P2/P3 are placed by rigid external position constraints** ("pins") toward a pose
   derived from the Raibert sandbox output.
3. **No angular/rest drive** on `P1_P2_P3` for attitude — the joint keeps its rest angle and is
   never re-rested (there is nothing to bend *to*).
4. Legs are initially **decoration**; the hip is the authority.

## 2. What we keep vs. drop

| Thing | Verdict |
|---|---|
| [`tick_spine_drive()`](bevy_vello/examples/game_lib/src/character/systems.rs:17) + [`calculate_spine_drive()`](bevy_vello/examples/game_lib/src/character/systems.rs:392) | **Drop** (no emitter; replaced by the pin emitter, §5) |
| [`SpineIndicator`](bevy_vello/examples/game_lib/src/character/mod.rs:249) as command/pose surface | **Drop** as an input surface; keep only if a debug overlay is wanted |
| [`SpineController`](bevy_vello/examples/game_lib/src/character/mod.rs:221) | **Keep the handle bundle** (`particles`/`angulars` entity IDs). It is also the AI + `tick_drag` + tuning-app read path; it never was the *drive*. Consider renaming later; do NOT delete it in step 1 |
| [`ExternalPositionConstraintConfig`](bevy_vello/src/integrations/physics/mod.rs:179) + [`queue_character_one_time_external_position_constraint()`](bevy_vello/src/integrations/physics/mod.rs:85) | **Keep — this is the whole actuator** |
| [`LimbConfig::rest_chase_alpha`](bevy_vello/examples/game_lib/src/character/mod.rs:181) work | Keep for the arms; the spine half becomes dead once the drive is gone |

## 3. Why external-position-only is the correct first step

The constraint is a **position projection**, not a PD law:

```
ExternalPositionConstraintConfig { target, compliance, damping }
```

With `compliance ~ 1e-7` the solve is effectively rigid — the particle is snapped onto the
target. This is exactly the "stiff external position constraint on the hip" the user asked for.
Notes:

- **The target is a full PD, not a bare position** (user correction, 2026-10-12):

  ```
  a = Kp·(x_target − x_actual) + Kd·(v_target − v_actual)
  ```

  The sandbox **supplies `v_target`** — [`ResolverState::hip_vel`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:205)
  is integrated by [`step_lower_body`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:853)
  (see the horizontal write-back at [`resolver.rs:946`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:946)),
  so we have both a hip position *and* a hip velocity target. This is the **asymmetry with the
  limb controller**: the arm/limb IK emits a *position* target only, so its D-term is smuggled in
  as the `kd·ω` rest-angle trail ([`LimbConfig::kd`](bevy_vello/examples/game_lib/src/character/mod.rs:167));
  there is no `v_target` there. The hip has one, so the D-term is the textbook `Kd(v_target − v_actual)`.

  Note the stock [`ExternalPositionConstraintConfig`](bevy_vello/src/integrations/physics/mod.rs:179)
  is position-only (no velocity slot). The PD is therefore realized in **position-target form**
  (implemented 2026-10-13, [`pd_target()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:121)):
  integrate the command acceleration over one step and add it to the pose target —
  `target = pose + (Kp·(pose − x) + Kd·(v_hip − v))·dt²`, so the constraint's force is the PD law.
  This is the same target-space D convention as the arm/spine (see
  [`spine_pd_three_linear.md`](bevy_vello/plans/spine_pd_three_linear.md:94) §3.2) and needs **no**
  impulse bridge or new config field. `softened_compliance()` softens `compliance` with particle
  speed (the limb's ω-schedule ported to velocity space).
  - **Step 1 (stiff):** `kp = kd = 0`, `soft_scale = 1` — the shape-only pin above; the rigid
    projection derives velocity for free.
  - **Soft phase (now reachable):** raise `kp`/`kd` and `soft_scale`; the PD rides on top of the
    (now speed-softened) pin. With `P1/P2/P3` lightened to `inv_mass = 1.0`, the particles respond
    to the PD (previously `1e-5` made them near-immovable anchors).
- **Frame shape is preserved by construction.** Because we pin *three* particles toward *one
  consistent rigid pose*, the `[P1,P2,P3]` distances hold without any distance/angular work — as
  long as the three targets are generated from one rigid transform (§4). No `bend_compliance`
  re-rest, no fighting.
- **`rest_chase_alpha` is not the knob here.** For a stiff pin the knob is `compliance`
  (→ ~0) and `damping`; "stiff now, soft later" is `compliance ↑`, and only *then* does a
  velocity/PD bridge earn its keep.

## 4. The rigid-frame derivation (P3 → P1, P2)

We already have the rigid rest shape: [`SpineIndicator::local_points`](bevy_vello/examples/game_lib/src/character/mod.rs:266)
is the **P2-centred rest offsets** of P1, P2, P3 (built in
[`spawn_spine_indicator()`](bevy_vello/examples/collision_detection/src/spine_indicator.rs:202)).
Reuse it as the template.

Given a target hip position `P3*` and a target attitude `θ` (rigid rotation of the whole frame):

```
R      = rot(θ)
P2*    = P3* + R · (local_P2 - local_P3)
P1*    = P3* + R · (local_P1 - local_P3)
```

i.e. translate P3 onto `P3*` and rotate the frame by `θ` about P3. Pin P1*, P2*, P3*.

The **free parameter is `θ`** — the sandbox emits a hip position + height, not a torso attitude.
See §10 open question 1.

## 5. The emitter (new system, replaces the drive's emission)

New system, e.g. `drive_hip_pin`, scheduled in the same slot as the old drive
([`tick_spine_drive.before(update_character_movement)`](bevy_vello/examples/game_lib/src/character/plugin.rs:27)):

1. Read the character's sandbox state (see §6).
2. Advance the pure resolver one fixed step → `LowerBodyPose { hip, ... }` (and `hip_pos/hip_vel/
   hip_height` in [`ResolverState`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:205)).
3. Map `pose.hip` → `P3*`; pick attitude `θ`; compute P1*/P2*/P3* (§4).
4. Emit three [`CharacterExternalPositionConstraintEvent`](bevy_vello/examples/game_lib/src/character/systems.rs:473)
   via the existing queue call, `compliance` from config (default ~1e-7).
5. **Emit nothing angular** — leave `P1_P2_P3` at rest.

This is a drop-in shape match: the old drive already emitted exactly these three events
([`systems.rs:472-482`](bevy_vello/examples/game_lib/src/character/systems.rs:472)); we are only
changing how the *targets* are produced (sandbox instead of input + rest-chase).

## 6. Cadence & input (the real integration risk)

The sandbox is currently **standalone**: only
[`raibert_sandbox_main.rs`](bevy_vello/examples/collision_detection/src/raibert_sandbox_main.rs:17)
adds [`LowerBodySandboxPlugin`](bevy_vello/examples/game_lib/src/lower_body/plugin.rs:14), and
[`drive_lower_body()`](bevy_vello/examples/game_lib/src/lower_body/systems.rs:69) reads the
keyboard directly in `Update` using `time.delta_secs()`. To pin at the physics cadence we must:

- Step the resolver in the **fixed** schedule with the **fixed** `dt` (not `delta_secs()`), so
  the emitted pin targets advance one physics step at a time.
- Feed the resolver from the **game input map** (`ActionState<PlayerAction>`), not the sandbox's
  own `ButtonInput<KeyCode>` reads.
- Keep the sandbox state **per character** (`LowerBodyController` already holds
  `ResolverState`/`Pose`, so this is an ownership move, not new state).

## 7. What must not fight the pin

1. **Gravity double-count.** The resolver already integrates gravity into `hip_vel`; XPBD also
   applies gravity to the pinned particles. With a stiff pin the pin wins, so step 1 is benign —
   but the particles' *stored velocity* will accumulate the free-fall term. Decide the rule now
   (recommended: the sandbox owns vertical dynamics; subtract/skip gravity on the three pinned
   particles) so the soft phase does not inherit a hidden double-count.
2. **Leg collision — there is NO per-collider flag (verified).** The only gate is the
   `collision_group` check at [`collision/mod.rs:69`](bevy_vello/src/collision/mod.rs:69)
   (pair collides iff at least one is soft **and** `group==0 || groups differ`). The group is
   **character-wide** ([`CharacterRoot::collision_group`](bevy_vello/examples/game_lib/src/character_factory/mod.rs:15)),
   and the floor is spawned with group `0`
   ([`make_static_collision_shape`](bevy_vello/examples/collision_detection/src/character_tuning_main.rs:487)),
   so you cannot exclude a leg by changing its group. "Dropping the bottom rect" is also **not
   clean**: the leg colliders are load-bearing in the joint graph
   ([`v10.character.json`](bevy_vello/examples/collision_detection/assets/character/v10.character.json:838)
   → `P40↔LUL`, `P40↔LLL`, `P41↔RUL`, `P41↔RLL`, `P3↔RUL`, `P3↔LUL`, `PLLL↔LLL`, `PRLL↔RLL`),
   and [`observers.rs:349`](bevy_vello/examples/game_lib/src/character_factory/observers.rs:349)
   **warns-and-skips** a joint whose part is missing — so a bare deletion detaches the legs and
   silently drops those joints. Options that actually work:
   - **(i) Zero-mass the legs.** Set `total_inv_mass` for `LUL/LLL/RUL/RLL` to `0` in the JSON.
     The parts still exist (joints resolve, legs render), but a zero-inverse-mass particle can
     never be moved by collision and injects no energy. Cleanest "decoration" semantics.
   - **(ii) Post-claim filter.** A one-line guard in the [`collision/mod.rs:66`](bevy_vello/src/collision/mod.rs:66)
     pair loop skipping pairs where both groups match a "no-collide" set. More code, more global.
   - **(iii) Defer.** Leave floor contact on for step 1; the hip pin's own height loop
     ([`tick_vertical`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:402)) holds the
     body up, so the leg contact may not even fire. Accept the jitter risk for the first look.
   - **Recommendation:** (i) — it is the "flag" the user was thinking of, expressed as mass.
3. **Rest angular joint.** Leave `P1_P2_P3` at its initial rest and never re-rest it (§3).

## 8. Testable gate ("check it works")

Headless, deterministic:

- Drive the resolver through a canned walk episode; capture `pose.hip` per step.
- Feed through the §4 mapping; step the constraint world.
- Assert: `|P3_live - P3*| < tol` after settling; the three bone lengths stay constant within
  eps (rigid shape held); P1/P2/P3 finite at extreme `dt` and extreme speeds (reuse the resolver
  fuzz bounds in [`resolver.rs`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:3909)).

**Note (dt envelope — verified during implementation, *not* changed by us).** The gate's "extreme
`dt`" clause is bounded by the resolver's own integrator, not by the pin. [`regulate_height()`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:391)
(the grounded height spring) is symplectic Euler with `VERTICAL_K_S = 400`, `VERTICAL_K_D = 40`
→ `ω = √400 = 20 rad/s`; it is only *conditionally* stable, and the shipped test envelope is
**30–240 Hz** ([`test_production_frame_rates_stay_stable_and_bounded`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:3722)).
At `dt = 1/15 s` (≈15 Hz) — *below* that minimum — a full episode **including a jump** lets the
grounded spring run away (observed `hip_pos.y ≈ 6.7e7`, losing f32 precision). Production ticks at
90 Hz, inside the band. The gate unit test therefore exercises the rigid-frame derivation over a
*bounded* walking episode (no jump); it does **not** assert resolver stability below its design rate.
Making [`step_lower_body()`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:853) dt-robust at
coarse rates (substepping / clamped `dt`) is out of scope here and would be a separate resolver
change. **This is a pre-existing resolver limitation; the hip-pin work does not touch the resolver.**

## 9. Blast radius / retire list (step 1 = gate, not delete)

`SpineIndicator`/`SpineController` are referenced well beyond the drive:

- [`spine_control_input()`](bevy_vello/examples/collision_detection/src/spine_indicator.rs:383) + [`apply_spine_config()`](bevy_vello/examples/collision_detection/src/spine_indicator.rs:435) (tuning app)
- [`spawn_spine_indicator()`](bevy_vello/examples/collision_detection/src/spine_indicator.rs:202) (needs `local_points` — keep the offset extraction, drop the visual if wanted)
- [`input_management/systems.rs`](bevy_vello/examples/game_lib/src/input_management/systems.rs:38), [`ai/mod.rs`](bevy_vello/examples/game_lib/src/ai/mod.rs:489), [`headless_ctrl.rs`](bevy_vello/examples/collision_detection/examples/headless_ctrl.rs:254), [`main.rs`](bevy_vello/examples/collision_detection/src/main.rs:91) `tick_drag`

**Recommended step-1 move:** *gate the emission* (a config flag / remove `tick_spine_drive` from
the schedule for the driven character) and leave the components in place; delete in a later pass
once the pin is proven. This keeps the diff small and the tuning app compiling.

## 10. Open questions

1. **Attitude `θ` for the rigid frame** — the sandbox gives a hip point, not a torso angle.
   - (a) *Upright*: `θ = 0` (world-vertical frame). Simplest; matches "spine rigid" literally.
   - (b) *Loop-3 attitude PD* drives `θ` (see [`raibert_loop3_attitude.md`](bevy_vello/plans/raibert_loop3_attitude.md:1)).
   - **Recommendation:** (a) for step 1; wire loop-3 as the second step.
2. **Leg collision in step 1** — no per-collider flag exists; options are (i) zero-mass the leg
   colliders in the JSON, (ii) a post-claim pair filter, or (iii) defer and accept jitter.
   (Rec: (i).)
3. **Spine drive removal** — gate now, delete later? (Rec: yes.)
4. **Gravity ownership** on the three pinned particles (§7.1). **Settled (2026-10-13, todo 68):**
   the sandbox owns vertical dynamics — [`drive_hip_pin`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:284)
   feeds the world `g·dt²` forward into the pin target so the constraint cancels the XPBD gravity
   term exactly (inert at `g = 0`). Verified in the `GRAVITY=on` gate: the light `P1/P2/P3`
   (`inv_mass = 1.0`) track within `0.34 px` of the gravity-free case with zero `-g` sag
   (see [`hip_pin_implementation_record.md`](bevy_vello/plans/hip_pin_implementation_record.md:123)).

## 11. Relation to other docs

- **Supersedes** the ownership fork in
  [`lower_body_pd_bridge.md`](bevy_vello/plans/lower_body_pd_bridge.md:58) (Replace / Complement /
  Consolidate): the answer is *Replace the spine drive's emitter with a sandbox-driven pin
  emitter; Complement nothing in step 1*.
- The **PD / velocity-target** portion of that doc is *not* discarded — it becomes the **soft
  phase** (§3), reached by raising `compliance` once the stiff pin is proven.
- [`raibert_loop3_attitude.md`](bevy_vello/plans/raibert_loop3_attitude.md:197) §9–§10 define the
  P3↔P2 topology this doc reuses, and feed open question 1(b).
