# Handoff: Solver-Baked Angular PD Damping (ζ) — Walking Foot Divergence Fix

**Status: PARTIAL — solver damping milestone DONE and committed; leg wiring + walk validation still TODO.**
**Date: 2026-10-14. Branches: `study_vello@gravity_walk`, `bevy_vello@world_scale`.**

---

## 1. The problem being fixed

Walking foot divergence in the headless Bevy/XPBD V10 character rig. Baseline defect:
the foot baseline diverges ~37px peak when walking (measured with
`tools/physics-qa/analyze_leg_divergence.py` against headless walk captures).

Root cause path (as understood): the leg controller's external D term
(`kd·ω` trailing velocity-offset applied on top of the desired rest angle)
interacted badly with the angular constraint solve — excess Kd drove
instability/overshoot, and the game-layer offset was fighting the solver.

## 2. The settled design (user decision)

**Bake the PD damping INTO the XPBD angular constraint solver** as an implicit
spring-damper. Remove `kd` from all settings. Expose a single damping knob:
**damping ratio `ζ` (zeta), default `1.0` = critical damping.**

- `Kp = 1 / compliance`
- generalized mass `m = 1 / w_sum`
- `Kd = 2·ζ·√(Kp·m)`
- `gamma = Kd · compliance / dt = 2·ζ·√(compliance / w_sum) / dt`
- solve: `delta_lambda = (-error − gamma·c_dot) / denom`, where
  `denom = w_sum·(1 + gamma) + alpha`, `alpha = compliance / dt²`
- `c_dot` uses the **current implicit velocities** (no θ̇ reference): `v = pos − previous_pos`.

**ζ semantics:** <1 underdamped (springy), =1 critical (fastest snap, no
overshoot), >1 overdamped (heavy). **ζ ≤ 0 disables damping** → reduces exactly
to the legacy undamped XPBD solve (strict superset, safe).

Because `gamma` now depends on `w_sum` (the generalized inverse-mass sum, which
varies with bone lengths/angles), `w_sum` MUST be computed before `gamma` in the
solve. `w_sum = w0·|dc/dp0|² + w1·|dc/dp1|² + w2·|dc/dp2|²`.

## 3. FINISHED (committed)

### 3a. `study_vello` — commit `79806ae`
`git -C study_vello log -1 --oneline`
> `79806ae Bake PD damping into XPBD angular solver via damping ratio zeta (stable, no blow-up)`

Files:
- [`integrations/vello_physics/src/constraints.rs`](../../study_vello/integrations/vello_physics/src/constraints.rs)
  - Module-level free fn `default_zeta()` → `1.0` (serde `default = "default_zeta"`;
    must be a module-level fn, NOT an associated fn — serde can't find it in scope otherwise).
  - `AngularConstraintConfig.zeta: f32` (replaces `kd`).
  - `impl AngularConstraint::damping_gamma(zeta, compliance, w_sum, dt)` static helper.
  - Solve rewrite: `c_dot` + `w_sum` computed first, then `gamma = Self::damping_gamma(...)`,
    then `denom = w_sum*(1+gamma) + alpha`.
  - `break_disabled` literal uses `zeta: 0.0`.
  - `get_config`/`set_config` use `zeta`.
- [`integrations/vello_physics/src/soft_body_connection.rs`](../../study_vello/integrations/vello_physics/src/soft_body_connection.rs)
  - `add_angular_constraint` literal `kd: 0.0` → `zeta: 0.0`.

### 3b. `bevy_vello` — commit `0f1056c`
`git -C bevy_vello log -1 --oneline`
> `0f1056c Expose solver damping ratio zeta on limb configs and drop external kd offset`

Files:
- [`examples/game_lib/src/character/mod.rs`](../examples/game_lib/src/character/mod.rs)
  - `LimbConfig`: removed `kd` and `kd_max_offset`; added `pub zeta: f32` with doc.
  - `Default` for `LimbConfig`: `zeta: 1.0`.
- [`examples/game_lib/src/character/leg_pd.rs`](../examples/game_lib/src/character/leg_pd.rs) (new)
  - Angular PD leg controller (no IK). **Position-only rest angles** (no velocity offset).
  - `LegPdTargets` has `#[derive(Clone)]`.
  - Computes current angles/rates (omega_spine/thigh/calf, theta_dot_hip/knee) for ω-scheduled compliance.
  - Target angles `theta_hip_t_cs` / `theta_knee_t_cs` with `a_t = targets.hip + (p2 - p3)`.
  - Emits hip/knee events each with `zeta: config.zeta`.
  - Dead zone via `FOREARM_ANGULAR_EPSILON`; `rest_chase_alpha` chase clamp via
    `rotate_toward` / `max_delta = max_angle_rate·dt`.
  - Tests: `standing_no_events`, `perfect_tracking_emits_no_events`,
    `perturbed_target_emits_both_joints_with_zeta`, `zeta_forwarded_unmodified`,
    `omega_schedule_softens_compliance`.
- [`examples/game_lib/src/character/systems.rs`](../examples/game_lib/src/character/systems.rs)
  - Removed the `kd_offset` closure + its sign-derivation comment.
  - Shoulder: `rest_shoulder_cs = desired_shoulder_cs`; config `zeta: config.zeta`.
  - Elbow: `rest_elbow_cs = desired_elbow_cs`; config `zeta: config.zeta`.
  - Spine config literal (in `calculate_spine_drive`): `zeta: 0.0`
    (spine bend is commanded by position constraints; no ζ damping).
  - NOTE: check whether `rotate_cs` is still used in systems.rs (it was only used
    for the kd offsets). `shoulder_omega`/`elbow_omega` still used for compliance
    scheduling — keep.
- [`examples/collision_detection/examples/headless_arm_pd.rs`](../examples/collision_detection/examples/headless_arm_pd.rs)
  - Harness: `apply_to_arm`/`apply_to_larm`/`apply_to_rleg`/`apply_to_lleg` now map
    `kd` param → `c.config.zeta = kd`; `apply_set` tweak `config.zeta = kd`.
  - The harness's old "kd sweep" axis now sweeps ζ.
- [`examples/collision_detection/src/character_tuning_main.rs`](../examples/collision_detection/src/character_tuning_main.rs)
  - Replaced kd/kd_max_offset sliders with a single ζ slider `0.0..=2.0`
    under heading "Solver ζ damping (implicit spring-damper)".

### 3c. Already present / unrelated (do NOT touch)
- `HipPinParams` (`lower_body/hip_pin.rs`) keeps its own `kp/kd/v_ref` — that is a
  separate rigid-frame PD, unrelated to angular-constraint damping.
- Prior committed work: resolver velocity fields on `LowerBodyPose`, `ResolverState.prev_positions`,
  `RightLegController`/`LeftLegController` hip/knee target+velocity fields, `leg_pd.rs`
  scaffolding, `drive_hip_pin` (already hooked in `headless_arm_pd.rs` HIPPIN mode).

## 4. Empirical validation (DONE, this machine)

Headless arm 180° wave via `headless_arm_pd` (built-in `right_arm` script:
Aim `[1,0]` → `[-1,0]`). `kp` → `rest_chase_alpha`, `kd` → `config.zeta`.

Build (both crates): `cd bevy_vello && cargo build -p collision_detection --example headless_arm_pd` → clean (only pre-existing warnings).

Run at ζ=1.0:
```
LIMB=right_arm KP=0.5 KD=1.0 OUT_CSV=tmp/headless/arm_wave_zeta1.csv ./target/debug/examples/headless_arm_pd
→ cell 0 (kp=0.500, kd=1.000): WAGGLE swing=158.8deg rms=12.47deg rev=0 settle=none inv=ok
```
No blow-up, no crash (`inv=ok`). But ζ=1.0 **over-damps** (never settles).

ζ sweep:
```
LIMB=right_arm SWEEP=1 GRID="0.5:0.0,0.5:0.2,0.5:0.5,0.5:1.0,0.5:1.5,0.5:2.0" OUT_CSV=tmp/headless/arm_wave_zeta_sweep.csv ./target/debug/examples/headless_arm_pd
→ kd=0.0 : PASS swing=212.7deg rms=0.41deg rev=2 settle=0.14s inv=ok
→ kd=0.2 : PASS swing=329.6deg rms=0.11deg rev=0 settle=0.00s inv=ok   <-- sweet spot
→ kd=0.5 : WAGGLE swing=158.0deg rms=1.88deg rev=0 settle=0.40s inv=ok
→ kd=1.0 : WAGGLE swing=113.1deg rms=5.85deg rev=0 settle=none inv=ok
→ kd=1.5 : WAGGLE swing=83.6deg rms=4.10deg rev=0 settle=none inv=ok
→ kd=2.0 : WAGGLE swing=66.8deg rms=3.22deg rev=0 settle=none inv=ok
```

Conclusions:
- **The damping is STABLE at every ζ (0→2)** — `inv=ok` everywhere, no NaN, no
  explosion. This is the FIRST angular-constraint damping that did not blow up the
  physics (it blew up twice before in earlier attempts).
- **ζ=1.0 over-damps** for the arm: swing shrinks monotonically with ζ
  (212°→66°) and settle is lost. The useful range is ζ ≲ 0.3; **ζ=0.2 is the
  sweet spot** (PASS, rms 0.11°, settle 0.00s).
- Over-damping mechanism: large `gamma` in `denom = w_sum·(1+gamma) + alpha`
  starves the per-iteration position correction → under-convergence, not instability.
- ⚠️ The default `LimbConfig.zeta = 1.0` is likely TOO HIGH for production.
  Recommendation: default to ~0.2 (or tune per-limb) before the walking test.

## 5. UNFINISHED (next session)

1. **`Hook drive_hip_pin` into the full resolver pose** — map the resolver's
   pose + velocities into the leg controllers (hip/knee/foot pos + vel). Partially
   scaffolded already in `headless_arm_pd.rs` HIPPIN mode; the `drive_hip_pin`
   bridge exists in `lower_body/hip_pin.rs`.
2. **Route `update_character_movement` leg branches through `drive_leg_pd`** —
   `systems.rs` currently routes legs through the old `drive_limb`/`calculate_limb_ik`;
   arms stay unchanged. This is the core wiring to actually apply `drive_leg_pd`.
3. **Update leg construction** in `character_factory/observers.rs` + harness
   `first_ready` so legs are assembled with the new controller/targets.
4. **Run resolver tests + new leg_pd tests**:
   `cd bevy_vello && cargo test -p game_lib` (and resolver tests in
   `lower_body/resolver.rs`).
5. **Decide final default ζ** — likely `~0.2`, not `1.0` (see §4).
6. **Run headless walk** and measure foot divergence vs the 37px baseline with
   `tools/physics-qa/analyze_leg_divergence.py`.
7. **Commit the fix.**

## 6. Repos / branches / commands

- Two separate git repos (no top-level repo at `/home/ubuntu/bender`):
  - `/home/ubuntu/bender/study_vello` — branch `gravity_walk` (physics engine).
  - `/home/ubuntu/bender/bevy_vello` — branch `world_scale` (game + harness + tests).
- Build the physics crate alone: `cd /home/ubuntu/bender/bevy_vello && cargo check -p vello_physics`
  (vello_physics is a path dep of the bevy_vello workspace; it is NOT a member of
  the study_vello workspace).
- Build + run harness: `cd /home/ubuntu/bender/bevy_vello && cargo build -p collision_detection --example headless_arm_pd`
  then `LIMB=right_arm KD=0.2 ./target/debug/examples/headless_arm_pd`.
- Telemetry CSVs land in `bevy_vello/tmp/headless/` (gitignored).
- Divergence analysis: `python3 tools/physics-qa/analyze_leg_divergence.py <csv>`.
