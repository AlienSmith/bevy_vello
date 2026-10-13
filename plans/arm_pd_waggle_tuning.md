# Plan: Headless Arm-PD Waggle-Tuning Harness

## Goal

A headless tool that drives the V10 **right arm** through a ~180° swing and
reports whether the limb PD controller **settles** or **waggles**, so we can
find good `Kp`/`Kd` values. A wrong pair is obvious — either it takes forever
to settle (under-damped) or it rings forever / diverges (over-driven).

## The rig (verified)

- V10 right arm particles: `[P2, P1, P13, PRLA]`, joints `[P2_P1_P13`
  shoulder, `P1_P13_PRLA` elbow] (`v10.character.json` `controllers.right_arm`).
- IK root `p12 = P1`; the user set **P1 `inv_mass = 1e-5`** (near-static anchor),
  so only the arm [P13, PRLA] actually moves under the PD.
- IK is solved in the character's `BlendedCoreFrame` **local** space; arm lengths
  L1 = P2→P1, L2 = P1→P13 (self-stabilizing, so the swing is ~rigid).
- Hips OFF (no `HipPin`) and `SpineDriveEnabled(false)` => the spine frame is
  passive and never fights the arm.

## The PD (verified, in `calculate_limb_ik`)

PD is implemented in **target space** — the solver is untouched:

| Knob | Field | Default | Role |
|---|---|---|---|
| **Kp** | `LimbConfig::rest_chase_alpha` | 0.5 | fraction of IK angle gap closed per tick, pre-clamp (shared with spine) |
| **D** | `LimbConfig::kd` | 0.065 s | rest angle trails by `kd·ω` (units [s]·[rad/s]=[rad]); **not** ×dt |
| stiff | `angular_compliance` | 5e-8 @ ref 30px | lower = stiffer spring (secondary) |

Safety clamps that can mask a bad Kp/Kd: `kd_max_offset` (0.12 rad),
`max_angle_rate` (2π rad/s), `FOREARM_ANGULAR_EPSILON` dead zone,
`convergence_threshold` (2px). The harness should report when a clamp binds.

## The command (user decision: **Aim**)

`update_character_movement` (FixedUpdate) reads `RightArmController.target`
(world) + `config.ik_mode`. The harness drives:

- **`IkMode::Aim { weapon_offset_y: 0.0 }`** — the user's choice. Repositions the
  wrist (mostly the elbow joint), and is the one that tends to **waggle**; it
  also makes a clean ~180° angular rotate of the wrist easy to command.
  (With `weapon_offset_y = 0.0` the "virtual target" degenerates to the true
  target, so `solve_aim_ik` reduces to a straight wrist reach and can flip
  between its full / elbow-only branches via `ELBOW_ONLY_THRESHOLD` — watch that
  threshold when interpreting a swing.)
- `IkMode::Reach` — whole-arm swing; kept as a secondary option only.

If `ik_mode == Disabled`, no events are emitted; when the wrist is within
`convergence_threshold` of the target, `calculate_limb_ik` early-exits.

## The swinging plan

1. **Swing**: set the **wrist target** to the "180° from the shoulder" point —
   `target = p12 + rotate(L2_rest_dir, 180°)`, giving a long, unambiguous angular
   command for the Aim solver. Fallback: offset the target far to the opposite
   side if the frame heading resists.
2. **Hold** the target to measure settle.
3. **Flip** the target back at a **slow** rate (~1–2 Hz over ~1 s) to sweep
   through DC, avoiding beating/aliasing a single step's response.
4. **Release** (hold the final target) to measure the residual ring.

## The tool: new `examples/headless_arm_pd.rs`

Modeled on `headless_pin.rs`:
- `MinimalPlugins`, `Time::<Fixed>::from_hz(90)`, zero gravity, no colliders,
  `SpineDriveEnabled(false)`, **no** `HipPin`.
- Full physics FixedUpdate chain; a `Probe` resource samples **every fixed
  tick** (90 Hz) — the same fixed-clock telemetry pattern as `headless_pin.rs`.
- Spawns V10; sets `RightArmController.config.ik_mode = Aim { weapon_offset_y: 0.0 }`;
  drives `.target` through the swing plan above; sweeps `rest_chase_alpha` (Kp)
  and `kd` live.
- Logs per tick: `t,phase,kp,kd,theta,omega_p13,waggle, ...` (p12/p13/pRLA
  positions + velocities + the wrist/forearm angle).
- **Env overrides** for automated grid runs: `KP`, `KD`, `TARGET_DIST`,
  `SWING_HZ`, `DUR`; plus a `SWEEP=1` mode that runs a (Kp,Kd) grid in one
  process and writes a summary table.
- Exit-time summary: `settle`, `ring_count`, `waggle_rms`, `ssh_max`, `verdict`.
- Output to `/home/ubuntu/bender/tmp/headless/arm_pd_log.csv` (bender-local,
  matching `headless_pin.rs`).

## The waggle metric (the deliverable)

Compute the forearm/wrist angle `theta(t)` (unwrapped). After the swing:
- **settle** = first `t` (after motion ends) where `|theta − theta_hold| < 2°`
  **forever** (the `forever` clause is the key — it rejects a swing that
  crosses the line then rings back).
- **waggle_rms** = RMS of `theta − EMA(theta)` (high-pass); **ring_count** =
  number of sign changes of the same; **ssh_max** = max steady-state error.

Verdict:
- `PASS` — `settle < τ_dead` and `waggle_rms < ~1°`
- `SLOW` — monotone but `settle > τ_slow` (under-damped / too soft)
- `WAGGLE` — `waggle_rms > ~1°` or ring persists to the end
- `DIVERGED` — `|theta|` grows past ~90°

### Metric v2 — settle-window + reversal detector (landed 2026-10-13)

The v1 metric measured RMS/rings over the **whole** release window. That window
opens while the flip is still coasting to rest, so a smooth monotonic catch-up
(e.g. cell12: `-93° → -114° → … → -246°` then a dead-flat `114.045°`) was
misread as a 50° wobble. v2 fixes this:

- **Settled tail**: the oscillation metrics use only the last `TAIL_S = 0.6 s`
  of the release window, about that slice's own mean. A one-way catch-up then
  cannot masquerade as oscillation.
- **Tail RMS** (`rms_deg`, about the tail mean) replaces the old whole-window
  RMS; threshold `TAIL_RMS_DEG = 1°`.
- **Angular-rate sign-reversals** (`reversals`): count sign changes of `omega`
  within the tail (deadband `|omega| > 0.05 rad/s`) — a direct, transient-immune
  wobble detector (the `detect_waggle_2d_zero_crossing` idea, applied to the 1-D
  joint rate). Threshold `WAGGLE_REVERSALS = 4`. A clean settle crosses ~0
  times; a limit cycle crosses every tick (~54 in a 0.6 s tail).
- **`catchup_deg`** (informational) = the furthest-flip excursion from the
  settled mean — large is fine, it only means a long flip.

Verdict (v2): `WAGGLE` iff `rms_deg > TAIL_RMS_DEG` **or**
`reversals >= WAGGLE_REVERSALS`.

## Verify first (prerequisite)

Run the harness once with the **default** config and confirm the arm actually
swings ~180° under `Aim` (i.e. that heavy-P1 + zero-g still lets the IK line
drive P13/PRLA). If it can't be placed 180° away, fall back to offsetting the
target both down-forward and up-back rather than assuming the initial heading.

## Touch list

- **New**: `bevy_vello/examples/collision_detection/examples/headless_arm_pd.rs`
- Read-only: `character/systems.rs` (`calculate_limb_ik`, `update_character_movement`),
  `character/mod.rs` (`LimbConfig`), possibly `character/ik.rs` (`solve_reach_ik`).
- No changes to game logic are required; the tool only sets controller fields.

## Findings (2026-10-13) — chosen Kp/Kd

Rig: `P2.inv_mass` raised `0.1 → 1e-5` (shoulder base now as stiff as the `P1`
anchor), zero-g, no colliders, `IkMode::Aim`, `DUR=3.0 HOLD=0.3` per cell;
25-cell sweep `Kp ∈ {0.2,0.35,0.5,0.65,0.8} × Kd ∈ {0,0.03,0.065,0.1,0.14}`.

The instability is driven by **excess Kd** (the `kd·omega` velocity feedback
over-corrects once the flip is fast), not by Kp alone. Clean stability boundary:

| Kp | stable Kd | breaks at |
|---|---|---|
| 0.20 | 0.00–0.14 (all PASS) | — |
| 0.35 | 0.00–0.14 (all PASS) | — |
| **0.50** | **0.03–0.065 PASS** | Kd ≥ 0.10 (rev 54, limit cycle) |
| 0.65 | 0.03 PASS | Kd ≥ 0.065 |
| 0.80 | 0.00–0.065 PASS | Kd ≥ 0.10 |

- **Chosen: `Kp = 0.5`, `Kd = 0.065`** (the current `LimbConfig` default) — the
  cleanest stable point: `rms = 0.17°`, `reversals = 0`. Confirms the user's
  "0.5 is not too far from ideal".
- `Kp = 0.5, Kd = 0` and `Kp = 0.65, Kd = 0` also show a few reversals
  (borderline), so 0.065 has margin.
- Because zero-g + a passive base is the easiest case, stay **inside** the
  stable cell (`Kp ≤ 0.5`, `Kd ≤ 0.065`) as gravity/compliance are added later.
- Evidence: `tmp/headless/arm_pd_sweep2.summary.csv` (v2 metric), per-cell CSVs
  `arm_pd_cellNN_kp…_kd….csv`.

Reproduce:
`DUR=3.0 HOLD=0.3 SWEEP=1 cargo run -p collision_detection --example headless_arm_pd`

## Resolved / context

- **Mode = `Aim`** (user decision). The arm is a stand-in: the real target is the
  **legs**, driven by the same limb-PD, but there is no leg controller yet — so
  this harness is where the Kp/Kd are found first, then reused for legs.
- **Kp/Kd found here should be reported** (values + the settle/waggle evidence),
  since they are intended to transfer to the future leg controller.
