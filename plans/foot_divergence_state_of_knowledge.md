# Foot Divergence — State of Knowledge (2026-10-16)

> Status: **ERROR PERSISTS.** This document is a snapshot of what we know and what
> we have ruled out. It is *not* a resolution.

> **Update 2026-10-17: RESOLVED.** The walk gate now reports **peak 3.5 px
> (hypot)** and **0.70 px mean**, tail `PASS` (rms 0.39 px). The three causes and
> the code changes are in §11. §1–§9 are kept as the historical snapshot; do not
> read them as current behaviour.

## 1. TL;DR

The walk rig shows a persistent foot divergence of **~38 px combined** (best
config so far), down from a **~63 px baseline**, but the error **cannot be
reduced below ~38 px by any solver/PD parameter we have tried** — damping (ζ),
compliance (σ), velocity projection, substep count, and a hip/knee-seed fix all
hit the same floor. The reason is structural, not tuning:

- The **hip is pinned perfectly** (target y == realized y, both 172.9).
- The **knee bends** (target oscillates 217–239; realized sits at ~225–227).
- The **swing foot NEVER lifts**: target lifts to y≈248–251, but the realized
  foot stays pinned at **y≈278–283 — it never leaves the floor**, and often
  sinks a few px *below* its own stance value.

In other words: with gravity off and no colliders, nothing physically holds the
foot, yet the angular constraints have **near-zero authority** over the free
(foot/knee) particles. The commanded lift is real (targets move) and the joints
*do* articulate, but the foot does not follow.

## 2. The rig (facts)

- Headless Bevy/XPBD V10 character, walk script
  [`walk_script.json`](walk_script.json) (wait 0.5 + move 10.0 + wait 1.0),
  FIXED_HZ=90. (Copy kept in `plans/` because `tmp/` is not committed.)
- Leg particles: `[P2(ref), P3(hip), knee(P41), foot(PRLL)]`, thigh=calf=55.5px
  (total leg 111px).
- Coordinate frame: resolver/leg-targets are bevy **y-up**; particle positions
  are vello **y-down** (`bevy_to_vello(p) = (p.x, -p.y)`). So *smaller* y =
  *higher up* in the telemetry.
- Floor / stance reference: rest foot y ≈ **275.9**.
- Divergence metric (`.leg.csv`): `left_err_px`/`right_err_px` = per-foot
  distance between target foot and realized foot; combined = `hypot(L,R)`.

## 3. What the data shows (hipknee_fix2 run, σ=0/ζ=2.5)

| particle | target y (swing) | realized y (swing) | verdict |
|---|---|---|---|
| P3 (hip) | 172.9 | 172.9 | pinned exactly |
| P41 (knee) | 217–239 | 225–227 | bends, ±1px off |
| PLLL/PRLL (feet) | 248–251 | 278–283 | **never lifts** |

- Realized foot stays in range **275.9–283.2** the whole move — it does not even
  return to its exact stance value; it drifts *down* over the run.
- Target swing-foot lift is ~**28px** (275.9 → ~248). Realized lift ≈ **0px**.
- The knee's small bend (14–19°) plus the straight-pole geometry means: even
  when the knee bends, the foot tip stays near the floor because the *thigh*
  never rotates forward/up enough — the hip stays put and the thigh is nearly
  vertical.

## 4. Root-cause chain (what we have established)

1. **Straight-pole root cause (FOUND, prior session):** `observers.rs` seeded
   `hip_target`/`knee_target` at spawn == foot rest → degenerate rest angles →
   the knee was never commanded to bend → the leg behaved as one rigid pole.
   Fixed in [`hip_pin.rs`](../examples/game_lib/src/lower_body/hip_pin.rs:392) by
   seeding resolver hip/knee targets from *actual* particle positions
   (P3 = particles[1], knee = particles[2]) via the `leg_target` delta-bridge.
2. **Result:** knees now bend (14–19°) **but the foot still does not lift** —
   and the combined metric got *worse* (49.99px vs 36.8px straight-pole
   baseline), because the angular constraints now actively pull against the pin
   without achieving foot motion.
3. **Substep hypothesis (REJECTED):** The user's theory — the two angular
   constraints fight over the shared knee/foot and need more XPBD substeps to
   converge — was tested directly. `SUBSTEPS` env lever added to
   [`headless_arm_pd.rs`](../examples/collision_detection/examples/headless_arm_pd.rs:769)
   (writes `VelloCollisionWorld.substeps`; default 0 → clamped to 1). Sweep:

   | SUBSTEPS | combined divergence | notes |
   |---|---|---|
   | 1 | 45.7px | baseline (default) |
   | 2 | 45.6px | byte-identical behavior |
   | 4 | 933.7px | explosion |
   | 8 | 3.3e9px | NaN (particles flew to ±3e9) |

   → The solver already reaches a per-substep fixed point at 1 substep (1 vs 2
   identical), so convergence is **not** the binding mechanism. 4+ substeps
   destabilize instead.
4. **Velocity projection (SYNCED, not yet proven useful):** the user refactored
   the solver to accept a substep-independent `angular_velocity` (rad/s);
   `apply_velocity_projection` converts to per-substep `c_dot_desired = ω·dt`
   internally (constraints.rs:698), and the one-shot reset runs once per frame.
   Game layer now emits `angular_velocity` (leg_pd.rs), `c_dot_scale` is a
   unitless rad/s gain. The projection does not lift the foot either.

## 5. Parameter sweeps — the ~38px floor

All with GRAVITY=off, LEG_SIGMA=0, walk script. Combined peak divergence:

| config | peakL | peakR | combined |
|---|---|---|---|
| baseline (`sw_sigma_baseline`) | 46.5 | 60.9 | **63.1** |
| ζ=2.0 | 36.8 | 37.8 | 38.8 |
| ζ=2.5 (= `lagcheck`) | 36.8 | 36.6 | 38.7 |
| ζ=3.0 | 36.7 | 36.5 | 38.6 |
| ζ=3.5 | 36.7 | 36.4 | 38.5 |
| ζ=4.0 (`sw_final_sig0_zeta4.0`) | 36.7 | 36.3 | **38.4** (best) |
| σ>0 sweeps | 36.9–46.7 | 36.6–56.9 | 41.7–63.1 |
| hipknee_fix2 (seed fix) | 45.7 | 35.0 | 50.0 |

Takeaways:
- ζ=2.5→4.0 removed ~40% of the baseline error (63 → 38px) but **saturates**.
- No σ, ω-schedule, chase-alpha, or seed change breaks below ~38px.
- The floor is a *position* problem: the free foot never reaches a *position*
  the angular constraints are meant to imply.

## 6. Why "nothing holds the foot" is the key clue

The user's own observation ("there is nothing holding the feet physically") is
confirmed and sharpened: at GRAVITY=off with no colliders, the foot is only
moved by the two angular constraints + the distance constraints. Yet:

- The **distance** constraints are doing their job (bone lengths hold ~55.5px).
- The **angular** constraints articulate the joints (knee bends) but the implied
  *foot position* does not track the resolver's foot target.

This points away from "solver not converging" and toward either:
1. **The commanded rest angles never encode the needed foot position** — i.e.
   the target hip/knee angles fed to the solver, even after the seed fix, don't
   correspond to a foot that lifts. The rest angles are derived from
   `targets.hip`/`targets.knee` in `calculate_leg_pd`
   (leg_pd.rs:150-152); if those targets aren't actually tracking the resolver's
   lift, the angular constraint is fighting to hold the foot *down*.
2. **A kinematic/geometry mismatch** between the resolver pose and the live leg
   frame (hip height, thigh angle reference), so the "target lift" in the
   telemetry is computed but never translated into a correct joint-angle command
   in the particle frame.

## 7. Open questions (next investigation)

1. **Do the emitted hip/knee rest angles actually correspond to a lifted foot?**
   Log `theta_hip_t_cs`/`theta_knee_t_cs` (target-space rest angles) vs the
   *forward-kinematic foot position they imply, and compare against the
   resolver's foot target. If the FK of the target angles points at the floor,
   the bug is upstream in how `targets.hip`/`targets.knee` are produced.
2. **Is the resolver hip/lift actually reaching `LegPdTargets`?** Trace
   `targets.foot` (used for nothing except the knee rest derivation?) and the
   `target_hips`/`target_knees` written by `drive_hip_pin` per tick
   (hip_pin.rs:392-424). Confirm the lift is in the *particle-frame targets* the
   leg PD reads, not just in the resolver's internal pose.
3. **Why does the realized foot drift *down* over the run** (275.9 → 283.2)
   rather than hovering at stance? Something is injecting downward velocity —
   check whether the angular constraints' velocity projection (or the ω-scheduled
   compliance) pumps the foot downward each cycle.
4. **Reconcile the asymmetric case:** `walk_foot_divergence` had peakL=37.07 /
   peakR=19.83 — one foot tracked ~2× better. Whatever the good foot does
   differently is a direct lead.

## 8. Files touched in the current commit pair

- bevy_vello `6a6704d` — `leg_pd.rs`, `systems.rs`, `mod.rs` (angular_velocity
  sync), `hip_pin.rs` (hip/knee seed fix), `headless_arm_pd.rs` (SUBSTEPS lever).
- study_vello `129d917` — `constraints.rs`, `soft_body_connection.rs`,
  `collision_response.rs` (angular_velocity + per-substep c_dot conversion +
  one-shot reset).

## 9. Verdict

The vertical foot divergence **persists** at ~38px. We have eliminated:
substep convergence (rejected by data), damping saturation (plateau), and the
degenerate-rest-angle bug (fixed but insufficient). The remaining suspect is the
**target-side**: the hip/knee rest angles commanded to the solver do not
correspond to a lifted foot in the live particle frame, so no solver tuning can
close the gap.

## 10. Running the rig on Windows

The harness was written against a Linux CI and every default path points at
`/home/ubuntu/bender/...`. On Windows exactly two settings must be overridden:

- `ASSET_ROOT` — there is no Windows fallback; unset, the character assets are
  loaded from `/home/ubuntu/...` and fail.
- `OUT_CSV` — the default is a Linux path; without it no useful artifact lands
  in the repo.

Everything else has a working default.

### 10.1 Prerequisites

| Requirement | Check |
|---|---|
| Rust MSVC toolchain | `rustc -V` (x86_64-pc-windows-msvc) |
| Python 3 (for the analyzers) | `py --version` |
| Repo at `D:\bender` | paths below assume it |

Use `py`, not `python`. On this machine `python` / `python3` resolve to the
0-byte Microsoft Store execution alias and print nothing; the real interpreter
is `C:\Users\<user>\AppData\Local\Programs\Python\Python312\python.exe`.
Check with `py -0p`.

### 10.2 One-time build

```powershell
cd D:\bender\bevy_vello
cargo build -p collision_detection --example headless_arm_pd --release
```

The example runs on `MinimalPlugins` — no winit window, no swapchain — so it
works from a normal PowerShell window or a headless session. `--release` is for
speed only; keep the profile fixed when comparing runs (float code layout can
differ between debug and release).

### 10.3 Reproduce the documented walk (PowerShell)

```powershell
cd D:\bender\bevy_vello

$env:ASSET_ROOT = "D:\bender\bevy_vello\examples\collision_detection\assets"
$env:SCRIPT     = "D:\bender\bevy_vello\plans\walk_script.json"
$env:OUT_CSV    = "D:\bender\bevy_vello\tmp\headless\walk.csv"

# Leave the LEG_* knobs unset: the leg controller now ships the fixed config
# (compliance 1e-10, ζ=0.5, no chase / no ω-schedule). Overriding them with the
# old sweep values (e.g. LEG_ZETA=2.5) reintroduces the divergence.
$env:GRAVITY = "off"   # default; "on" or "x,y" in the vello frame (y down)

cargo run -p collision_detection --example headless_arm_pd --release
```

The same block under `cmd.exe`:

```bat
cd /d D:\bender\bevy_vello
set ASSET_ROOT=D:\bender\bevy_vello\examples\collision_detection\assets
set SCRIPT=D:\bender\bevy_vello\plans\walk_script.json
set OUT_CSV=D:\bender\bevy_vello\tmp\headless\walk.csv
cargo run -p collision_detection --example headless_arm_pd --release
```

Notes:

- `SCRIPT` must be [`walk_script.json`](walk_script.json). Without it the harness
  loads the built-in limb script for `LIMB` (default `right_arm`), which is
  **limb mode** — no `HipPin`, no whole-body telemetry, no `.leg.csv`.
- A `move`/`jump` command in the script (or `HIPPIN=1`) is what switches on body
  mode: `drive_hip_pin` runs, the per-leg verdict is computed, and the
  `.leg.csv` sidecar is written. See
  [`headless_arm_pd.rs:668`](../examples/collision_detection/examples/headless_arm_pd.rs:668).
- PowerShell env vars persist for the session. Clear them with
  `Remove-Item Env:ASSET_ROOT,Env:SCRIPT,Env:OUT_CSV` when switching runs.
- The asset root is the `assets` directory itself, not the crate root — the
  harness appends `character/v10.character.svg` and `.json` to it
  ([`headless_arm_pd.rs:150`](../examples/collision_detection/examples/headless_arm_pd.rs:150)).

### 10.4 Expected outputs

`flush_rows` creates the parent directory, so `tmp\headless\` does not need to
exist beforehand ([`csv.rs:14`](../examples/game_lib/src/headless/csv.rs:14)).

| File | Content |
|---|---|
| `walk.csv` | one row per fixed tick: 13 particle channels + targets + `act`/`kind`/`cmd_vx`/`vphase` |
| `walk.leg.csv` | `t,left_err_px,right_err_px,left_dx_px,right_dx_px` |
| `walk.events.csv` | input timeline (`t,event_id,label,keys,kind`) |
| `walk.summary.csv` | limb-cell verdict + hash (not the leg verdict) |

A healthy walk run is deterministic: **1036 rows** (11.5 s at 90 Hz), with
events at `t=0.5333` (`move`) and `t=10.5333` (`wait`). A different row count
means `SCRIPT` did not load or the process was interrupted.

### 10.5 Analyze

```powershell
cd D:\bender

py bevy_vello\tools\physics-qa\analyze_leg_divergence.py `
   bevy_vello\tmp\headless\walk.leg.csv --threshold 25

py bevy_vello\tools\physics-qa\analyze_frame_divergence.py `
   bevy_vello\tmp\headless\walk.csv
```

- `analyze_leg_divergence.py` prints peak L/R, early/mid/late means and the
  fraction of ticks above the threshold. Exit codes: 0 = under threshold,
  1 = diverged, 2 = no rows.
- Its "combined" column is `max(L,R)`, **not** the `hypot(L,R)` used in §2/§5.
  For comparisons with the sweep table, compute hypot explicitly:

```powershell
py -c "import csv,math; r=list(csv.DictReader(open(r'bevy_vello\tmp\headless\walk.leg.csv'))); c=[math.hypot(float(x['left_err_px']),float(x['right_err_px'])) for x in r]; print('peak hypot=%.2f mean=%.2f'%(max(c),sum(c)/len(c)))"
```

- `analyze_frame_divergence.py` prints P2/P3 velocity tracking and the residual
  against the integrated `cmd_vx`. It defines the move window as
  `cmd_vx != 0`, so a standstill script shows an empty move section (expected).
- Do not gate the legs on `walk.summary.csv`: in body mode the focus limb is
  still the default `right_arm`, so the row reports `NO_SWING` while the legs
  are the actual subject.

### 10.6 Env levers (identical names on Windows)

| Var | Meaning | Default |
|---|---|---|
| `ASSET_ROOT` | character asset directory | Linux CI path — **set on Windows** |
| `SCRIPT` | JSON command script | built-in limb script |
| `OUT_CSV` | main telemetry path | Linux CI path — **set on Windows** |
| `HIPPIN` | force body mode (`1`) | off |
| `GRAVITY` | `off` / `on` / `x,y` (vello y-down) | `off` |
| `LEGS` | `off` = pin only `P1/P2/P3` | on |
| `COMPLIANCE`, `DAMPING`, `TARGET_SPEED` | hip-pin params | struct defaults |
| `LEG_ZETA` | solver damping ratio ζ (leg default) | **0.5** |
| `LEG_SIGMA` | projection strength σ (0 = off; angle-derived reference) | 0 (off) |
| `LEG_COMPLIANCE` | angular stiffness (Kp of the XPBD spring) | **1e-10** |
| `LEG_C_DOT_SCALE` | commanded joint-rate gain | 1.0 |
| `LEG_NO_CHASE` | command rest angle directly (no chase/rate clamp) | off |
| `LEG_NO_SCHEDULE` | disable ω-softened compliance | off |
| `SUBSTEPS` | XPBD substeps | 0 → 1 |
| `LIMB`, `KP`, `KD`, `TARGET_DIST`, `SWEEP`, `GRID` | limb-cell mode only; `SWEEP` is ignored in body mode | `right_arm`, 0.5, 0.065, auto, off, built-in grid |

The leg controllers are spawned with a dedicated config (see §11): compliance
`1e-10`, `ζ=0.5`, `soft_scale=1`, `rest_chase_alpha=1`, `max_angle_rate=1e6`.
The arm controllers keep the arm defaults.

### 10.7 Windows gotchas

1. **`python` is a Store alias.** Use `py` for the analyzers; otherwise the
   command exits silently with no output.
2. **`ASSET_ROOT` is mandatory.** The observed failure mode is a pair of
   `bevy_asset::server: Path not found: .../character/v10.character.*` errors
   and a rig that never becomes ready (then aborts after 10 sim-seconds).
3. **`OUT_CSV` defaults to Linux.** If omitted, the run still succeeds but writes
   outside the repo. Always set it explicitly.
4. **Use absolute paths in env vars.** PowerShell passes them through unchanged;
   forward slashes also work (`D:/bender/...`) if quoting is awkward.
5. **Determinism check.** With the fixed clock, identical env + identical binary
   profile must reproduce the same `.summary.csv` hash. A changed hash means an
   input (script, env, asset) or the binary changed.
6. **`tmp/` is gitignored.** Artifacts are local only; the doc's reference copy
   of the script lives in `plans/` precisely for that reason.

## 11. Resolution (2026-10-17)

Three independent causes stacked. None of them alone explained the whole error,
and all three had to be fixed.

### 11.1 The target bridge was translation-only

[`drive_hip_pin`](../examples/game_lib/src/lower_body/hip_pin.rs:415) captured
only `live_hip − resolver_hip` and then wrote `resolver_pose + offset`. That is
rigid **between resolver poses**, but it is not congruent with the character's
spawn pose: the resolver's standing pose is a ~61° knee bend
(`hip_height=90`, `thigh=calf=55`), while the V10 blueprint spawns nearly
straight. At the first logged tick the target knee angle was **+60.8°** and the
realized **−0.3°**, so the entire walk ran inside a multi-second catch-up
transient.

The fix ([`hip_pin.rs:104`](../examples/game_lib/src/lower_body/hip_pin.rs:104))
stores both legs' **live** and **resolver** poses at seed. Each tick it applies
only the resolver's per-segment **rotation deltas** to the live pose
(`transfer_segment`), preserving the character's own bone lengths. At seed every
delta is zero, so the targets equal the live pose exactly; afterwards the swing
is layered on top without moving the rest pose.

### 11.2 Pinned particles diluted the angular mass

[`AngularConstraint::solve`](../../study_vello/integrations/vello_physics/src/constraints.rs:871)
computed `w_sum` over all three particles, including `P2`/`P3` which the
`HipPin` owns. Most of every angular correction was applied to particles the pin
reset on the next frame, leaving only a small fraction for the knee/foot. That
is the "near-zero authority" of §6.

The fix adds `free_mask` to `AngularConstraintConfig` (bit 0/1/2 = p0/p1/p2)
and zeroes the pinned particles' mass in both the positional solve and the
(disabled) projection. When `HipPin` drives the legs it sets
`LimbConfig::anchor_chain`, and [`calculate_leg_pd`](../examples/game_lib/src/character/leg_pd.rs:225)
emits `free_mask = 0b100` for the hip joint (only the knee free) and `0b110` for
the knee joint (only `P3` anchored).

### 11.3 The leg config was tuned for arms

The leg controllers inherited the arm defaults: `rest_chase_alpha=0.5`,
`soft_scale=20`, `angular_compliance=5e-8`, `zeta=2.5`. That is three lags in
series plus a spring that deliberately softens by 20× during the fast swing plus
over-damping. The leg controllers now spawn with a dedicated config
([`observers.rs`](../examples/game_lib/src/character_factory/observers.rs:863)):

| knob | arms | legs (fixed) |
|---|---|---|
| `rest_chase_alpha` | 0.5 | **1.0** |
| `max_angle_rate` | 2π rad/s | **1e6** (no clamp) |
| `soft_scale` | 20 | **1.0** (no ω-schedule) |
| `angular_compliance` | 5e-8 | **1e-10** |
| `zeta` | 1.0 (arm config) | **0.5** |

The arms are unchanged.

### 11.4 Measured result

Same walk script, no `LEG_*` overrides, release build:

| metric | before (`foot_divergence_fixed`) | after (`walk_default`) |
|---|---|---|
| peak hypot | 60.12 px @ 1.40 s | **3.52 px @ 10.52 s** |
| move-phase mean | 32.10 px | **0.70 px** |
| move-phase p95 | 63.04 px | **1.14 px** |
| late mean (last 1.5 s) | 16.43 px | **0.43 px** |
| ticks above 25 px | 42.2 % | **0.0 %** |
| harness tail | WAGGLE (rms 10.03 px) | **PASS (rms 0.394 px)** |
| P3 position residual | 0.60 px | 0.58 px |
| P2 position residual | 0.59 px | 0.56 px |
| wait-phase rocking | 1.91 px/s | 0.55 px/s |

Ablation (bridge + anchor already applied, `ζ=0.5`; numbers are the harness's
`max(L,R)` peak / late mean):

| config | peak | late mean |
|---|---|---|
| compliance 5e-8, chase + schedule (as shipped before) | 56.9 | 13.3 |
| compliance 1e-10, chase + schedule | 10.0 | 3.3 |
| compliance 1e-9, no chase, no schedule | 6.8 | 1.5 |
| **compliance 1e-10, no chase, no schedule (shipped)** | **3.5** | **0.5** |

### 11.5 Status of the old open questions (§7)

1. **Do the emitted rest angles imply a lifted foot?** Yes. With the live-pose
   seed the target and realized joint angles match at t=0 and track within a few
   pixels throughout; the vertical foot error is ~1–3 px rather than ~30 px.
2. **Does the resolver lift reach `LegPdTargets`?** Yes. The target knee/foot
   are the resolver pose rotated onto the live leg and the realized particles
   follow them.
3. **Why did the realized foot drift down?** It no longer does; the late stance
   error is 0.43 px.
4. **The asymmetric case?** Gone: peak left 1.20 px, peak right 3.48 px.

### 11.6 Velocity projection: reference fixed, still not a win

The reference rate is now derived from the **target angles** (per-tick
difference with wrap, a 0.25 rad jump gate, and a 20 rad/s clamp) instead of the
resolver's finite-differenced particle velocities, and the projection call in
[`post_step`](../../study_vello/integrations/vello_physics/src/soft_body_connection.rs:387)
is enabled (inert at σ=0). The remaining problem is the projection mechanism
itself: it treats the XPBD solve's post-correction displacement
`pos − previous_pos` as the joint velocity and rewrites `previous_pos`, which is
the state the next frame integrates.

Measured (peak hypot / mid mean):

| config | σ=0 | σ=1 |
|---|---|---|
| compliance 1e-10 (shipped) | 3.48 / 0.61 | 3.47 / 0.74 |
| compliance 1e-9 | — | 6.96 / 2.22 |
| compliance 1e-8 | 12.49 / 5.90 | 17.88 / 8.85 |

So the feed-forward is right in theory but not useful in this solver as written.
Keep the leg `sigma=0` (the default); the `angular_velocity` command is now
clean and ready for a velocity-level constraint that adjusts
`particle.velocity` before the next prediction instead of rewriting the
post-solve `previous_pos`.

### 11.7 Reproduce

```powershell
cd D:\bender\bevy_vello
$env:ASSET_ROOT = "D:\bender\bevy_vello\examples\collision_detection\assets"
$env:SCRIPT     = "D:\bender\bevy_vello\plans\walk_script.json"
$env:OUT_CSV    = "D:\bender\bevy_vello\tmp\headless\walk_default.csv"
target\release\examples\headless_arm_pd.exe

cd D:\bender
py bevy_vello\tools\physics-qa\analyze_leg_divergence.py `
   bevy_vello\tmp\headless\walk_default.leg.csv --threshold 25
```

Expected: `RESULT: OK`, peak ≈ 3.5 px, `tail=PASS(rms≈0.4px)`. The harness log
also prints `leg: peak left=1.2px ... right=3.5px ... tail=PASS`.
