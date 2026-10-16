# Foot Divergence — State of Knowledge (2026-10-16)

> Status: **ERROR PERSISTS.** This document is a snapshot of what we know and what
> we have ruled out. It is *not* a resolution.

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
  [`walk_script.json`](../tmp/headless/walk_script.json) (wait 0.5 + move 10.0 +
  wait 1.0), FIXED_HZ=90.
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
