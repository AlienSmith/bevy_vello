# Hop-in-place jump — measured findings (2026-11-18)

This records what was *measured*, not what was intended. Every number below is
from `tools/physics-qa/run_walker_qa.sh` telemetry
(`tmp/walker_qa/*.csv`), script `plans/walker_jump_hop_script.json`.

## 1. What works

- `LocoMode::Jump` + `Command::jump` + `Cell::Jump` exist end to end
  (state machine, script verb, telemetry `mode=jump`).
- A hand-rolled loop-2 controller runs the phase machine
  `Load → Launch → Flight → Land` on **measured** transitions.
- The hop now genuinely leaves the ground (0.30 s continuous airborne;
  before the launch fix the hip never rose above standing).
- Backward *translation* is gone: the pose is world-anchored (fixed plants +
  fixed `anchor_x`) instead of riding the measured hip.

## 2. The hard geometric ceiling

| Quantity | Value |
|---|---|
| Standing hip→foot chord | 104.5 px |
| Max leg reach (thigh 55 + calf 55) | 110 px |
| **Extension budget from standing** | **5.5 px** |
| Achieved apex (apex=28 requested) | **5.55 px** |

The leg stands at **95 % of full reach**, so a rest-length strut has only
5.5 px of travel to launch from. A 28 px apex needs `v0 = sqrt(2·g·28) = 234 px/s`.
The achieved apex saturates on the extension budget, not on the law's target.

## 3. The crouch/tip cliff (the real blocker)

The body does **not** fall backwards by translating. `hip.x` is pinned and holds.
It **rotates**: `meas_theta` runs to −1.59 rad (91°) and stays there — the
character lies down.

Sweep of `JUMP_APEX` (crouch depth = `0.5·apex`):

| apex | crouch | meas_theta min | \|ω_com\| max | real apex |
|---|---|---|---|---|
| 4 | 2 px | −0.012 | 0.120 | 0.07 |
| 8 | 4 px | −0.048 | 0.281 | 0.05 |
| 12 | 6 px | −0.102 | 0.547 | 2.28 |
| 16 | 8 px | −0.303 | 0.969 | 3.29 |
| **20** | **10 px** | **−1.601** | 1.956 | 4.40 |
| 24 | 12 px | −1.599 | 1.804 | 5.27 |
| 28 | 14 px | −1.594 | 2.433 | 5.55 |

**Stable up to ~8 px of crouch; collapses past ~10 px.** It is a cliff, not a
gain slider.

## 4. Hypotheses that were tested and REFUTED

- **"Foot slip causes the rotation."** Refuted. During Load the feet sit at the
  plants exactly (`dL = dR = 0.0`) while `omega_com` ramps −0.08 → −1.32. The
  slide (5.9 px) appears only *after*, at t=1.51.
- **"Non-zero attitude command causes it."** Refuted. Forcing
  `upper_angle = 0` made the tip *worse* (−1.59 → −1.67).
- **"The spin-null differential ratio is wrong."** The ratio *was* wrong (it
  used the asymmetric rest offsets −33/+39.5 against symmetric plants
  −37.3/+37.2), and it was corrected to use the real plants — **the tip was
  unchanged**. So the ratio is not the mechanism either.

## 5. What the evidence points at

The tip scales with **crouch depth** and the CoM cannot stay over the base while
the hip travels 10–14 px *down* between two contacts only 74 px apart. The
strut is a rest-length position servo: to crouch, the chord is shortened, and
the resulting leg fold is resolved against friction — which is exactly the
regime where a soft, sliding foot trades vertical travel for body rotation.

This is consistent with the repo's own prior art at
[`walker_control/src/walker.rs:240`](../walker_control/src/walker.rs:240):
> "A pose-level push can only move FEET, and a sliding foot delivers almost
> nothing (measured: 6 px of foot drag → 0.6 px of hip travel)."

The boost solves this by applying a **velocity impulse directly to the P3 hip
particle** (`queue_connect_particle_velocity`). A jump needs the same thing:
a real impulse, not a rest-length target.

## 6. Honest conclusion

"Hop in place" is **not** achievable by tuning this controller:

- the achievable apex is ~5.5 px (extension budget), and
- any crouch deep enough to add stroke past that tips the body over.

Fixing it is a design change, not a parameter change. Two principled options:

1. **Impulse launch.** Drive `Launch` as a vertical velocity impulse on the hip
   particle over a few ticks (mirroring `boost_hip_accel`), with the legs only
   providing support/attitude. This decouples hop height from the 5.5 px strut
   budget.
2. **PPO / learned controller.** Only justified if the *learned* policy can use
   contact friction in a way this pose-level servo structurally cannot. Note the
   refuted hypotheses above: the failure is not a control-gain problem, so a
   learned policy would have to learn a different *actuation channel*, which the
   impulse option already provides analytically.

Recommendation: option 1 first. It reuses the proven boost mechanism, keeps the
loop-2 law and the measured phase machine, and is a bounded change.

## 7. The impulse launch WORKS (option 1, implemented 2026-11-18)

The vertical impulse launch was implemented and **validated**. The hop now
genuinely leaves the ground and lands upright:

| Metric | Before (chord-only) | After (impulse) |
|---|---|---|
| Rise | 5.55 px (saturated) | **25.1 px** |
| Airborne | 0.30 s (marginal) | **0.44 s** |
| `meas_theta` range | −1.59 rad (91°, fell) | **−0.10 … +0.04 rad (5.5° max)** |
| `omega_com` range | 1.72 mean / 3.89 max | **−0.27 … +0.43** |
| Landing | tipped over | grounded, upright (final θ 0.02–0.06) |

Run: `imp28c` — `JUMP_APEX=8 JUMP_IMPULSE=234` (shallow crouch, below the tip
cliff; the height comes from the impulse, so no deep crouch is needed).

### What it took (three things, all necessary)

1. **The impulse itself** (`ControlConfig::jump_launch_impulse`, `JUMP_IMPULSE`
   env knob). `JumpCtrl` fires it once on the Launch→Flight transition;
   `actuate_stage` writes it via `queue_connect_particle_velocity`. With the
   knob unset it derives `v0 = sqrt(2·g·apex)`, so `JUMP_APEX` alone also works.
2. **Apply it to the WHOLE leg chain, not `P3` alone.** The legs are rigid bone
   rods: kicking only `P3` left the feet pinned to the floor by contact, and the
   closed hip→floor chain ate the impulse in ~1 frame (`P3vy` reached only −113
   of −234; body rose 6 px = the extension budget). Writing the same `Δv` to
   every particle of both leg assemblies launches the chain rigidly, so the feet
   actually leave the ground.
3. **Tuck the feet to the body in Flight** (`fly_l`/`fly_r`, captured at release
   relative to the measured hip). Holding the fixed world plants let the strut
   length `|hip − foot|` grow without bound as the hip rose, so the compliant
   leg dragged the hip back down — the visible ground "bump" (`P3` rose 6 px,
   `grounded` toggling). Riding the hip keeps `rest_length = |fly|` constant, so
   the leg neither launches nor damps.

### Still to address

Deepening toward the full 28 px is a tuning matter now (the mechanism is
proven): raise `JUMP_IMPULSE` toward `sqrt(2·g·28) ≈ 234` at the *foot* and let
the loop-2 law close on the measured rise. The `jump → standing` handover pose
discontinuity (feet y 308 → 403, hip x 175.7 → 105.2 for one frame) is a
mode-switch bug and is still independent of the launch mechanism.