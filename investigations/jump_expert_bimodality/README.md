# The jump expert is bimodal (investigation notes, 2026-11-25)

**Status: the behaviour is characterised, the root cause is NOT found.**

## Summary

The authored jump expert (`walker_control::jump::JumpCtrl`, no policy) does not
produce one jump. Depending on the character's spawn x it leaves the floor in
one of two modes:

| mode | peak CoM spin after liftoff | what the body does |
|---|---|---|
| high-spin | ~1.5 - 1.8 rad/s | tumbles (-81 deg at touchdown), ends lying down |
| low-spin | ~0.3 - 0.6 rad/s | spins less but slides sideways (~285 px), also ends lying down |

The expert fails to land in **both** modes. Which mode you get is
deterministic and reproducible for a given spawn x, but it is
hypersensitive: a 0.001 px change in x can flip it.

This matters for the residual-PPO task (`walker_ppo`): a residual policy has to
cancel the expert's spin, and the right correction for one mode overshoots in
the other.

## Evidence

All numbers are from `walker_qa` with `SCRIPT=plans/walker_jump_hop_script.json`
(stand 1.25 s -> jump 2.5 s -> stand 2.5 s), expert only unless stated.
"Peak spin" = max |omega_com| in the 0.3 s after liftoff.

**1. Spin is bimodal in spawn x** (20 px steps, x = 0..400, original arms):
0:1.70 20:0.31 40:1.60 60:1.52 80:1.59 100:1.64 120:1.54 140:0.42 160:0.35
180:0.47 200:0.42 220:0.39 240:0.40 260:0.40 280:0.39 300:1.48 320:0.62
340:0.37 360:1.67 380:1.67 400:1.48 - 10 high, 11 low. With the four arm
particles 100x lighter (`inv_mass` 10 -> 1000): 23 high, 8 low of 31.

**2. Hypersensitive.** x = 0 -> 1.70, x = 0.001 -> 0.38, x = 0.01 -> 1.69.

**3. Deterministic through the jump.** Repeated runs at the same x are identical
for every row until t ~ 2.9 s or later (after the body has fallen); a small
non-determinism then appears (28-36 columns drift). Cause not checked
(`vello_physics` is built with `parallel`).

**4. The mode is set at one tick.** At the Launch -> Flight release tick
(t = 1.600 s) the hip and both feet are displaced 8 - 17 px sideways in a single
tick, the foot velocity spikes to ~ -1400 px/s for one frame, and the lean steps
by 1 - 2 deg. At that moment the feet are 16 - 21 px below the floor top
surface (y = 312). The size of the kick is similar in both modes; the lean step
correlates with the mode (about -2 deg high-spin, -1 deg low-spin) but not
cleanly (one light-arm run, x = 220, is high-spin with -1.1 deg).

**5. The policy is not the cause, it reacts to it.** Expert vs u90
(`checkpoints/jump_land_u90`) at the same x: identical for all 113 standing
frames, diverging from the first jump tick. u90 injects a counter-spin before the
kick (omega ~ +0.95 at t = 1.55 s) so it leaves with ~ -0.35 rad/s instead of
-1.7. Matching u90 against the expert's mode at the same x (single body, QA):

| expert mode at that x | u90 landed | u90 fell |
|---|---|---|
| high-spin | 9 | 2 |
| low-spin | 1 | 5 |

So u90 learned to cancel a large spin; where the spin is already small, the
same correction overshoots (the +1.05 rad lean tip-over seen in the trainer).

**6. This explains the confusing training numbers.** Deterministic u90 in the
trainer lands 94% (N=1, body at x=0) but 34 - 58% for other body counts, and 18%
sampled with the learned noise (std ~ 0.3), because the body positions put a
different share of bodies in each mode. The policy sees no absolute x, so it
cannot tell which mode it is in. The default QA scenario (spawn x = 0) is a
high-spin start, so it only measured one mode.

## Ruled out

| hypothesis | test | result |
|---|---|---|
| number of bodies | one body in QA at different x | still position-dependent |
| floor position / length / shape | floor shifted with or against the character | outcome follows the character's world x |
| GPU collision coordinates (tile/bin phase) | shifted all collision geometry by +/-210 px before the GPU pass, added back after | no change |
| contact normals | logged every soft-vs-static normal; forced them vertical | median 0.000 deg off vertical, 4 of 2576 contacts > 1 deg; forced vertical: modes unchanged (1.66 / 0.43) |
| which foot is lifted | foot-height difference at stroke start | +3.00 px and the same foot in every run |
| the arms | arm particles 100x lighter | two modes remain |
| penetration depth / substeps | 1, 2, 4, 8 solver substeps | penetration 17-21 -> 7-9 px, kick still -8..-14 px, spin spread not smaller (0.48 - 2.16 at 8) |
| stance pin | `STANCE_PIN=0` | **not tested**: that switch lives in the walking controller, the jump path ignores it |

One observation that is only a correlation: at ticks 143 - 145 the number of
soft-vs-static contacts differs between a high-spin and a low-spin run
(4/2/1 vs 1/1/2), i.e. the contact set differs just before the kick.

## Open questions

1. Where does the *horizontal* part of the release-tick kick come from? The
   launch impulse in `walker.rs` is purely vertical (`Vec2::new(0.0, impulse)`).
   Look at everything that changes at the Launch -> Flight tick: which pins /
   constraints switch off, how the impulse reaches each particle, and how the
   soft-body contact solver responds to the foot contacts at that moment.
2. What in the pre-stroke state selects the mode? Lean, omega, angular
   momentum and positions at t = 1.24 s did not separate the modes.
3. Is the small late-run non-determinism (item 3) the `parallel` feature?

## Consequences for training

- Training survival % mixes both modes and is not a reliable bar. Judge
  checkpoints with deterministic eval at several spawn x (for example 0, 60,
  150, 210, 300), not a single QA start.
- Fixing the expert so it has one consistent mode is probably worth more than
  more PPO tuning.
- u90 is a good candidate in the high-spin mode only.

## Reproducing

`tools/physics-qa/compare_runs.py A.csv B.csv` (committed) diffs two QA CSVs
frame by frame and reports the takeover frame and the first divergent columns.

The experiments above used small **diagnostic patches that are not committed**
(env-gated, defaults unchanged):

- `walker_qa/src/main.rs`: `SPAWN_X`, `FLOOR_X`, `FLOOR_HALF_LEN`
- `walker_ppo/src/main.rs`: env id and spawn x in the `EXPERT_ONLY` episode log,
  `CHAR_SPACING`
- `src/integrations/physics/systems.rs`: `COLLISION_SHIFT_X`, `DEBUG_NORMALS`,
  `FLOOR_NORMAL_FLAT`, `PHYS_SUBSTEPS`

Typical run (expert only, spawn 20 px to the right):

```
tools/physics-qa/run_walker_qa.sh t SCRIPT=plans/walker_jump_hop_script.json SPAWN_X=20
```

Raw CSVs and logs were kept only in a scratch directory and are not in the repo.
