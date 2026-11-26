# jump_land_simple_u120 - residual jump-and-land policy (proof of concept)

Update-120 snapshot of a residual PPO run on the contact-mass physics (bevy_vello branch
`jump-contact-mass` + study_vello branch `contact-mass`; the feet carry `contact_mass_max`
in `v10.character.json`). Valid only with that physics: a different collision model changes
the jump the policy corrects.

## Training recipe
Reward: **+1 only when the landing completes** (stands through the 1.5 s recovery window),
nothing per tick. Every other term is zeroed through environment knobs.

```
R_BASE=0 W_UPRIGHT=0 W_ANG=0 W_LAUNCH_SPIN=0 W_RECOVER=0 R_SURVIVE=1.0 GAMMA=0.999 \
GENS_PER_UPDATE=4 INIT_STD=0.05 ENT=0 LR=1e-4 SEED=7 N=32 UPDATES=120 \
SNAPSHOT_EVERY=5 PATIENCE=100000 WIN_HIDDEN=1 ./target/release/walker_ppo
```
(4 generations of 32 episodes per update, noise std at its 0.05 floor, no entropy bonus.)

## Evaluation (deterministic: mean action, no sampling noise)
| | expert alone | this policy |
|---|---|---|
| trainer environment, 32 bodies, ~830 episodes | 94.4% (645/683) | **98.1%** (813/829) |
| walker_qa, 31 spawn positions (stand -> jump -> stand) | 29/31 | **30/31** |
| positions that fall | 0.001, 0.1 | 80 (a slow topple ~2.5 s after a good landing) |
| jump rise, median / min | 44 / 35 px | 41 / 33 px |
| max torso lean during the jump, median / max | 25 / 39 deg | 39 / 51 deg |

Snapshots fluctuate: u80 of the same run scored only 80.1% deterministic, u100 98.0%, u120
98.1%. Pick a checkpoint by deterministic evaluation, not by the last update.

## What it is not
- It is a small correction on top of the expert (the residual is clamped; the expert's launch
  impulse passes through unchanged). The jump is not perfectly vertical (~50 px sideways drift
  at x=0, uneven foot lift): landing position was not rewarded.
- It uses more torso lean than the expert (median 39 deg vs 25): the reward only checks
  tilt < 60 deg. `FALL_ANGLE_DEG=45` would tighten that.
- The reward window (1.5 s) cannot see a topple later than that; `RECOVERY_WINDOW=3.5` would.

## Why this reward (what failed)
- The original dense reward made the deterministic policy worse as it trained (4/5 positions at
  u10 -> 0/5 from u60). Adding each removed term back to the simple reward one at a time: the
  **launch payment** (`R_BASE`, reward for low spin at liftoff) reproduced the steady decline;
  the pre-liftoff spin penalty and flight terms were neutral; recovery shaping looked helpful
  in the sampled survival.
- But the sampled survival is misleading: with dense terms (even without the launch payment) the
  sampled policy kept improving (0.81 -> 0.94) while the deterministic one got worse
  (93.9% -> 84.5% -> 81.3%): the policy learns to use its own exploration noise. Only the plain
  completion reward kept the deterministic policy at or above the expert.
- Not the cause: the PPO implementation (toy-task test in `walker_nn/tests`), batch size,
  entropy/noise growth, resource leaks.

## Run it
```
tools/physics-qa/run_walker_qa.sh jump_simple_u120 POLICY=checkpoints/jump_land_simple_u120/policy.json \
    SCRIPT=plans/walker_jump_hop_script.json
```
