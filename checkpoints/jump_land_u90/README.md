# jump_land_u90 — residual jump-and-land policy

Update-90 snapshot of a residual PPO run (obs 22 / act 5, `JumpResidual` on top
of the authored expert). The expert alone jumps but tips over every time
(spin ≈ −1.7 rad/s right after liftoff, lean ≈ −81° at touchdown); this policy
cancels the spin and lands upright.

## Training run
```
LR=1e-4 SEED=7 N=64 UPDATES=150 PATIENCE=100000 WIN_HIDDEN=1 ./target/release/walker_ppo
```
Reward defaults at the time: `W_LAUNCH_SPIN=2.0`, `W_RECOVER=1.0`, `R_SURVIVE=2.0`
(`walker_ppo/src/reward.rs`). This is the `snap/policy_0090.json` snapshot.

## Deploy check (walker_qa, deterministic)
`plans/walker_jump_hop_script.json` (stand 1.25 s -> jump 2.5 s -> stand 2.5 s):
touchdown lean -13°, final lean 2°, final hip height back at standing (~204 px).
Varying the stand time before the jump (1.0 / 1.5 / 2.0 / 3.0 s): 4/4 landed.

Not tested: other start poses, pushes, other seeds. The neighbouring snapshots
(u100-u120) fell, so this is a good candidate, not a proven stable plateau.

## Run it
```
tools/physics-qa/run_walker_qa.sh jump_u90 POLICY=checkpoints/jump_land_u90/policy.json SCRIPT=plans/walker_jump_hop_script.json
```
