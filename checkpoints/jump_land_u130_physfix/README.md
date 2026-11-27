# jump_land_u130_physfix — residual jump-and-land policy (candidate, not committed)

Update-130 snapshot of a residual PPO run trained against the **fixed** collision
physics (angular-friction gain capped as a bounded rate in
`study_vello/integrations/vello_physics/src/constraints.rs`), with the expert's
jump now consistent across spawn positions.

## Training run
```
LR=1e-4 SEED=7 N=64 UPDATES=150 PATIENCE=100000 WIN_HIDDEN=1 ./target/release/walker_ppo
```
Rig: `v10.character.json` with the four arm particles at `inv_mass` 1000 (light
arms) — NOT the repo's committed v10. Reward defaults as in commit 31d9343.

## QA (expert replaced by this policy; `plans/walker_jump_hop_script.json`)
Ends standing (lean < 30 deg, hip back near standing) at spawn x = 0, 60, 210,
300; falls at x = 150. Neighbouring snapshots (u120, u140, u150) land only 1-2 of
5 positions, so this is a fragile candidate, not a stable plateau.

## Run it
```
tools/physics-qa/run_walker_qa.sh jump_u130 POLICY=checkpoints/jump_land_u130_physfix/policy.json SCRIPT=plans/walker_jump_hop_script.json
```
Only valid with the physics fix in place (a different collision model changes
the jump the policy was trained to correct).
