# The jump expert diverges under the collision physics (investigation notes)

Started 2026-11-25, updated 2026-11-26.

**Status (2026-11-26): the jump expert now lands from nearly every start (29 of 31 spawn positions) but its liftoff spin still takes several discrete levels; the heavy foot absorbs them. The large penetration was caused by the light foot being driven into the floor by the heavy hip through the leg's constraints; a depth-driven "contact mass" on the feet fixes that (branches `jump-contact-mass` / `contact-mass`). Walking is not yet re-validated with it.**

The sections below up to "Resolution" are the investigation that led there: the
original physics behaves inconsistently under the stroke, and the diagnostic experiments
on friction, the spin estimate and contact reporting were run on the original masses.

## Headline

The authored jump expert (`walker_control::jump::JumpCtrl`, no policy) is
deterministic code: it issues the same stroke every time. The *physics response*
to that stroke is not consistent. Tiny differences in the start (a 0.001 px change
in spawn x) send the same stroke to very different outcomes, so the expert's
behaviour diverges a lot from run to run even though the expert itself does not
change. This is what the PPO residual has been fighting.

With the original physics the expert's peak spin after liftoff is either
~1.5 - 1.8 rad/s ("high") or ~0.3 - 0.6 rad/s ("low"), and it falls in both modes.

## Where the divergence comes from

All of this is in the frame collision response
(`FrameCollisionConstraint::solve` in `constraints.rs`) and in how contacts are
produced for it. Confirmed means "removing/changing it changes the outcome as
stated"; the rest is observed but not shown to be causal.

**Confirmed**

1. **Angular friction saturates at full strength.** The gain was
   `friction * |penetration|`, clamped at 1: the whole measured body spin was
   cancelled in one step for any foot more than ~3 px deep (friction 0.35). The
   feet are 8 - 20 px deep during the push-off. Removing only this term cut the
   spread of peak spin across spawn positions from 1.39 to 0.14; capping the gain
   gave a smooth sweep (0.06 at gain 0.03, 0.20 at 0.1, 0.58 at 0.25, 0.85 at 0.5,
   1.39 uncapped). It amplifies a difference that already exists; it does not
   create it.
2. **The spin estimate treats deformation as rotation.** It is
   `sum(r x v_rel) / sum(r^2)` over four frame particles that move relative to
   each other as the crouch compresses the frame. Logged at the contacts it read
   2 - 8x the body's real spin (median 2.3x at x=0, 4.2x at x=0.001), so a nominal
   gain of 1 is an effective gain well above 1 and overshoots by an unpredictable
   amount. Estimating the spin from the spine axis only (spine_base -> spine_mid)
   makes the original strong friction usable: the expert alone then ends standing.

**Observed, not proven causal**

3. **Contact flicker.** The GPU narrow phase reports one contact per collider pair
   per tick. A flat-bottomed foot has several nearly-equally-deep curves, and the
   reported curve flips between them (2, 4, 6) from tick to tick. Two runs started
   0.001 px apart already have different contact sets at tick 8 (the first ground
   contact), and while standing the count alternates between 1 and 2. This is the
   best candidate for the selector that friction then amplifies.
4. **Stale contact and a wandering contact point.** A contact is detected one tick
   before it is applied and its position on the curve is found by projecting that
   stale point onto the current curve. The curve parameter `t` jumps (0.00, 0.99,
   0.20, ...) and the penetration measured there jumps with it (4 -> 16 -> 10 -> 20
   px). Using the deepest point of the curve instead makes `t` continuous, and the
   split did not change, so this is not the selector.
5. **The frame solver averages over however many contacts exist**, and the count
   differed between modes (4/2/1 against 1/1/2 at ticks 143 - 145). Some ticks also
   contain two identical constraints in one solve. The sum-versus-average test was
   interrupted and not run.

**Unexplained**

6. A single-tick sideways kick of 8 - 17 px (hip and both feet) at the
   Launch -> Flight release tick (t = 1.60 s). It survives friction = 0, deeper
   substeps and the other changes below, so friction does not cause it. The launch
   impulse in `walker.rs` is purely vertical.

## Experiments and results

Expert only, `walker_qa`, `plans/walker_jump_hop_script.json`. "Spread" = max - min
of peak spin over the spawn positions tested (x = 0, 0.001, 20, 60, 140, 200, 220,
300 unless stated).

| change | result |
|---|---|
| original physics | two modes, spread 1.39; falls in both |
| frame angular friction removed | spread 0.14 (one mode, falls) |
| frame tangential friction removed | spread 2.73 (worse) |
| fine tangential friction removed | spread 1.30 (no change) |
| angular gain capped as a rate (`ANGULAR_FRICTION_RATE_MAX = 10/s`, ~0.1 per 90 Hz tick) | spread 0.16; 31 positions: spin 0.94 - 1.42, stdev 0.14; falls consistently |
| Coulomb friction at the contact point (friction cone, effective mass), full stick | spread 1.33 (still bimodal) |
| Coulomb with the slip correction scaled by 0.5 / 0.25 / 0.1 | spread 0.87 / 0.48 / 0.40 |
| original friction + spine-axis spin estimate | expert alone STANDS in 7 of 31 positions (23%); the landings are lateral hops of 150 - 190 px, not vertical |
| deepest-point contact parameter | spread unchanged (1.26 with original friction) |
| solver substeps 1 / 2 / 4 / 8 | penetration 17-21 -> 7-9 px, kick and spread not reduced |

Ruled out as causes: number of bodies, floor position / length / shape, GPU
collision coordinates (shifting them by +/-210 px changed nothing), contact normals
(median 0.000 deg off vertical; forcing them vertical changed nothing), the arms
(100x lighter, same split; the committed v10 rig is restored), which foot is lifted.
`STANCE_PIN` is not tested: it only exists in the walking controller.

Determinism: repeated runs at the same x are identical through the jump (until
t ~ 2.9 s or later); after the body has fallen a small non-determinism appears
(cause unchecked, `vello_physics` builds with `parallel`).

## Resolution for the jump: contact mass (2026-11-26)

Reading the rig showed the hip is 20x heavier than the feet (`inv_mass` 0.05 vs 1.0).
The leg's distance/angular constraints split a correction by inverse mass, so when
the stroke lengthens the leg ~95% of it drives the light foot into the floor and the
hip barely rises. That is where the 8 - 20 px penetration came from (the contact
solver only removes a fraction of the depth per tick), and the friction terms scale
with that depth. Probe with the foot `inv_mass` edited in the rig (expert alone,
8 positions): feet x10 heavier -> depth 11 px, 5/8 stand; x100 -> 4 px, **8/8 stand**,
nearly vertical; x1000 -> 3.7 px, 8/8 but a lower posture and jump; adding heavier
knees made it worse. Constant heavy feet break walking, so the heaviness is applied
only in contact.

Implementation (`study_vello`, `vello_physics`): a particle flag `contact_mass_max`
(blueprint, default off; set to 100 on `PLLL`/`PRLL` in `v10.character.json`). After
every substep, each flagged skeleton particle takes the **deepest** penetration seen by
the frame collision constraints of the soft bodies it is connected to (mass has no
direction, so the maximum), maps it through a smoothstep between `D0 = 0.3` px and
`D1 = 1.5` px (world px; a resting foot already shows ~2 px, so this means "heavy
whenever the foot touches"), keeps the level with a hold (`decay 0.99` per substep,
half-life ~0.8 s) and sets `inv_mass = base / (1 + (M-1) * level)`. The hold matters:
without it the mass toggles every tick on ticks where a contact is not reported, which
was unstable; gating on depth alone (D0 5, D1 9) was too late and gave a much higher
jump. Env overrides: `CONTACT_MASS_D0`, `CONTACT_MASS_D1`, `CONTACT_MASS_DECAY`.

Expert alone, 31 spawn positions (x = 0 - 400 in 20 px steps plus 0.001 - 10 nudges),
original physics, no environment overrides:

| | mass update off | new defaults |
|---|---|---|
| ends standing | 0 / 31 | **29 / 31** (still falls at x = 0.001 and 0.1) |
| signed spin 0.1 s after liftoff | two bundles: 17 runs at -1.9 .. -1.5, 14 at -0.55 .. -0.2; gap 0.93 | still several levels: 7 at about -1.1, 17 spread over -0.9 .. -0.3, 3 near -0.1, 4 at about +0.5 (opposite direction); largest gap 0.53 |
| peak foot depth (median) | 19.0 px | 3.6 px |
| foot slide in the push-off (median / max) | 18.5 / 22 px | 8.1 / 24 px |
| hip sideways travel (median) | 63 px | 36 px |
| apex rise (median) | 73 px | 44 px |

Correction: an earlier version of this table (and the commit message of the commit that
introduced it) used the maximum of |omega| after liftoff, which folds +0.5 and -0.5 together
and wrongly suggested a single spin mode ("largest gap 0.16"). The signed values above, and
the overlay of `omega_com` over the 31 runs, show that the discrete spin levels are still
there. After landing, the two runs that fall (x = 0.001 and 0.1) do so slowly, about 1 - 2 s
after touchdown, not in flight.

Caveats: the earlier friction/estimator/contact-flicker experiments above were run on the
original masses; they were diagnostics and their switches are not in this branch. The
jump is lower than before (apex 44 vs 73 px) so the launch impulse may need re-tuning.
Walking has not been re-checked with the flagged feet; the plan is to train one PPO model
on both. Two positions still fall.

## Consequences for PPO

- Old physics: u90 (`checkpoints/jump_land_u90`) lands in QA at the default start,
  but that start is a high-spin one; vs the expert's mode at the same x it landed
  9 of 11 high-spin starts and 1 of 6 low-spin starts. Trainer survival % mixed
  both modes and was not a usable bar.
- Capped-gain physics (expert consistent): snapshots 10 - 110 fail everywhere,
  u130 lands at 4 of 5 positions (fragile; neighbours land 1 - 2 of 5). u130 is
  saved but NOT committed in `checkpoints/jump_land_u130_physfix` (trained on a
  light-arm rig).
- Low-noise run (`INIT_STD=0.15 STD_MAX=0.15`): sampled survival reached 55 - 72%
  but the deterministic policy fails (0/5 at every snapshot but one; 1 - 2% in the
  trainer's own deterministic evaluation). The policy depends on the exploration
  noise; nothing from this run is deployable.
- Compare deterministic and sampled behaviour before trusting any survival number.

## State of the code

Two repos, committed on paired branches (the `bevy_vello` workspace depends on the sibling
`../study_vello` by path, so check out both):

- `study_vello` branch `contact-mass`: the contact-mass flag and update in
  `integrations/vello_physics` (defaults as above; everything else is the original physics).
- `bevy_vello` branch `jump-contact-mass`: the rig flag on the feet, a `walker_qa`
  `SPAWN_X=<px>` knob (default 0), a `Default` fix in `game_lib`, and
  `tools/physics-qa/compare_runs.py`.

The diagnostic switches used in the investigation above (`FRAME_FRICTION_MODEL`,
`FRAME_OMEGA_EST`, `COLLISION_SHIFT_X`, `PHYS_SUBSTEPS`, ...) were reverted and are not in
either branch.

## Open questions / next steps

1. Train the PPO residual on the new physics and judge checkpoints with deterministic
   evaluation at several spawn positions.
2. Re-check walking with the flagged feet (planned: one model for both).
3. The two positions that still fall; whether the launch impulse needs re-tuning for the
   lower apex.
4. Explain the one-tick sideways kick at the Launch -> Flight release (it survived every
   physics change tried; it is much smaller in effect now that the foot is not driven in).
5. The frame solver's partial correction (interpolation weights, averaging, skeleton
   propagation `collision_scale = 1 - collision_damping` = 0.3 in this rig) is still there;
   the contact mass works around it for the feet rather than removing it.

## Reproducing

```
# expert only, spawn 20 px to the right
tools/physics-qa/run_walker_qa.sh t SCRIPT=plans/walker_jump_hop_script.json SPAWN_X=20

# with the contact mass disabled (control)
CONTACT_MASS_D0=1000 CONTACT_MASS_D1=1001 tools/physics-qa/run_walker_qa.sh t \
    SCRIPT=plans/walker_jump_hop_script.json SPAWN_X=20
```

Raw CSVs and logs were kept only in a scratch directory and are not in the repo.
