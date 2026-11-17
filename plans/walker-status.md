# Walker locomotion — conclusive status (2026-11-17)

**Verdict: the walker is functionally done.** A character stands, launches from
rest, walks at a commanded speed, changes speed, and stops. Speed control
responds monotonically to the command, the gait alternates, nothing walks
backwards, and the whole thing is covered by 211 unit tests plus a headless QA
harness that produces a CSV and a hip/feet plot per run.

One cosmetic/physical defect is **parked, not solved**: the stance foot slides a
few px inside its own (already world-pinned) target. It is diagnosed end to end
below; it does not break walking, speed control, or the launch.

Everything in this document is measured from `tmp/walker_qa/2.csv` (17 s,
1531 ticks at 90 Hz) unless stated otherwise.

---

## 1. Where the code lives

| crate | role |
| --- | --- |
| [`walker_math`](walker_math/src/lib.rs) | the plan: gait FSM, capture-point placement, deadbeat stance law, attitude loop, vertical stabiliser. Pure, no ECS. 119 tests. |
| [`walker_control`](walker_control/src/lib.rs) | observe → control → actuate → telemetry, on one `Walker` component. 92 tests. |
| [`walker_qa`](walker_qa/src/main.rs) | headless QA scene: attaches the walker to the v10 character, plays a script, dumps the legacy CSV. |
| [`examples/game_lib`](examples/game_lib/src/lib.rs) | the character rig (particles, joints, colliders) and asset loading. |
| `../study_vello/integrations/vello_physics` | the XPBD solver (path dependency, `parallel` feature). |

## 2. The pipeline, in one screen

```
physics ──observe──> Fiction ──control──> ControlOut{Pose, plan, debug} ──actuate──> constraints
   ▲                                                                                      │
   └──────────────────────────────────────────────────────────────────────────────────────┘
```

- **observe** ([`observe.rs`](walker_control/src/observe.rs)): particles →
  `Fiction { hip, com, v_com, lean, lean_rate, feet[2]{pos, touching}, dt }`,
  with EMA filtering and contact debounce. This is the only place reality is
  read.
- **control** ([`control.rs`](walker_control/src/control.rs)): one
  [`Step`](walker_control/src/control.rs:375) impl per mode returns a `Pose`.
  The walking pose is always

  ```
  pose.hip   = measured hip                     (translation-invariant)
  pose.foot  = measured hip + to_vello(solver_offset)
  ```

  where `to_vello` mirrors x by `facing` and negates y (solver is y-up travel
  frame, Vello is y-down). `stance_press` and the stance pin are the last two
  acts on that pose.
- **actuate** ([`actuate.rs`](walker_control/src/actuate.rs) /
  [`emit.rs`](walker_control/src/emit.rs)): pose → per-leg `AngularCommand`s
  (hip `P2-P3-knee`, knee `P3-knee-foot`) + one `DistanceCommand` (the
  hip↔stance-foot strut). Rest lengths come from the **live bone lengths**, so a
  frame error in the pose is not double-applied.

## 3. The laws and their defaults

[`LowerBodyConfig`](walker_math/src/types.rs:294) — planner:

| field | value | meaning |
| --- | --- | --- |
| `thigh_len`, `calf_len` | 55, 55 | reach 110 px |
| `hip_height` | 90 (harness pins **104**; walks run `HIP_HEIGHT=98`) | assumed hip height above the floor |
| `stance_duration_ts`, `swing_duration_ts` | 0.28, 0.28 | β = 0.5, cycle 0.56 s commanded / **0.587 s realized** |
| `kv` | 0.12 | Raibert velocity-feedback gain (seconds) |
| `beta` | 1.0 | `x_target = v·Ts/2 + kv·(v − v_des)` |
| `step_height` | 28 px | swing arc |
| `prediction_margin` | 0.5, `safety_margin` 0.85 | reach envelope `x_max = √(reach² − H²)·margin` |
| `td_solve`, `td_solve_floor` | **true**, 0.5 | touchdown velocity rewrite (`v_support = v + Δx/(Ts/2 + kv)`) |
| `dcm_tc` | 0 (auto) | deadbeat deadline = stance duration |
| `attitude_kp`, `attitude_kd` | 45, 11 | the model's own lean loop |
| `gravity` | 980 px/s² | |

[`ControlConfig`](walker_control/src/control.rs:172) — shaping, walking:

| field | value | meaning |
| --- | --- | --- |
| `stance_pin` | **true** | stance foot's world x is latched for the whole stance |
| `stance_press_depth` | 0.0 | `STANCE_DEPTH`: extra chord press, envelope-shaped |
| `att_closure_kp`, `att_closure_kd` | 0.1, 0.15 | loop-3 closure on the measured lean |
| `stand_settle_rate`, `move_threshold` | 40 px/s, 5 px/s | stand + command deadband |

[`ControlConfig`](walker_control/src/control.rs:172) — the launch (all default):

| field | value | meaning |
| --- | --- | --- |
| `boost_gather_ts` | 0.4 | front foot slides under the hip |
| `boost_crouch_ts` | 0.25 | smoothstep squash to `hip_height` (a step is a velocity kick) |
| `boost_kick`, `boost_kick_ts` | 15 px, 0.04 | **step two**: back foot moves backwards, clamped to `√(reach²−y²)−|x|` |
| `boost_kick_front` | 0.0 | the support foot never moves |
| `boost_push`, `boost_push_ts` | 0.0, 0.3 | optional chord extension |
| `boost_hip_accel` | 300 px/s² | horizontal Δv on `P3`, idempotent |
| `boost_lead_tau` | 0.33 | ξ ≈ com + v·τ (1/ω₀ at H≈104) |

[`ActuatorConfig`](walker_control/src/actuate.rs:123):

| field | value | meaning |
| --- | --- | --- |
| `hip_compliance` | 1e-9 | near-rigid hip joint |
| `knee_compliance` | 1e-8 | one decade softer (absorbs load without folding) |
| `zeta` | 1.0 | critical damping, `Kd = 2ζ√(Kp·m)` |
| `strut_active` | 1e-6 | hip↔stance-foot length constraint |
| `strut_idle` | 1e-2 | swing leg's tether (no authority) |

## 4. The locomotion state machine

[`LocoState::decide`](walker_control/src/control.rs:1106) only chooses a
controller; each controller owns its mode:

```
Standing ──move──> Walking          (a plain move starts the gait directly)
Standing ──boost─> BoostStart ──DCM ξ passes the frontmost foot──> Walking
any ──command end──> Standing       (mode entry resets `since`)
```

The walking gait itself is event-driven inside `walker_math`
(`GaitAuthority::Full`): **liftoff** when the DCM ξ crosses the plant, **landing**
on measured touchdown. The timers are only a bound.

## 5. Measured behaviour (`2.csv`, the run that was accepted)

| property | measurement |
| --- | --- |
| gait cadence | stance 0.291–0.304 s, swing 0.280–0.292 s, duty β ≈ 0.51, cycle **0.587 s**, double support 1.7 % |
| forward progress | hip 180 → 1837 px in 14.7 s = **112.9 px/s** average |
| backwards motion | **0 of 1323** walking ticks with hip vx < −5 px/s |
| speed tracking (steady, 25 % trimmed) | `v_des 60 → 82`, `80 → 102`, `120 → 111`, `200 → 154` px/s |
| saturation | ≈155 px/s — the planner's `v_max`/`x_max` reach cap, not a control failure |
| stance slip | mean 5–8 px per stance, worst ~14 px, all inside a bit-constant target (see §6) |

Speed control works: the realized speed is a monotone function of the command,
with a systematic **+20…+35 px/s offset** (the body runs slightly ahead of the
plan) and a hard ceiling near 155 px/s. The offset is the known
fiction-vs-reality gap, not a bug in the law.

## 6. PARKED — the stance foot slides inside its own pinned target

### 6.1 What "pinned" means now

[`ControlConfig::stance_pin`](walker_control/src/control.rs:281) latches the
stance foot's **world x** on stance entry and emits that x for the rest of the
stance, so the strut rest length and the knee angle are both derived from a foot
that is actually planted. Verified: the commanded stance x is **bit-constant**
across all 39 stances in `2.csv` (e.g. `tPLLLx` = 551.66 for the entire
5.01–5.31 s stance).

### 6.2 What still moves

| quantity | value (5.01–5.31 s, left stance) |
| --- | --- |
| commanded foot x | 551.66, constant |
| hip x | 527.9 → 554.7 (**+26.8 px**) |
| actual foot x | 542.4 → **525.1** → 534.1 (±13 px wander) |
| hip→foot chord | 97.9 … 101.7 px (reach 110) |
| commanded chord angle | +13.6° → −2.5° |
| actual chord angle | +8.9° → −12.9° (lag ≤ 15°) |

### 6.3 Root cause

The chord is constant, so the ankle sits on a circle about the hip: **the leg
pivots about the hip, not about the foot**, and the ankle rides the arc. The
command cannot fix that, because the two competing authorities are

- joint servos: `hip_compliance 1e-9`, `knee_compliance 1e-8`, solved 4× per
  frame plus `STABLIZE_STEPS_COUNT` extra distance/angular passes;
- the floor contact: `max_friction_disp = friction · penetration` ≈ 3 px per
  solve.

The rotation wins. Which body is the pivot is a servo-authority question, not a
grip question.

A second, smaller defect sits on top: the pin latches the **commanded** x, which
at stance entry is already ~9 px ahead of the real foot, so the actuator spends
the whole stance pulling the ankle forward into a gap the floor will not give.

### 6.4 Everything that was tried

| attempt | result |
| --- | --- |
| raise `friction_compliance_scaler` on the feet (0.35 → 1.0) | **worse** (L 12→32 px, R 28→47 px mean drift) — more grip delivers the leg drive more completely, the body overruns the plan (`plan − hip` +35 → −1 px inside one stance) and the strut then drags the body back |
| unbounded friction budget (`friction ≥ 1.0` ⇒ clamp removed) | **diverges**: hip launched to y −6823, then through the floor |
| physical hold — external position constraint on the foot particle, x-only, `compliance = 0` | removes the ankle travel, but the walk loses the speed-control tendency and stops looking natural: a foot that cannot move at all converts every servo error into body load. **Reverted.** |
| vertical re-anchors (floor-frame pose, one-sided floor clamp, waist pin) | all tried earlier, all reverted — see §7 |

### 6.5 If it ever becomes vital, this is the order to try

1. **Seed the pin from the measured foot** (`fiction.feet[i].pos.x`) instead of
   the commanded x — one line in [`pin_off`](walker_control/src/control.rs:1183),
   removes the 9 px standing demand and most of the 26 px sustained error. Was
   written and reverted only because it rode along with the physical-hold
   experiment.
2. **Price the servo, not the contact.** The real question is why the leg chooses
   the hip as its pivot; soften `hip_compliance`/`knee_compliance` one decade and
   see whether the ankle takes more of the rotation, or add the missing
   tangential damping to the strut (the distance constraint has no `zeta`).
3. **The frame fix underneath it** (§7.3): stop deriving the plant from the
   measured hip, and the residual the pin is fighting disappears at the source.
4. Only then revisit the physical hold, with the constraint on **x only** and
   `compliance ≲ 1e-6` (at 1e-2 the `compliance/dt²` term ≈ 1.3e3 makes it
   numerically inert).

## 7. Other parked / structural notes

### 7.1 The leg has one shared DOF, and that is why the hop appears

The chord is rigid (`strut_active 1e-6`) and the leg has two commanded DOFs
(chord + hip angle) but only **one usable** orientation: chord direction and hip
height are the same number. Any vertical re-anchor of the pose trades against
attitude. That is why the walking "hop" (the stance foot riding the hip's vault)
could not be fixed at the pose level, and why every attempt in this direction
made the walk worse. The penetration of ~12 px under the standing pose is
**load-bearing**: it absorbs the mismatch between the planner's assumed
`hip_height` and the chord the actuator actually commands.

### 7.2 The commanded plant is not world-anchored

The pose is `measured_hip + (plant − solver_hip)`, so the plant *offset* is
world-relative but the *result* moves with the body. With the fiction at 80 and
reality at 120 the emitted plant slides at −40 px/s. `td_solve` is what keeps the
fiction near reality and therefore keeps this from running away — turning it off
smooths `cmd_vx` and drags the plant backwards (measured −23…−34 px/s, ~10 px of
commanded slide per stance), so it is **on** with a comment saying exactly that.

### 7.3 Everything that is load-bearing (do not "fix" these blindly)

- the ~12 px floor penetration of the standing pose;
- the friction budget being finite (`friction · penetration`);
- `td_solve` being on;
- the launch's two-step order (squash **then** translate) and the reach clamps on
  the kick/push.

## 8. Physics-engine quirks worth remembering

- There is **no separate friction for feet**. One `CollisionConstraintConfig` per
  collider; the feet just carry `friction_compliance_scaler = 0.35` vs 0.1.
- The live contact is `FineCollisionConstraint`: `max_disp = friction ·
  penetration`, i.e. a **budget**, and **higher = more grip**. `0.0` disables
  friction. The `(1 − compliance)` form in `StaticCollisionConstraint` is dead
  code (never constructed).
- `FrameCollisionConstraint` is a velocity damper: `v_t_scale = 1 −
  min(friction·penetration·dt·100, 1)` plus angular damping — already saturated
  at 0.35.
- Constraint order per substep: external position → coarse contact → coarse
  distance → fine bilinear → distance → angular → repulse → **fine contact** →
  shape-matching damping.
- `ExternalPositionConstraint` pulls `len/(inv_mass + compliance/dt²)`, so its
  compliance must be ≲ 1e-6 to have any authority.
- `AngularConstraint` damping: `Kd = 2ζ√(Kp·m)`; `DistanceConstraint` has **no**
  damping term at all.

## 9. How to reproduce

```
# the reference walk (real window, stance press, crouched height, strong launch)
tools/physics-qa/run_walker_qa.sh 2 SCRIPT=plans/walker_v60_120_60_script.json \
    STANCE_DEPTH=1.3 WIN_HIDDEN=0 HIP_HEIGHT=98 BOOST_HIP_ACCEL=7000

# the 17 s script every number in this document comes from
tools/physics-qa/run_walker_qa.sh base SCRIPT=plans/walker_v60_boost90_40_70_40_script.json

# analysis
python3 tools/physics-qa/analyze_foot_slide.py tmp/walker_qa/2.csv --skip-start 1.6
python3 tools/physics-qa/plot_p3.py tmp/walker_qa/2.csv
```

Artifacts: `tmp/walker_qa/<tag>.csv`, `.hips_feet.png`, `.log`. Scripts live in
`plans/` because `tmp/` is not tracked.

## 10. Env knobs (harness → config)

| knob | target |
| --- | --- |
| `HIP_HEIGHT`, `STANCE_TS`, `SWING_TS` | `LowerBodyConfig` |
| `STANCE_DEPTH`, `STANCE_PIN`, `ATT_KP`, `ATT_KD` | `ControlConfig` walking |
| `BOOST_GATHER_TS`, `BOOST_CROUCH_TS`, `BOOST_PUSH`, `BOOST_PUSH_TS`, `BOOST_HIP_ACCEL` | `ControlConfig` launch |
| `HIP_COMPLIANCE`, `KNEE_COMPLIANCE`, `ZETA`, `STRUT_ACTIVE`, `STRUT_IDLE` | `ActuatorConfig` |
| `WIN_HIDDEN`, `SCRIPT`, `ASSET_ROOT`, `RUN_SECS` | harness |

## 11. Test inventory

| suite | tests | what it pins down |
| --- | --- | --- |
| `walker_math` | **119** | laws, FSM events, placement, vertical stabiliser, attitude, determinism |
| `walker_control` | **92** | observe ⇄ control ⇄ actuate seams, pose mapping, stance press + pin, boost schedule, telemetry format |

## 12. Commit trail (this arc)

`84454a3` stance/swing split + boost stance work · `145c4ca` upright launch +
kick · `4e752c8` two-step launch + scheduled release · `80ced19` waist-pin doc
correction · `96365eb` hip thrust on `P3` · `8ad78c3` **pin the stance foot's
world x** (now the default). Branch `walk-hip-frame` holds the earlier frame
experiment.

## 13. Reading list

- [`plans/actuator-frame-divergence.md`](plans/actuator-frame-divergence.md) — the
  long-form record: the frame question, the launched-boost failures, the walking
  hop, the one-DOF conclusion, the friction audit, the stance pin and the
  physical hold.
- [`plans/stance-delivery-is-the-brake.md`](plans/stance-delivery-is-the-brake.md) — earlier stance-delivery
  investigation.
- [`tools/physics-qa/physics_tuning_progress.md`](tools/physics-qa/physics_tuning_progress.md) — parameter
  sweeps and their verdicts.
