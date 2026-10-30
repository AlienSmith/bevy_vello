# Stance delivery is the brake — why the plateau cannot be tuned away (2026-10-30)

## The question this answers

The body cannot be slowed from ~200 px/s to the commanded 90–120 px/s by *any*
setting of `HIP_V_MIX`, `ADAPT_TC`, `V_MEAS_TAU` or `DCM_TC`. Earlier sessions
framed this as "too little mix cannot decelerate, too much mix cannot
accelerate". Those tuning results are **retracted** (§7). This document states
the measured mechanism, on the only upright runs, and it is a **delivery**
problem, not a **law** problem.

Reproduce with:

```
python3 tools/physics-qa/analyze_stance_delivery.py \
    tmp/headless/walks/walk20_mix07_s90.csv
```

## 0. Sign conventions (this bit me twice, so it is stated once, up front)

The resolver frame is **y-down** and the walk goes in **+x**.

```
rel  = hip_x − foot_x        rel > 0  ⇒ foot is BEHIND the hip
                                         (LIPM ACCELERATES: body falls forward)
                                          rel < 0  ⇒ foot is AHEAD of the hip
                                         (LIPM BRAKES)
Δv   = v_end − v_start
shortfall (world frame) = swing target x − actual foot x     (>0 ⇒ foot landed SHY of target)
plant error = rel at stance start − rel the law demands      (>0 ⇒ foot further BEHIND than wanted)
```

Earlier drafts of this document said the foot lands "ahead" — **wrong**. `rel`
is positive throughout, so the **foot lands ~33 px behind the hip**. That is
literally the user's original observation ("the feet are always behind the hip
when it should decelerate"), and it is now measured.

## 1. The stance clock is correct (the hypothesis under test was not it)

`walk20_mix07_s90` (`GAIT_AUTHORITY=timer`, the only upright regime), 70 stances:

| quantity | measured | nominal |
|---|---|---|
| stance window median | 0.289 s | 0.28 |
| swing window median | 0.278 s | 0.28 |
| double support | 5.1 % | — |
| airborne (both feet in swing) | 0.0 % | — |

[`plan_target_x()`](../examples/game_lib/src/lower_body/resolver/placement.rs:159)
is re-evaluated every swing tick with the shrinking `leg.phase_timer`, so the
"remaining swing = 0" evaluation does happen each frame. **The stance time is
not miscalibrated.** The stance arm of the investigation is closed.

## 2. The swing delivers only 74–80 % of its target advance

Per swing, ground frame:

| | left | right |
|---|---|---|
| target advance (px) | 98.8 | 91.4 |
| target ground speed (px/s) | 355.5 | 328.9 |
| **actual foot advance (px)** | **74.2** | **66.4** |
| **actual foot speed (px/s)** | **267.2** | **239.0** |
| delivery ratio | 0.80 | 0.74 |
| shortfall at touchdown (px) | **55.3** | **58.2** |
| peak foot speed (px/s) | 739 | 762 |

Peak foot speed ~740 px/s against a 267 px/s average, so this is **not** a rate
cap (`LEG_MAX_ANG_RATE` defaults to `1e6`, disabled). The swing is
**time-starved**: the Hermite advance segment asks for more displacement in
0.28 s than the leg resolves. Lag vs target by swing decile (positive = short):

```
L  32.9  4.7  -15.1  -8.4  10.2  24.0  47.2  59.3  65.3  61.1
R  31.6  -0.1 -14.4  -5.9  13.6  27.1  50.9  62.6  64.7  60.0
```

The foot is *ahead* of target through the advance phase (deciles 1–4 negative),
then falls ~60 px short by touchdown. It runs out of time, not of speed.

## 3. The unresolved contradiction: shortfall (55 px) and plant error (30 px) are different frames

The two numbers that looked self-contradictory are both true, and they measure
different things:

| quantity | value | frame |
|---|---|---|
| shortfall | 55.3 / 58.2 px | **world**: swing target x − actual foot x (foot landed shy) |
| `rel` at stance start | **+32.7 / +35.4 px** | **hip-relative**: foot is 33 px *behind* the hip |
| `rel` the law demands at stance start | −1.9 / +2.3 px | foot essentially under the hip |
| plant error = actual − demanded | **+30.6 / +23.5 px** | foot 24–31 px further behind than wanted |

They coexist because the **swing target and the stance demand are different
points on the same stride**:

- the swing target is a *world* position, clamped to `hip ± x_max` (≈48 px),
  re-planned every frame, so its total ground advance is the ±x_max sweep ≈96 px
  — which matches the measured target advance of 91–99 px;
- the stance demand, re-evaluated at `t_flight = 0`, wants the foot ~under the
  hip.

So the swing aims ~+48 px ahead of the hip, lands 33 px ahead of it, i.e. misses
by 55 px and is still 33 px short of being *under* the hip. There is no sign
error; there is a **reference-point mismatch** between the two evaluations of
the same law.

The plant error is almost perfectly explained by the swing shortfall:

| correlation | left | right |
|---|---|---|
| corr(plant error, swing shortfall) | **+0.946** | **+0.932** |
| corr(plant error, over-speed `v0 − v_des`) | **+0.893** | **+0.922** |

Over-speed ⇒ less time per unit of ground travel ⇒ the swing under-delivers ⇒
the foot lands behind the hip ⇒ a foot behind the hip **accelerates** the body
⇒ more over-speed. That is the closed loop.

## 4. And the "pinned" foot slides, so the pendulum load is lost

| | left | right |
|---|---|---|
| stance foot drag (px) | **+24.5** | **+26.5** |
| hip advance over stance (px) | +45.7 | +47.9 |
| corr(drag, swing shortfall) | **+0.66** | **+0.69** |
| peak stance foot speed (px/s) | 248 | 255 |

[`analyze_foot_slide.py`](../tools/physics-qa/analyze_foot_slide.py) reports
`SLIDING`: `drift mean=45.0px max=71.7px` against a 2.0 px gate. The slide is
bursty and front-loaded (foot 99/149 px/s early vs 73/74 px/s late in stance).

## 5. The pendulum is disconnected, in the accelerating direction

`rel_mean` over stance is **+43.4 / +38.6 px** (foot behind the hip). With
`ω = 3.25 rad/s`, `T = 0.29 s`:

| | left | right |
|---|---|---|
| LIPM `Δv = v(cosh−1) + ω·sinh(ωT)·rel` | **≈ +163 px/s (accelerating)** | **≈ +139 px/s** |
| (core term `ω·sinh(ωT)·rel`) | +149 | +125 |
| measured `Δv` | −12.9 | +6.7 |
| delivered / predicted | **0.09** | **−0.05** |

Correlations over all 70 stances:

```
corr(rel, Δv)      L +0.061   R −0.129
corr(hip_adv, Δv)  L −0.028   R +0.019
corr(T, Δv)        L +0.222   R +0.146
delivered fraction L +0.01    R −0.01
```

Two facts, both bad:

1. **The geometry is the acceleration geometry.** Feet behind the hip ⇒ even a
   perfect pendulum would accelerate. Braking needs the foot *ahead* of the
   hip, which never happens because the swing cannot reach it (§2–3).
2. **Even the acceleration the LIPM promises is not delivered** (+163 predicted
   vs ~0 measured). So the stance leg is not transmitting the pendulum reaction
   at all — the law's demand is genuinely unconnected to the realised `Δv`.

This is why the failure sign-flips with the command instead of disappearing: the
placement law changes what it *asks for*, while the realised stance geometry
stays "feet behind the hip, sliding".

## 6. Is it a friction problem? No — and here is the number that says so

Friction is **enabled**: every collider in the asset carries
`friction_compliance_scaler = 0.1` (feet 0.35), and the static-collision solver
(`constraints.rs:292`) uses `friction_strength = 0.9`, budgeting
`max_friction_disp = 0.9·penetration` per solve.

Measured on `walk20_mix07_s90` (floor surface at Vello y **278.97**, from the run
log; 90 Hz ⇒ 0.0111 s/frame):

| | left | right |
|---|---|---|
| stance frames actually **inside** the floor (`penetration > 0`) | **7.0 %** | **5.9 %** |
| penetration when in contact | 1.21 px | 1.42 px |
| **friction budget** = 0.9·pen | 1.09 px/frame ≈ **98 px/s** | 1.28 px/frame ≈ **115 px/s** |
| slide **while in contact** | 0.52 px/frame ≈ **47 px/s** | 0.72 px/frame ≈ **65 px/s** |
| slide **while NOT in contact** | 1.28 px/frame ≈ **115 px/s** | 1.19 px/frame ≈ **107 px/s** |
| slide / budget when in contact | **0.48** | **0.56** |

Two conclusions:

1. **Friction is not saturated.** Where contact actually exists, the foot slides
   at ~50 % of what friction would allow. Making friction stronger cannot help.
2. **Most "stance" frames have no contact at all** (93–94 % of them), and the
   slide is *larger* when there is no contact (115 px/s vs 47 px/s). Air has no
   friction coefficient. That is the dominant term.

Why no contact: the feet **hover above the floor**.

| | measured |
|---|---|
| stance foot y (median) | 272.7 / 272.8 |
| floor surface y | **278.97** |
| **foot above surface** | **6.2 / 6.3 px** |
| commanded foot y (median) | 257.4 / 257.1 |
| **commanded foot above surface** | **21.6 / 21.9 px** |

The commanded stance foot is parked ~22 px above the real collider. Root cause
is measurable and specific: [`spawn_collision_floor`](../examples/collision_detection/examples/headless_arm_pd/collision.rs:80)
places the floor at `hip_y − pin.stance_height()`, and the run log shows

```
live_stance_h=106.00   resolver_hip_height=90.00
```

`standing_height` is latched from the **straight-legged seed pose (106 px)**
while the resolver is configured at **90 px** — a **16 px** disagreement. The
virtual floor the gait plans against therefore sits 16 px ABOVE the real
collider, the leg PD parks the foot in that gap, and the "stance" leg is
standing on nothing.

So: **the slide is not a friction problem. It is a floor-reference problem
(16 px) plus a swing-reach problem (~55 px).** Friction would only matter once
the foot is actually on the ground.

## 7. Retractions

- Every "ADAPT_TC / mix / τ improves the speed error" number from the previous
  sweep was measured on `GAIT_AUTHORITY=full` runs whose body had **collapsed to
  the floor** (hip above floor 108–111 px through t≈10, then 98, 70, **9**,
  30, 99, 74, 63, 75, 66, 65, 11 px — a floor crawl from t≈12). Retracted.
- `ADAPT_TC` is **byte-identical** under `GAIT_AUTHORITY=timer`
  (`atc11_timer` vs `walk20_mix07_s90`: 1936 / 1936 rows), so it can only act in
  the collapse-prone regime. No adaptive-horizon tuning can help the upright walk.
- **Sign corrections in this very document**: an earlier draft claimed the foot
  lands *ahead* of the hip (it lands 33 px *behind*) and that the LIPM predicts
  −54/−150 px/s of *braking* (it predicts +163/+139 px/s of **acceleration**). A
  "+163 accel predicted, ~0 delivered" gap is a *stronger* indictment than the
  one I originally wrote, because it shows the stance leg transmits almost none
  of the reaction in either direction.

## 8. Open questions (honest, unresolved)

1. **Soft-body geometry.** The legs are soft-body constraints and the foot
   particle sits at the **centre of a round cap**, and the cap itself deforms on
   contact. My "penetration" is therefore the *particle centre* vs the floor
   surface, not the true contact depth — the round cap means the real gap/hover
   is roughly `cap_radius` smaller than the 6.2 px measured. The 7 %/6 % contact
   fraction and the 0.9·pen budget are **approximations** at best, and the
   friction verdict ("not saturated, 0.48–0.56 of budget") should be re-tested
   against a real contact-depth measure before being relied on.
2. **Whether the 16 px floor disagreement is the whole hover.** It is the
   largest single candidate, but `thrust.stance_extend` (applied only when
   `left_stance_loaded`) also moves the commanded foot down by a few px and was
   not separated out.
3. **Why the LIPM reaction is ~10 % instead of ~100 %.** With the foot hovering,
   the answer may be "there is no reaction to transmit". But if it stays at 10 %
   *after* the foot is genuinely on the ground, the leak is elsewhere
   (`transfer_segment` bridge seed, or the per-frame rebuild of both leg targets
   from the moving hip at `LEG_COMPLIANCE=1e-8`: the stance leg is a position
   servo chasing the hip, not a strut).
4. **Whether the swing target and the stance demand should be the same
   quantity.** §3 shows they disagree by design (±x_max sweep vs ~0). If the
   intended semantics is "the swing plants the foot where the next stance wants
   it", then the two evaluations of `plan_target_x` should be reconciled — that
   is a *law consistency* question, not a tuning question.

## 9. Ordered next experiments (delivery, not mix)

Cheapest first; each gated on numbers, not plots:

1. **Fix the floor reference** — make `standing_height` and the resolver
   `hip_height` agree (or derive the collider from the live foot line). Success:
   median stance foot 6.2 px above surface → **≤ 1 px**, stance frames with
   contact 7 % → **> 70 %**.
2. **Then re-measure the slide.** If the slide drops with contact restored,
   friction was never the issue and §6 stands. If it does not, escalate to
   question 3.
3. **Swing time budget** — lengthen `SWING` or make the Hermite advance segment
   actually reach `swing_to`. Success: delivery ratio 0.74–0.80 → **> 0.95**,
   shortfall 55–58 px → **< 10 px**.
4. Only after 1–3: revisit whether the placement law needs a different
   reference point (§8.4).

Do **not** resume `HIP_V_MIX` / `DCM_TC` / `ADAPT_TC` sweeps until contact exists
— those knobs act on a law whose output cannot reach the physics, and any
"improvement" measured without contact is, as already demonstrated, an artifact.
