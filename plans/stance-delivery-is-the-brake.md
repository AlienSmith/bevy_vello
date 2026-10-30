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

The commanded stance foot is parked ~22 px above the real collider. The leading
candidate cause is specific and measurable: [`spawn_collision_floor`](../examples/collision_detection/examples/headless_arm_pd/collision.rs:80)
places the floor at `hip_y − pin.stance_height()`, and the run log shows

```
live_stance_h=106.00   resolver_hip_height=90.00
```

`standing_height` is latched from the **straight-legged seed pose (106 px)**
while the resolver is configured at **90 px** — a **16 px** disagreement, which
would put the gait's virtual floor 16 px ABOVE the real collider.

### 6a. Why the penetration test above cannot decide contact (retracted)

**The `penetration > 0` test compares the foot PARTICLE CENTRE to the floor
surface — and the foot particle is a point at the centre of a CAPSULE.** The
collider has non-zero radius, so a particle centre 6.2 px above the surface can
be in solid, load-bearing contact: the cap is already touching the plane.

This invalidates the headline claim of the previous commit:

- **"Both feet are in the air ~93 % of the time" is RETRACTED.** It only says the
  particle *centre* is above the plane.
- **The derived "we land the feet too hard / the pose is too bouncy" reading is
  RETRACTED too** — nothing measured supports it, and the hover→bounce story was
  built on the invalid number. (The user's objection to it was correct.)
- **The slide/friction-budget ratios (0.48 / 0.56) are approximations only**,
  because the `0.9·penetration` budget uses that same suspect penetration. They
  are not evidence either way.
- The 16 px floor-reference disagreement remains a *candidate*, not a cause:
  with a capsule radius on the order of the 6.2 px gap it may be entirely
  harmless.

Also retracted: §9 item 1's success gate "stance frames with contact 7 % →
> 70 %", because 7 % was measured with the invalid test.

What **survives**, because it does not depend on the contact test at all:

- friction is enabled (strength 0.9, scaler 0.1, feet 0.35);
- the stance foot slides ~25 px per stance, front-loaded;
- the swing delivers only 74–80 % of its target advance (55–58 px short);
- the foot lands ~33 px BEHIND the hip and the LIPM therefore predicts
  **acceleration**, of which ~10 % is realised.

So the current best statement is: **not a friction problem, and the contact
fraction is unknown.** It is a swing-reach problem (~55 px) with an unexplained
loss of pendulum transmission, on a contact state we have not yet measured.

## 6b. The measurement that settles it: emit collision events to the CSV

Contact truth already exists in the collision layer, and the pin already
consumes it (`ground_support` → `left_stance_loaded`/`right_stance_loaded` in
[`drive_hip_pin`](../examples/game_lib/src/lower_body/hip_pin/drive.rs:641),
and `gait_authority >= GaitAuthority::Contact`). The resolver *uses* it; the
telemetry does not *record* it. Every contact argument in this document — mine
and any future one — is therefore a geometric proxy. Stop guessing from y and
log the ground truth.

| proposed column | source | why |
|---|---|---|
| `contact_l`, `contact_r` | the **debounced** per-foot `grounded` flag the pin already computes | 1 = the physics says this foot is on the floor |
| `hit_l`, `hit_r` | whether that foot's collider appears in `CollisionEventBatch` this tick | separates "the physics is resting on it" from "a collision was resolved" |
| `n_contacts` | `batch.entries.len()` | total collision events this tick |
| `pen_l`, `pen_r` | penetration/normal carried by the event, if available | tells a skim from a load |

The **cheap version is the first row alone**: `contact_l` / `contact_r` is a
two-line addition next to the existing `gait_l`/`gait_r` writes in
[`sample.rs`](../examples/collision_detection/examples/headless_arm_pd/sample.rs:129),
plus a reader in
[`analyze_stance_delivery.py`](../tools/physics-qa/analyze_stance_delivery.py:1)
that bins stance frames by `contact` / `no-contact` instead of by particle y.

That single column makes every open question here decidable:

- is the stance foot genuinely on the ground, or skimming?
- does the ~25 px slide happen **while loaded**, or only while losing contact?
- is the LIPM reaction ~10 % because there is no contact, or because the leg
  servo (`LEG_COMPLIANCE=1e-8`, targets rebuilt from the moving hip every frame)
  is too soft to transmit it?

Decisive outcome either way — no more inference from geometry:

- if `contact_l/r` show the foot down for **~95 %** of stance while the y-test
  said 7 %, the "hover" was a **capsule-radius artefact**, the 16 px floor
  reference is a non-issue, and the real story is the swing reach (§2–3);
- if the flags agree at **~7 %**, the hover is real, and the 16 px floor
  disagreement (§6) becomes the thing to fix.

**Rule going forward: no contact claim from particle y.** Use the collision
event / debounced flag or say it is unknown.

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

1. **Soft-body / capsule geometry (now the top open question).** The foot
   particle is a point at the centre of a capsule, and the capsule deforms on
   contact, so particle-centre-vs-surface is **not** contact depth (§6a). Until
   the collision events are logged, the contact fraction is unknown and no
   hover/bounce conclusion can be drawn. See §6b for the proposed columns.
2. **Whether the 16 px floor disagreement matters at all.** It is a candidate
   cause of the commanded-foot 22 px gap, but with an unknown capsule radius it
   may be harmless, and `thrust.stance_extend` (applied only when
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

1. **Log contact truth** (§6b) — emit `contact_l` / `contact_r` (the pin's
   debounced `grounded` flag) into the telemetry, and teach
   [`analyze_stance_delivery.py`](../tools/physics-qa/analyze_stance_delivery.py:1)
   to bin stance frames by it. This is the gate for everything else: it decides
   whether the hover is real or a capsule artefact.
2. **Re-measure the slide against contact truth.** Success: fricional slide
   while `contact == 1` is small, OR the slide turns out to happen only in
   `contact == 0` frames (in which case the "stance" clock and the real foot
   contact disagree, which is a different bug).
3. **Swing time budget** — lengthen `SWING` or make the Hermite advance segment
   actually reach `swing_to`. Success: delivery ratio 0.74–0.80 → **> 0.95**,
   shortfall 55–58 px → **< 10 px**. This one is independent of contact and can
   run in parallel.
4. **Only if** step 1 shows `contact == 1` for most of stance AND step 3 has
   lifted the delivery ratio: re-check the ~10 % LIPM transmission (§8.3). If it
   is still ~10 % with a loaded foot, the leak is the per-frame rebuild of both
   leg targets from the moving hip at `LEG_COMPLIANCE=1e-8` — the stance leg is
   a position servo chasing the hip, not a strut. That is the next place to
   instrument.
5. Only after 1–4: revisit whether the placement law needs a different
   reference point (§8.4).

Do **not** resume `HIP_V_MIX` / `DCM_TC` / `ADAPT_TC` sweeps until contact exists
— those knobs act on a law whose output cannot reach the physics, and any
"improvement" measured without contact is, as already demonstrated, an artifact.
