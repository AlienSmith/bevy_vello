# Stance delivery is the brake — why the plateau cannot be tuned away (2026-10-30)

## The question this answers

The body cannot be slowed from ~200 px/s to the commanded 90–120 px/s by *any*
setting of `HIP_V_MIX`, `ADAPT_TC`, `V_MEAS_TAU` or `DCM_TC`. Earlier sessions
framed this as "too little mix cannot decelerate, too much mix cannot
accelerate". Those tuning results are **retracted** (see §6). This document
states the mechanism, measured on the only upright runs, and it is a
**delivery** problem, not a **law** problem.

Reproduce every number here with:

```
python3 tools/physics-qa/analyze_stance_delivery.py \
    tmp/headless/walks/walk20_mix07_s90.csv
```

## 1. The stance clock is correct (the hypothesis under test was not it)

`walk20_mix07_s90` (`GAIT_AUTHORITY=timer`, the upright regime), 70 stances:

| quantity | measured | nominal |
|---|---|---|
| stance window median | 0.289 s | 0.28 |
| swing window median | 0.278 s | 0.28 |
| double support | 5.1 % | — |
| airborne | 0.0 % | — |

[`plan_target_x()`](../examples/game_lib/src/lower_body/resolver/placement.rs:159)
*is* re-evaluated every swing tick with the shrinking `leg.phase_timer`, so the
"remaining swing time = 0" evaluation does happen each frame. **The stance time
is not miscalibrated** — the foot is down for the configured window.

## 2. The swing delivers only 74–80 % of its target advance

Per swing, ground frame:

| | left | right |
|---|---|---|
| target advance (px) | 98.8 | 91.4 |
| target ground speed (px/s) | 355.5 | 328.9 |
| **actual foot advance (px)** | **74.2** | **66.4** |
| **actual foot speed (px/s)** | **267.2** | **239.0** |
| **delivery ratio** | **0.80** | **0.74** |
| shortfall at touchdown (px) | **55.3** | **58.2** |
| peak foot speed (px/s) | 739 | 762 |

Peak instantaneous foot speed is ~740 px/s, so this is **not** a rate cap
(`LEG_MAX_ANG_RATE` defaults to `1e6`, i.e. disabled). The deficit is that the
Hermite advance segment asks for more displacement in 0.28 s than the leg PD
resolves. Lag vs target by swing decile:

```
L  32.9  4.7  -15.1  -8.4  10.2  24.0  47.2  59.3  65.3  61.1
R  31.6  -0.1 -14.4  -5.9  13.6  27.1  50.9  62.6  64.7  60.0
```

Note the shape: the foot is *ahead* of target through the advance phase
(deciles 1–4 negative), then falls ~60 px *behind* by touchdown. The swing runs
out of time, not out of speed.

## 3. The deficit becomes a **plant error**: the foot lands 24–31 px too far forward

At stance start, the hip→foot offset should equal the law's demand. It does not:

| | left | right |
|---|---|---|
| relative offset at stance start (px) | **+32.7** | **+35.4** |
| offset the deadbeat law demands (px) | −1.9 | +2.3 |
| **plant error (actual − demanded)** | **+30.6** | **+23.5** |

Sign convention: positive = foot ahead of the hip, which **accelerates** the
body. So every stride begins with a 24–31 px forward error that the law never
asked for, and the law then spends the stance trying to remove it.

That error is almost perfectly explained by the swing shortfall:

| correlation | left | right |
|---|---|---|
| corr(plant error, swing shortfall at TD) | **+0.946** | **+0.932** |
| corr(plant error, over-speed `v0 − v_des`) | **+0.893** | **+0.922** |

**This is the whole loop**: over-speed ⇒ the swing has less time per unit of
ground travel ⇒ it under-delivers ⇒ the foot plants ahead of the demand ⇒ the
foot ahead of the hip pushes the body faster ⇒ more over-speed.

## 4. And the supposedly pinned stance foot slides forward, so the pendulum never brakes

| | left | right |
|---|---|---|
| stance foot drag (physics px) | **+24.5** | **+26.5** |
| hip advance over stance (px) | +45.7 | +47.9 |
| corr(drag, swing shortfall) | **+0.66** | **+0.69** |
| peak stance foot speed (px/s) | 248 | 255 |

[`analyze_foot_slide.py`](../tools/physics-qa/analyze_foot_slide.py) already
gates this: `drift mean=45.0px max=71.7px` → `RESULT: SLIDING` against a 2.0 px
gate. The slide is bursty and front-loaded (foot 99/149 px/s early vs 73/74 px/s
late in stance), i.e. the leg PD dragging the foot toward a target that is
chasing the advancing hip — not passive friction slip.

So the "pinned" foot moves ~25 px of the ~46 px the hip moves. The leg
therefore never closes on the hip enough to load it backward.

## 5. The pendulum is inert — this is the plateau

| | left | right |
|---|---|---|
| relative sweep over stance (px) | **−18.5** | **−11.6** |
| LIPM prediction `Δv = −ω·sinh(ωT)·rel_sweep` | **+47.8 px/s** | **+29.5 px/s** |
| measured median `Δv` | −12.9 px/s | +6.7 px/s |
| delivered fraction of the law's demand | **+0.01** | **−0.01** |

Sign note: `rel_sweep < 0` means the foot ends *nearer* the hip than it started —
the classic accelerating stance, and the LIPM correctly predicts **+47.8 px/s**
of acceleration from it. The measured change is ~0 to +7 px/s, i.e. the LIPM's
own prediction is ~4× larger than what the physics delivers, **in the
accelerating direction**. The body is fighting an acceleration the drag creates.

Correlations over all 70 stances:

```
corr(rel_sweep, Δv)   L +0.06   R −0.13
corr(hip_adv,   Δv)   L −0.03   R +0.02
corr(stance T,  Δv)   L +0.22   R +0.15
```

**The delivered velocity change has no correlation with any state the placement
law is built on.** The law's demand is ≈ ±2 px, the actual plant offset ≈ +33 px,
and delivered fraction ≈ 0.01. The deadbeat law is not weak — it is *unconnected*.

## 6. Retractions

- Every "ADAPT_TC / mix / τ improves the speed error" number from the previous
  sweep was measured on `GAIT_AUTHORITY=full` runs whose body had **collapsed to
  the floor** (hip-above-foot 105 → 59 → −26 px at t≈12; hip→foot distance
  reaching 108.0 / 107.5 px against a 110 px leg). Retracted.
- `ADAPT_TC` is **byte-identical** under `GAIT_AUTHORITY=timer`
  (`atc11_timer` vs `walk20_mix07_s90`: 1936 / 1936 identical rows), because
  `swing_wait` only accumulates under contact-gated landing. It can therefore
  *only* act in the collapse-prone regime. No adaptive-horizon tuning can help
  the upright walk.
- An earlier draft of this document stated the LIPM prediction as −53.8 px/s
  (braking). That was my own sign error: with `rel_sweep ≈ −15 px` the pendulum
  *accelerates*. Corrected above; the conclusion is unchanged and in fact
  sharper, because the drag is now seen to *inject* the acceleration rather than
  merely cancel the braking.

## 7. The missing physics

[`drive_hip_pin()`](../examples/game_lib/src/lower_body/hip_pin/drive.rs:573)
rebuilds `left_leg.target` / `right_leg.target` **every frame** from the
resolver's `pose.left_foot`, which is generated from the *moving* hip, at
`LEG_COMPLIANCE=1e-8` with the joint rate limiter off. The stance leg is
therefore a position servo chasing a moving hip, not a rigid strut against the
ground. Its implicit forward leak is ~25 px/stance ≈ 85 px/s — comparable to the
46 px/stance the hip moves, which is why the relative sweep stays near zero.

`HIP_V_MIX` changes **what the law asks for**. It cannot change what the
delivery channel can produce. That is why the failure flips from "cannot
decelerate" to "cannot accelerate" with the sign of the command but never
disappears: the leak is signed with the motion, in both directions.

## 8. Predicted behaviour under the fix (falsifiable)

If the stance foot is genuinely pinned (drag → ~0), then per stance:

- relative sweep becomes ≈ +46 px (the full hip advance)
- LIPM predicts ≈ −46 px/s of **braking** per stance
- over 2 strides that is ~120 px/s of authority, enough to go from 200 → 90

Success criteria for the next experiment, on numbers rather than plots:

| metric | now | target |
|---|---|---|
| stance drag | 24.5 / 26.5 px | **< 5 px** (`analyze_foot_slide.py` gate 2.0) |
| plant error at TD | +30.6 / +23.5 px | **< 10 px** |
| corr(rel_sweep, Δv) | +0.06 / −0.13 | **> +0.5** |
| delivered fraction | 0.01 | **> 0.5** |

## 9. The decisive experiment (stance delivery, not another mix sweep)

Lever order, cheapest first, all measured with
[`analyze_stance_delivery.py`](../tools/physics-qa/analyze_stance_delivery.py):

1. **Swing pacing** — the swing is time-starved, not speed-capped (peak 740 px/s
   against a 267 px/s average). Either lengthen `SWING` or make the Hermite
   advance segment reach `swing_to` (currently derivative `1.20·dx` overshoots
   early and lands short).
2. **`LEG_COMPLIANCE` down** (5e-9 → 1e-9, via
   [`sweep_leg_compliance.py`](../tools/physics-qa/sweep_leg_compliance.py)).
3. **`LEG_MAX_ANG_RATE` engaged** (finite, e.g. 30 rad/s) so the PD cannot drag
   the stance foot.
4. Optionally a friction/contact gate on the stance foot.

Note that (1) and (2)–(4) are coupled: fixing the swing shortfall reduces the
plant error, which reduces the drag the PD has to fight. Either alone should
move `corr(rel_sweep, Δv)`; if neither does, the leak is in the
`transfer_segment` bridge seed and that becomes the next thing to instrument —
do **not** resume mix tuning.
