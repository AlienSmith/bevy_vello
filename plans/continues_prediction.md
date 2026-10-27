# Continuous Prediction — Per-Frame Swing Trajectory Recalculation

Formatted notes (converted 2026-10-27 from rough capture; original source was a
chat answer about Unity/Godot loops). Companion to
[`dcm-prediction-math.md`](dcm-prediction-math.md) (the placement law and
parameter meanings) and [`swing-phase-timing.md`](swing-phase-timing.md) (the
full plan). Units here: whatever the sim uses; our project runs px with
**100 px = 1 m**, `g = 980 px/s²`.

---

## The idea

Do **not** pick a spot on the ground at liftoff and force the foot there
blindly. Recalculate the DCM every single frame of the swing phase and
smoothly **steer** the foot toward the moving target. Foot placement is a
continuous steering problem, not a single decision made at the start of a
step. By tracking the time left in the air and updating the landing
prediction every frame, the character dynamically adjusts its step mid-air to
match whatever chaos the physics engine throws at the hip.

## The per-frame loop (what runs inside FixedUpdate)

### 1. Track time remaining — `t_remaining`

Keep track of exactly how many seconds are left until the foot is scheduled
to land. If the swing phase lasts 0.4 s, then at liftoff `t_remaining = 0.4`;
ten frames later it might be `t_remaining = 0.25`.

> In our resolver this is `leg.phase_timer` during Swing.

### 2. Recompute the instantaneous capture point every frame

Look at the hip's current position `x_hip` and velocity `v_hip` and compute
the DCM right now with the pendulum constant `ω = √(g/h)`:

```
DCM_current = x_hip + v_hip / ω
```

Because the velocity fluctuates, `DCM_current` jitters and slides along the
floor like a laser pointer. That is expected — it is a *state*, not a target.

### 3. Predict the DCM at landing

Project the DCM forward over the remaining flight time. **Correction vs. the
rough notes** (`DCM·e^{-ω·t}` — wrong sign and missing pivot): under the LIPM
the DCM *diverges from the currently planted foot* `p` exponentially:

```
DCM_landing = p + (DCM_current − p) · e^{+ω · t_remaining}
```

- `+` sign: an uncaught imbalance grows, it does not shrink.
- `p` (the stance foot x): the pendulum pivots there; only the *offset from
  the pivot* propagates. Measuring the DCM against the world origin conflates
  balance state with position.

As `t_remaining → 0`, the prediction converges to the actual state at the
moment of touchdown, which is what makes the step land stable. (Our variant
skips the fixed-point iteration of plan §4: the swing timer *is* the touchdown
estimate, so a single evaluation per frame, re-done every frame, closes the
loop itself.)

### 4. Steer the foot smoothly to the landing DCM — do not snap

Do not teleport the foot to `DCM_landing` each frame; the leg will shake
violently. Use interpolation or a spring-damper to continuously steer the
foot's horizontal target toward the shifting point:

```
frame 1:  target here    x
frame 2:  target shifted    x   ← steer swing foot toward new spot
frame 3:  target shifted      x
```

> In our resolver the swing spline already smooths this (the profile s→foot
> map); the replan moves the endpoint `swing_to.x` every frame. A per-frame
> *rate limit* on `swing_to.x` (this steering trick, made explicit) is **not
> yet implemented** — candidate improvement, see §"Mapping" below.

Finally aim the landing point itself with the placement offset — the DCM must
land *inside* where the foot will plant, i.e. apply the velocity-tracking
law at the landing state (see `dcm-prediction-math.md` §4):

```
x_foot = DCM_landing − v_des / (T_conv · ω)
```

## Three game-physics safety filters (apply before the target reaches the leg)

Even with per-frame updates, physics engines are chaotic. These three go
**last** in the pipeline:

### A. Terminal freeze

Stop updating the landing target during the last 10–15 % of the swing
(`t_remaining < ~0.05 s`). Changing foot placement a millisecond before
impact demands torques the leg cannot deliver — it produces a violent glitch.
**Lock the target just before landing.**

> **New idea for our pipeline.** We currently re-plan until touchdown. Our
> measured symptom — swing gates failing `NO_ADVANCE` on late swings, feet
> chasing moving targets — is exactly what a terminal freeze prevents.
> Simplest form: latch `swing_to.x` when `phase_timer < 0.15·T_swing`.

### B. Kinematic envelope clamp

If the distance from hip to `DCM_landing` exceeds what the leg can physically
reach, clamp the target to maximum leg extension.

> Already ours: every `plan_target_x` result is clamped to `hip ± x_max`, with
> `capture_saturated` raised when the *unclamped* demand was outside reach
> (the "uncapturable → fall path" signal, plan §4/§8).

### C. Velocity deadband

If the player is trying to stand still but `v_hip` micro-oscillates from
solver jitter, the DCM dances. Below a small speed threshold (e.g.
0.05 m/s = 5 px/s at our scale), force the target DCM to sit exactly
underneath the hip.

> Partially ours: `SWING_TRIGGER_SPEED` gates *stepping*, and the standstill
> path plants under the hip. The deadband applied to *placement* (zeroing the
> DCM term at low speed rather than the step gate) is a distinct, cheap
> addition — mostly protects the stop-settle phase.

## Mapping to the current resolver

| notes item | status in `resolver.rs` |
|---|---|
| per-frame DCM recompute | ✅ `plan_target_x` re-called every tick during Swing |
| landing-time prediction (+pivot, e^{+ωt}) | ❌ stale-state law today; exponential variant was implemented & reverted (handoff §3) |
| smooth steering / no snap | ◐ spline smooths position; no explicit target-rate limit |
| terminal freeze | ❌ — **adopt**, clearest new idea in this doc |
| envelope clamp | ✅ `hip ± x_max` + `capture_saturated` |
| velocity deadband | ◐ gate exists; placement zeroing at low speed not done |
| `DCM·e^{-ωt}` formula as written | ✏️ corrected above (sign + pivot) |

## What this suggests we do next

1. **Terminal freeze** in the swing replan (latch target in the last ~15 % of
   flight). Tiny, testable in isolation, and plausibly the direct fix for the
   late-swing gate failures.
2. Re-land the landing-time prediction (the corrected §3) as the *requested*
   target, on top of the freeze — the freeze then locks the *correct* number,
   not the stale one.
3. Low-speed placement deadband for the stop-settle tail.
4. Keep the envelope clamp + saturation exactly as is.

---

Related: [`dcm-prediction-math.md`](dcm-prediction-math.md) — every parameter
and the placement law; [`event-gait-phaseA-handoff.md`](event-gait-phaseA-handoff.md) —
build state, measured A/Bs, and the reactive-lead alternative.
