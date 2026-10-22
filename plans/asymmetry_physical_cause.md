# Root-cause analysis: left-fails-to-advance / right-steps asymmetry

**Status:** Investigation / plan only — no committed code changed (d34f728 untouched).
**Scope:** Find a *physical* mechanism in the code/simulation that makes the left
leg consistently fail to gain ground while the right leg steps, and propose the
cheapest confirming experiment.

---

## 1. What the measurement actually shows (restated)

From the walk3 gesture analysis (`tools/physics-qa/analyze_gesture.py`):

* Right leg is a **consistent stepper**: `adv` (foot − body x gain) +28…+78 px,
  ~13–14/16 swings pass all three gates (lift, advance, overtake).
* Left leg fails **specifically on ADVANCE**: its `adv` oscillates −40…+60 px
  swing-to-swing, ~half end `NO` — despite near-identical knee flexion
  (~45–63°). It moves kinematically, it just never *gains ground on the body*.
* Spatial clue: **max planted left-foot y is consistently HIGHER than the right**.
* Best walk so far: `LEG_COMPLIANCE=5e-9`, `STEP_HEIGHT=70..110`, body 60–84 px/s.

The leg moves but does not advance ⇒ this is **not** a stiffness/muscle failure.
It is a *foot-placement / ground-reference / phase-order* failure that is
systematically different for the two legs.

---

## 2. What is symmetric (rule these OUT with evidence)

I read the actual code and the V10 asset, not guessed:

* **No per-leg controller config difference.**
  [`hip_pin.rs`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1213)
  writes the same `angular_compliance`, `zeta`, `rest_chase_alpha`,
  `max_angle_rate` to both `left_leg.config` and `right_leg.config`.
  [`observers.rs`](bevy_vello/examples/game_lib/src/character_factory/observers.rs:869)
  builds both legs from one identical `leg_config()`. → mechanism #4
  (asymmetric knee compliance/torque caps) is **not** present.
* **No per-leg foot-placement sign error.** Both swing legs share the single
  `plan_target_x` ([`resolver.rs`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1436),
  re-planned at :1455) and the shared `bend_sign`. → mechanism #3 (sign error in
  one leg's foot-placement x) is **not** present.
* **Asset legs are near-symmetric (V10).** Computed from
  `v10.character.json` particle rest positions:
  * left  thigh(P3→P40)=111.0, calf(P40→PLLL)=111.0, total **222.0** px
  * right thigh(P3→P41)=110.1, calf(P41→PRLL)=110.5, total **220.6** px
  * knees/foot rest x are mirrored around hip x=347: P40×314, P41×386;
    PLLL×281, PRLL×426; foot y PLLL 557.75 (vello-down), PRLL 551.75.
  * Distance joint compliance identical (`1e-6`); angular compliance identical
    (`5e-7`, `null` rest angle → solver keeps current). → mechanism #1 (unequal
    thigh/calf rest lengths) is **~1.4 px at most, not the tens-of-px stride gap**.

> One genuine asset asymmetry exists and **matters** (see §4): the two feet seed
> at different rest heights (left foot vello-down y 557.75 vs right 551.75,
> ≈6 px), i.e. left and right are not on exactly the same floor line at seed.

* Dispatcher/contacts treat feet symmetrically
  ([`dispatch.rs`](bevy_vello/examples/collision_detection/examples/headless_arm_pd/dispatch.rs:327),
  [`hip_pin.rs`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:964)).
  → mechanism #2 (different foot friction/collider radius) is possible but runs
  *after* the placement failure and is a symptom-amplifier, not the primary
  cause (foot colliders LLL/RLL both exist; radii not dumped — see §6).

---

## 3. The two asymmetric *control* structures actually in the code

### A. The right leg is hard-wired as the leading leg (H1 — leading-leg bias)

[`resolver.rs`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:448)
seeds the right leg **half a stance out of phase**:

```rust
let right = LegState {
    phase: GaitPhase::Stance,
    phase_timer: config.stance_duration_ts * 0.5,   // ← right finishes stance first
    ...
```

There is no `lead_leg`/`swap` mechanism anywhere in the resolver, hip_pin, or
dispatch. Combined with the **anti-phase interlock** in
[`step_leg`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1430)
(`let wants_step = … && !other_swinging`), this **deterministically makes the
right leg the first to swing and the left the second, forever.** There is no
contact authority override that re-balances this ordering in the runner
(`GAIT_AUTHORITY=timer` is the documented default for the best walk).

### B. The virtual floor is latched from the LEFT leg only (H2 — floor anchor)

[`hip_pin.rs`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1260)
latches the per-character standing height from **left only**:

```rust
pin.standing_height = (live_left.hip.y - live_left.foot.y).max(1.0);
```

and exposes it for the *shared* collision floor via
[`stance_height()`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:571)
(`s.live[0]` = left), which [`collision.rs`](bevy_vello/examples/collision_detection/examples/headless_arm_pd/collision.rs:91)
uses to place the single floor rect. The same left-derived value overrides the
resolver's `hip_height` ([`hip_pin.rs`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1005))
and the CoM floor line ([`measure.rs`/`hip_pin.rs:818`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:818)).

Because the left and right feet seed at different rest heights (asset, §2), a
**floor sized for the left foot is wrong for the right foot** (and vice versa).
Meanwhile the per-foot swing-floor clamp
[`foot_l.y` / `foot_r.y`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1313)
each clamp to *their own* seed foot y — so the two feet are told to plant on two
*different* implicit floor lines while the single collision plane only has one.

---

## 4. Why these produce "left fails to advance but right steps"

### H2 → the left foot is the floor's reference, but the *right foot* is the one
that always steps first.

During the walk the resolver plans (and the pin commands) a symmetric forward
stride for both legs. The difference enters through **which leg actually lands
and drives the shared hip velocity**. Only **one** foot lands per swing, and the
backward solve
([`resolver.rs:1778`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1778))
sets **one** shared `state.hip_vel.x = v_support` from that touchdown's
`delta = plant_pos.x − hip_x`. Because the right leg is always the first of a
pair to swing (H1), it is the leg that lands on the freshly-accelerated hip and
locks the cruise velocity. The left leg then swings as the **trailer**: by the
time it lifts, its `plant_pos` has gone stale (it was pinned while the right
leg's whole swing+land moved the hip forward), and under `Contact` authority
the resolver **re-anchors** its stance plant to the *measured realized* foot x
([`resolver.rs:1384`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1384),
`leg.plant_pos.x = measured foot`). Re-anchoring the plant to a foot that sits on
the **higher** left floor means the left stride is repeatedly collapsed onto a
stale / mis-level plant → the left foot lifts but returns **in front of where it
started in body space without gaining net ground on the body** — exactly the
`NO_ADVANCE` signature, and exactly the "left plant sits higher" clue.

In short: **H1 (right swings first) + H2 (floor sized to left) couple through
the single shared backward-solve velocity and the per-foot plant re-anchor, so
the leading leg (right) is credited with every successful catch, while the
trailing left leg re-anchors to a higher/stale plant and is denied ground gain.**

### Why it reads as "kinematically identical knee flexion" (matches the data):
both legs fold ~45–63° through a symmetric swing profile
([`swing_profile_foot`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1290)).
The knee plays correctly; the failure is purely in the *landing x relative to the
moving hip* — the resolver's `plan_target_x` re-plans against the shared `v`,
and the trailing left leg inherits a `v` already settled by the right leg's
touchdown (`kv(v−v_desired)` term shrinks/stales for the left's swing window),
so left `x_target` is consistently short of the hip's actual travel.

---

## 5. Mechanism ranking (by evidence strength)

| # | Mechanism | Where | Evidence | Rank |
|---|-----------|-------|----------|------|
| H1 | Right leg hard-wired to swing first (half-stance phase stagger, no swap) | [`resolver.rs:453`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:453) | Direct source; matches the user's "which foot swings first" hypothesis; explains persistent (non-random, not 50/50) left failure | **1 (primary)** |
| H2 | Floor/standing_height latched from LEFT leg only; asset feet seed at different heights | [`hip_pin.rs:1260`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1260), [`hip_pin.rs:571`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:571), [`collision.rs:91`](bevy_vello/examples/collision_detection/examples/headless_arm_pd/collision.rs:91), [`hip_pin.rs:1313`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1313) | Direct source; the asset has 6 px foot-height asymmetry (§2); explains the "left floor sits higher" clue and the left-plant collapse | **2 (strong contributor / enabler)** |
| H3 | Stance plant re-anchor `Contact` collapses trailing leg onto measured foot | [`resolver.rs:1384`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1384) | Reason to exist (landing error) but demonstrably transfers stride to slip under re-anchor; compounds H1+H2 | **3 (amplifier)** |
| H4 | Single shared backward-solve velocity credits only the landing leg each swing | [`resolver.rs:1778`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1778) | Real; converts the H1 ordering into a systematic leading/trailing velocity difference | 4 (consequence of H1+H2) |
| H5 | Asset: unequal thigh/calf rest length | (asset v10) | Only ~1.4 px; not the tens-of-px gap | 5 (minor/rule out) |
| H6 | Foot friction / collider radius difference | asset + collision | Plausible amplifier of the higher-left-floor symptom, but no asymmetric placement cause found in code | 6 (symptom) |

---

## 6. Concrete confirmatory / fix experiments (pick the cheapest, decisive one)

All three are one-line **swap tests** — exactly what the task asks for — and
each cleanly disambiguates one ranked mechanism.

### Experiment 1 — Flip the leading leg (confirms H1, cheapest)
Change [`resolver.rs:453`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:453)
so the **left** leg gets the `* 0.5` phase stagger instead of the right
(swap `phase_timer` on the two `LegState`s in `ResolverState::new`).
**Prediction if H1 correct:** the asymmetry **reverses** — the *left* leg is the
one with +28…+78 px `adv`, the *right* fails `ADVANCE`, and the *right* floor
now sits higher. If the failure stays on the left, H1 is not the cause.

### Experiment 2 — Anchor the floor to the average (or to the right) (confirms H2)
Change [`hip_pin.rs:1260`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1260)
(and `stance_height()` at :571 and the `foot_l/foot_r` clamps at :1313) to use
`0.5·(left + right)` hip→foot, or to the **right** leg. **Prediction if H2
correct:** the left-floor-higher symptom disappears and the left `ADVANCE` NO
rate drops; the legs plant on one common line.

### Experiment 3 — Disable the Contact plant re-anchor (confirms H3)
Benchmark the same walk at `GAIT_AUTHORITY=timer` (already the runner default)
and at `contact`, and diff `left adv` and the left-floor-Y columns. **Prediction
if H3 amplifiers:** `timer` (no re-anchor) shows a smaller or symmetric
left/right stride difference; `contact` re-introduces the left collapse.

**Recommended quick path:** run Experiment 1 first — it is a 2-line change, is
decisive, and directly tests the user's "which foot swings first" hypothesis.
Then Experiment 2 to remove the floor-reference asymmetry.

### Measurement to add (no physics change) for unambiguous attribution
In `headless_arm_pd/sample.rs` per-leg row, add, per stance, the **signed
touchdown offset `plant_pos.x − hip_x` separately for left and right**, and the
**realized foot-behind/shortfall** `foot.x − plan_target_x` per swing. This
directly shows whether the left swing is *landing short* (targeting smaller x)
vs *planning the same but landing short* (physics/contact), which is the one
datum that cleanly splits "resolver plans a shorter step for left" from
"resolver plans equal but physics denies left ground." If the two legs' planned
`plan_target_x` differ while both use the same `hip_x`, that localizes H1/H4 in
the resolver; if they are equal but the *realized* left foot lands short, it's
H2 (floor) / H3 (re-anchor) in the physics path.

---

## 7. Crisp statement of the physical story

The two legs are commanded with a **symmetric** stride, but the walk is built on
two one-sided control facts that the measurement surfaces:

1. **Right always steps first** ([`resolver.rs:453`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:453))
   → right is always the leg that catches the fresh-forward hip and locks cruise
   velocity; **left is always the trailer**.
2. **The single floor is sized to the left foot** ([`hip_pin.rs:1260`](bevy_vello/examples/game_lib/src/lower_body/hip_pin.rs:1260))
   while the two feet seed 6 px apart in the asset → the trailing left foot, on
   `Contact` re-anchor ([`resolver.rs:1384`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:1384)),
   repeatedly re-plants onto a **higher, stale** point, so standing still in body
   space (its knee still flexes, its foot still lifts, but it gains no ground).

The compound effect is precisely the observed gate: left knee flexion identical
to right, left `adv` oscillating around zero / `NO_ADVANCE`, and the left stance
floor sitting higher than the right. The fix is structural, not a gain sweep:
decide the leading leg explicitly (and/or balance it), and size the one floor to
both feet (average), not to the left only.