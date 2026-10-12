# Plan (draft): Lower-Body Tracking Bridge — Per-Particle PD Following the Raibert Sandbox

Status: **Draft for discussion** · Area: XPBD ↔ Raibert-sandbox integration

This supersedes the earlier spine-remodel scratch idea
([`spine_pd_three_linear.md`](bevy_vello/plans/spine_pd_three_linear.md), now **deferred**).

## 1. The pivot (and why it is a better target)

The existing spine drive, [`calculate_spine_drive`](bevy_vello/examples/game_lib/src/character/systems.rs:392),
is built around a **stiff XPBD external-position constraint** pulling the spine
particles toward a virtual pose. Its "force" is `Kp·(pose − pos)` with
`Kp = 1/compliance`, and its only velocity term is the constraint's own
`damping: 0.01` — there is **no target velocity**, so it cannot track a moving
reference, only drag toward a static-ish one.

That design assumes the input can be *arbitrarily bad* — the user can command any
pose and the controller has to cope. The Raibert sandbox is the opposite: it has
already **planned a dynamically consistent trajectory** (gravity, velocity
clamps, feasible foot placement, loop-1 height). Feeding that as a target, the
tracking problem is easy, so the controller can be trivial — and *should* be,
because a simpler controller is easier to reason about, tune, and test.

**Proposal (user):** for the lower body, drive each particle with a plain PD
**acceleration** law and nothing else:

```text
a_i = Kp · (x_target_i − x_actual_i) + Kd · (v_target_i − v_actual_i)
```

applied as an XPBD **impulse** (`impulse_i = a_i · dt / inv_mass_i`, additive),
so the solver stays untouched and collision push-back survives
(see [`spine_impulse_interface.md`](plans/spine_impulse_interface.md:32)).

## 2. Why this is sound

- **It has a real D term.** The `Kd(v_target − v_actual)` half is exactly the
  target-velocity tracking the spine drive lacks; a proper PD, not a raw damper.
- **Targets are feasible by construction.** The sandbox already respects gravity
  and the velocity/acceleration envelope, so `Kp` does not have to fight to keep
  the legs under the body — the reference is already reachable.
- **Additive ⇒ collision-preserving.** Impulses correct *from* the post-physics
  velocity instead of overwriting it, unlike the old velocity path.
- **Directly tunable in physical units** (px/s²), with the standard
  critical-damping rule `Kd = 2·sqrt(Kp)` as a starting point.

## 3. What the sandbox already provides (verified)

| Quantity | Source | Status |
|---|---|---|
| `hip_pos`, `hip_vel`, `hip_height` | [`ResolverState`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:205) | present |
| hip / knee / foot **positions** | [`LowerBodyPose`](bevy_vello/examples/game_lib/src/lower_body/resolver.rs:264) | present |
| per-particle **velocity** | — | **missing** → `v_target` must be derived |
| gravity, velocity/accel clamp | sandbox internals | present (targets feasible) |

So `x_target_i` is direct; `v_target_i` is the one thing to add (see §4, D).

## 4. Decisions to settle before coding

1. **Scope — which particles does the PD own?** "Hip particles" is ambiguous
   between (a) the hip P3 + the leg chain it parents (P40/P41 knees, PLLL/PRLL
   feet), and (b) P2/P3 (CoM + hip). This decides the whole ownership model.
2. **Ownership fork — does the PD replace or complement the spine controller?**
   - *Replace*: PD drives the lower body end-to-end (sandbox is one controller for
     the body); the spine frame becomes passively carried. Simplest, matches the
     sandbox's single-controller shape.
   - *Complement*: spine drive keeps P1/P2/P3 (upper body), PD takes P3→legs.
     **Overlap at P3** — two controllers on one particle — must be resolved.
   - *Consolidate*: PD takes P2 + P3 + legs; spine drive trimmed to P1 only, so
     the split is upper-body vs lower-body with a single owner each.
3. **`v_target` source.** Options: (i) add per-particle velocity to the sandbox
   output (cleanest, makes the sandbox self-describing); (ii) finite-difference
   positions against the previous tick's pose (no sandbox change, one-frame
   memory, statelessness slightly relaxed). Recommend (i) long-term, (ii) to
   prototype.
4. **Gravity bookkeeping.** The sandbox trajectory already *includes* gravity. If
   XPBD *also* applies gravity to these particles, the controller must not
   double-count: either the PD output is the *total* acceleration with gravity
   handled inside the reference, or gravity is applied separately and the PD only
   corrects the residual. Needs an explicit rule (cf. the same question in
   [`gravity_four_layer.md`](plans/gravity_four_layer.md:179)).
5. **The P3→P2 offset still exists.** The sandbox's `hip` is character **P3**
   ([`raibert_loop3_attitude.md`](bevy_vello/plans/raibert_loop3_attitude.md:299),
   §10). If the PD drives the *leg* chain from P3, the spine↔hip junction still
   needs `P3 = P2 + R(θ)·r_hip` to attach P1/P2 — so the frame rigid offset does
   not vanish, it just moves to the junction.
6. **Where it runs.** Prototype in the pure sandbox (headless, like the vertical
   loop) or wire straight into the game layer? Recommend sandbox first, so the
   PD gains are provable before the XPBD port.

## 5. Relation to loop 3

If the sandbox reference already governs the body's CoM trajectory and foot
placement (loops 1 and 2, plus its attitude-relevant step planner), then a
separate loop-3 attitude torque may be **subsumed**: the tracked pose already
encodes "upright / leaning as planned," and the PD enforces it. That would park
[`raibert_loop3_attitude.md`](bevy_vello/plans/raibert_loop3_attitude.md) rather
than integrate it. To confirm once §4.1/§4.2 are settled.

## 6. Testability

- Pure PD + pure sandbox ⇒ the whole bridge is unit-testable headlessly:
  feed the sandbox a gait, run the PD, assert the particle tracks `x_target`
  within tolerance and settles with no overshoot at the critical-damping `Kd`.
- Regression: `Kp`/`Kd` = 0 ⇒ no impulse (behaviour-neutral baseline).

## 7. Out of scope (this draft)

- The spine-drive Kd / three-linear-velocity remodel (deferred scratch).
- The impulse-vs-velocity transport change (separate plan).
