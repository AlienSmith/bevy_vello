# Controllable-speed walking — the plan (2026-10-29)

Goal: the scripted per-`move` `speed` becomes an actual setpoint the body
tracks — speed changes bend the measured hip velocity toward the command at the
command edge, within ~1–1.5 s, with bounded overshoot, without cliffs, and the
character stays upright through start, cruise, step-down, step-up, and stop.

This is the gap the `speed3` runs exposed: fiction `cmd_vx` tracks the command
(120.3 / 110.4 means), measured `P3vx` does not (248 / 169), a 120→60 step-down
decelerates late and collapses into a standstill cliff ~4 s after the edge, and
a 60→100 step-up from rest walks 0. The DCM/deadbeat machinery supports
velocity control in theory — the problem is which channels are closed, and what
happens when the placement channel saturates.

## 0. Background: the fiction model, the hookup, and why speed is uncontrolled

### 0a. Why we have a fiction model

The physics (XPBD) is **reactive, not proactive**: each tick it resolves
constraints — particles move toward targets, joints hold, feet friction against
the floor. It has no notion of "in 0.3 s the body will be over its front foot
and must catch itself." Walking *is* that notion: foot-placement decisions made
now for where the body will be later (the DCM / capture-point math in
`resolver/placement.rs`). Those decisions need a model that can be integrated
forward cheaply, deterministically, in the resolver's own frame. That model is
the **fiction**: a mass-on-legs inverted pendulum the planner owns
(`hip_pos`, `hip_vel`, `v_ref`, leg timers, plants).

History forced it, it was not chosen for elegance:
- physics-only startup deadlocked (`PUSH_OFF`, Stage 4b — a resting body has no
  state-derivable reason to take the first step);
- hard velocity feedback from physics into the resolver **stopped a walking body
  dead** (`VEL_FEEDBACK`: net +6 px — see §5).

The fiction persists because it is the only thing that can *disagree* with
reality — and that disagreement is what moves the body.

### 0b. How it's hooked up

Fiction → physics every tick, through the hip-pin servo
(`hip_pin/state.rs::delta_frame_targets`):

```
target_delta = hip_delta + (v_hip − v_c)·dt + gravity_term
   position plan            ^^^^^^^^^^^^^^^
                            THE LEAD: fiction velocity − measured velocity
```

The body is dragged toward a target sitting **ahead of itself** by the fiction's
velocity advantage; friction at the planted feet converts that lead into forward
push. The boot kick is pure lead (fiction pinned to 160 px/s while the feet hold).

Physics → fiction feeds back:
- **hip position** (`drive.rs` FEEDBACK write-back) — planning stays honest about
  *where* the body is;
- **CoM + foot-contact measurement** (`pin.measured`) — feeds the capture point
  `ξ`, and under `PLAN_MEAS_V` the placement law's lead term;
- **landing error** at each touchdown (the `v_support` backward solve,
  `walk.rs` §6) — a per-stride nudge.

**Deliberately excluded: measured velocity → fiction velocity.** The one direct
wire (overwriting `state.hip_vel.x`) zeroes the lead and kills the engine, so the
fiction's velocity runs on its own command schedule and *never learns the true
speed*. The hookup is half-open by design: the position channel is closed, the
velocity channel is not.

### 0c. Why that equals "no control of speed"

Velocity control needs a loop: sense error → act → error shrinks. Trace ours:

1. **Command** `speed=120` → fiction ramps to 120. Works — the `cmd_vx` track
   does this.
2. **Fiction → body**: the lead pushes the body. Works at startup (the kick
   proves it: fiction 160 → body 160+).
3. **Body → speed controller**: should say "you're at 230, command was 120,
   stride shorter." The only channel that can shrink stride is the **placement
   law**, and by default its lead term is fed the *fiction* `v` (obediently 120
   while the body runs 230) — so the error is invisible. **Open loop.** This is
   the runaway.
4. `PLAN_MEAS_V` patches #3 (routes the measured EMA into the law; measured
   236→205→127 bleed). But there is a hard wall: feet reach only
   `x_max ≈ 54 px`, while a 230 px/s body's capture point sits `v/ω₀ ≈ 70 px`
   ahead — **unreachable above ~177 px/s**. There the law saturates, every stride
   aims max-reach, and a *speed-down produces no control event at all* until the
   fiction-vs-body race crosses by accident: the 8.5 s cliff (brake engages ~4 s
   after the 4.5 s edge, then overshoots to 0 — nothing senses the over-braking
   either).
5. **Speed-UP has no engine from rest**: at rest the lead is zero
   (fiction ≈ measured ≈ 0) and the only thing that ever seeds a big lead — the
   kick — is one-shot in the boot FSM. The 60→100 segment after the cliff walked
   ~0 px/s.

So "not hooked up properly" is precise: **position closed, velocity deliberately
broken (for good reason), and the one legitimate speed authority — stride length
— is fed a blind signal by default and saturates beyond ~177 px/s.** The command
can *start* motion (lead engine) and *lean on* feet (placement) but has no
continuous governor between them. Stages B (saturation-aware horizon → an
N-stride brake expression within reach), C (bounded lead clamp → command edges
produce real control events), and D (re-armable surge → a speed-UP engine) close
exactly those three holes.

## 1. The control structure, honestly

There are exactly **two channels** by which the controller can change the
body's horizontal velocity:

1. **Foot placement (the deadbeat law).** `p = ξ + (v·e^(−ω₀t_c) − v_d)/(ω₀·sinh(ω₀t_c))`
   is a velocity controller: with placement `p` realized, it promises `v(t_c) = v_d`.
   It is *discrete* (one correction per stance), *bounded* (reach clamp), and — since
   `PLAN_MEAS_V` — it can see the EMA'd measured speed through `v_law`, while `ξ`
   always used measured `com_x/v_com`.
2. **The fiction lead (the hip-pin servo).** `d = hip_delta + (v_hip − v_c)·dt + g_eff·dt²`
   (`hip_pin/state.rs::delta_frame_targets`). Every tick the body is dragged toward a
   reference that runs at fiction velocity. This is *continuous* and *powerful* — it is
   what the boot kick exploits — but it is **unregulated**: nothing in it knows the
   command changed or that the body outran the plan.

The velocity loop through channel 1 is what `PLAN_MEAS_V` opened. The loop through
channel 2 was deliberately never closed (the `VEL_FEEDBACK` experiment proved that
overwriting `state.hip_vel.x` with measured v kills the walk — net +6 px — because it
zeroes the lead the feed-forward gait runs on).

**The design error to avoid:** treating channel 2 as the regulator. Channel 2 is the
*engine* (feed-forward). Channel 1 is the *steering* (feedback). Controllable speed
means letting channel 1 actually steer — and both channels saturate differently.

## 2. Why speed changes don't track — three separable blockers

**(B1) Saturation = no expression for "brake over several strides."**
The capture point sits `v/ω₀` ahead of the CoM. With `ω₀ ≈ 3.30/s` (g=980, z≈90) and
`x_max ≈ 53.8` px, the one-step capture wall is `ω₀·x_max ≈ 177 px/s`. Above it, the
demanded brake foot is outside max reach, the law clamps to `hip + x_max`, and every
subsequent tick re-fires the same max-reach step with a *committed* stance clock.
The controller has no state that says "I am 2 strides from feasible"; it just sprints
as hard as geometry allows. Observed: at measured 227–265 px/s, feet land behind the
racing hip (nothing ahead to brake onto) until fiction coasts cross below reality —
then a hard brake engages ~4 s late and overshoots to a standstill (the 8.5 s cliff).
Note the s3 OFF/ON bit-identical moves: `v_law` cannot matter when the target is
clamped anyway — the saturation regime is where all speed-down error lives.

**(B2) Late/absent engagement is a *timing* problem, not a blindness problem.**
`stopping`/`v_brake`/`brake_step` (commit 412606f) fixed blindness at the *stop*
edge. Mid-walk step-downs still engage when fiction-vs-measured races cross, because
channel 2 keeps pushing until placement catches. The command edge does not produce a
control event — only a fiction ramp event (`v_ref` at accel 800 → 0.075 s for a
60 px/s step). The body's velocity has no reason to respect the fiction's schedule.

**(B3) Re-acceleration has no engine from rest.** The 60→100 segment after the cliff:
the body at ~0, fiction at 100, but the hip particle is behind its own capture
geometry for stepping-from-rest and the *only* proven way to seed stride velocity is
the boot kick, which is one-shot (Stand→Lunge at input edge; the walk loop never
re-arms it). Feed-forward alone walks in place: legs cycle, hip oscillates ±40, net ~0.

Secondary: the touchdown backward-solve (`v_support`) pumps landing error into the
fiction (green 60↔124 sawtooth), which both muddies what "the command" looks like in
telemetry and (open question) injects energy into the limit cycle we're trying to tame.

## 3. What theory guarantees (sanity anchors, not to re-litigate)

- Deadbeat placement **can** assign `v(t_c) = v_d` exactly for any `v_d` while
  `|v|·(lead coefficients) ≤ x_max`. Velocity tracking is a solved problem for LIPM —
  **at speeds under the reach budget, with horizon freedom.** The law's own knob for
  horizon is `t_c`: α = coth-ish gain shrinks as `t_c` grows, which is precisely how a
  demanded Δv is *spread over time* instead of demanded now.
- `Δv` per stance at max reach: `x_max/α(t_c)` → at the default stance horizon that's
  the gentle ~100 px/s² sustained authority measured earlier; the one-step ceiling
  ~177 px/s. A 230→60 transition is therefore *physically* a ≥1.5–3 s maneuver at
  fixed step rate. "Controllable" must mean **bounded and graded, not instant.**
- The clock is fixed (stance/swing durations constant). We agreed not to modulate
  cadence — the DCM horizon `t_c = stance_duration` assumes it. So all velocity
  authority is stride length. (Decision D1 below.)

## 4. The plan — stages, each small, flagged, measurable

Every stage: default-off flag, sandbox unit test, A/B on `plans/speed3_script.json`
(120→60→100→stop) and `plans/stop_script.json`, off-path bit-identical, `cargo test -p
game_lib lower_body` stays green, artifacts through `run_walk.sh` (the new vx panel
makes every verdict readable on one plot).

### Stage A — instrument the saturation regime (no behavior change)
Add `DebugInfo`/CSV columns: `plan_v_law`, `v_meas_filt` (`brake_v`), `plan_saturated`
(per-leg `capture_offset`/clamp hit), and the commanded `v_cmd` at the edge. Without
these we cannot see *why* a run fails; we've been inferring from `P3vx`.
**Gate:** columns present; runs bit-identical to their pre-A CSVs (hash the data cols).

### Stage B — saturation-aware horizon (the multi-step brake promise)
When the deadbeat target clamps (`p_raw` outside `hip ± x_max`) *and* the velocity
error is a deceleration (`v·v_d < 0` or `v_d == 0`), re-evaluate the law at a
stretched deadline `t_c·k` (k grown while saturated, reset on feasibility) so the
promise is "v_d in N stances" with **feasible** steps, instead of "v_d next stance"
delivered max-clamped. Implementation lives in `plan_target_x_raw`/`StepPlan`
(carry `t_c_eff` choice per tick; the clamp detection already exists in the
capture-saturation report path). This is the principled version of what `DCM_TC>0`
already lets a human do by hand.
**Gate:** 120→60 step-down measured speed begins bending **within 1 stride of the
4.5 s edge** and decays monotonically (no >1-stride plateau, no cliff); step-up
unaffected. Kill-switch: if the stretch reintroduces the 2026-10-28 `PROPAGATE`
pathologies (mid-flight arc collapse), B is re-scoped to stretch only the *deadline*
(the existing horizon-stretch machinery), not the state.

### Stage C — lead management at command edges (fix B2 without VEL_FEEDBACK's death)
Keep the lead as the engine but make it *track the command reality*: when
`plan_meas_v` is on and a command edge is in flight (`|v_cmd − v_meas_filt|` large),
clamp the fiction to `v_meas_filt + LEAD` with `LEAD` a config knob (default ~the
cruise lead the pin needs, empirically 30–50 px/s) — a *floor+ceiling on the lead*,
never a replacement of fiction velocity. The fiction may not drag a 170 px/s body
with a 60 px/s reference, nor a 0 px/s body pretend to run 100.
**Gate:** post-edge settling ≤1.5 s; overshoot ≤30%; cruise segments unchanged
(off-flag bit-identical; on-flag steady-state |v̄−cmd| < 15%); the walk must NOT
degrade like VEL_FEEDBACK (guard test: cruise net displacement stays within 20% of
the fiction-commanded value).

### Stage D — re-acceleration: reuse the kick as a gated surge (or accept ramps)
Two candidate answers to B3, pick after C's data:
- **D1 surge:** a *continuous* boot-style kick — when `v_cmd − v_meas_filt > SURGE`
  for ≥1 stance and the body is grounded+quiet, queue the P3 external-position kick
  for one window (the proven mechanism, now re-armed by velocity error instead of
  boot FSM). Risk: reintroduces the runaway we just learned to brake (B/C must be
  shipped first so the surge is automatically bled back by the placement brake).
- **D2 ramp contract:** no new engine; scripts must step speed up in ≤`~40 px/s`
  increments the feed-forward lead can carry; the resolver *verifies* this and logs
  violations. Cheap, honest, game-facing (sprint = several small steps or a kick).
**Gate:** whichever lands, 0→100 from a standstill reaches ≥80% of command within
3 strides (D1) or the A/B quantifies exactly how much the lead carries per command
step (D2 → document the envelope).

### Stage E — stop revisited (the standing acceptance test)
With B+C (and D1 if chosen), re-run `stop_script.json` from post-kick speed. Today
`stopk_*` still falls (P3y 182→265; one-step wall 177 < release speed ~230). B's
graded horizon is exactly what a >177 release needs.
**Gate:** `stopk_on` stands: P3y stays < 200 after release; coast ≤ ~1 stride
budget; `fall_check.py` upright; gates: swing/SLIDING verdicts no worse than
`stop_bl` baseline.

### Stage F — authority leaks: foot slide + friction
Stance drift 66 px at kick-speed brake (gate 2.0). A slipping stance foot converts
"placement authority" into heat — every B/C stage's effect is damped by it. Sweep
`LEG_COMPLIANCE`/foot friction at the brake transients; decide whether a hard brake
needs a temporary stance-compliance stiffening (params knob, flagged).
**Gate:** stance drift < ~35 px during commanded step-downs (the observed baseline
cruise number is ~25 px; do not aim below physics).

## 5. Known-false paths (do not re-try; each cost a session)

- Hard velocity feedback into the integrator (`VEL_FEEDBACK`) → walk dies (+6 px).
  The lead *is* the drive; only bounded *modification* (Stage C) is admissible.
- Raw measured `v_com` into the placement law → ±140 ripple rings the brake target
  (V-sweep net −239…+450). Only the EMA (`v_meas_filt`) is admissible.
- EMA τ too long (1.2) leaks the runaway; 0.3–0.6 works.
- Torso *lean* cannot boost hip velocity (light torso): it topples (+309 px HOLD
  fall) or drags backward. Kick = particle constraint, settled.
- Fiction-only kick (through the leg-PD reference) is absorbed (feet dragged).
- Propagated/state-clamped deadbeat variants measured worse (2026-10-28) — horizon
  stretch is the accepted form; Stage B must reuse it, not invent state propagation.

## 6. Decision points for the user

- **D1 (cadence):** confirmed — velocity authority = stride length only, fixed clock?
  (Then B/C/D are the whole game. Modulating stance duration would raise authority
  but re-grounds every `t_c`/trigger formula — I recommend NO for now.)
- **Spec of "controllable":** settle ≤1.5 s / overshoot ≤30% on 40–60 px/s steps in
  [60, 120] is the proposal; 120 is already the cruise ceiling (~126), so "fast walk"
  means *up to* ceiling, and *above* it is explicitly the kick/surge's job.
- **Stage order:** B → C → E is the deceleration spine; D is independent; F is a
  cleanup that will likely matter more once B/C demand harder brakes.

## 7. Status

- Committed: `412606f` (stop-path measured routing + brake-step gate), `2ef3427`
  (placement-law EMA brake), `bf65865` (boot kick). Working tree: speed `move`
  schema + `MoveSpeed` resource + vx plot panel (uncommitted).
- 131/131 lower_body tests pass; QA scripts: `plans/speed_script.json`,
  `plans/speed2_script.json`, `plans/speed3_script.json`, `plans/stop_script.json`;
  `run_walk.sh` now accepts `SCRIPT=/abs/path` KEY=VALUE args.
