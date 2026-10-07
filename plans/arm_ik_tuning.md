# Arm IK Tuning Guide

How to find the parameters for the right-arm aim controller. All knobs are live
in the **"Arm IK Tuning"** egui window (`arm_tuning_ui` in
`examples/collision_detection/src/character_tuning_main.rs`), editing
`ArmConfig` on the Player entity. Changes take effect immediately — no restart.

---

## 1. What each knob does

The controller is a PD-style system built entirely in the game layer (the XPBD
solver is untouched). Four mechanisms, in order of authority:

```
IK solution (desired angle)
   │
   ├─ kd velocity feedback .... moves the TARGET (D term, active)
   │
   ├─ alpha-0.5 chase ......... rate-limits how fast the rest angle follows
   │                            (max_angle_rate)
   │
   └─ ω-scheduled compliance .. scales SPRING SOFTNESS by joint speed
                                (omega_ref, soft_scale) — "the schedule"
   │
AngularConstraint solve (stiffness = angular_compliance, length-normalized)
```

| Knob | Default | Units | Mechanism | One-liner |
|---|---|---|---|---|
| `angular_compliance` | 5e-8 | XPBD compliance @ ref length | P spring softness at rest | How hard the arm locks onto the aim |
| `compliance_ref_length` | 30.0 | px | normalization anchor | Match to typical upper-arm length; don't "tune" it |
| `omega_ref` | 8.0 | rad/s | schedule: saturation speed | "Fast" starts here |
| `soft_scale` | 8.0 | × | schedule: max softening | How much authority is withheld while fast |
| `kd` | 0.1 (tuned: 0.05) | seconds | D term: target trails motion by `kd·ω` | Kills overshoot ring; amplifies noise if too high |
| `kd_max_offset` | 0.12 | rad | safety clamp on the kd offset | Leave alone |
| `max_angle_rate` | 2π | rad/s | chase clamp per frame | Rarely needs changing |

### The schedule (ω-compliance scheduling)

Each joint measures the angular velocity ω of its own rotating bone
(`bone_angular_velocity`: shoulder → upper arm, elbow → forearm) and softens
its compliance linearly in stiffness space:

```
s = min(|ω| / omega_ref, 1)
compliance_joint = angular_compliance · (1 + s · (soft_scale − 1))
```

- **At rest** (ω ≈ 0): s = 0 → full stiffness → crisp aim lock.
- **Swinging fast** (|ω| ≥ omega_ref): s = 1 → `soft_scale`× softer → the
  spring withholds correction authority instead of fighting/pumping the motion
  (back-EMF analogy: a fast-spinning motor has less torque left).

**Key property: it is passive.** It only ever *reduces* spring authority, never
moves the target and never adds energy. Therefore it **cannot cause rest
jitter** — at rest it is exactly the un-scheduled controller. This is why swing
problems go to the schedule, and rest problems go to kd.

### kd velocity feedback

The rest target is rotated *behind* the IK-desired angle by `kd·ω`
(sign-derived: constraint angle is `φ_in − φ_out`, ω measures the outgoing
bone, so `θ̇ = −ω` and the textbook `−kd·θ̇` becomes `+kd·ω` — see
`kd_offset` in `systems.rs`). Brakes the arm before it overshoots; subtracts
noise velocities at rest. **Time-independent by design**: `kd·ω` is an angle;
kd *is* the time constant. Never multiply by dt.

---

## 2. Symptom → knob table

| Symptom | Where | Knob | Direction |
|---|---|---|---|
| Aim ray jitters / shimmers at rest, never settles | rest | `kd` | **lower** (noise amplification: kd maps solver noise into target motion, spring chases it, loop gain ∝ kd) |
| Arm overshoots & rings after fast flicks | swing | `soft_scale` ↑ or `omega_ref` ↓ | schedule softens more / earlier during the swing |
| Arm feels floaty, sluggish mid-swing | swing | `soft_scale` ↓ or `omega_ref` ↑ | give authority back |
| Aim drifts when the body sways / gets bumped | rest | `angular_compliance` | **lower value** = stiffer lock |
| Arm snaps/rings when hit or on landing spikes | any | `kd_max_offset` | lower (0.05–0.1) |
| Aim permanently lags a *moving* mouse target | tracking | `kd` | lower (trailing error = kd·ω_target) |
| Rest angle crawls to target, feels laggy | any | `max_angle_rate` | raise |

**Rule of thumb:** swing regime → schedule (`omega_ref`, `soft_scale`).
Rest regime → `kd` then `angular_compliance`. kd's benefit saturates early
(0.05 s of trail at a 20 rad/s swing is already ~1 rad, clamped anyway), but
its noise cost grows linearly — so keep kd at the *low end* of "just kills the
ring", and push remaining swing problems into the schedule.

---

## 3. Tuning procedure (fixed aim point)

Do this in order; each step isolates one regime.

### Step 0 — Baseline
Set `kd = 0`, `soft_scale = 1` (both disabled). Stiffness only. This is the
raw P controller; note how bad the flick overshoot is — that's your reference.

### Step 1 — Rest stiffness (`angular_compliance`)
Aim at a fixed point and hold. Raise stiffness (lower compliance value, log
slider) until the aim line holds firm against body sway without visible
shimmer. Too stiff amplifies every solver correction; too soft and the aim
droops. Record the value. (Currently 5e-8 @ ref length 30.)

### Step 2 — Swing damping (`omega_ref`, `soft_scale`)
Flick the mouse between two far targets repeatedly. With kd still 0:
1. Start `omega_ref = 8`, `soft_scale = 8`.
2. If the arm still rings on arrival → raise `soft_scale` (8 → 15 → 30).
3. If the ring only dies when the swing feels mushy mid-flight → lower
   `omega_ref` (8 → 5 → 3) so softening kicks in earlier, then back off
   `soft_scale`.
4. Stop at the first setting where flicks arrive without a visible ring.

Sanity check: at rest nothing should have changed (schedule is identity at
ω = 0). If rest got worse, something else moved.

### Step 3 — Residual overshoot (`kd`)
If small overshoot remains after Step 2 (usually near the very end of the
swing, where ω drops below `omega_ref` and the schedule hands authority back):
1. Raise `kd` from 0 in small steps: 0.02 → 0.04 → 0.06 → 0.08.
2. **At each step, hold the aim still and watch for rest shimmer.** The first
   kd that produces shimmer is your ceiling — back off to roughly half.
3. Empirically: 0.1 shimmered, 0.05 was clean (for compliance 5e-8). If you
   change stiffness in Step 1, re-check the kd ceiling — stiffer spring =
   lower kd ceiling.

### Step 4 — Final pass
Re-run the flick test and the hold test a few times. Adjust only one knob at a
time; every symptom in §2 maps to exactly one knob.

---

## 4. Interactions cheat-sheet

```
kd ceiling      ∝ 1 / stiffness        (stiffer spring amplifies kd's noise)
kd ceiling      ∝ 1 / solver noise     (more contacts/substeps = noisier ω)
ring amplitude  ∝ soft_scale⁻¹         (while ω > omega_ref)
lag on tracking ∝ kd                   (permanent, = kd·ω_target)
lock crispness  ∝ stiffness            (at rest only — schedule is identity)
```

- kd and `soft_scale` both damp swings (target-trail vs. authority-withholding).
  Prefer the schedule; kd is the fine trim.
- `compliance_ref_length` is a rig constant, not a feel knob: set it once to
  the character's typical upper-arm length so `angular_compliance` means the
  same thing across scales.
- `max_angle_rate` interacts with kd: if the per-frame clamp
  (`max_angle_rate·dt`) is smaller than `kd·ω`, the clamp dominates and kd is
  effectively capped. Keep the rate fast (2π) so kd has room to work.

## 5. Known-good starting point (current build)

```
angular_compliance    = 5e-8   @ compliance_ref_length = 30
omega_ref             = 8
soft_scale            = 8
kd                    = 0.05   ← 0.1 causes sustained aim-ray jitter
kd_max_offset         = 0.12
max_angle_rate        = 2π
FOREARM_ANGULAR_EPSILON = 1e-4 (event-spam guard only, not a settle mechanism)
```
