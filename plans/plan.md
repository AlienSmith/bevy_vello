# Residual PPO — 3-Stage Jump & Landing Reward Design

> End-to-end plan for training a character to jump straight up and land, using a
> residual policy on top of an expert controller with a staged, positive-only
> reward structure.

---

## At a glance

| Stage | Condition | Reward | Ends with |
|---|---|---|---|
| **1 — Launch** | on ground, before liftoff | `0.0` per step | liftoff gate |
| **2 — Flight** | airborne, before touchdown | upright − angular penalty | touchdown |
| **3 — Recovery** | 1.5 s window after touchdown | upright × momentum decay | completion bonus, `terminated` |

```
Launch (r=0) ──[liftoff gate]──▶ Flight (upright) ──[touchdown]──▶ Recovery (1.5s) ──▶ terminated
```

---

## 1. Residual Control Setup

- **Action composition:** execute

  $$a = a_{\text{expert}} + a_{\text{res}}$$

  with the residual actor's final layer **zero-initialized** and clamped to a tight
  trust radius (**±15%–20%**). The expert automatically drives the robot into the
  jump from Episode 1, and zero-init guarantees the residual starts as a no-op.

- **High discount factor:** set

  $$\gamma = 0.99 \text{–} 0.995$$

  so rewards from Flight and Landing propagate cleanly backward through GAE and
  adjust pre-liftoff push-off torques.

---

## 2. Stage-by-Stage Reward Structure (Positive-Only)

### Stage 1 — Launch Phase (on ground, before liftoff)

- **Reward:** `0.0` per step.
- **Why:** keeping pre-liftoff reward at zero eliminates any temptation to stall or
  camp on the ground, while the bounded expert naturally handles the crouch and
  push-off timing.

### Stage 1 → Stage 2 transition — Cheat-Proof Liftoff Gate

Unlock Stage 2 rewards only on the exact frame **all three** physical checks hold
simultaneously (this prevents knee-tuck / "fast squash" exploits):

1. **No ground contact:** `left_foot_contact == false && right_foot_contact == false`
2. **Upward CoM velocity:** `com_vel_z > v_min`
3. **Vertical CoM clearance:** `com_pos_z > z_standing + delta_z`

**One-time liftoff bonus** — award an immediate positive bonus on the transition
frame, scaled by low angular velocity:

$$R_{\text{liftoff}} = R_{\text{base}} \cdot \exp\!\left(-k \cdot \omega^2\right)$$

### Stage 2 — Flight Phase (airborne until touchdown)

- **Per-step reward:** reward staying upright while airborne (optionally with light
  angular-velocity damping applied only in the air):

  $$r_{\text{flight}} = w_{\text{upright}} \cdot \exp\!\left(-k_1 \cdot \text{tilt}^2\right) - w_{\text{ang}} \cdot \omega^2$$

- **Why:** gives immediate feedback to keep the torso level, and teaches pre-liftoff
  push-off torques to minimize backward rotation.

### Stage 3 — Landing & Recovery Phase (1.5 s window after touchdown)

Triggered once the robot passes apex and at least one foot regains ground contact;
runs for a fixed **1.5 s recovery window**.

- **Per-step recovery reward:** reward staying upright while smoothly bleeding off
  horizontal momentum (a dead stop by 1.5 s is *not* required):

  $$r_{\text{land}} = w_{\text{upright}} \cdot \exp\!\left(-k_1 \cdot \text{tilt}^2\right)
  \cdot \exp\!\left(-k_2 \cdot \lVert v_{xy,\,\text{com}} \rVert^2\right)$$

- **Window completion bonus:** if the robot survives the full 1.5 s window without
  tipping over, award a final completion bonus `+R_survive` and end the episode
  cleanly with `terminated = True`.

---

## 3. Termination Rules (`terminated = True`)

**Failure termination** — set `terminated = True` (zeroing future bootstrap value,
`V(s_{t+1}) = 0`) immediately if:

- torso tilt exceeds the failure angle (e.g. `|pitch| > 60°`), **or**
- any non-foot body link touches the ground, **or**
- the robot fails to trigger the liftoff gate before the expert's launch window
  expires.

**No negative crash penalty needed.** Cutting off all remaining positive `r_flight`
and `r_land` rewards via `terminated = True` penalizes crashes naturally, without
making the agent afraid to leave the ground.

---

## 4. Two-Phase Training Rollout

1. **Phase 1 — Task Mastery.** Train with the exact rewards above until the robot
   reliably lifts off, stays upright in flight, and survives the 1.5 s landing
   window (**80–90 %+ success**).
2. **Phase 2 — Smoothing & Polish.** Fine-tune from the Phase 1 checkpoint by adding
   an action-rate smoothness term

   $$-\,w_{\text{smooth}} \, \lVert a_t - a_{t-1} \rVert^2$$

   with `a_{t-1}` fed into the MLP observation, to eliminate wobbling or actuator
   chatter before final distillation.

---

## Symbol reference

| Symbol | Meaning |
|---|---|
| `a_expert`, `a_res` | expert action, residual action |
| `com_vel_z`, `com_pos_z` | vertical CoM velocity, vertical CoM position |
| `z_standing` | CoM height while standing |
| `tilt` | torso tilt from upright |
| `ω` | angular velocity |
| `v_xy,com` | horizontal CoM velocity |
| `v_min`, `delta_z` | gate thresholds for liftoff |
| `R_base`, `R_survive` | liftoff bonus scale, full-window bonus |
| `k`, `k_1`, `k_2` | sharpness constants |
| `w_upright`, `w_ang`, `w_smooth` | reward weights |