# Raibert Step Planner & Velocity Clamp

Building a pure kinematic (non-physics) prototype in your game first is an exceptional strategy. It allows you to lock down the geometric limits, line-tracing, and gait timing in a sterile environment before introducing the chaos of gravity, mass ratios, and friction.

Here is the complete math for both the Forward Step Planner and the Backward Velocity Clamp, broken down term-by-term so you can implement them flawlessly.

## 1. The Forward Math: Step Target Position

This formula runs during the Swing Phase to calculate exactly where the airborne foot should land horizontally relative to the hip.

$$x_{target} = \frac{v \cdot T_s}{2} + k_v (v - v_{desired})$$

### Detailed Term Explanation

- **$x_{target}$** [Output, Meters]: The horizontal distance from the hip where the foot should plant. A positive value means stepping ahead of the hip; a negative value means stepping behind it.
- **$v$** [Input, Meters/Second]: The actual current velocity of your hip particle. In your kinematic prototype, this is just your script's current speed. Later, in the physics version, this will be `hip_particle.velocity.x`.
- **$T_s$** [Constant, Seconds]: Stance Duration. The total amount of time a single foot is expected to remain pinned to the ground during a stride.
- **$\frac{v \cdot T_s}{2}$** [The Neutral Point Component]: This is predictive feed-forward math. If your hip is moving at velocity $v$, it will travel a distance of $v \cdot T_s$ while the foot is on the ground. To ensure the hip passes perfectly over the foot exactly halfway through the step (maximizing stability), the foot must land exactly half that distance ahead.
- **$v_{desired}$** [Input, Meters/Second]: Your target cruise speed (e.g., driven by player joystick input).
- **$k_v$** [Tuning Parameter, Dimensionless]: The Proportional Feedback Gain. This dictates how aggressively the character reacts to speed errors.
  - If moving too fast ($v > v_{desired}$): The error is positive, making $x_{target}$ longer. The foot steps far forward, introducing a braking force.
  - If moving too slow ($v < v_{desired}$): The error is negative, making $x_{target}$ shorter. The foot steps closer, letting momentum carry the body forward.

## 2. The Backward Math: Velocity Limit Clamp

> **Correction (redesign, see `plans/raibert_redesign.md`)**: a pinned stance
> foot's offset from the hip sweeps monotonically from $+x_{target}$ at
> touchdown to $-(v T_s - x_{target})$ at lift-off, and the swing foot's
> offset interpolates between the same bounds — so $T_{swing}$ does **not**
> enter the reach constraint. Feasibility is two-sided:
>
> $$\frac{v T_s}{2} + k_v\,|v - v_{desired}| \le X_{max}$$
>
> solving on each side of $v_{desired}$ and taking the binding minimum:
>
> $$v_{max} = \min\!\left(\frac{X_{max} + k_v v_{desired}}{\frac{T_s}{2} + k_v},\ \frac{X_{max} - k_v v_{desired}}{\frac{T_s}{2} - k_v}\right)$$
>
> The original "floating foot" bug was an *uncapped, infeasible*
> $v_{desired}$ (e.g. 600 px/s where $\frac{v T_s}{2} = 168 > X_{max}$)
> violating the end-of-stance side — fixed by capping the desired speed
> itself (Plan cap below), not by adding a $T_{swing}$ term.

### The Generalized Backward Solve (foot → hip)

The same rearrangement at an *arbitrary* plant offset $\Delta x$ (signed, from
the hip at touchdown to the support foot) gives the hip velocity whose forward
plan would land exactly on that offset:

$$v_{support} = \frac{\Delta x + k_v \cdot v_{desired}}{\frac{T_s}{2} + k_v}$$

$v_{max}$ is this evaluated at $\Delta x = X_{max}$. Composing backward∘forward
is the identity: a plant error $e$ maps to the velocity correction
$\frac{e}{T_s/2 + k_v}$, annihilated in one stride (deadbeat). The
implementation fires this at each **touchdown event** and blends it into the
hip velocity with gain $\beta$ ($\beta = 1$ = deadbeat; lower $\beta$ damps the
correction for the future XPBD integration, where feet lag their kinematic
targets).

### How This Formula Was Derived

This isn't a new magic equation—it is the exact same Raibert formula from above, mathematically rearranged to solve for velocity ($v$) when your step target reaches its absolute physical limit ($x_{target} = X_{max}$).

### Detailed Term Explanation

- **$v_{max}$** [Output, Meters/Second]: The maximum safe speed the hip can move right now without outrunning its own feet.
- **$X_{max}$** [Dynamic Input, Meters]: The Leg's Maximum Geometric Reach at the current height.
  - **Crucial Detail**: This must be calculated dynamically every frame! If your leg segments are $L_1$ (thigh) and $L_2$ (calf), and your current hip height is $H$, your maximum horizontal reach using the Pythagorean theorem is:

$$X_{max} = \sqrt{(L_1 + L_2)^2 - H^2} \cdot \text{SafetyMargin}$$

  - When you squash the character down ($H$ gets small), $X_{max}$ automatically shrinks.
- **$T_s$, $k_v$, $v_{desired}$** [Inputs]: These are identical to the constants used in the forward formula, ensuring perfect mathematical symmetry between your planner and your brake.

## 3. The Precomputed Feasibility Envelope

All limits are closed-form in the config, so the planner is clamped *before*
it can produce unreachable targets (`GaitLimits::from_config`):

- **Reach**: $X_{max} = \sqrt{(L_1+L_2)^2 - H^2} \cdot \text{SafetyMargin}$
- **Acceleration cap**: the swing-phase prediction error $\frac{1}{2} a T_{swing}^2$
  must fit inside the reserved budget $m_e \cdot X_{max}$ (config
  `prediction_margin`), giving $a_{max} = \dfrac{2\, m_e\, X_{max}}{T_{swing}^2}$
- **Absolute speed ceiling**: $v_{abs} = \dfrac{2\, X_{max} \sqrt{m_e}}{T_{swing}}$ —
  at this speed the stop decel floor $\frac{v^2}{2 X_{max}}$ equals $a_{max}$ exactly
- **Cruise ceiling**: the corrected $v_{max}$ above
- **Plan cap**: $v_{plan} = \dfrac{2 X_{max}}{T_s}$ — the self-consistent
  cruise speed where the $k_v$ bias vanishes ($v = v_{desired}$ ⇒ the offset
  peaks at $\frac{v T_s}{2}$); the desired speed fed to the planner is capped
  at this so $k_v(v - v_{desired})$ can never persistently push late-stance
  feet out of reach

While braking to a stop the deceleration is raised to at least the **stop
floor** $a_{stop} = \frac{v^2}{2 X_{max}}$, which guarantees the capture point
stays within reach.

## 4. Capture-Point Stop

On a sudden stop (intent released while $|v|$ is above the swing trigger), the
swing foot targets the **capture point** (Pratt) instead of the forward plan:

$$x_{capture} = \frac{v \cdot |v|}{2\, a_{eff}}$$

so the hip brakes *onto* its foot rather than past it. Swing targets are also
**re-planned every frame** from the current hip state with the shrinking
remaining flight time (at constant velocity the world-space target is
stationary, since hip advance $v\,dt$ cancels the prediction shrink), and all
targets are clamped to $\text{hip} \pm X_{max}$. Once speed bleeds off and both
feet are planted, the stop-settle servo glides the hip to the support centre.

## 🔄 The Execution Pipeline in Your Game

When you implement this kinematic placeholder, your frame-by-frame loop will look like this:

1. Read `target_hip_height` (Normal vs. Squashed)
2. Precompute the feasibility envelope (`GaitLimits`) at the current height
3. Shape the hip velocity: accel clamped to `min(accel, a_max)` (or the stop
   floor while braking), ceiling `min(v_abs, v_cruise_max)`
4. Stop-settle servo once braking completes (both feet planted, no intent)
5. Clamp the hip into the stance feet's reach windows (`plant ± X_max`) so it
   can never outrun its own support
6. Run the Forward Math (or the capture-point plan while stopping) with
   per-frame re-planning; clamp targets to `hip ± X_max`
7. At each touchdown event, run the Backward Solve and blend `v_support` into
   the hip velocity with gain `beta`
8. Cast a vertical Line Trace at the target x to get `Ground_Y`
9. Feed `(x_target, Ground_Y)` to your leg IK solver

Because your forward and backward loops share the exact same parameters ($k_v$ and $T_s$), they will never fight each other. The moment you press the crouch button, the backward math will gracefully drag the character's speed down, and the forward math will cleanly shorten the steps to match.

Once your non-physics prototype is walking smoothly, let me know when you are ready to inject it into the XPBD solver so we can talk about syncing the kinematic target angles with your XPBD angular compliance ($\alpha$) parameters!