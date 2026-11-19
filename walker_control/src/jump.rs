//! The **hand-rolled jump** — a scripted crouch→extend stroke.
//!
//! # Why this lives here and not in `game_lib`
//! `game_lib`'s loop-2 vertical resolver (`examples/game_lib/src/lower_body/
//! resolver/vertical.rs`) owns a jump FSM (`Stand→Load→Launch→Flight→Land`)
//! with the per-hop law `Δl = Δl_prev + k_p·(h_desired − h_apex)` — but it
//! drives the legacy `drive_hip_pin` stack, which `walker_qa` disables
//! (`SpineDriveEnabled(false)`). The walker_control stack actuates through a
//! different channel, so the stroke needs its own port here.
//!
//! # The actuation channel this exploits
//! [`crate::actuate`] consumes only the **shape**: the two hip→foot chord
//! offsets and the lean. `pose.hip` is a free anchor (no position constraint),
//! and a `stance: true` foot gets a stiff distance strut pulling hip↔foot to
//! the commanded chord length. So:
//!
//! - **Load** shortens the chord (crouch) against planted feet ⇒ the solver
//!   pulls the hip down. The crouch keeps the **rest x-offsets**: sweeping the
//!   feet inward while they are friction-planted drags them and kicks the
//!   torso off-center (measured: it spun the body a −95° backflip).
//! - **Launch** re-extends the chord to full rest fast ⇒ the strut pushes the
//!   hip up against the friction-planted feet. (This exact channel lifted the
//!   hip ~25 px in the PPO deploy — the physics is proven, only the clock is
//!   ours now.)
//! - **Flight** holds the extended pose. No tuck: the leg hip angular
//!   constraints are the torso's ONLY joints, so swinging the airborne legs
//!   (the first attempt swept them 45 px through an arc) reaction-rotates the
//!   torso — a pencil jump keeps the shape and therefore keeps the attitude.
//! - **Land** re-arms the stroke with the feet at the rest offsets, then (while
//!   the jump intent is held) **re-arms Load for the next hop** — bunny-
//!   hopping, matching loop-2's held-command semantics.
//!
//! The stroke's *shape* is time-phased and **open-loop** (no height feedback):
//! the authored expert PPO is trained against exactly this. The one exception
//! is the `Load→Launch` boundary, which is **event-gated** on the measured
//! chord (`ControlConfig::jump_fire_r`) rather than fired on the clock — a
//! run-up hop whose feet have not loaded must not extend into an unbraced
//! stance (`plans/jump-ppo-residual-polar-action.md` §11). `Flight→Land` is
//! event-gated on contact the same way. Every other transition, and every
//! keyframe, remains pure replay.

use glam::Vec2;

use crate::actuate::{FootPose, Pose};
use crate::control::{Command, ControlConfig, ControlOut, LocoMode, Step};
use crate::observe::Fiction;

/// One stroke cycle. Time-phased; `Land` re-arms `Load` while the command is
/// held (bunny-hop) — the mode machine (`LocoState::decide`) ends the stroke
/// externally when the jump intent goes away.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JumpPhase {
    /// Chord rest → crouch, feet stance-pinned, hip descends.
    Load,
    /// Crouch → rest fast, feet pinned: the power stroke.
    Launch,
    /// Airborne: stance off (soft struts), shape held extended (pencil).
    Flight,
    /// Touchdown recovery: stance on, chord ramps back to rest.
    Land,
}

/// The scripted jump controller — the authored expert for the jump intent
/// (`Command::is_jump`). The *shape* comes from the keyframed [`jump_clip`]
/// track; this type only owns the stroke clock, the contact-driven flight exit,
/// and the ride-along hip anchor.
#[derive(Debug, Clone)]
pub struct JumpCtrl {
    phase: JumpPhase,
    /// Seconds since the stroke began (the clip's local time, unwrapped across
    /// re-hops so a held jump keeps advancing through the track).
    t: f32,
    /// Completed touchdown count (telemetry/tests).
    hops: u32,
    /// The hip target the stroke started from, and its measured velocity at
    /// that moment (see the ride-along in [`Step::step`]).
    anchor: Vec2,
    anchor_v: f32,
    /// True once the ride-along has been seeded (on the first Load entry).
    seeded: bool,
    /// Seconds spent **held at the Load gate** this stroke (see the advance
    /// rule in [`Step::step`]). Capped by `ControlConfig::jump_load_cap` so a
    /// body that never compresses still launches.
    gate_wait: f32,
    /// True once this stroke has passed the Load→Launch gate. Latching it is
    /// what stops the gate re-arming: during Launch the measured chord is
    /// rising again (that is the extension), so an unlatched condition would
    /// yank the cursor back to the crouch key forever.
    fired: bool,
    /// Set on the single tick the power stroke is released; consumed by the
    /// same tick's output as [`ControlOut::jump_impulse`]. The strut alone
    /// cannot launch the hop (standing already sits at ~104.5 of 110 px, so
    /// there are only ~5.5 px of extension left — see
    /// `plans/walker-jump-findings.md`), so the height is handed to the body
    /// as one vertical impulse, exactly as the pre-clip stroke did.
    launch_pending: bool,
    /// Loop-2's design apex (px above standing) — the impulse target when
    /// `jump_launch_impulse == 0` (derive from the apex).
    jump_apex: f32,
    /// Gravity (px/s²), for that same derivation.
    gravity: f32,
    /// Stroke-local phase boundaries (seconds): load end and flight end. From
    /// [`crate::jump_clip::jump_clip`]; kept here so a re-hop can rebase `t`
    /// cheaply. (Launch end needs no field: `phase_at(self.t)` derives every
    /// transition from the clip itself.)
    load_end: f32,
    flight_end: f32,
    /// End of the authored Land (absorb) segment — the clip's last key. The
    /// landing hold starts counting only once the body has played the absorb
    /// out to here (see [`Self::landing_complete`]).
    land_end: f32,
    /// Monotone **time since touchdown**, seconds. Starts when the stroke enters
    /// `Land` and ticks every step regardless of contact, so the landing window
    /// can never hang. This is the `ControlConfig::jump_land_hold` clock.
    land_hold: f32,
    /// Latched when the landing hold has **elapsed** and the body was upright:
    /// the stroke is over and the body is standing. The episode may now respawn;
    /// the stroke will not re-hop.
    landed: bool,
    /// Latched when the landing window has been judged (either way): the grace
    /// period is over and the fall gate may speak again. Distinct from `landed`
    /// so a body that fails to stabilize still leaves the window.
    landing_resolved: bool,
    /// Chord length used to turn the normalized pose into px offsets.
    leg_len: f32,
}

impl JumpCtrl {
    /// `lower` supplies `thigh_len + calf_len` (the chord the normalized clip
    /// multiplies into px). Defaults to the v10 blueprint when zero.
    pub fn new(lower: &walker_math::LowerBodyConfig) -> Self {
        let leg_len = match lower.thigh_len + lower.calf_len {
            l if l > 0.0 => l,
            _ => 110.0,
        };
        // Keep the boundaries in lockstep with the clip rather than re-deriving
        // them: read the key times straight off the track.
        let clip = crate::jump_clip::jump_clip();
        let ts: Vec<f32> = clip.keys().iter().map(|k| k.t).collect();
        Self {
            phase: JumpPhase::Load,
            t: 0.0,
            hops: 0,
            anchor: Vec2::ZERO,
            anchor_v: 0.0,
            seeded: false,
            gate_wait: 0.0,
            fired: false,
            launch_pending: false,
            jump_apex: lower.jump_apex,
            gravity: lower.gravity,
            load_end: ts.get(1).copied().unwrap_or(0.22),
            flight_end: ts.get(3).copied().unwrap_or(0.79),
            // The clip's final key is the end of the land-absorb (1.04).
            land_end: ts.last().copied().unwrap_or(1.04),
            land_hold: 0.0,
            landed: false,
            landing_resolved: false,
            leg_len,
        }
    }

    /// Completed hops (for tests and later telemetry).
    pub fn hops(&self) -> u32 {
        self.hops
    }

    /// True once a landing has been declared **complete**: the authored absorb
    /// has played out and the body has then held a stable grounded stance for
    /// `cfg.jump_land_hold` seconds (`plans/jump-ppo-residual-polar-action.md`
    /// §12.2). This is the signal a harness uses to end the jump episode and
    /// respawn — touchdown alone is not "landed", the body still has to absorb
    /// the impact and settle.
    ///
    /// Returns `false` while the hold is disabled (`cfg.jump_land_hold < 0`), so
    /// the old re-hop loop is preserved for stroke-only tests.
    pub fn landing_complete(&self, cfg: &ControlConfig) -> bool {
        cfg.jump_land_hold >= 0.0 && self.landed
    }

    /// True while the **landing grace window** is open: touchdown has happened
    /// and the hold has not yet elapsed. The fall gate must stay silent during
    /// it — the body is supposed to be folding, wobbling and recovering, and
    /// cutting the episode there is exactly the "respawns too soon on landing"
    /// failure. Closes on elapse (judged once, see [`Self::landing_complete`]).
    pub fn landing_window_open(&self, cfg: &ControlConfig) -> bool {
        cfg.jump_land_hold >= 0.0 && self.phase == JumpPhase::Land && !self.landing_resolved
    }

    /// Measured chord ratio over the **touching** feet: `mean(|foot − hip|) /
    /// leg_len`. `None` when neither foot reports contact (airborne ⇒ nothing
    /// to gate on). This is the reality signal the §11 Load→Launch gate reads.
    fn measured_r(&self, fiction: &Fiction) -> Option<f32> {
        let mut sum = 0.0;
        let mut n = 0u32;
        for f in fiction.feet.iter().filter(|f| f.touching) {
            sum += (f.pos - fiction.hip).length();
            n += 1;
        }
        if n == 0 {
            None
        } else {
            Some(sum / (n as f32 * self.leg_len.max(1e-6)))
        }
    }

    /// Sample the authored clip at the current stroke time, returning the
    /// normalized 5-param pose, the stance flag, and the px hip→foot offsets.
    fn sample_shape(&self) -> ([f32; 5], bool, Vec2, Vec2) {
        let clip = crate::jump_clip::jump_clip();
        let key = clip.sample(self.t);
        let (l, r) = crate::jump_clip::key_to_offsets(key.pose, self.leg_len);
        (key.pose, key.stance, l, r)
    }
}

impl Step for JumpCtrl {
    fn step(
        &mut self,
        _cmd: &Command,
        fiction: &Fiction,
        cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        let dt = dt.max(0.0);

        // Advance rule — NOT a bare clock through the whole Load segment. The
        // Load segment itself runs on time (the crouch must actually be
        // performed, or there is nothing to measure); the gate only holds the
        // cursor at the Load→Launch BOUNDARY until the MEASURED chord has
        // compressed past `jump_fire_r` (§11). This is the same shape as
        // `Flight→Land` below: authored keys, reality-gated transition.
        // Fallbacks keep the stroke from ever hanging — `jump_load_cap` seconds
        // since the boundary, and a disabled knob (`<= 0`) is the old pure
        // clock replay.
        let gate_on = cfg.jump_fire_r > 0.0;
        // The predicate is "would this tick cross into Launch?", not "are we
        // already past it": a hold parks `t` just under the boundary, so a
        // position test would disengage on the very next tick.
        let crossing = !self.fired && self.t + dt >= self.load_end;
        if gate_on && crossing {
            let compressed = self
                .measured_r(fiction)
                .map(|r| r < cfg.jump_fire_r)
                .unwrap_or(false);
            let capped = self.gate_wait >= cfg.jump_load_cap.max(0.0);
            if compressed || capped {
                // Release: latch and step across the boundary.
                self.fired = true;
                self.t = (self.t + dt).max(self.load_end);
            } else {
                // Hold at the crouch key: freeze the cursor just under the
                // boundary (so `phase_at` still reports Load) and count the
                // wait so the cap can fire.
                self.gate_wait += dt;
                self.t = self.load_end - 1e-4;
            }
        } else {
            // No gate, or already past the boundary: the clock is free. Latch
            // `fired` on the crossing so "has this stroke launched yet" has one
            // meaning whether or not the gate is enabled.
            if crossing {
                self.fired = true;
            }
            self.t += dt;
        }

        // Phase boundaries come straight from the clip; the phase itself is a
        // function of the clock. Flight can end **early** on real contact even
        // though the clip's shape keeps running (a clip cannot see the world).
        // The contact check only fires once flight is already established (the
        // entry tick still reads the feet touching, which is the *launch*, not
        // a landing).
        let was_flight = self.phase == JumpPhase::Flight;
        let was_launch = self.phase == JumpPhase::Launch;
        self.phase = crate::jump_clip::phase_at(self.t);
        // Release: the single tick the power stroke ends (Launch→Flight) is
        // the jump's one real authority. The strut cannot lift the body (only
        // ~5.5 px of extension left at standing), so arm the vertical impulse
        // here and emit it below.
        if was_launch && self.phase == JumpPhase::Flight {
            self.launch_pending = true;
        }
        let contact = fiction.feet.iter().any(|f| f.touching);
        if was_flight && self.phase == JumpPhase::Flight && contact {
            // Touchdown: skip ahead to the clip's Land segment. The wrap keeps
            // the pose continuous (the Land key starts at rest, the Flight key
            // is rest too, so this is a soft jump).
            self.t = self.flight_end;
            self.phase = JumpPhase::Land;
        }
        // Landing is a **window**, not an instant (user: "the landing doesn't
        // end the second the character touches the ground"). Three stages, all
        // in the Land phase:
        //   1. the authored absorb segment plays out (t: flight_end → land_end),
        //      easing the stance back to rest and killing the impact;
        //   2. once the absorb is done AND the body is grounded and upright, the
        //      clock parks at `land_end` (holding the rest stance) and the
        //      stable-grounded time accumulates into `land_hold`;
        //   3. when `land_hold` reaches `cfg.jump_land_hold`, the landing is
        //      *complete* — latch `landed`, report it via `landing_complete`.
        //
        // The re-hop below is the fallback when the hold is **disabled**
        // (`jump_land_hold < 0`): the stroke loops instead of ever declaring a
        // landing. That keeps the stroke-only tests (and any caller that wants a
        // continuous hop loop) behaving exactly as before.
        let hold_enabled = cfg.jump_land_hold >= 0.0;
        let lean = fiction.lean.abs();
        let upright = cfg.jump_land_lean <= 0.0 || lean <= cfg.jump_land_lean;
        // The landing is a **grace window**, not an instant, and it must not be
        // cut short by the fall gate while the body is still absorbing and
        // recovering. So `land_hold` is a MONOTONE clock from touchdown (entering
        // `Land`), independent of contact — it can never hang, and the harness
        // suppresses the fall gate while the window is open.
        //
        // Stage 1 (t < land_end): the authored absorb plays out, easing the
        // stance back to rest. Stage 2 (t pinned at land_end): hold that rest
        // stance until the window elapses.
        if hold_enabled && self.phase == JumpPhase::Land && !self.landing_resolved {
            self.land_hold += dt;
            if self.t < self.land_end {
                self.t = (self.t + dt).min(self.land_end);
            } else {
                // Pinned at the last key: `clip.sample` clamps, so the rest
                // stance is what holds.
                self.t = self.land_end;
            }
            if self.land_hold >= cfg.jump_land_hold {
                // Window elapsed — judge it ONCE, now:
                //  - landed "properly" only if the body is upright on the ground
                //    at the end of the window (`landed`);
                //  - either way the grace window is over (`landing_resolved`), so
                //    a topple past it goes back to the normal fall gate.
                self.landing_resolved = true;
                if fiction.grounded && upright {
                    self.landed = true;
                    self.hops += 1;
                }
            }
        }
        // Re-hop fallback: only when the landing hold is disabled, once the Land
        // segment has played out while still grounded and the intent is held,
        // rebase the clock to the top of the clip. The gate re-arms with it —
        // every hop has to earn its own launch.
        if !hold_enabled
            && self.phase == JumpPhase::Land
            && self.t >= self.flight_end + 0.25
            && fiction.grounded
        {
            self.hops += 1;
            self.t = 0.0;
            self.phase = JumpPhase::Load;
            self.fired = false;
            self.gate_wait = 0.0;
            self.launch_pending = false;
        }

        // Shape: sample the authored clip at the current stroke time. Stance is
        // authored from the **phase**, not the blended key: the Land segment
        // starts on the (soft) Flight key, so a lerped flag would keep the
        // struts soft through touchdown. Ground = stiff, air = soft.
        let (_pose5, _key_stance, left, right) = self.sample_shape();
        let stance = self.phase != JumpPhase::Flight;

        // Anchor: the pose is translation-invariant in WORLD x — only the
        // chord shape actuates (see [`crate::actuate`]). But while a stance
        // foot is planted the strut drags the hip toward that foot's
        // commanded position, so a *stationary* hip target becomes a brake:
        // walking into a jump would dead-stop the 120 px/s gait mid-stroke
        // (measured on the first run-up attempt). The ground phases therefore
        // ride the target along at the velocity captured at stroke entry
        // (measured `fiction.v_hip.x`, constant through the arc); during the
        // airborne phase the pose floats — re-anchored to the measured hip,
        // which also re-syncs before the next stroke.
        let hip = if stance {
            if !self.seeded {
                self.anchor = fiction.hip;
                self.anchor_v = fiction.v_hip.x;
                self.seeded = true;
            } else {
                self.anchor.x += self.anchor_v * dt;
            }
            Vec2::new(self.anchor.x, fiction.hip.y)
        } else {
            self.seeded = false;
            fiction.hip
        };
        // Consume the launch: `JUMP_IMPULSE` semantics match the pre-clip
        // stroke — > 0 literal speed, == 0 derive `sqrt(2·g·apex)` so
        // `JUMP_APEX` alone still produces the whole hop, < 0 disabled (the
        // pure-strut control experiment). vello y is down, so an upward
        // launch is a NEGATIVE Δv_y.
        let jump_impulse = if self.launch_pending {
            self.launch_pending = false;
            let v0 = if cfg.jump_launch_impulse > 0.0 {
                cfg.jump_launch_impulse
            } else if cfg.jump_launch_impulse == 0.0 {
                // Apex measured from the CURRENT rise, not a blind constant:
                // the body may still be below standing when the stroke fires.
                let rise = (fiction.hip.y - self.anchor.y).max(0.0);
                let apex_needed = (self.jump_apex - rise).max(1.0);
                (2.0 * self.gravity.max(1.0) * apex_needed).sqrt()
            } else {
                0.0
            };
            -v0.max(0.0)
        } else {
            0.0
        };

        ControlOut {
            pose: Pose {
                hip,
                left_foot: FootPose {
                    pos: hip + left,
                    stance,
                },
                right_foot: FootPose {
                    pos: hip + right,
                    stance,
                },
                facing,
                // Upright throughout: the hand-rolled jump is a pure vertical
                // hop; any lean here is drift we would have to fight later.
                upper_angle: 0.0,
                stance_split: false,
            },
            // No horizontal intent: the hip_vel target is zero so telemetry
            // reads the stroke as in-place.
            hip_vel: Vec2::ZERO,
            jump_impulse,
            plan: [Vec2::new(f32::NAN, f32::NAN); 2],
            mode: LocoMode::Jumping,
            debug: None,
        }
    }

    fn mode(&self) -> LocoMode {
        LocoMode::Jumping
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observe::FootState;

    /// The clip's rest hip→foot offsets at the default `leg_len` (110 px).
    fn rest_offsets() -> (Vec2, Vec2) {
        crate::jump_clip::key_to_offsets(crate::jump_clip::rest_pose(), 110.0)
    }

    /// Config with the §11 Load→Launch gate disabled — the pre-§11 pure clock
    /// replay. The stroke-mechanics tests below drive a fiction whose chord
    /// never compresses (a static standing body), so they must not be gated;
    /// the gate has its own tests further down.
    fn clock_cfg() -> ControlConfig {
        ControlConfig {
            jump_fire_r: 0.0,
            ..ControlConfig::default()
        }
    }

    /// The re-hop loop: landing-hold **disabled** (`< 0`), so a held jump intent
    /// keeps cycling the stroke instead of ever declaring a landing. This is
    /// the behaviour the two `re_hop` tests assert; the hold is a deliberate
    /// opt-in and has its own test.
    fn hop_loop_cfg() -> ControlConfig {
        ControlConfig {
            jump_fire_r: 0.0,
            jump_land_hold: -1.0,
            ..ControlConfig::default()
        }
    }

    /// A fiction whose touching feet sit at `chord_ratio · leg_len` from the
    /// hip — the compression signal the gate reads.
    fn crouched(chord_ratio: f32) -> Fiction {
        let mut f = standing();
        let leg = 110.0;
        let chord = chord_ratio * leg;
        // Keep the rest x-direction, shrink only the depth: this reproduces
        // "the crouch actually loaded" without moving the feet in x.
        let (l, r) = rest_offsets();
        let dir = |o: Vec2| o.normalize_or_zero();
        f.feet[0].pos = f.hip + dir(l) * chord;
        f.feet[1].pos = f.hip + dir(r) * chord;
        f
    }

    /// Standing fiction: feet at the rest offsets under a hip at (0, −140),
    /// both touching.
    fn standing() -> Fiction {
        let hip = Vec2::new(0.0, -140.0);
        let (l, r) = rest_offsets();
        Fiction {
            dt: 1.0 / 90.0,
            hip,
            v_hip: Vec2::ZERO,
            lean: 0.0,
            lean_rate: 0.0,
            com: hip,
            v_com: Vec2::ZERO,
            ang_momentum: 0.0,
            omega_com: 0.0,
            feet: [
                FootState {
                    pos: hip + l,
                    vel: Vec2::ZERO,
                    touching: true,
                },
                FootState {
                    pos: hip + r,
                    vel: Vec2::ZERO,
                    touching: true,
                },
            ],
            grounded: true,
        }
    }

    fn airborne() -> Fiction {
        let mut f = standing();
        f.feet[0].touching = false;
        f.feet[1].touching = false;
        f.grounded = false;
        f
    }

    fn run(
        cfg: &ControlConfig,
        ctrl: &mut JumpCtrl,
        fiction: &Fiction,
        ticks: usize,
    ) -> Vec<ControlOut> {
        let cmd = Command::jump_vertical(100.0);
        (0..ticks)
            .map(|_| ctrl.step(&cmd, fiction, cfg, 1.0 / 90.0, 1.0))
            .collect()
    }

    #[test]
    fn the_stroke_crouches_then_extends_then_tucks() {
        let cfg = clock_cfg();
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        let depth = |o: &ControlOut| o.pose.left_foot.pos.y - o.pose.hip.y;

        // Load (grounded) shortens the chord depth toward the crouch. At
        // tick 20 (t = .222 ≥ load_ts) the stroke hands over to Launch.
        let outs = run(&cfg, &mut ctrl, &standing(), 20);
        assert!(depth(&outs[19]) < 100.0, "crouching during load");

        // Launch ramps back to the full rest chord; at its tick 11 it enters
        // Flight (launch_ts = .12 s).
        let outs = run(&cfg, &mut ctrl, &standing(), 11);
        assert!(
            (depth(&outs[10]) - 106.0).abs() < 2.0,
            "fully extended after the power stroke, got {}",
            depth(&outs[10])
        );

        // Flight (airborne fiction — ANY contact would land early): stance
        // dropped, shape held extended (pencil: no leg swing to react
        // against).
        let outs = run(&cfg, &mut ctrl, &airborne(), 26);
        let flight = &outs[25];
        assert!(!flight.pose.left_foot.stance, "flight is airborne");
        assert!(!flight.pose.right_foot.stance);
        assert!(depth(flight) > 100.0, "shape held extended in flight");
        let (rest_l, _) = rest_offsets();
        assert!(
            (flight.pose.left_foot.pos.x - flight.pose.hip.x - rest_l.x).abs() < 2.0,
            "flight keeps the rest x-offsets"
        );

        // Land (touchdown) brings stance back on.
        let outs = run(&cfg, &mut ctrl, &standing(), 10);
        assert!(outs[5].pose.left_foot.stance, "stance re-arms at land");
    }

    #[test]
    fn flight_ends_early_on_real_contact() {
        // The ballistic clock may not expire, but a touchdown does: land.
        let cfg = clock_cfg();
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // Drive Load + Launch grounded until Flight begins (tick 31), a few
        // ticks airborne, then the fiction reports contact again.
        run(&cfg, &mut ctrl, &standing(), 20);
        run(&cfg, &mut ctrl, &standing(), 11);
        assert_eq!(ctrl.phase, JumpPhase::Flight, "power stroke ended");
        run(&cfg, &mut ctrl, &airborne(), 2);
        assert_eq!(ctrl.phase, JumpPhase::Flight, "airborne: still flying");
        ctrl.step(
            &Command::jump_vertical(100.0),
            &standing(),
            &cfg,
            1.0 / 90.0,
            1.0,
        );
        assert_eq!(ctrl.phase, JumpPhase::Land, "contact ends the flight");
    }

    #[test]
    fn a_held_command_re_hops() {
        let cfg = hop_loop_cfg();
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // One full hop ≈ load .22 + launch .12 + short flight + land .25.
        run(&cfg, &mut ctrl, &standing(), 26); // grounded throughout
                                               // Let flight terminate on contact immediately (feet always touching).
        run(&cfg, &mut ctrl, &standing(), 40);
        assert!(ctrl.hops() >= 1, "the stroke re-armed after landing");
    }

    #[test]
    fn land_waits_for_the_ground() {
        // Commanded contact gone (airborne) ⇒ Land must not hand over;
        // stance-on in the air does no harm (strut target vs free feet).
        let cfg = clock_cfg();
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // Run to the end of flight without any contact.
        run(&cfg, &mut ctrl, &airborne(), 75);
        assert_eq!(
            ctrl.phase,
            JumpPhase::Land,
            "flight clock expired into Land (airborne)"
        );
        ctrl.step(
            &Command::jump_vertical(100.0),
            &airborne(),
            &cfg,
            1.0 / 90.0,
            1.0,
        );
        assert_eq!(ctrl.phase, JumpPhase::Land, "still no ground, no re-arm");
    }

    #[test]
    fn the_gate_holds_launch_until_the_chord_actually_compresses() {
        // Static standing body: the chord never compresses, so the cursor must
        // stall at the Load→Launch boundary instead of firing the extension
        // into an unbraced stance. This is the whole point of §11.
        let cfg = ControlConfig {
            jump_fire_r: 0.8,
            jump_load_cap: 5.0, // effectively "no cap" for this test
            ..ControlConfig::default()
        };
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // Enough ticks to pass the clip's 0.22 s load and then some.
        run(&cfg, &mut ctrl, &standing(), 60);
        assert_eq!(
            ctrl.phase,
            JumpPhase::Load,
            "an unloaded body must not fire the power stroke"
        );
        assert!(!ctrl.fired, "the gate never latched");
    }

    #[test]
    fn the_gate_releases_the_moment_the_measured_chord_compresses() {
        let cfg = ControlConfig {
            jump_fire_r: 0.8,
            jump_load_cap: 5.0,
            ..ControlConfig::default()
        };
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // Pass the load window still unloaded: still held.
        run(&cfg, &mut ctrl, &standing(), 21);
        assert_eq!(ctrl.phase, JumpPhase::Load, "still held before load");
        // Now the measured chord compresses past 0.8: the very next tick fires.
        ctrl.step(
            &Command::jump_vertical(100.0),
            &crouched(0.7),
            &cfg,
            1.0 / 90.0,
            1.0,
        );
        assert!(ctrl.fired, "compression latched the gate");
        assert_eq!(ctrl.phase, JumpPhase::Launch, "the power stroke begins");
    }

    #[test]
    fn the_gate_cap_fires_even_a_body_that_never_loads() {
        // Fallback: a body that never compresses must still launch at the cap,
        // so the stroke can never hang.
        let cfg = ControlConfig {
            jump_fire_r: 0.8,
            jump_load_cap: 0.10, // 9 ticks past the boundary
            ..ControlConfig::default()
        };
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        run(&cfg, &mut ctrl, &standing(), 60);
        assert!(ctrl.fired, "the cap released the gate");
        assert_ne!(ctrl.phase, JumpPhase::Load, "the stroke got past Load");
    }

    #[test]
    fn a_disabled_gate_is_the_pure_clock_replay() {
        // `jump_fire_r <= 0` is the pre-§11 behaviour: launch on the clock even
        // with an uncompressed body.
        let cfg = ControlConfig {
            jump_fire_r: 0.0,
            ..ControlConfig::default()
        };
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        run(&cfg, &mut ctrl, &standing(), 26);
        assert!(ctrl.fired, "no gate ⇒ latched on the clock");
        assert_eq!(ctrl.phase, JumpPhase::Launch, "fired on schedule");
    }

    #[test]
    fn each_re_hop_has_to_earn_its_own_launch() {
        // The gate re-arms at every re-hop: a compressed first hop must not
        // leave the latch open for a second hop that never loads.
        let cfg = ControlConfig {
            jump_fire_r: 0.8,
            jump_load_cap: 5.0,
            // Re-hop loop: no landing hold, so the second hop can arm.
            jump_land_hold: -1.0,
            ..ControlConfig::default()
        };
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // Hop 1: compressed throughout, so it fires and completes.
        run(&cfg, &mut ctrl, &crouched(0.7), 120);
        assert!(ctrl.hops() >= 1, "the compressed hop completed");
        assert!(!ctrl.fired, "the latch re-armed for the next hop");
    }

    #[test]
    fn touchdown_is_not_the_end_of_the_landing() {
        // The point of the landing hold: the stroke must NOT declare a landing
        // on the touchdown tick. It plays the authored absorb, then holds a
        // stable stance for `jump_land_hold` before `landing_complete` latches.
        let cfg = ControlConfig {
            jump_fire_r: 0.0,
            jump_land_hold: 1.0,
            jump_land_lean: 0.35,
            ..ControlConfig::default()
        };
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // Grounded throughout: the flight-clock ends early on contact, so the
        // stroke lands, plays the absorb and then enters the hold.
        // The hold runs 1.0 s ≈ 90 ticks from touchdown; the absorb (~0.25 s)
        // is inside it. Stop at 80 ticks after entry: still short of the hold.
        run(&cfg, &mut ctrl, &standing(), 80);
        assert!(
            !ctrl.landing_complete(&cfg),
            "touchdown + absorb alone is not a completed landing"
        );

        // Now give it the remaining hold: the stroke reaches the 1 s mark within
        // another 1.5 s of stable ground.
        run(&cfg, &mut ctrl, &standing(), 140);
        assert!(
            ctrl.landing_complete(&cfg),
            "after the hold the landing is complete"
        );
        assert_eq!(
            ctrl.hops(),
            1,
            "exactly one hop, no re-hop past the landing"
        );
    }

    #[test]
    fn the_default_landing_window_is_one_and_a_half_seconds() {
        // The shipped default (1.5 s) must survive a full stroke + hold without a
        // timeout truncation, and must not complete before 1.5 s of ground.
        let cfg = ControlConfig {
            jump_fire_r: 0.0,
            ..ControlConfig::default()
        };
        assert_eq!(cfg.jump_land_hold, 1.5, "default hold is 1.5 s");
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        // 1.4 s of grounded stroke+hold: still short of 1.5 s.
        run(&cfg, &mut ctrl, &standing(), 126);
        assert!(
            !ctrl.landing_complete(&cfg),
            "1.4 s of ground is not yet 1.5 s of hold"
        );
        // Past 1.5 s it completes.
        run(&cfg, &mut ctrl, &standing(), 40);
        assert!(
            ctrl.landing_complete(&cfg),
            "the 1.5 s default hold completes"
        );
    }

    #[test]
    fn a_toppling_body_never_completes_the_landing() {
        // The hold's lean band: a body that keeps tipping must never be declared
        // landed (the fall gate owns that failure). Drive a full stroke on a
        // heavily leaned fiction.
        let cfg = ControlConfig {
            jump_fire_r: 0.0,
            jump_land_hold: 0.2,
            jump_land_lean: 0.35,
            ..ControlConfig::default()
        };
        let mut leaned = standing();
        leaned.lean = 1.2; // well past the band
        let mut ctrl = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        run(&cfg, &mut ctrl, &leaned, 200);
        assert!(
            !ctrl.landing_complete(&cfg),
            "a toppling body is never 'landed'"
        );
    }

    #[test]
    fn the_stroke_is_deterministic() {
        let cfg = ControlConfig::default();
        let mut a = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        let mut b = JumpCtrl::new(&walker_math::LowerBodyConfig::default());
        for _ in 0..120 {
            let oa = a.step(
                &Command::jump_vertical(100.0),
                &standing(),
                &cfg,
                1.0 / 90.0,
                1.0,
            );
            let ob = b.step(
                &Command::jump_vertical(100.0),
                &standing(),
                &cfg,
                1.0 / 90.0,
                1.0,
            );
            assert_eq!(oa.pose.left_foot.pos, ob.pose.left_foot.pos);
            assert_eq!(oa.pose.right_foot.pos, ob.pose.right_foot.pos);
        }
    }
}
