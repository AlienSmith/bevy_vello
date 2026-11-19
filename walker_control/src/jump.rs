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
//! The stroke is time-phased and deliberately **open-loop** (no height
//! feedback yet): the authored expert PPO will later be trained against. When
//! the measured hop is right, the same phases get a closed Δl law on top.

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
    /// Stroke-local phase boundaries (seconds): load end, launch end, flight
    /// end. From [`crate::jump_clip::jump_clip`]; kept here so a re-hop can
    /// rebase `t` cheaply.
    load_end: f32,
    launch_end: f32,
    flight_end: f32,
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
            load_end: ts.get(1).copied().unwrap_or(0.22),
            launch_end: ts.get(2).copied().unwrap_or(0.34),
            flight_end: ts.get(3).copied().unwrap_or(0.79),
            leg_len,
        }
    }

    /// Completed hops (for tests and later telemetry).
    pub fn hops(&self) -> u32 {
        self.hops
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
        _cfg: &ControlConfig,
        dt: f32,
        facing: f32,
    ) -> ControlOut {
        let dt = dt.max(0.0);
        self.t += dt;

        // Phase boundaries come straight from the clip; the phase itself is a
        // function of the clock. Flight can end **early** on real contact even
        // though the clip's shape keeps running (a clip cannot see the world).
        // The contact check only fires once flight is already established (the
        // entry tick still reads the feet touching, which is the *launch*, not
        // a landing).
        let was_flight = self.phase == JumpPhase::Flight;
        self.phase = crate::jump_clip::phase_at(self.t);
        let contact = fiction.feet.iter().any(|f| f.touching);
        if was_flight && self.phase == JumpPhase::Flight && contact {
            // Touchdown: skip ahead to the clip's Land segment. The wrap keeps
            // the pose continuous (the Land key starts at rest, the Flight key
            // is rest too, so this is a soft jump).
            self.t = self.flight_end;
            self.phase = JumpPhase::Land;
        }
        // Re-hop: once the Land segment has played out while still grounded and
        // the intent is held, rebase the clock to the top of the clip.
        if self.phase == JumpPhase::Land && self.t >= self.flight_end + 0.25 && fiction.grounded {
            self.hops += 1;
            self.t = 0.0;
            self.phase = JumpPhase::Load;
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
            jump_impulse: 0.0,
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
        let cfg = ControlConfig::default();
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
        let cfg = ControlConfig::default();
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
        let cfg = ControlConfig::default();
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
        let cfg = ControlConfig::default();
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
