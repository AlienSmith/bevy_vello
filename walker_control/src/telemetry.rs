//! The **telemetry** module — fidelity-preserving CSV export (plan §3b).
//!
//! The frozen harness header is 110 columns: 24 harness scalars, 30 commanded
//! lower-body targets, then 52 generic particle channels that `capture_telemetry`
//! fills from the live particles. This module produces the **scalar block**
//! (the first 54 columns plus the two appended ones). The particle channels are
//! untouched by the rewrite, so ~40 of the old columns keep reporting exactly
//! what they always did — that is the A/B anchor for the validation gates.
//!
//! # The record lives on the character, one writer per field
//! [`TickRecord`] is a field of [`crate::walker::Walker`], not a shared
//! resource: the record's lifetime is the character's lifetime, and multiple
//! characters need no keying or bookkeeping. Each block has exactly **one
//! producer**, which is the same ownership rule every other `Walker` field
//! follows:
//!
//! | block | producer | method |
//! |---|---|---|
//! | reality (`v_filt`, attitude, contacts, `grounded`) | [`crate::walker::observe_stage`] | [`TickRecord::write_reality`] |
//! | command (`cmd_vx`, `v_des`, gait, `mode`, target feet/hip) | [`crate::walker::control_stage`] | [`TickRecord::write_control`] |
//! | knees (`tP40`/`tP41`) | [`crate::walker::actuate_stage`] | [`TickRecord::write_knees`] |
//! | playhead/classifier (`tau`, `act`, `kind`, `kp`, `kd`, `theta`, `omega`) | the host harness | [`TickRecord::harness`] |
//!
//! The formatter is the only place a value becomes text, so precision and
//! column order have one owner: [`TickRecord::format`].
//!
//! # Where each column comes from now
//!
//! | column(s) | old source | new source |
//! |---|---|---|
//! | `tau`,`t`,`act`,`kind`,`kp`,`kd`,`theta_deg`,`omega` | playhead/limb classifier (`Probe`) | [`HarnessSlice`] (host-written) |
//! | `cmd_vx` | `pin.state.hip_vel.x` (commanded) | `ControlOut.hip_vel.x` |
//! | `v_filt` | `pin.state.v_meas_filt` (EMA of measured CoM speed) | `Fiction.v_com.x` (the observer's EMA) |
//! | `v_des` | `pin.state.v_ref` | `Command::vx()` |
//! | `vphase` | `pin.state.vs.phase` | **0** — no vertical phase in the new control module |
//! | `gait_l`,`gait_r` | resolver leg phase | `ControlOut.pose.*_foot.stance` (0 = stance, 1 = swing) |
//! | `body_theta`,`body_omega`,`meas_theta` | pin attitude state | `Observer::raw.lean` / `raw.lean_rate` (pre-filter) |
//! | `att_torque`,`hip_accel_x` | pin latched loop-3 internals | **0** — loop 3 is applied by actuate, not a torque |
//! | `att_enabled` | `HipPinParams.attitude_enabled` | **0** — no on/off fork (Law 4) |
//! | `ext_pin` | `pin.last_external_pin` | `false` — the pin is gone (Law 1) |
//! | `att_target` | `pin.last_att_target` | `ControlOut.pose.upper_angle` |
//! | `att_rest_cos`,`att_rest_sin` | emitted rest basis | **1.0 / 0.0** — no rest basis is emitted; the rest is upright |
//! | `ns_l`,`ns_r` | contact closing speed | **0** — no normal channel in `Fiction` yet |
//! | `tP3*` | `left_leg.hip_target`/`hip_vel` | `ControlOut.pose.hip` + `hip_vel` |
//! | `tP40*`,`tP41*` | frozen spawn values | [`crate::actuate::ik_knee_target`] — the knee actuation aims at |
//! | `tPLLL*`,`tPRLL*` | `left/right_leg.target` + `target_vel` | `ControlOut.pose.*_foot.pos` + finite difference |
//!
//! Two columns are **appended** after `tPRLLay`: `mode` and `grounded`. Appending
//! keeps every pre-existing column at its frozen index, so the fixture header and
//! `judge.rs`'s rectangular/rows-hash checks stay valid.
//!
//! # Frame
//! Everything here is Vello **y-down**, the frame the old `t*` columns were
//! exported in, so `t<name>x - <name>x` remains directly readable as tracking
//! error.

use glam::Vec2;

use crate::actuate::Pose;
use crate::control::{Command, ControlOut, LocoMode};
use crate::observe::{Fiction, Observer};

/// Columns of the scalar block, in the exact `TelemetryRow::set` order used by
/// the harness, plus the two appended columns. The formatter emits exactly these
/// keys, in exactly this order.
pub const SCALAR_COLUMNS: [&str; 56] = [
    "tau",
    "t",
    "act",
    "kind",
    "kp",
    "kd",
    "theta_deg",
    "omega",
    "cmd_vx",
    "v_filt",
    "v_des",
    "vphase",
    "gait_l",
    "gait_r",
    "body_theta",
    "body_omega",
    "att_torque",
    "hip_accel_x",
    "att_enabled",
    "ext_pin",
    "att_target",
    "att_rest_cos",
    "att_rest_sin",
    "meas_theta",
    "tP3x",
    "tP3y",
    "tP3vx",
    "tP3vy",
    "tP3ax",
    "tP3ay",
    "tP40x",
    "tP40y",
    "tP40vx",
    "tP40vy",
    "tP40ax",
    "tP40ay",
    "tP41x",
    "tP41y",
    "tP41vx",
    "tP41vy",
    "tP41ax",
    "tP41ay",
    "tPLLLx",
    "tPLLLy",
    "tPLLLvx",
    "tPLLLvy",
    "tPLLLax",
    "tPLLLay",
    "tPRLLx",
    "tPRLLy",
    "tPRLLvx",
    "tPRLLvy",
    "tPRLLax",
    "tPRLLay",
    "mode",
    "grounded",
];

/// The commanded-target column groups, in header order: `tP3` (hip), `tP40`
/// (left knee), `tP41` (right knee), `tPLLL` (left foot), `tPRLL` (right foot).
const TARGET_NAMES: [&str; 5] = ["tP3", "tP40", "tP41", "tPLLL", "tPRLL"];

/// The *measured* particle channels appended after the scalar block, in the
/// exact `game_lib::V10_PARTICLES` order the old headless harnesses used
/// (`capture_telemetry` emitted `x,y,vx,vy` per name, raw Vello y-down frame).
///
/// The observer supplies the hip (`P3`) and both feet (`PLLL`/`PRLL`); every
/// other slot is zero-filled. The full rectangular header is the contract with
/// the analysis tools (`plot_p3.py`, `pose_viewer.py`), so an absent feature
/// exports zeros rather than dropped columns — the same law as the scalars.
pub const REALITY_PARTICLES: [&str; 13] = [
    "P0", "P1", "P2", "P3", "P40", "P41", "P10", "P13", "PH", "PLLA", "PRLA", "PRLL", "PLLL",
];

/// The complete legacy-format header: the frozen scalar block, then `x`, `y`,
/// `vx`, `vy` for every [`REALITY_PARTICLES`] name. A CSV written with this
/// header and [`TickRecord::format`] rows loads directly in the old tools.
pub fn legacy_header() -> String {
    let mut cols: Vec<String> = SCALAR_COLUMNS.iter().map(|s| (*s).to_string()).collect();
    for name in REALITY_PARTICLES {
        cols.push(format!("{name}x"));
        cols.push(format!("{name}y"));
        cols.push(format!("{name}vx"));
        cols.push(format!("{name}vy"));
    }
    cols.join(",")
}

/// Stable CSV spelling of a mode (`mode` column).
pub fn mode_name(mode: LocoMode) -> &'static str {
    match mode {
        LocoMode::Standing => "standing",
        LocoMode::Walking => "walking",
        LocoMode::BoostStart => "boost",
    }
}

/// `0` = stance (loaded support), `1` = swing — the frozen `gait_*` encoding.
fn gait_code(stance: bool) -> String {
    if stance {
        "0".to_string()
    } else {
        "1".to_string()
    }
}

fn finite_difference(cur: Vec2, prev: Vec2, dt: f32) -> Vec2 {
    if dt <= 0.0 {
        Vec2::ZERO
    } else {
        (cur - prev) / dt
    }
}

/// The host harness's per-tick playhead/classifier values. Written by whichever
/// system owns the script playhead and limb classifier; the walker crate never
/// writes it, it only reads it when formatting.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct HarnessSlice {
    /// Script-relative playhead, seconds.
    pub tau: f32,
    /// Index of the active script cell.
    pub act: usize,
    /// Script cell kind (e.g. `"walk"`, `"none"`).
    pub kind: String,
    /// Proportional gain the limb classifier tracked with.
    pub kp: f32,
    /// Derivative gain the limb classifier tracked with.
    pub kd: f32,
    /// Tracked limb angle, radians.
    pub theta: f32,
    /// Tracked limb angular velocity, rad/s.
    pub omega: f32,
}

impl HarnessSlice {
    /// `kind` defaults to `"none"` on the wire rather than the empty string, so
    /// an unwritten harness slice still reads like the old `unwrap_or("none")`.
    pub fn new() -> Self {
        Self {
            kind: "none".to_string(),
            ..Self::default()
        }
    }
}

/// Finite-difference history for the commanded target columns.
///
/// Two histories are kept because the two derivatives need two different
/// differences: velocity from **positions**, acceleration from **velocities**.
/// (One shared slot is the bug that turns a 10 px foot step into a six-figure
/// `ax`.)
#[derive(Debug, Clone, Default)]
struct FdState {
    prev_pos: [Vec2; 5],
    prev_vel: [Vec2; 5],
    primed: bool,
}

/// One tick's telemetry, stored on the character. Every block has one producer
/// (see the module docs); [`TickRecord::format`] is the only formatter.
#[derive(Debug, Clone)]
pub struct TickRecord {
    // ---- written by `observe_stage` (reality block) ----
    /// Filtered CoM velocity along travel (`Fiction.v_com.x`).
    pub v_filt: f32,
    /// Pre-filter measured attitude, rad.
    pub body_theta: f32,
    /// Pre-filter measured attitude rate, rad/s.
    pub body_omega: f32,
    /// Debounced per-foot support (0/1 columns `contact_*`).
    pub contact: [bool; 2],
    /// Raw per-foot touch this tick (0/1 columns `hit_*`).
    pub hit: [bool; 2],
    /// Debounced whole-body support flag.
    pub grounded: bool,
    /// Measured hip (`P3`) position, raw Vello frame.
    pub real_hip: Vec2,
    /// Measured hip velocity (pre-filter — the legacy particle channels are
    /// raw, exactly like the old `capture_telemetry` output).
    pub real_hip_vel: Vec2,
    /// Measured foot positions `[left (PLLL), right (PRLL)]`.
    pub real_feet: [Vec2; 2],
    /// Measured foot velocities `[left, right]`, unfiltered.
    pub real_feet_vel: [Vec2; 2],

    // ---- written by `control_stage` (command block) ----
    /// Commanded hip velocity x (the frozen `cmd_vx` / `tP3vx` signal).
    pub cmd_vx: f32,
    /// Desired speed (`Command::vx()`).
    pub v_des: f32,
    /// Per-foot stance flags, traded to 0/1 by the formatter (`gait_*`).
    pub stance: [bool; 2],
    /// Commanded attitude target, rad.
    pub att_target: f32,
    /// Which controller produced this tick.
    pub mode: LocoMode,
    /// Commanded hip position.
    pub hip: Vec2,
    /// Commanded hip velocity (full vector, for `tP3vy`).
    pub hip_vel: Vec2,
    /// Commanded foot positions.
    pub feet: [Vec2; 2],

    // ---- written by `actuate_stage` (knees) ----
    /// IK knee targets; `None` when the bones were degenerate.
    pub knees: [Option<Vec2>; 2],

    // ---- written by the host harness ----
    pub harness: HarnessSlice,

    // ---- formatter's own bookkeeping ----
    /// Accumulated physics time, seconds — the `t` column. It is *simulation*
    /// time, deliberately not the script playhead (`tau`): the two coincide
    /// only when the script starts at the first tick.
    clock: f32,
    fd: FdState,
}

impl Default for TickRecord {
    /// Absent features are zero-filled (Law: the frozen header never changes).
    /// `stance` and `hip_vel` are the two exceptions that must not default to a
    /// *lie*: the frozen `gait_*` encoding is `0` = stance / `1` = swing, and a
    /// record that has never been written describes a **standing** character,
    /// not a swinging one. So stance starts loaded and `begin_tick` restores
    /// that same benign value if the control stage stalls a tick.
    fn default() -> Self {
        Self {
            v_filt: 0.0,
            body_theta: 0.0,
            body_omega: 0.0,
            contact: [false; 2],
            hit: [false; 2],
            grounded: false,
            real_hip: Vec2::ZERO,
            real_hip_vel: Vec2::ZERO,
            real_feet: [Vec2::ZERO; 2],
            real_feet_vel: [Vec2::ZERO; 2],
            cmd_vx: 0.0,
            v_des: 0.0,
            stance: [true; 2],
            att_target: 0.0,
            mode: LocoMode::default(),
            hip: Vec2::ZERO,
            hip_vel: Vec2::ZERO,
            feet: [Vec2::ZERO; 2],
            knees: [None; 2],
            harness: HarnessSlice::default(),
            clock: 0.0,
            fd: FdState::default(),
        }
    }
}

impl TickRecord {
    /// Record the reality block. Called by `observe_stage` right after the
    /// fiction update, from the values it just wrote.
    pub fn write_reality(&mut self, observer: &Observer) {
        let fiction = &observer.fiction;
        self.v_filt = fiction.v_com.x;
        self.body_theta = observer.raw.lean;
        self.body_omega = observer.raw.lean_rate;
        for i in 0..2 {
            self.contact[i] = observer.foot_debounce()[i].grounded;
            self.hit[i] = fiction.feet[i].touching;
        }
        self.grounded = fiction.grounded;
        self.real_hip = fiction.hip;
        self.real_hip_vel = observer.raw.v_hip;
        for i in 0..2 {
            self.real_feet[i] = fiction.feet[i].pos;
            self.real_feet_vel[i] = fiction.feet[i].vel;
        }
    }

    /// Record the command block. Called by `control_stage` from the output it
    /// just produced, so the columns can never describe a different pose than
    /// the one being actuated.
    pub fn write_control(&mut self, out: &ControlOut, command: &Command) {
        self.cmd_vx = out.hip_vel.x;
        self.v_des = command.vx();
        self.stance = [out.pose.left_foot.stance, out.pose.right_foot.stance];
        self.att_target = out.pose.upper_angle;
        self.mode = out.mode;
        self.hip = out.pose.hip;
        self.hip_vel = out.hip_vel;
        self.feet = [out.pose.left_foot.pos, out.pose.right_foot.pos];
    }

    /// Record the IK knee targets. Called by `actuate_stage`, which computes
    /// them for the joint commands — the columns are a byproduct of the real
    /// actuation, never a separate derivation.
    pub fn write_knees(&mut self, knees: [Option<Vec2>; 2]) {
        self.knees = knees;
    }

    /// Clear the tick-scoped blocks so a stalled stage cannot report last tick's
    /// values. The harness slice and the FD history survive by design (the
    /// harness owns its lifetime; the history is the derivative's memory).
    pub fn begin_tick(&mut self) {
        self.contact = [false; 2];
        self.hit = [false; 2];
        self.grounded = false;
        self.real_hip = Vec2::ZERO;
        self.real_hip_vel = Vec2::ZERO;
        self.real_feet = [Vec2::ZERO; 2];
        self.real_feet_vel = [Vec2::ZERO; 2];
        self.v_filt = 0.0;
        self.body_theta = 0.0;
        self.body_omega = 0.0;
        self.cmd_vx = 0.0;
        self.v_des = 0.0;
        // Not `[false; 2]`: `gait_*` encodes `1` as *swing*, and a stalled
        // control tick must not claim a swing it never commanded. The benign
        // resting value is the same one `Default` uses.
        self.stance = [true; 2];
        self.att_target = 0.0;
        self.hip = Vec2::ZERO;
        self.hip_vel = Vec2::ZERO;
        self.feet = [Vec2::ZERO; 2];
        self.knees = [None; 2];
    }

    /// The five commanded targets in header order: hip, both knees, both feet.
    fn target_positions(&self) -> [Vec2; 5] {
        [
            self.hip,
            self.knees[0].unwrap_or(Vec2::ZERO),
            self.knees[1].unwrap_or(Vec2::ZERO),
            self.feet[0],
            self.feet[1],
        ]
    }

    /// Format one row of `(key, value)` pairs in [`SCALAR_COLUMNS`] order, and
    /// advance the finite-difference history. `dt` is the physics tick; `t` is
    /// accumulated internally so the record owns its own clock-free column.
    ///
    /// Only the root's velocity is commanded (it is not a derivative); the
    /// knees and feet get a one-tick position difference, and every target's
    /// acceleration is a velocity difference. On the first tick there is no
    /// history, so both derivatives are zero — the same priming the old exporter
    /// used.
    pub fn format(&mut self, dt: f32) -> Vec<(String, String)> {
        let dt = if dt > 0.0 { dt } else { 1.0 / 90.0 };
        self.clock += dt;
        let positions = self.target_positions();

        let primed = self.fd.primed;
        let mut vel = [Vec2::ZERO; 5];
        let mut acc = [Vec2::ZERO; 5];
        for i in 0..5 {
            if i == 0 {
                // Commanded, not differentiated: it exists from tick 0.
                vel[i] = self.hip_vel;
            } else if primed {
                vel[i] = finite_difference(positions[i], self.fd.prev_pos[i], dt);
            }
            if primed {
                acc[i] = finite_difference(vel[i], self.fd.prev_vel[i], dt);
            }
            self.fd.prev_pos[i] = positions[i];
            self.fd.prev_vel[i] = vel[i];
        }
        self.fd.primed = true;

        let mut rows: Vec<(String, String)> = Vec::with_capacity(SCALAR_COLUMNS.len());
        let mut push = |k: &str, v: String| rows.push((k.to_string(), v));

        // ---- Harness-owned playhead / classifier ----
        push("tau", format!("{:.4}", self.harness.tau));
        push("t", format!("{:.4}", self.clock));
        push("act", self.harness.act.to_string());
        push("kind", self.harness.kind.clone());
        push("kp", format!("{:.3}", self.harness.kp));
        push("kd", format!("{:.4}", self.harness.kd));
        push(
            "theta_deg",
            format!("{:.3}", self.harness.theta.to_degrees()),
        );
        push("omega", format!("{:.3}", self.harness.omega));

        // ---- Velocity / command block ----
        push("cmd_vx", format!("{:.2}", self.cmd_vx));
        push("v_filt", format!("{:.2}", self.v_filt));
        push("v_des", format!("{:.2}", self.v_des));
        push("vphase", "0".to_string()); // no vertical phase in the new control module
        push("gait_l", gait_code(self.stance[0]));
        push("gait_r", gait_code(self.stance[1]));

        // ---- Loop 3 / attitude block ----
        push("body_theta", format!("{:.4}", self.body_theta));
        push("body_omega", format!("{:.4}", self.body_omega));
        push("att_torque", "0.00".to_string()); // applied by actuate, not a torque
        push("hip_accel_x", "0.00".to_string()); // not modelled
        push("att_enabled", "0".to_string()); // no on/off fork (Law 4)
        push("ext_pin", "false".to_string()); // the pin is gone (Law 1)
        push("att_target", format!("{:.4}", self.att_target));
        push("att_rest_cos", "1.00000".to_string()); // fixed upright rest
        push("att_rest_sin", "0.00000".to_string());
        push("meas_theta", format!("{:.4}", self.body_theta));

        // ---- Commanded targets: x, y, vx, vy, ax, ay each ----
        for (i, name) in TARGET_NAMES.iter().enumerate() {
            let p = positions[i];
            let v = vel[i];
            let a = acc[i];
            push(&format!("{name}x"), format!("{:.2}", p.x));
            push(&format!("{name}y"), format!("{:.2}", p.y));
            push(&format!("{name}vx"), format!("{:.2}", v.x));
            push(&format!("{name}vy"), format!("{:.2}", v.y));
            push(&format!("{name}ax"), format!("{:.1}", a.x));
            push(&format!("{name}ay"), format!("{:.1}", a.y));
        }

        // ---- Appended: the two signals the new architecture owns ----
        push("mode", mode_name(self.mode).to_string());
        push("grounded", format!("{}", self.grounded as u8));

        // ---- Measured particle channels, legacy V10 order (plot_p3.py,
        // pose_viewer.py): hip + feet real, every other slot zero-filled. ----
        for name in REALITY_PARTICLES {
            let (pos, vel) = match name {
                "P3" => (self.real_hip, self.real_hip_vel),
                "PLLL" => (self.real_feet[0], self.real_feet_vel[0]),
                "PRLL" => (self.real_feet[1], self.real_feet_vel[1]),
                _ => (Vec2::ZERO, Vec2::ZERO),
            };
            push(&format!("{name}x"), format!("{:.2}", pos.x));
            push(&format!("{name}y"), format!("{:.2}", pos.y));
            push(&format!("{name}vx"), format!("{:.2}", vel.x));
            push(&format!("{name}vy"), format!("{:.2}", vel.y));
        }

        rows
    }
}

/// Convenience for adapters that build a record from a bare pose (e.g. a
/// pre-spawn preview). Keeps the "no pose" fallback in one place.
pub fn pose_or_default(pose: Option<Pose>, facing: f32) -> Pose {
    pose.unwrap_or(Pose {
        hip: Vec2::ZERO,
        left_foot: crate::actuate::FootPose::default(),
        right_foot: crate::actuate::FootPose::default(),
        facing,
        upper_angle: 0.0,
    })
}

/// The `v_filt` source for callers without a `TickRecord`.
pub fn filtered_vx(fiction: &Fiction) -> f32 {
    fiction.v_com.x
}

#[cfg(test)]
#[allow(clippy::field_reassign_with_default)] // the tests deliberately start from the unwritten record
mod tests {
    use super::*;
    use crate::actuate::FootPose;
    use crate::control::{ControlConfig, StandingCtrl, Step};
    use crate::observe::FootState;

    fn keys(rows: &[(String, String)]) -> Vec<String> {
        rows.iter().map(|(k, _)| k.clone()).collect()
    }

    fn value<'a>(rows: &'a [(String, String)], key: &str) -> &'a str {
        rows.iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
            .unwrap_or_else(|| panic!("missing column {key}"))
    }

    fn fiction(lean: f32, vx: f32, grounded: bool, touching: [bool; 2]) -> Fiction {
        Fiction {
            dt: 1.0 / 90.0,
            hip: Vec2::new(0.0, -140.0),
            v_hip: Vec2::new(vx, 0.0),
            lean,
            lean_rate: 0.1,
            com: Vec2::new(0.0, -140.0),
            v_com: Vec2::new(vx, 0.0),
            feet: [
                FootState {
                    pos: Vec2::new(-10.0, -90.0),
                    vel: Vec2::ZERO,
                    touching: touching[0],
                },
                FootState {
                    pos: Vec2::new(10.0, -90.0),
                    vel: Vec2::ZERO,
                    touching: touching[1],
                },
            ],
            grounded,
        }
    }

    fn standing_out() -> ControlOut {
        StandingCtrl::new(&walker_math::LowerBodyConfig::default()).step(
            &Command::default(),
            &fiction(0.0, 0.0, true, [true, true]),
            &ControlConfig::default(),
            1.0 / 90.0,
            1.0,
        )
    }

    #[test]
    fn format_emits_exactly_the_frozen_columns_in_order() {
        let mut rec = TickRecord::default();
        let rows = rec.format(1.0 / 90.0);
        // Scalars first, then the legacy particle channels — exactly the
        // `legacy_header()` contract.
        let mut expected: Vec<String> = SCALAR_COLUMNS.iter().map(|s| s.to_string()).collect();
        for name in REALITY_PARTICLES {
            for suffix in ["x", "y", "vx", "vy"] {
                expected.push(format!("{name}{suffix}"));
            }
        }
        assert_eq!(keys(&rows), expected);
        assert_eq!(rows.len(), legacy_header().split(',').count());
    }

    #[test]
    fn an_unwritten_record_is_all_zeros_and_still_rectangular() {
        let mut rec = TickRecord::default();
        let rows = rec.format(1.0 / 90.0);
        assert_eq!(
            rows.len(),
            SCALAR_COLUMNS.len() + 4 * REALITY_PARTICLES.len()
        );
        for key in ["cmd_vx", "v_filt", "v_des", "att_torque", "hip_accel_x"] {
            assert_eq!(value(&rows, key), "0.00", "{key} is zero");
        }
        assert_eq!(value(&rows, "vphase"), "0");
        assert_eq!(value(&rows, "att_enabled"), "0");
        assert_eq!(value(&rows, "ext_pin"), "false");
        assert_eq!(value(&rows, "att_rest_cos"), "1.00000");
        assert_eq!(value(&rows, "att_rest_sin"), "0.00000");
        assert_eq!(value(&rows, "gait_l"), "0");
        assert_eq!(value(&rows, "mode"), "standing");
        assert_eq!(value(&rows, "grounded"), "0");
        assert_eq!(value(&rows, "tP3x"), "0.00");
        // Unwritten reality channels are zero-filled too, so the legacy tools
        // load a stalled record without missing-column crashes.
        assert_eq!(value(&rows, "P3x"), "0.00");
        assert_eq!(value(&rows, "PLLLvx"), "0.00");
    }

    #[test]
    fn write_reality_fills_the_legacy_hip_and_feet_channels() {
        // The observer's fiction carries the measured hip/feet; the other v10
        // particles the old harness logged stay zero-filled.
        let mut observer = Observer::new();
        observer.fiction = fiction(0.0, 0.0, true, [true, true]);
        observer.raw.v_hip = Vec2::new(7.5, -1.0);
        let mut rec = TickRecord::default();
        rec.write_reality(&observer);
        let rows = rec.format(1.0 / 90.0);
        assert_eq!(value(&rows, "P3x"), "0.00");
        assert_eq!(value(&rows, "P3y"), "-140.00");
        assert_eq!(value(&rows, "P3vx"), "7.50", "raw pre-filter hip velocity");
        assert_eq!(value(&rows, "PLLLx"), "-10.00");
        assert_eq!(value(&rows, "PLLLy"), "-90.00");
        assert_eq!(value(&rows, "PRLLx"), "10.00");
        assert_eq!(value(&rows, "P1x"), "0.00", "absent particle ⇒ zero-fill");
        assert_eq!(value(&rows, "P40vy"), "0.00");
    }

    #[test]
    fn write_control_records_the_pose_the_actuator_will_consume() {
        let mut rec = TickRecord::default();
        let out = standing_out();
        rec.write_control(&out, &Command::walk(120.0));
        assert_eq!(rec.mode, LocoMode::Standing);
        assert_eq!(rec.hip, out.pose.hip);
        assert_eq!(rec.feet[0], out.pose.left_foot.pos);

        let rows = rec.format(1.0 / 90.0);
        // `{:.2}` of a +0.0 has no sign to print; `-0.00` only appears if the
        // signal itself is negative zero, which standing never produces.
        assert_eq!(value(&rows, "tP3x"), "0.00");
        // The v10 rest offsets under `P3` (hip at the origin in this fixture).
        assert_eq!(value(&rows, "tPLLLx"), "-33.00");
        assert_eq!(value(&rows, "tPRLLx"), "39.50");
        assert_eq!(value(&rows, "gait_l"), "0");
        assert_eq!(value(&rows, "v_des"), "120.00");
    }

    #[test]
    fn write_reality_records_the_observer_product() {
        let mut observer = Observer::new();
        let f = fiction(0.25, 88.0, true, [true, false]);
        // The observer's debounce needs a couple of ticks to arm; write the
        // fiction directly (this is what `observe()` would have produced).
        observer.fiction = f;
        observer.raw.lean = f.lean;
        observer.raw.lean_rate = f.lean_rate;

        let mut rec = TickRecord::default();
        rec.write_reality(&observer);
        let rows = rec.format(1.0 / 90.0);
        assert_eq!(value(&rows, "v_filt"), "88.00");
        assert_eq!(value(&rows, "body_theta"), "0.2500");
        assert_eq!(value(&rows, "meas_theta"), "0.2500");
        assert_eq!(value(&rows, "grounded"), "1");
    }

    #[test]
    fn each_block_has_exactly_one_writer() {
        // Reality and command are disjoint: writing one cannot perturb the other.
        let mut rec = TickRecord::default();
        let out = standing_out();
        rec.write_control(&out, &Command::default());
        let before = rec.format(1.0 / 90.0);
        let cmd_before = value(&before, "cmd_vx").to_string();

        let mut observer = Observer::new();
        observer.fiction = fiction(0.0, 10.0, true, [true, true]);
        observer.raw.lean = 0.1;
        rec.write_reality(&observer);
        let after = rec.format(1.0 / 90.0);
        assert_eq!(
            value(&after, "cmd_vx"),
            cmd_before,
            "reality must not touch the command block"
        );
        assert_eq!(value(&after, "v_filt"), "10.00");
    }

    #[test]
    fn begin_tick_clears_this_tick_but_keeps_the_harness_slice() {
        let mut rec = TickRecord::default();
        rec.harness = HarnessSlice {
            tau: 1.5,
            act: 3,
            kind: "walk".to_string(),
            kp: 0.5,
            kd: 1.0,
            theta: 0.2,
            omega: 0.05,
        };
        rec.hip = Vec2::new(5.0, -140.0);
        rec.grounded = true;
        rec.begin_tick();
        assert_eq!(rec.hip, Vec2::ZERO);
        assert!(!rec.grounded);
        let rows = rec.format(1.0 / 90.0);
        assert_eq!(value(&rows, "kind"), "walk", "harness owns its lifetime");
        assert_eq!(value(&rows, "act"), "3");
        assert_eq!(value(&rows, "tau"), "1.5000");
    }

    #[test]
    fn harness_slice_defaults_kind_to_none() {
        assert_eq!(HarnessSlice::new().kind, "none");
        assert_eq!(
            HarnessSlice::default().kind,
            "",
            "Default is dumb on purpose"
        );
    }

    #[test]
    fn first_tick_has_no_derivatives_then_position_differences_kick_in() {
        let mut rec = TickRecord::default();
        rec.feet = [Vec2::new(-20.0, -90.0), Vec2::new(20.0, -90.0)];
        let rows1 = rec.format(1.0 / 90.0);
        assert_eq!(value(&rows1, "tPLLLvx"), "0.00", "no history ⇒ no velocity");
        assert_eq!(value(&rows1, "tPLLLax"), "0.0");

        // Move the left foot 10 px in one tick at 90 Hz ⇒ 900 px/s.
        rec.feet[0] = Vec2::new(-10.0, -90.0);
        let rows2 = rec.format(1.0 / 90.0);
        assert_eq!(value(&rows2, "tPLLLvx"), "900.00");
        assert!(value(&rows2, "tPLLLax").parse::<f32>().unwrap().abs() > 1.0);
    }

    #[test]
    fn root_velocity_is_the_commanded_hip_velocity_not_a_difference() {
        // Written through the real producer, not by poking the field: `cmd_vx`
        // comes from the command block and `tP3vx` from the target block, and
        // both must carry the same number. On the first tick a *differentiated*
        // root would read `0.00`; `120.00` proves it is the commanded value.
        let mut rec = TickRecord::default();
        let mut out = standing_out();
        out.hip_vel = Vec2::new(120.0, 0.0);
        rec.write_control(&out, &Command::walk(120.0));
        let rows = rec.format(1.0 / 90.0);
        assert_eq!(value(&rows, "tP3vx"), "120.00");
        assert_eq!(value(&rows, "cmd_vx"), "120.00", "same signal, two columns");
    }

    #[test]
    fn knee_columns_come_from_the_record_not_a_second_derivation() {
        let mut rec = TickRecord::default();
        rec.write_knees([Some(Vec2::new(0.0, -55.0)), None]);
        let rows = rec.format(1.0 / 90.0);
        assert_eq!(value(&rows, "tP40y"), "-55.00");
        assert_eq!(value(&rows, "tP41y"), "0.00", "missing knee ⇒ zeros");
    }

    #[test]
    fn gait_columns_trade_stance_flags_for_the_frozen_encoding() {
        let mut rec = TickRecord::default();
        rec.stance = [true, false];
        let rows = rec.format(1.0 / 90.0);
        assert_eq!(value(&rows, "gait_l"), "0", "stance");
        assert_eq!(value(&rows, "gait_r"), "1", "swing");
    }

    #[test]
    fn mode_column_tracks_the_recorded_mode() {
        for (mode, name) in [
            (LocoMode::Standing, "standing"),
            (LocoMode::Walking, "walking"),
            (LocoMode::BoostStart, "boost"),
        ] {
            let mut rec = TickRecord::default();
            rec.mode = mode;
            let rows = rec.format(1.0 / 90.0);
            assert_eq!(value(&rows, "mode"), name);
        }
    }

    #[test]
    fn formatting_is_deterministic_for_identical_records() {
        let mut a = TickRecord::default();
        let mut b = TickRecord::default();
        let out = standing_out();
        let mut ra = Vec::new();
        let mut rb = Vec::new();
        for tick in 0..20 {
            a.harness.tau = tick as f32 / 90.0;
            b.harness.tau = tick as f32 / 90.0;
            a.write_control(&out, &Command::walk(100.0));
            b.write_control(&out, &Command::walk(100.0));
            ra = a.format(1.0 / 90.0);
            rb = b.format(1.0 / 90.0);
        }
        assert_eq!(ra, rb, "same record ⇒ same row (D1)");
    }

    #[test]
    fn zero_dt_falls_back_to_the_physics_rate() {
        let mut rec = TickRecord::default();
        rec.feet = [Vec2::ZERO, Vec2::ZERO];
        let _ = rec.format(0.0);
        rec.feet[0] = Vec2::new(1.0, 0.0);
        let rows = rec.format(0.0);
        assert_eq!(value(&rows, "tPLLLvx"), "90.00", "1 px at 90 Hz");
    }

    #[test]
    fn pose_or_default_produces_a_finite_pose() {
        let p = pose_or_default(None, 1.0);
        assert_eq!(p.facing, 1.0);
        assert!(p.hip.is_finite());
        assert_eq!(p.left_foot, FootPose::default());
    }
}
