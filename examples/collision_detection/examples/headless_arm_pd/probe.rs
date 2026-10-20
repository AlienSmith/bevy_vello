//! The harness state resource + the per-cell accumulation structs.
//!
//! [`Probe`] is the single `Resource` the dispatcher, sampler, and reporter share;
//! [`Sample`] is one telemetry sample and [`CellReport`] is the classified result
//! of one `(Kp,Kd)` cell.

use bevy::prelude::{Entity, Resource, Vec2};

use crate::command::{Command, LimbId};

/// One telemetry sample (Bevy y-up frame) — the minimum the classifier reads.
#[derive(Clone, Copy)]
pub struct Sample {
    pub tau: f32,
    /// Script action index this sample belongs to.
    pub act: usize,
    pub theta: f32, // end-bone angle (rad), Bevy frame
    pub omega: f32, // end-bone angular velocity (rad/s)
}

/// The classified result of one `(Kp,Kd)` cell — the deliverable readout.
pub struct CellReport {
    pub cell: usize,
    pub kp: f32,
    pub kd: f32,
    pub verdict: String,
    pub swing_deg: f32,
    pub rms_deg: f32,
    pub catchup_deg: f32,
    pub reversals: u32,
    pub settle_s: Option<f32>,
    pub max_omega: f32,
    pub inv_ok: bool,
    pub inv_detail: String,
    pub hash: u64,
}

#[derive(Resource)]
pub struct Probe {
    pub character: Option<Entity>,
    /// Monotonic fixed-tick counter (the harness clock).
    pub tick: u64,
    pub spawn_tick: Option<u64>,
    pub ever_ready: bool,

    // --- rig (captured once, when the focus limb is first ready) ---
    pub focus: LimbId,
    pub ready: bool,
    pub ticks_ready: u64,
    /// Per-limb pivot anchor (world Bevy y-up) = the limb's `particles[1]`.
    pub anchors: [Vec2; 4],
    /// Per-limb characteristic length = upper + lower bone, in px.
    pub limb_len: [f32; 4],
    pub anchored: bool,
    /// Resolved `TARGET_DIST` override for `aim` (px; ≤0 = auto).
    pub target_dist: f32,

    // --- script ---
    pub script: Vec<Command>,
    pub script_src: String,
    pub action: usize,
    pub action_start_tick: u64,
    pub applied_action: Option<usize>,
    pub script_done: bool,

    // --- sweep ---
    pub grid: Vec<(f32, f32)>,
    pub cell: usize,

    // --- per-cell accumulation ---
    pub kp: f32,
    pub kd: f32,
    pub samples: Vec<Sample>,
    pub reports: Vec<CellReport>,
    pub done: bool,
    pub csv_path: String,
    pub sweep: bool,

    // --- whole-body (hip-pin + legs) mode ---
    /// True when the script drives the whole body (`move`/`jump` present, or
    /// `HIPPIN` set): the pin is attached, `drive_hip_pin` runs, and the
    /// track/rigidity/jump→walk telemetry is folded into the row.
    pub body: bool,
    /// Previous held key mask, for edge-detecting the scripted body input.
    pub held_prev: u8,
    /// Events sidecar rows (`t,event_id,label,keys,kind`).
    pub events: Vec<String>,
    pub next_event_id: u32,
    /// Cached `P3` particle entity (the realized hip) for the jump→walk trace.
    pub p3_entity: Option<Entity>,
    /// The jump→walk window `(begin, end)` in seconds, derived from the script
    /// (the `move` immediately after a `jump`); `None` disables the verdict.
    pub jw_window: Option<(f32, f32)>,
    pub jw_prev_cmd_vx: f32,
    pub jw_prev_real_vx: f32,
    pub jw_primed: bool,
    pub jw_cmd_step: f32,
    pub jw_cmd_step_t: f32,
    pub jw_real_step: f32,
    pub jw_real_step_t: f32,
    pub jw_lag: f32,
    pub jw_lag_t: f32,
    pub jw_phase: u8,
    pub nan_rows: u32,
    /// Per-leg realized foot lag vs the commanded IK target.
    pub leg_active: bool,
    pub max_leg_err: f32,
    pub leg_peak_t: f32,
    pub max_leg_err_lr: [f32; 2],
    pub max_leg_err_lr_t: [f32; 2],
    pub leg_t: Vec<f32>,
    pub leg_err: Vec<f32>,
    pub leg_err_lr: Vec<[f32; 2]>,
    /// Signed world-frame x offset `foot.x - target.x` per leg, so a peak can
    /// be classified as lag (foot behind target, negative) vs overshoot (foot
    /// ahead of target, positive) for the +x walking direction.
    pub leg_dx_lr: Vec<[f32; 2]>,
    /// Commanded leg-target velocity (bevy y-up) from the previous tick, one per
    /// lower-body particle in `[P3, P40, P41, PLLL, PRLL]` order, for the
    /// finite-difference target acceleration export.
    pub prev_tgt_vel: [Vec2; 5],
    /// Whether `prev_tgt_vel` holds a real previous tick (false on the first).
    pub tgt_primed: bool,
}

/// Constructor inputs for [`Probe`] — the env/script-derived fields that differ
/// between harnesses (headless vs visual). This keeps the two binaries from
/// duplicating the ~55-field struct literal.
pub struct ProbeArgs {
    pub focus: LimbId,
    pub script: Vec<Command>,
    pub script_src: String,
    pub body: bool,
    pub sweep: bool,
    pub grid: Vec<(f32, f32)>,
    pub kp: f32,
    pub kd: f32,
    pub csv_path: String,
    pub target_dist: f32,
    pub jw_window: Option<(f32, f32)>,
}

impl Probe {
    pub fn new(args: ProbeArgs) -> Self {
        Self {
            character: None,
            tick: 0,
            spawn_tick: None,
            ever_ready: false,
            focus: args.focus,
            ready: false,
            ticks_ready: 0,
            anchors: [Vec2::ZERO; 4],
            limb_len: [0.0; 4],
            anchored: false,
            target_dist: args.target_dist,
            script: args.script,
            script_src: args.script_src,
            action: 0,
            action_start_tick: 0,
            applied_action: None,
            script_done: false,
            kp: args.kp,
            kd: args.kd,
            grid: args.grid,
            cell: 0,
            samples: Vec::new(),
            reports: Vec::new(),
            done: false,
            csv_path: args.csv_path,
            sweep: args.sweep,
            body: args.body,
            held_prev: 0,
            events: Vec::new(),
            next_event_id: 0,
            p3_entity: None,
            jw_window: args.jw_window,
            jw_prev_cmd_vx: 0.0,
            jw_prev_real_vx: 0.0,
            jw_primed: false,
            jw_cmd_step: 0.0,
            jw_cmd_step_t: 0.0,
            jw_real_step: 0.0,
            jw_real_step_t: 0.0,
            jw_lag: 0.0,
            jw_lag_t: 0.0,
            jw_phase: 0,
            nan_rows: 0,
            leg_active: false,
            max_leg_err: 0.0,
            leg_peak_t: 0.0,
            max_leg_err_lr: [0.0, 0.0],
            max_leg_err_lr_t: [0.0, 0.0],
            leg_t: Vec::new(),
            leg_err: Vec::new(),
            leg_err_lr: Vec::new(),
            leg_dx_lr: Vec::new(),
            prev_tgt_vel: [Vec2::ZERO; 5],
            tgt_primed: false,
        }
    }
}
