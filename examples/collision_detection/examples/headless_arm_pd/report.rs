//! Whole-body (hip-pin) reporting: translate the command playhead into the real
//! `ButtonInput<KeyCode>` for the leafwing → hip-pin input path, then post-process
//! the captured buffer into the body verdicts (frame/leg bone rigidity, jump→walk
//! handoff, foot-lag wobble) and write the `.events.csv` / `.leg.csv` sidecars.

use bevy::input::keyboard::KeyCode;
use bevy::input::ButtonInput;
use bevy::prelude::{error, info, ResMut};

use game_lib::{
    check_bone_rigidity, tail_verdict, Bone, GaitPhase, HipPinParams, TelemetryBuffer,
    VerticalPhase,
};

use crate::config::{
    FIXED_HZ, KEYS, LEG_RATE_DEADBAND, LEG_REV_MAX, LEG_RMS_MAX, LEG_TAIL_S, PIN_RIGID_TOL,
};
use crate::probe::Probe;

/// Compact code for the vertical phase, for the `vphase` CSV column (matches the
/// `VerticalPhase` declaration order).
pub fn vphase_code(p: VerticalPhase) -> u8 {
    match p {
        VerticalPhase::Stand => 0,
        VerticalPhase::Load => 1,
        VerticalPhase::Launch => 2,
        VerticalPhase::Flight => 3,
        VerticalPhase::Land => 4,
    }
}

/// Compact code for the per-leg gait phase, for the `gait_l`/`gait_r` CSV
/// columns (matches the `GaitPhase` declaration order). 0 = Stance, 1 = Swing.
pub fn gait_code(p: GaitPhase) -> u8 {
    match p {
        GaitPhase::Stance => 0,
        GaitPhase::Swing => 1,
    }
}

/// `pin_log.csv` + `.events.csv` → `pin_log.events.csv` (same dir, suffix before
/// extension).
pub fn path_with_suffix(main: &str, suffix: &str) -> String {
    let p = std::path::Path::new(main);
    let stem = p.file_stem().and_then(|s| s.to_str()).unwrap_or("log");
    match p.parent() {
        Some(dir) => dir
            .join(format!("{stem}{suffix}"))
            .to_string_lossy()
            .to_string(),
        None => format!("{stem}{suffix}"),
    }
}

/// Translate the active `move`/`jump` command into the real `ButtonInput<KeyCode>`
/// (edge transitions only, so held keys stay held) — the whole
/// `keyboard → leafwing ActionState → hip pin` path, same as the retired
/// `headless_pin` harness, but driven by the command playhead. Also records the
/// input timeline for the events sidecar.
pub fn drive_body_keys(mut keys: ResMut<ButtonInput<KeyCode>>, mut probe: ResMut<Probe>) {
    if !probe.body || probe.character.is_none() {
        return;
    }
    let cur = probe.action.min(probe.script.len().saturating_sub(1));
    let mask = if probe.script_done {
        0
    } else {
        probe.script.get(cur).map(|c| c.active_mask()).unwrap_or(0)
    };
    let kind: &'static str = if probe.script_done {
        "done"
    } else {
        probe.script.get(cur).map(|c| c.kind()).unwrap_or("none")
    };
    let t = probe.tick as f32 / FIXED_HZ as f32;
    for i in 0..3 {
        let bit = 1u8 << i;
        let now = mask & bit != 0;
        let was = probe.held_prev & bit != 0;
        if now && !was {
            keys.press(KEYS[i]);
            let id = probe.next_event_id;
            probe.next_event_id += 1;
            probe.events.push(format!("{t:.4},{id},{kind},{mask},0"));
        } else if !now && was {
            keys.release(KEYS[i]);
            let id = probe.next_event_id;
            probe.next_event_id += 1;
            probe.events.push(format!("{t:.4},{id},{kind},{mask},1"));
        }
    }
    probe.held_prev = mask;
}

/// Whole-body verdicts, computed from the captured buffer + the accumulators:
/// the pinned-frame (and leg) **bone rigidity**, the **jump→walk** commanded-vs-
/// realized hip-velocity comparison, the **leg foot-lag/wobble** verdict, and the
/// `<csv>.events.csv` input timeline. Pure post-processing (no ECS writes).
pub fn report_body(probe: &Probe, buf: &TelemetryBuffer, params: Option<&HipPinParams>) {
    // Rigid bones that must keep their length: the pinned `P1/P2/P3` frame (the
    // pin's job) and, when the pin drives the legs, both legs' thigh + calf.
    let mut bones = vec![
        Bone {
            a: ("P1x".into(), "P1y".into()),
            b: ("P2x".into(), "P2y".into()),
            rest_len: 0.0,
        },
        Bone {
            a: ("P2x".into(), "P2y".into()),
            b: ("P3x".into(), "P3y".into()),
            rest_len: 0.0,
        },
        Bone {
            a: ("P1x".into(), "P1y".into()),
            b: ("P3x".into(), "P3y".into()),
            rest_len: 0.0,
        },
    ];
    let mut labels: Vec<String> = vec!["P1-P2".into(), "P2-P3".into(), "P1-P3".into()];
    let legs_on = params.map(|p| p.hip_drives_legs).unwrap_or(false) && probe.leg_active;
    if legs_on {
        for (a, b) in [
            ("P3", "P40"),
            ("P40", "PLLL"),
            ("P3", "P41"),
            ("P41", "PRLL"),
        ] {
            bones.push(Bone {
                a: (format!("{a}x"), format!("{a}y")),
                b: (format!("{b}x"), format!("{b}y")),
                rest_len: 0.0,
            });
            labels.push(format!("{a}-{b}"));
        }
    }
    let rig = check_bone_rigidity(&buf.header, &buf.rows, &bones, PIN_RIGID_TOL);
    let worst_rel = rig.max_rel_dev.iter().copied().fold(0.0f32, f32::max);
    let mut rig_detail = String::new();
    for (i, nm) in labels.iter().enumerate() {
        let tt = buf
            .rows
            .get(rig.worst_row[i])
            .and_then(|r| r.split(',').next())
            .unwrap_or("?");
        rig_detail += &format!(" {nm}={:.4}@t{tt}", rig.max_rel_dev[i]);
    }
    info!(
        "body: rows={} nan_rows={} rig={} max_rel_dev={:.4}",
        buf.rows.len(),
        probe.nan_rows,
        if rig.ok { "ok" } else { "FAIL" },
        worst_rel,
    );
    info!("rigidity:{}", rig_detail);

    if let Some((b, e)) = probe.jw_window {
        info!(
            "jumpwalk(t{:.1}-{:.1}s): handoff_phase={} cmd_step={:.2}px/s@t{:.2} real_step={:.2}px/s@t{:.2} cmd_vs_real_gap={:.2}px/s@t{:.2}",
            b, e, probe.jw_phase, probe.jw_cmd_step, probe.jw_cmd_step_t,
            probe.jw_real_step, probe.jw_real_step_t, probe.jw_lag, probe.jw_lag_t,
        );
    }

    if legs_on {
        let mut rate = vec![0.0f32; probe.leg_err.len()];
        for i in 1..probe.leg_err.len() {
            let d = (probe.leg_t[i] - probe.leg_t[i - 1]).max(1e-6);
            rate[i] = (probe.leg_err[i] - probe.leg_err[i - 1]) / d;
        }
        let v = tail_verdict(
            &probe.leg_t,
            &probe.leg_err,
            &rate,
            LEG_TAIL_S,
            LEG_RMS_MAX,
            LEG_REV_MAX,
            LEG_RATE_DEADBAND,
        );
        info!(
            "leg: peak left={:.1}px@t{:.2} right={:.1}px@t{:.2} combined={:.1}px@t{:.2} tail={}(rms={:.3}px rev={})",
            probe.max_leg_err_lr[0], probe.max_leg_err_lr_t[0],
            probe.max_leg_err_lr[1], probe.max_leg_err_lr_t[1],
            probe.max_leg_err, probe.leg_peak_t,
            if v.pass { "PASS" } else { "WAGGLE" }, v.rms_deg, v.reversals,
        );
    }

    // The input timeline sidecar, so any CSV slice can be tied to a key event.
    let ev_path = path_with_suffix(&probe.csv_path, ".events.csv");
    let mut ev_out = String::from("t,event_id,label,keys,kind\n");
    if !probe.events.is_empty() {
        ev_out += &(probe.events.join("\n") + "\n");
    }
    if let Some(dir) = std::path::Path::new(&ev_path).parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    match std::fs::write(&ev_path, ev_out) {
        Ok(_) => info!("wrote {ev_path}"),
        Err(e) => error!("write failed ({ev_path}): {e}"),
    }

    // Per-leg foot-lag trace so the worst lag can be tied to a gait phase. The
    // signed `*_dx_px` columns are `foot.x - target.x`: negative = lagging
    // behind the +x command, positive = overshooting ahead of it.
    if legs_on && !probe.leg_t.is_empty() {
        let leg_path = path_with_suffix(&probe.csv_path, ".leg.csv");
        let mut leg_out = String::from("t,left_err_px,right_err_px,left_dx_px,right_dx_px\n");
        for (i, tt) in probe.leg_t.iter().enumerate() {
            let lr = probe.leg_err_lr.get(i).copied().unwrap_or([0.0, 0.0]);
            let dx = probe.leg_dx_lr.get(i).copied().unwrap_or([0.0, 0.0]);
            leg_out += &format!(
                "{tt:.4},{:.3},{:.3},{:.3},{:.3}\n",
                lr[0], lr[1], dx[0], dx[1]
            );
        }
        if let Some(dir) = std::path::Path::new(&leg_path).parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        match std::fs::write(&leg_path, leg_out) {
            Ok(_) => info!("wrote {leg_path}"),
            Err(e) => error!("write failed ({leg_path}): {e}"),
        }
    }
}
