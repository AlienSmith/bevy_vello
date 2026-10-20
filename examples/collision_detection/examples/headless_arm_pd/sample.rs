//! Telemetry collection + cell classification.
//!
//! [`limb_sample`] writes the frozen 104-column row (harness scalars + body
//! telemetry + per-leg gait phase + per-leg foot lag + lower-body target
//! exports) via the shared
//! [`TelemetryRow`]; [`limb_finalize`] closes a cell once the script completes,
//! and [`finalize_cell`] classifies the settled tail.

use bevy::app::AppExit;
use bevy::ecs::event::EventWriter;
use bevy::prelude::{info, Entity, Query, Res, ResMut};

use bevy_vello::integrations::physics::VelloParticle;

use game_lib::{
    cell_verdict, check_invariants, flush_csv, flush_rows, rows_hash, tail_verdict, HipPin,
    HipPinParams, InvariantConfig, LeftArmController, LeftLegController,
    ResetLimbControlConstraintsEvent, RightArmController, RightLegController, TelemetryBuffer,
    TelemetryRow,
};

use crate::command::LimbId;
use crate::config::{
    FIXED_HZ, MIN_SWING_DEG, PI, REVERSAL_DEADBAND, SETTLE_BAND_DEG, SLOW_SETTLE_S, TAIL_RMS_DEG,
    TAIL_S, WAGGLE_REVERSALS,
};
use crate::math::{bone_ang_vel, to_bevy, to_vello};
use crate::probe::{CellReport, Probe, Sample};
use crate::report::{gait_code, report_body, vphase_code};

/// Whether telemetry should collect this tick: only while the rig is up and a
/// cell is in progress, so no row lands in the brief not-ready gap between cells.
pub fn telemetry_active(probe: Res<Probe>) -> bool {
    probe.character.is_some() && probe.ready && !probe.done
}

/// The focus limb's end-bone (particles[2] → particles[3]) angle + rate, in the
/// Bevy frame. Returns `None` if the focus limb's controller is absent.
fn focus_bone(
    probe: &Probe,
    character: Entity,
    right_arm_q: &Query<&RightArmController>,
    left_arm_q: &Query<&LeftArmController>,
    right_leg_q: &Query<&RightLegController>,
    left_leg_q: &Query<&LeftLegController>,
    particle_q: &Query<&VelloParticle>,
) -> Option<(f32, f32)> {
    let parts: [Entity; 4] = match probe.focus {
        LimbId::RightArm => right_arm_q.get(character).ok()?.particles,
        LimbId::LeftArm => left_arm_q.get(character).ok()?.particles,
        LimbId::RightLeg => right_leg_q.get(character).ok()?.particles,
        LimbId::LeftLeg => left_leg_q.get(character).ok()?.particles,
    };
    let mid = particle_q.get(parts[2]).ok()?;
    let end = particle_q.get(parts[3]).ok()?;
    let arm = to_bevy(end.particle.pos) - to_bevy(mid.particle.pos);
    let dvel = to_bevy(end.particle.velocity) - to_bevy(mid.particle.velocity);
    let theta = arm.y.atan2(arm.x);
    let omega = bone_ang_vel(arm, dvel);
    Some((theta, omega))
}

/// Collect one sample: the harness scalars (into [`TelemetryRow`], in a fixed
/// key order) and the theta/omega the classifier needs (into `Probe::samples`).
#[allow(clippy::too_many_arguments)]
pub fn limb_sample(
    mut probe: ResMut<Probe>,
    mut row: ResMut<TelemetryRow>,
    params: Option<Res<HipPinParams>>,
    pin_q: Query<&HipPin>,
    right_arm_q: Query<&RightArmController>,
    left_arm_q: Query<&LeftArmController>,
    right_leg_q: Query<&RightLegController>,
    left_leg_q: Query<&LeftLegController>,
    particle_q: Query<&VelloParticle>,
) {
    let Some(character) = probe.character else {
        return;
    };
    if probe.done || !probe.ready {
        return;
    }
    let Some((theta, omega)) = focus_bone(
        &probe,
        character,
        &right_arm_q,
        &left_arm_q,
        &right_leg_q,
        &left_leg_q,
        &particle_q,
    ) else {
        return;
    };

    let tick = probe.tick;
    let tau = (tick.saturating_sub(probe.ticks_ready)) as f32 / FIXED_HZ as f32;
    let act = probe.action.min(probe.script.len().saturating_sub(1));
    let kind = probe.script.get(act).map(|c| c.kind()).unwrap_or("none");

    let (kp, kd) = (probe.kp, probe.kd);
    probe.samples.push(Sample {
        tau,
        act,
        theta,
        omega,
    });

    row.clear();
    row.set("tau", format!("{:.4}", tau));
    row.set("t", format!("{:.4}", tick as f32 / FIXED_HZ as f32));
    row.set("act", act.to_string());
    row.set("kind", kind.to_string());
    row.set("kp", format!("{:.3}", kp));
    row.set("kd", format!("{:.4}", kd));
    row.set("theta_deg", format!("{:.3}", theta.to_degrees()));
    row.set("omega", format!("{:.3}", omega));

    // --- Whole-body (hip-pin) telemetry, folded into the row as extra scalars.
    // The verbatim commanded hip vx (`cmd_vx`) + vertical phase (`vphase`), the
    // jump→walk handoff accumulators, and the per-leg foot lag vs the commanded
    // IK target. All body timing uses the **script-relative** `tau` (the playhead
    // resets when the rig is ready), so it lines up with `jumpwalk_window`'s
    // cumulative `dur` — not the spawn-relative `t` column the limb gate uses.
    if probe.body {
        let tb = tau;
        // The pin seeds lazily; degrade to zeros rather than dropping the row, so
        // the frozen header (scalars set in this exact order on the first capture)
        // always matches the values.
        let (cmd_vx, vphase, gait_l, gait_r) = match pin_q.get(character) {
            Ok(pin) => (
                pin.state.hip_vel.x,
                vphase_code(pin.state.vs.phase),
                gait_code(pin.state.left.phase),
                gait_code(pin.state.right.phase),
            ),
            Err(_) => (0.0, 0, 0, 0),
        };
        row.set("cmd_vx", format!("{cmd_vx:.2}"));
        row.set("vphase", vphase.to_string());
        // Per-leg gait phase (0 = Stance, 1 = Swing) from the resolver. This is
        // the event signal that segments stance from swing independently of the
        // realized foot height, so the foot-slide analyzer no longer needs a
        // geometric floor tolerance to tell the two apart.
        row.set("gait_l", gait_l.to_string());
        row.set("gait_r", gait_r.to_string());

        // --- Loop 3 (attitude/tilt) telemetry, from the same pin. The resolver's
        // persistent `theta`/`omega` (rad, rad/s) plus the per-tick control input
        // `u` (rad/s²) and measured hip acceleration (px/s²) latched by
        // `drive_hip_pin`. Keyed `body_*`/`att_*` so they never collide with the
        // limb classifier's `theta_deg`/`omega` columns above. Order is fixed so
        // the frozen header stays stable.
        let (
            body_theta,
            body_omega,
            att_torque,
            hip_accel_x,
            att_target,
            att_rest_cos,
            att_rest_sin,
            meas_theta,
            ext_pin,
        ) = match pin_q.get(character) {
            Ok(pin) => (
                pin.state.attitude.theta,
                pin.state.attitude.omega,
                pin.last_attitude_torque,
                pin.last_hip_accel_x,
                pin.last_att_target,
                pin.last_att_rest_cos,
                pin.last_att_rest_sin,
                pin.last_meas_theta,
                pin.last_external_pin,
            ),
            Err(_) => (0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, false),
        };
        row.set("body_theta", format!("{body_theta:.4}"));
        row.set("body_omega", format!("{body_omega:.4}"));
        row.set("att_torque", format!("{att_torque:.2}"));
        row.set("hip_accel_x", format!("{hip_accel_x:.2}"));
        row.set(
            "att_enabled",
            format!(
                "{}",
                params.as_ref().map(|p| p.attitude_enabled).unwrap_or(false)
            ),
        );
        row.set("ext_pin", format!("{ext_pin}"));
        row.set("att_target", format!("{att_target:.4}"));
        row.set("att_rest_cos", format!("{att_rest_cos:.5}"));
        row.set("att_rest_sin", format!("{att_rest_sin:.5}"));
        row.set("meas_theta", format!("{meas_theta:.4}"));

        // Realized hip vx = the pinned `P3` (each leg controller's `pivot`,
        // `particles[1]`). Cached on the first tick it resolves.
        if probe.p3_entity.is_none() {
            probe.p3_entity = left_leg_q.get(character).ok().map(|l| l.particles[1]);
        }
        let p3_vx = match probe.p3_entity.and_then(|e| particle_q.get(e).ok()) {
            Some(p) => to_bevy(p.particle.velocity).x,
            None => cmd_vx,
        };

        // Jump→walk handoff: the *commanded* hip vx (Raibert resolver) vs the
        // *realized* hip vx (the pinned `P3`). A jagged `real_step` over a smooth
        // `cmd_step` ⇒ the physics/pin layer injects a jolt the controller never
        // asked for; a persistent `lag` ⇒ the frame is not tracking the command.
        if let Some((b, e)) = probe.jw_window {
            if (b..e).contains(&tb) {
                if !probe.jw_primed {
                    probe.jw_phase = vphase;
                    probe.jw_primed = true;
                } else {
                    let s = (cmd_vx - probe.jw_prev_cmd_vx).abs();
                    if s > probe.jw_cmd_step {
                        probe.jw_cmd_step = s;
                        probe.jw_cmd_step_t = tb;
                    }
                    let r = (p3_vx - probe.jw_prev_real_vx).abs();
                    if r > probe.jw_real_step {
                        probe.jw_real_step = r;
                        probe.jw_real_step_t = tb;
                    }
                }
                let lag = (p3_vx - cmd_vx).abs();
                if lag > probe.jw_lag {
                    probe.jw_lag = lag;
                    probe.jw_lag_t = tb;
                }
                probe.jw_prev_cmd_vx = cmd_vx;
                probe.jw_prev_real_vx = p3_vx;
            }
        }

        // Per-leg foot lag: the realized foot (`PRLL`/`PLLL`) vs the commanded IK
        // `target` the pin wrote into each leg controller. A large value = a foot
        // lagging far from (or fighting) its commanded pose.
        let legs_on = params
            .as_deref()
            .map(|p| p.hip_drives_legs)
            .unwrap_or(false);
        if legs_on {
            if let (Ok(left_leg), Ok(right_leg)) =
                (left_leg_q.get(character), right_leg_q.get(character))
            {
                probe.leg_active = true;
                let mut emax_leg = 0.0f32;
                let mut lr = [0.0f32, 0.0f32];
                for (li, target) in [left_leg.target, right_leg.target].into_iter().enumerate() {
                    let foot_ent = if li == 0 {
                        left_leg.particles[3]
                    } else {
                        right_leg.particles[3]
                    };
                    if let Ok(p) = particle_q.get(foot_ent) {
                        let err = (to_bevy(p.particle.pos) - target).length();
                        lr[li] = err;
                        if err > probe.max_leg_err_lr[li] {
                            probe.max_leg_err_lr[li] = err;
                            probe.max_leg_err_lr_t[li] = tb;
                        }
                        emax_leg = emax_leg.max(err);
                    }
                }
                if emax_leg > probe.max_leg_err {
                    probe.max_leg_err = emax_leg;
                    probe.leg_peak_t = tb;
                }
                let mut dx_lr = [0.0f32, 0.0f32];
                for (li, target) in [left_leg.target, right_leg.target].into_iter().enumerate() {
                    let foot_ent = if li == 0 {
                        left_leg.particles[3]
                    } else {
                        right_leg.particles[3]
                    };
                    if let Ok(p) = particle_q.get(foot_ent) {
                        // Signed world-frame x offset: foot.x - target.x. Negative
                        // = lagging behind the +x command, positive = overshoot.
                        // Both are bevy y-up world, so the x is directly comparable.
                        dx_lr[li] = to_bevy(p.particle.pos).x - target.x;
                    }
                }
                probe.leg_t.push(tb);
                probe.leg_err.push(emax_leg);
                probe.leg_err_lr.push(lr);
                probe.leg_dx_lr.push(dx_lr);

                // --- Commanded lower-body target state, exported in the same
                // vello y-down frame as the realized particle channels (which
                // `capture_telemetry` emits raw), so `<name>x - t<name>x` is
                // directly the tracking error: negative x = lagging behind the
                // +x command, positive x = overshoot. Velocity is the
                // resolver/controller command; acceleration is a per-tick finite
                // difference (fixed cadence `1/FIXED_HZ`). Order matches the 5
                // lower-body particles: P3 (hip), P40 (left knee), P41 (right
                // knee), PLLL (left foot), PRLL (right foot). NOTE: the pin
                // currently writes only the foot `target` + the joint
                // velocities; `hip_target`/`knee_target` are frozen at spawn,
                // so a stationary `tP3`/`tP40`/`tP41` is expected and is itself
                // diagnostic.
                let lb_tgt = [
                    ("tP3", left_leg.hip_target, left_leg.hip_vel),
                    ("tP40", left_leg.knee_target, left_leg.knee_vel),
                    ("tP41", right_leg.knee_target, right_leg.knee_vel),
                    ("tPLLL", left_leg.target, left_leg.target_vel),
                    ("tPRLL", right_leg.target, right_leg.target_vel),
                ];
                let dt_tgt = 1.0 / FIXED_HZ as f32;
                for (i, (nm, tgt_pos, tgt_vel)) in lb_tgt.iter().enumerate() {
                    let p = to_vello(*tgt_pos);
                    let v = to_vello(*tgt_vel);
                    let (ax, ay) = if probe.tgt_primed {
                        let dv = v - probe.prev_tgt_vel[i];
                        (dv.x / dt_tgt, dv.y / dt_tgt)
                    } else {
                        (0.0, 0.0)
                    };
                    let key_x = format!("{}x", nm);
                    let key_y = format!("{}y", nm);
                    let key_vx = format!("{}vx", nm);
                    let key_vy = format!("{}vy", nm);
                    let key_ax = format!("{}ax", nm);
                    let key_ay = format!("{}ay", nm);
                    row.set(&key_x, format!("{:.2}", p.x));
                    row.set(&key_y, format!("{:.2}", p.y));
                    row.set(&key_vx, format!("{:.2}", v.x));
                    row.set(&key_vy, format!("{:.2}", v.y));
                    row.set(&key_ax, format!("{:.1}", ax));
                    row.set(&key_ay, format!("{:.1}", ay));
                    probe.prev_tgt_vel[i] = v;
                }
                probe.tgt_primed = true;
            }
        }
    }
}

/// Close out a cell once the script has run to completion: classify, flush the
/// per-cell CSV from the shared buffer, then advance to the next cell or write
/// the summary and exit.
pub fn limb_finalize(
    mut probe: ResMut<Probe>,
    mut buf: ResMut<TelemetryBuffer>,
    params: Option<Res<HipPinParams>>,
    mut reset_ev: EventWriter<ResetLimbControlConstraintsEvent>,
    mut exit: EventWriter<AppExit>,
) {
    if probe.done || !probe.ready || !probe.script_done {
        return;
    }

    let mut report = finalize_cell(&probe);

    let inv = check_invariants(&buf.header, &buf.rows, &InvariantConfig::default());
    report.inv_ok = inv.ok;
    report.inv_detail = inv.detail.clone();
    report.hash = rows_hash(&buf.header, &buf.rows);

    // Whole-body verdicts + the events sidecar (no-op in limb mode).
    if probe.body {
        report_body(&probe, &buf, params.as_deref());
    }

    info!(
        "cell {} (kp={:.3}, kd={:.4}): {} swing={:.1}deg rms={:.2}deg rev={} settle={} inv={} hash={:016x}",
        report.cell,
        report.kp,
        report.kd,
        report.verdict,
        report.swing_deg,
        report.rms_deg,
        report.reversals,
        report
            .settle_s
            .map(|t| format!("{t:.2}s"))
            .unwrap_or_else(|| "none".into()),
        if report.inv_ok {
            "ok"
        } else {
            report.inv_detail.as_str()
        },
        report.hash,
    );

    let cell_path = if probe.sweep {
        let p = std::path::Path::new(&probe.csv_path);
        let dir = p.parent().map(|d| d.to_path_buf());
        let name = format!(
            "limb_pd_cell{:02}_kp{:.3}_kd{:.4}.csv",
            report.cell, report.kp, report.kd
        );
        match dir {
            Some(d) => d.join(name).to_string_lossy().to_string(),
            None => name,
        }
    } else {
        probe.csv_path.clone()
    };
    flush_csv(&cell_path, &buf);
    probe.reports.push(report);

    probe.cell += 1;
    if probe.cell >= probe.grid.len() {
        finish(&probe);
        probe.done = true;
        exit.write(AppExit::Success);
    } else {
        let (kp, kd) = probe.grid[probe.cell];
        probe.kp = kp;
        probe.kd = kd;
        // Reset the playhead for the next cell and restore rest angles so the
        // next cell starts from the same pose.
        if let Some(character) = probe.character {
            reset_ev.write(ResetLimbControlConstraintsEvent {
                limb: probe.focus.which(),
                character,
            });
        }
        probe.action = 0;
        probe.action_start_tick = probe.tick + 1;
        probe.applied_action = None;
        probe.script_done = false;
        probe.samples.clear();
        buf.clear_rows();
    }
}

/// Classify one cell from its samples: the transported arc over the whole script
/// plus the settled-tail quality over the **final** action's hold window.
fn finalize_cell(probe: &Probe) -> CellReport {
    let samples = &probe.samples;
    let mut rep = CellReport {
        cell: probe.cell,
        kp: probe.kp,
        kd: probe.kd,
        verdict: "SHORT".into(),
        swing_deg: 0.0,
        rms_deg: 0.0,
        catchup_deg: 0.0,
        reversals: 0,
        settle_s: None,
        max_omega: 0.0,
        inv_ok: true,
        inv_detail: String::new(),
        hash: 0,
    };
    if samples.len() < 5 {
        return rep;
    }

    // Unwrap theta so a swing across ±π doesn't fold.
    let mut un: Vec<f32> = Vec::with_capacity(samples.len());
    let mut prev: Option<f32> = None;
    for s in samples {
        let mut th = s.theta;
        if let Some(p) = prev {
            while th - p > PI {
                th -= 2.0 * PI;
            }
            while th - p < -PI {
                th += 2.0 * PI;
            }
        }
        prev = Some(th);
        un.push(th);
    }

    let mut mn = f32::INFINITY;
    let mut mx = f32::NEG_INFINITY;
    let mut max_omega = 0.0f32;
    for (i, s) in samples.iter().enumerate() {
        mn = mn.min(un[i]);
        mx = mx.max(un[i]);
        max_omega = max_omega.max(s.omega.abs());
    }
    let swing_deg = (mx - mn).to_degrees();
    rep.swing_deg = swing_deg;
    rep.max_omega = max_omega;

    if !swing_deg.is_finite() || !max_omega.is_finite() || swing_deg > 360.0 {
        rep.verdict = "DIVERGED".into();
        return rep;
    }
    if swing_deg < MIN_SWING_DEG {
        rep.verdict = "NO_SWING".into();
        return rep;
    }

    // The settled window = the final action's samples (the script should end on a
    // hold). Fall back to the whole run if the last action has too few samples.
    let last_act = samples.last().map(|s| s.act).unwrap_or(0);
    let mut win: Vec<(f32, f32, f32)> = samples
        .iter()
        .enumerate()
        .filter(|(_, s)| s.act == last_act)
        .map(|(i, s)| (s.tau, un[i].to_degrees(), s.omega))
        .collect();
    if win.len() < 5 {
        win = samples
            .iter()
            .enumerate()
            .map(|(i, s)| (s.tau, un[i].to_degrees(), s.omega))
            .collect();
    }
    if win.len() < 5 {
        return rep; // stays SHORT
    }

    let taus: Vec<f32> = win.iter().map(|r| r.0).collect();
    let thetas: Vec<f32> = win.iter().map(|r| r.1).collect();
    let omegas: Vec<f32> = win.iter().map(|r| r.2).collect();
    let v = tail_verdict(
        &taus,
        &thetas,
        &omegas,
        TAIL_S,
        TAIL_RMS_DEG,
        WAGGLE_REVERSALS,
        REVERSAL_DEADBAND,
    );
    let tail_mean = v.mean;
    rep.rms_deg = v.rms_deg;
    rep.reversals = v.reversals;

    let mut catchup = 0.0f32;
    for (_, th, _) in &win {
        catchup = catchup.max((th - tail_mean).abs());
    }
    rep.catchup_deg = catchup;

    // Settle time: first instant after the final window opens at which every
    // remaining sample stays in band about the settled tail mean.
    let rel_t0 = win[0].0;
    let nrels = win.len();
    let mut last_out = None;
    for (i, (_, th, _)) in win.iter().enumerate() {
        if (th - tail_mean).abs() > SETTLE_BAND_DEG {
            last_out = Some(i);
        }
    }
    rep.settle_s = if let Some(li) = last_out {
        if li + 1 < nrels {
            Some(win[li + 1].0 - rel_t0)
        } else {
            None
        }
    } else {
        Some(0.0)
    };

    rep.verdict = cell_verdict(v.pass, rep.settle_s, SLOW_SETTLE_S).to_string();
    rep
}

fn finish(probe: &Probe) {
    let p = std::path::Path::new(&probe.csv_path);
    let stem = p
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("limb_pd_log");
    let dir = p.parent().map(|d| d.to_path_buf());
    let name = format!("{stem}.summary.csv");
    let path = match dir {
        Some(d) => d.join(name).to_string_lossy().to_string(),
        None => name,
    };
    let header =
        "cell,kp,kd,verdict,swing_deg,rms_deg,catchup_deg,reversals,settle_s,max_omega,inv_ok,hash";
    let mut rows: Vec<String> = Vec::with_capacity(probe.reports.len());
    for r in &probe.reports {
        rows.push(format!(
            "{},{:.3},{:.4},{},{:.2},{:.2},{:.2},{},{},{:.3},{},{:016x}",
            r.cell,
            r.kp,
            r.kd,
            r.verdict,
            r.swing_deg,
            r.rms_deg,
            r.catchup_deg,
            r.reversals,
            r.settle_s
                .map(|t| format!("{t:.3}"))
                .unwrap_or_else(|| "none".into()),
            r.max_omega,
            r.inv_ok as u8,
            r.hash,
        ));
    }
    flush_rows(&path, header, &rows);
    info!(
        "done: {} cell(s) on {} ({}); summary: {}",
        probe.grid.len(),
        probe.focus.label(),
        probe.script_src,
        path
    );
}
