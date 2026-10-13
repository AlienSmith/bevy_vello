//! The shared **judge**: pure post-processing over captured telemetry.
//!
//! Everything here is *log-only* — it never touches game logic or physics. It
//! turns the descriptive CSV stream into **assertions**, which is what lets a
//! headless harness catch weird physics (divergence, constraint fights,
//! non-determinism) instead of merely plotting it.
//!
//! Four tools, all harness-agnostic:
//!
//! - [`tail_verdict`] — is a 1-D signal (e.g. a joint angle) *settled*, or
//!   oscillating? Tail-RMS + angular-rate sign-reversals over the final window,
//!   so a monotonic catch-up is never mistaken for a wobble. (This is the
//!   generalization of the arm harness's metric v2 and of the user's
//!   `detect_waggle_2d_zero_crossing` idea, in 1-D joint form.)
//! - [`check_invariants`] — are the rows well-formed and bounded? Finite,
//!   rectangular, `|v|`/`|a|` inside sanity ceilings.
//! - [`check_bone_rigidity`] — do the particles that *must* stay rigid
//!   (`P1/P2/P3` under the hip pin) hold their rest lengths? A stretch/compression
//!   here is the direct fingerprint of two constraints fighting.
//! - [`rows_hash`] / [`diff_rows`] — determinism at the integration level: run an
//!   episode twice and compare, catching cross-system ordering bugs the resolver's
//!   own unit tests cannot see.

/// Config for [`check_invariants`]. Defaults are generous sanity ceilings for the
/// V10 rig in pixels / px·s⁻¹ / px·s⁻² at 90 Hz; tighten per harness if wanted.
#[derive(Clone, Copy, Debug)]
pub struct InvariantConfig {
    /// Reject rows whose field count ≠ the header field count.
    pub require_rectangular: bool,
    /// `|pos|` ceiling (px from origin).
    pub max_abs_pos: f32,
    /// `|vel|` ceiling (px/s).
    pub max_vel: f32,
    /// `|accel|` ceiling (px/s²).
    pub max_accel: f32,
}

impl Default for InvariantConfig {
    fn default() -> Self {
        Self {
            require_rectangular: true,
            max_abs_pos: 1.0e6,
            max_vel: 1.0e5,
            max_accel: 1.0e7,
        }
    }
}

/// The result of an invariant sweep: `ok` is the headline; the counters localize
/// the failure so a log line can point straight at the offending column/row.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct InvariantReport {
    pub ok: bool,
    pub rows_checked: usize,
    /// Fields that failed `f32::parse` (blanks, `nan`, `inf`, text).
    pub non_finite: usize,
    /// Rows whose field count ≠ the header's.
    pub ragged: usize,
    /// Columns whose names look numeric (end in `x|y|vx|vy|ax|ay|fx|fy`).
    pub numeric_cols: Vec<usize>,
    /// Column index of the first coordinate that broke a magnitude ceiling.
    pub worst_col: Option<usize>,
    /// Coarse reason string for the log (`"non_finite"`, `"ragged"`, `"bound:3"`).
    pub detail: String,
}

/// One judged signal segment (typically the settled tail of a release window).
#[derive(Clone, Debug, PartialEq)]
pub struct Verdict {
    /// Mean of the judged segment (`*_deg`); exposed so callers can reuse it for
    /// catch-up / settle-time calculations without re-slicing.
    pub mean: f32,
    /// Residual RMS about the segment mean, in `*_deg`.
    pub rms_deg: f32,
    /// Deadbanded sign-reversals of the supplied rate over the segment.
    pub reversals: u32,
    /// Number of samples in the segment.
    pub len: usize,
    /// `PASS` iff `rms_deg <= rms_deg_max` **and** `reversals < reversals_max`.
    pub pass: bool,
}

/// Classify a 1-D signal tail. `thetas_deg` are the unwrapped angles (deg),
/// `omegas` their rates (rad/s, same length), `taus` their timestamps (s).
///
/// `tail_s <= 0` uses the whole slice; otherwise only the final `tail_s` seconds
/// are measured — that is the transient-immune part, where the flip catch-up has
/// already ended and only genuine ringing remains.
pub fn tail_verdict(
    taus: &[f32],
    thetas_deg: &[f32],
    omegas: &[f32],
    tail_s: f32,
    rms_deg_max: f32,
    reversals_max: u32,
    reversal_deadband: f32,
) -> Verdict {
    let empty = Verdict {
        mean: 0.0,
        rms_deg: 0.0,
        reversals: 0,
        len: 0,
        pass: false,
    };
    let n = taus.len().min(thetas_deg.len()).min(omegas.len());
    if n == 0 {
        return empty;
    }
    // Slice the final `tail_s`.
    let start = if tail_s > 0.0 {
        let t_end = taus[n - 1];
        let t0 = t_end - tail_s;
        let mut s = n;
        for i in 0..n {
            if taus[i] >= t0 {
                s = i;
                break;
            }
        }
        s
    } else {
        0
    };
    let idx = start..n;
    let len = idx.len();
    if len == 0 {
        return empty;
    }

    let mut sum = 0.0f64;
    for i in idx.clone() {
        sum += thetas_deg[i] as f64;
    }
    let mean = (sum / len as f64) as f32;

    let mut sq = 0.0f64;
    for i in idx.clone() {
        let d = (thetas_deg[i] - mean) as f64;
        sq += d * d;
    }
    let rms_deg = (sq / len as f64).sqrt() as f32;

    let mut reversals = 0u32;
    let mut last_om = 0.0f32;
    for i in idx {
        let om = omegas[i];
        if om.abs() < reversal_deadband {
            continue;
        }
        if last_om != 0.0 && om.signum() != last_om.signum() {
            reversals += 1;
        }
        last_om = om;
    }

    Verdict {
        mean,
        rms_deg,
        reversals,
        len,
        pass: rms_deg <= rms_deg_max && reversals < reversals_max,
    }
}

/// The shared cell verdict string. `pass` is [`Verdict::pass`] from
/// [`tail_verdict`]; `settle_s` is the harness-measured settle time (or `None`
/// when the signal never came to rest within the observed window).
pub fn cell_verdict(pass: bool, settle_s: Option<f32>, slow_settle_s: f32) -> &'static str {
    if !pass {
        "WAGGLE"
    } else if settle_s.map(|t| t > slow_settle_s).unwrap_or(true) {
        "SLOW"
    } else {
        "PASS"
    }
}

/// Column index whose name looks like a coordinate/velocity/accel channel.
fn is_numeric_col(name: &str) -> bool {
    for suf in ["x", "y", "vx", "vy", "ax", "ay", "fx", "fy"] {
        if name.ends_with(suf) {
            return true;
        }
    }
    false
}

/// Sweep every row for finiteness, rectangularity and magnitude ceilings.
///
/// Only columns named like a numeric channel (see [`is_numeric_col`]) are range
/// checked; the `pos` column is `x,y` (indices differ by one) so magnitude is
/// checked per-field against the axis ceiling.
pub fn check_invariants(header: &str, rows: &[String], cfg: &InvariantConfig) -> InvariantReport {
    let hcols: Vec<&str> = header.split(',').collect();
    let numeric: Vec<usize> = (0..hcols.len())
        .filter(|&i| is_numeric_col(hcols[i]))
        .collect();

    let mut rep = InvariantReport {
        ok: true,
        rows_checked: 0,
        non_finite: 0,
        ragged: 0,
        numeric_cols: numeric,
        worst_col: None,
        detail: String::new(),
    };

    // Only the physics channels are asserted on: harness scalars (e.g. a string
    // `phase`/`label` column) are informational and must not be read as numbers.
    let numeric = rep.numeric_cols.clone();
    for (ri, row) in rows.iter().enumerate() {
        rep.rows_checked += 1;
        let fields: Vec<&str> = row.split(',').collect();
        if cfg.require_rectangular && fields.len() != hcols.len() {
            rep.ragged += 1;
            rep.ok = false;
            if rep.detail.is_empty() {
                rep.detail = format!("ragged@row{ri}: {} vs {}", fields.len(), hcols.len());
            }
            continue;
        }
        for &ci in &numeric {
            let Some(f) = fields.get(ci) else { continue };
            let Ok(v) = f.trim().parse::<f32>() else {
                rep.non_finite += 1;
                rep.ok = false;
                if rep.detail.is_empty() {
                    rep.detail = format!("non_finite@row{ri},col{ci}='{f}'");
                }
                continue;
            };
            if !v.is_finite() {
                rep.non_finite += 1;
                rep.ok = false;
                if rep.detail.is_empty() {
                    rep.detail = format!("non_finite@row{ri},col{ci}='{f}'");
                }
                continue;
            }
            let name = hcols[ci];
            let ceiling = if name.ends_with("vx") || name.ends_with("vy") {
                cfg.max_vel
            } else if name.ends_with("ax")
                || name.ends_with("ay")
                || name.ends_with("fx")
                || name.ends_with("fy")
            {
                cfg.max_accel
            } else {
                cfg.max_abs_pos
            };
            if v.abs() > ceiling {
                rep.ok = false;
                if rep.worst_col.is_none() {
                    rep.worst_col = Some(ci);
                    rep.detail = format!("bound:{}={v:.3e}@row{ri}", hcols[ci]);
                }
            }
        }
    }
    rep
}

/// One bone: the two column names of its endpoints (`"P1x","P1y","P2x","P2y"`).
/// Endpoint coordinate fields must be numeric channels present in the header.
#[derive(Clone, Debug)]
pub struct Bone {
    pub a: (String, String),
    pub b: (String, String),
    /// Authored rest length. `<= 0` means "learn it from the first row".
    pub rest_len: f32,
}

/// Rigidity verdict for a set of bones.
#[derive(Clone, Debug, Default)]
pub struct RigidityReport {
    pub ok: bool,
    pub rest_lens: Vec<f32>,
    /// Max `|len - rest|` seen per bone over the run.
    pub max_dev: Vec<f32>,
    /// Max relative deviation `|len-rest|/rest` per bone.
    pub max_rel_dev: Vec<f32>,
    /// Row index at which each bone hit its `max_rel_dev` — pinpoints *when*
    /// the frame deformed (e.g. the tick of an impulse), so a failure is
    /// actionable rather than just a number.
    pub worst_row: Vec<usize>,
    /// The tolerance the verdict used.
    pub max_rel_tol: f32,
}

/// Check that each bone's length holds its rest value across all rows. A
/// `rest_len <= 0` bone learns its reference from row 0.
pub fn check_bone_rigidity(
    header: &str,
    rows: &[String],
    bones: &[Bone],
    max_rel_tol: f32,
) -> RigidityReport {
    let hcols: Vec<&str> = header.split(',').collect();
    let col = |name: &str| hcols.iter().position(|c| *c == name);
    let rest: Vec<f32> = bones.iter().map(|b| b.rest_len).collect();
    let mut rep = RigidityReport {
        ok: !rows.is_empty(),
        rest_lens: rest.clone(),
        max_dev: vec![0.0; bones.len()],
        max_rel_dev: vec![0.0; bones.len()],
        worst_row: vec![0usize; bones.len()],
        max_rel_tol,
    };
    if rows.is_empty() {
        return rep;
    }

    // Resolve column indices once.
    let idx: Vec<Option<(usize, usize, usize, usize)>> = bones
        .iter()
        .map(|b| Some((col(&b.a.0)?, col(&b.a.1)?, col(&b.b.0)?, col(&b.b.1)?)))
        .collect();
    if idx.iter().any(|o| o.is_none()) {
        rep.ok = false;
        return rep;
    }
    let idx: Vec<(usize, usize, usize, usize)> = idx.into_iter().map(|o| o.unwrap()).collect();

    let read = |row: &str, c: usize| -> f32 {
        row.split(',')
            .nth(c)
            .and_then(|s| s.trim().parse::<f32>().ok())
            .unwrap_or(f32::NAN)
    };

    let mut first = true;
    for (ri, row) in rows.iter().enumerate() {
        for (bi, &(ax, ay, bx, by)) in idx.iter().enumerate() {
            let a = (read(row, ax), read(row, ay));
            let b = (read(row, bx), read(row, by));
            let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
            if !len.is_finite() {
                rep.ok = false;
                continue;
            }
            if first && rep.rest_lens[bi] <= 0.0 {
                rep.rest_lens[bi] = len;
            }
            let rest = rep.rest_lens[bi];
            if rest > 0.0 {
                let dev = (len - rest).abs();
                let rel = dev / rest;
                if dev > rep.max_dev[bi] {
                    rep.max_dev[bi] = dev;
                }
                if rel > rep.max_rel_dev[bi] {
                    rep.max_rel_dev[bi] = rel;
                    rep.worst_row[bi] = ri;
                }
            }
            if rest > 0.0 && (len - rest).abs() / rest > max_rel_tol {
                rep.ok = false;
            }
        }
        first = false;
    }
    rep
}

/// A stable FNV-1a hash of the header + rows — the run fingerprint for
/// determinism replay. Two runs of the same episode must produce the same value.
pub fn rows_hash(header: &str, rows: &[String]) -> u64 {
    const FNV_OFFSET: u64 = 0xcbf29ce484222325;
    const FNV_PRIME: u64 = 0x100000001b3;
    let mut h = FNV_OFFSET;
    let mut eat = |bytes: &[u8]| {
        for &byte in bytes {
            h ^= byte as u64;
            h = h.wrapping_mul(FNV_PRIME);
        }
    };
    eat(header.as_bytes());
    eat(b"\n");
    for r in rows {
        eat(r.as_bytes());
        eat(b"\n");
    }
    h
}

/// Diff two row sets; `None` = identical (deterministic replay held), otherwise
/// a human-readable first divergence (`(row_idx, a, b)`).
pub fn diff_rows(a: &[String], b: &[String]) -> Option<(usize, String, String)> {
    if a.len() != b.len() {
        return Some((
            a.len().min(b.len()),
            format!("len={}", a.len()),
            format!("len={}", b.len()),
        ));
    }
    for (i, (ra, rb)) in a.iter().zip(b.iter()).enumerate() {
        if ra != rb {
            return Some((i, ra.clone(), rb.clone()));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ramp(n: usize, f: impl Fn(usize) -> (f32, f32, f32)) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
        let mut t = Vec::new();
        let mut th = Vec::new();
        let mut om = Vec::new();
        for i in 0..n {
            let (a, b, c) = f(i);
            t.push(a);
            th.push(b);
            om.push(c);
        }
        (t, th, om)
    }

    #[test]
    fn clean_settle_passes() {
        // 1 s, constant angle, ~0 rate.
        let (t, th, om) = ramp(90, |i| (i as f32 / 90.0, 10.0, 0.0));
        let v = tail_verdict(&t, &th, &om, 0.6, 1.0, 4, 0.05);
        assert!(v.pass, "{v:?}");
        assert!(v.rms_deg < 1e-3);
        assert_eq!(v.reversals, 0);
    }

    #[test]
    fn monotonic_catchup_is_not_a_wobble() {
        // A one-way coast: angle marches, rate constant → 0 reversals, and the
        // last 0.6 s is (nearly) flat even though the whole slice is not.
        let mut t = Vec::new();
        let mut th = Vec::new();
        let mut om = Vec::new();
        for i in 0..180 {
            let s = i as f32 / 90.0;
            t.push(s);
            // fast for 1 s, then flat
            if s < 1.0 {
                th.push(-90.0 * s);
                om.push(-1.0);
            } else {
                th.push(-90.0);
                om.push(0.0);
            }
        }
        let v = tail_verdict(&t, &th, &om, 0.6, 1.0, 4, 0.05);
        assert!(v.pass, "{v:?}");
        assert_eq!(v.reversals, 0);
    }

    #[test]
    fn ringing_tail_waggles() {
        // Alternating rate sign every sample in the tail → many reversals.
        let mut t = Vec::new();
        let mut th = Vec::new();
        let mut om = Vec::new();
        for i in 0..120 {
            let s = i as f32 / 90.0;
            t.push(s);
            th.push(5.0 * (s * 60.0).sin());
            om.push(if i % 2 == 0 { 0.5 } else { -0.5 });
        }
        let v = tail_verdict(&t, &th, &om, 0.6, 1.0, 4, 0.05);
        assert!(!v.pass, "{v:?}");
        assert!(v.reversals >= 4);
    }

    #[test]
    fn invariants_flag_non_finite_and_bound() {
        let header = "t,P2x,P2y,P2vx,P2vy";
        let rows = vec![
            "0,1.00,2.00,0.00,0.00".to_string(),
            "1,nan,2.00,0.00,0.00".to_string(),
            "2,1.00,2.00,9.99e9,0.00".to_string(),
        ];
        let rep = check_invariants(header, &rows, &InvariantConfig::default());
        assert!(!rep.ok);
        assert_eq!(rep.non_finite, 1);
        assert!(rep.detail.starts_with("non_finite") || rep.detail.starts_with("bound"));
    }

    #[test]
    fn invariants_flag_ragged() {
        let header = "t,P2x,P2y";
        let rows = vec!["0,1.00".to_string(), "1,1.00,2.00".to_string()];
        let rep = check_invariants(header, &rows, &InvariantConfig::default());
        assert!(!rep.ok);
        assert_eq!(rep.ragged, 1);
    }

    #[test]
    fn bone_rigidity_passes_when_lengths_hold() {
        let header = "P1x,P1y,P2x,P2y,P3x,P3y";
        // P1..P2 length 4, P2..P3 length 3, held across rows.
        let rows = vec![
            "0.00,0.00,4.00,0.00,4.00,3.00".to_string(),
            "1.00,0.00,5.00,0.00,5.00,3.00".to_string(),
        ];
        let bones = vec![
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
        ];
        let rep = check_bone_rigidity(&header, &rows, &bones, 1e-3);
        assert!(rep.ok, "{rep:?}");
        assert!((rep.rest_lens[0] - 4.0).abs() < 1e-4);
        assert!((rep.rest_lens[1] - 3.0).abs() < 1e-4);
        assert!(rep.max_rel_dev[0] < 1e-4);
    }

    #[test]
    fn bone_rigidity_flags_stretch() {
        let header = "P1x,P1y,P2x,P2y";
        // Rows 0–1 hold length 4; row 2 stretches to length 6 (+50%).
        let rows = vec![
            "0.00,0.00,4.00,0.00".to_string(),
            "1.00,0.00,5.00,0.00".to_string(),
            "2.00,0.00,8.00,0.00".to_string(),
        ];
        let bones = vec![Bone {
            a: ("P1x".into(), "P1y".into()),
            b: ("P2x".into(), "P2y".into()),
            rest_len: 0.0,
        }];
        let rep = check_bone_rigidity(&header, &rows, &bones, 1e-2);
        assert!(!rep.ok);
        assert!(rep.max_rel_dev[0] > 0.4);
    }

    #[test]
    fn hash_is_stable_and_sensitive() {
        let h = "t,P2x";
        let a = vec!["0,1".to_string(), "1,2".to_string()];
        let b = vec!["0,1".to_string(), "1,2".to_string()];
        let c = vec!["0,1".to_string(), "1,3".to_string()];
        assert_eq!(rows_hash(h, &a), rows_hash(h, &b));
        assert_ne!(rows_hash(h, &a), rows_hash(h, &c));
        assert!(diff_rows(&a, &b).is_none());
        assert!(diff_rows(&a, &c).is_some());
    }
}
