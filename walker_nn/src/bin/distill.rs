//! `distill` — the merge step of `plans/distill-single-model.md`: regenerate
//! authority targets and fit one student net, with no physics and no Bevy.
//!
//! ```text
//! cargo run -p walker_nn --bin distill gen \
//!   --obs  tmp/distill/jump01.obs \
//!   --expert tmp/walker_ppo/policy_B.json \
//!   --best   tmp/walker_ppo/best_checkpoint.json \
//!   --out    tmp/distill/jump01.target          [--delta-max 1.0]
//!
//! cargo run -p walker_nn --bin distill fit \
//!   --obs tmp/distill/jump01.obs \
//!   --targets tmp/distill/jump01.target \
//!   --expert tmp/walker_ppo/policy_B.json \
//!   --out tmp/walker_ppo/policy_new.json \
//!   [--epochs 8] [--lr 3e-4] [--val-frac 0.1] [--jitter 0.01]
//!   [--report tmp/distill/distill_report.csv]
//! ```
//!
//! `gen` is pure and re-runnable: changing the best checkpoint `A*` only reruns
//! this step (two forwards per row), never the sim. `fit` writes `policy_new.json`
//! in the same `PolicyBundle` shape `walker_qa POLICY=` / `walker_ppo RESUME=`
//! load, plus a per-epoch report CSV.
//!
//! Exit codes: 0 = ok, 1 = runtime error, 2 = usage error.

use std::path::Path;

use walker_nn::dataset::{self, DatasetMeta};
use walker_nn::distill::{self, DistillError, FitConfig, GenConfig};
use walker_nn::PolicyBundle;

fn need<'a>(args: &'a [String], key: &str) -> Option<&'a str> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).map(|s| s.as_str()))
}
fn parse<T: std::str::FromStr>(args: &[String], key: &str, default: T) -> Result<T, String> {
    match need(args, key) {
        Some(s) => s
            .parse::<T>()
            .map_err(|_| format!("--{key} expects a number, got '{s}'")),
        None => Ok(default),
    }
}

fn usage() -> ! {
    eprintln!(
        r#"usage: distill <gen|fit> ...
  gen:
    distill gen --obs <tag.obs> --expert <B.json> --best <A.json>
                --out <tag.target> [--delta-max <f>] [--upto <update>]
  fit:
    distill fit --obs <tag.obs> --targets <tag.target> --expert <B.json>
                --out <policy_new.json> [--epochs <n>] [--lr <f>]
                [--val-frac <f>] [--jitter <f>] [--val-threshold <f>]
                [--report <csv>] [--upto <update>]

  --upto <update>: checkpoint-consistent slice. Uses the <tag>.windows.csv
    sidecar the walker_ppo drain wrote, keeping only rows [0, end_row):
    exactly the training inputs that produced checkpoint <update>. Rows after
    the promoted checkpoint reflect a *later* (possibly reward-hacked)
    authority and are excluded. Required whenever the dataset came from the
    trainer drain (it has a windows sidecar); a QA-probe dataset without one
    is used whole."#
    );
    std::process::exit(2);
}

/// Resolve `--upto <update>` into a row limit, or `None` if the flag is absent.
/// Reads `<tag>.windows.csv` (the drain's per-update boundaries) and returns
/// the `end_row` recorded at the requested update — the exact checkpoint-
/// consistent cut. Errors if the flag is given but no sidecar exists (a QA
/// probe dataset without boundaries) or the update isn't in the records.
fn upto_row_limit(args: &[String], obs_path: &str) -> Result<Option<usize>, String> {
    let Some(update_str) = need(args, "--upto") else {
        return Ok(None);
    };
    let update: usize = update_str
        .parse()
        .map_err(|_| format!("--upto expects an update number, got '{update_str}'"))?;
    let windows_path = dataset::windows_path_for(Path::new(obs_path));
    let windows = dataset::read_windows(windows_path.to_str().unwrap_or_default())
        .map_err(|e| format!("reading windows sidecar {}: {e}", windows_path.display()))?;
    let end_row = dataset::end_row_for_update(&windows, update).ok_or_else(|| {
        format!(
            "--upto {update}: no window record at or before that update {} (datasets without a trainer drain have no boundaries)",
            windows_path.display()
        )
    })?;
    println!(
        "distill: --upto {update} -> rows [0, {end_row}) (checkpoint-consistent slice; file has {} window records)",
        windows.len()
    );
    Ok(Some(end_row))
}

fn err_exit(msg: impl std::fmt::Display) -> ! {
    eprintln!("distill: {msg}");
    std::process::exit(1);
}

/// `--obs <tag.obs>`'s manifest, if it exists — used to spot a shape that does
/// not match the bundles being loaded (a wiring error, not a crash).
fn cross_check_manifest(obs_path: &str, b: &PolicyBundle) {
    let mp = dataset::manifest_path_for(Path::new(obs_path));
    if !mp.is_file() {
        return;
    }
    match dataset::read_manifest(mp.to_str().unwrap_or_default()) {
        Ok(meta) => {
            if meta.obs_dim != b.obs_dim() || meta.action_dim != b.action_dim() {
                eprintln!(
                    "distill: WARNING manifest {} says obs={}/act={} but bundle B is obs={}/act={}",
                    mp.display(),
                    meta.obs_dim,
                    meta.action_dim,
                    b.obs_dim(),
                    b.action_dim()
                );
            }
        }
        Err(e) => eprintln!("distill: WARNING unreadable manifest {}: {e}", mp.display()),
    }
}

fn cmd_gen(args: &[String]) {
    let Some(obs_path) = need(args, "--obs") else {
        usage()
    };
    let Some(expert_path) = need(args, "--expert") else {
        usage()
    };
    let Some(best_path) = need(args, "--best") else {
        usage()
    };
    let Some(out_path) = need(args, "--out") else {
        usage()
    };
    let delta_max: f32 = parse(args, "delta-max", 0.0f32).unwrap_or_else(|e| err_exit(e));
    let cfg = GenConfig {
        delta_max: if delta_max > 0.0 {
            Some(delta_max)
        } else {
            None
        },
    };

    let b = PolicyBundle::load_json(expert_path)
        .unwrap_or_else(|e| err_exit(format!("--expert '{expert_path}': {e}")));
    let a = PolicyBundle::load_json(best_path)
        .unwrap_or_else(|e| err_exit(format!("--best '{best_path}': {e}")));
    cross_check_manifest(obs_path, &b);

    // Checkpoint-consistent slice: `--upto` keeps only rows [0, end_row) of the
    // promoted update, cutting the late-training tail (a different authority).
    let upto = upto_row_limit(args, obs_path).unwrap_or_else(|e| err_exit(e));
    let obs = match upto {
        Some(end_row) => dataset::read_rows_upto(obs_path, b.obs_dim(), end_row),
        None => dataset::read_rows(obs_path, b.obs_dim()),
    }
    .unwrap_or_else(|e| err_exit(format!("reading '--obs {obs_path}': {e}")));
    println!(
        "distill gen: {} obs rows (obs {}) from {obs_path}",
        obs.len(),
        b.obs_dim()
    );

    let targets =
        distill::generate(&obs, &b, &a, &cfg).unwrap_or_else(|e: DistillError| err_exit(e));
    dataset::write_rows(out_path, b.action_dim(), &targets)
        .unwrap_or_else(|e| err_exit(format!("writing '--out {out_path}': {e}")));

    // Emit a small manifest for the generated .target too (regenerable, so the
    // row count is a snapshot — `gen` rewrites it every run).
    let meta = DatasetMeta::derive(
        Path::new(out_path)
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_default()
            .as_str(),
        "distill gen",
        b.obs_dim(),
        b.action_dim(),
        &b.actor.dims[1..b.actor.dims.len().saturating_sub(1)],
        targets.len(),
    );
    dataset::write_manifest(
        dataset::manifest_path_for(Path::new(out_path))
            .to_str()
            .unwrap_or_default(),
        &meta,
    )
    .unwrap_or_else(|e| err_exit(format!("writing target manifest: {e}")));
    println!(
        "distill gen: {} target rows (act {}) -> {out_path}",
        targets.len(),
        b.action_dim()
    );
}

fn cmd_fit(args: &[String]) {
    let Some(obs_path) = need(args, "--obs") else {
        usage()
    };
    let Some(targets_path) = need(args, "--targets") else {
        usage()
    };
    let Some(expert_path) = need(args, "--expert") else {
        usage()
    };
    let out_path = need(args, "--out").unwrap_or("policy_new.json");
    let epochs: usize = parse(args, "--epochs", 8usize).unwrap_or_else(|e| err_exit(e));
    let lr: f32 = parse(args, "--lr", 3e-4f32).unwrap_or_else(|e| err_exit(e));
    let val_frac: f32 = parse(args, "--val-frac", 0.1f32).unwrap_or_else(|e| err_exit(e));
    let jitter: f32 = parse(args, "--jitter", 0.0f32).unwrap_or_else(|e| err_exit(e));
    let val_threshold: f32 =
        parse(args, "--val-threshold", 0.05f32).unwrap_or_else(|e| err_exit(e));
    let report_path = need(args, "--report").unwrap_or("distill_report.csv");
    let cfg = FitConfig {
        epochs,
        lr,
        val_frac,
        jitter: if jitter > 0.0 { Some(jitter) } else { None },
        val_threshold,
        ..FitConfig::default()
    };

    let expert = PolicyBundle::load_json(expert_path)
        .unwrap_or_else(|e| err_exit(format!("--expert '{expert_path}': {e}")));
    cross_check_manifest(obs_path, &expert);

    // Same checkpoint-consistent slice as `gen`; `--targets` was produced by a
    // `gen` with the same `--upto`, so slicing it identically keeps the pair in
    // lockstep (a mismatch here surfaces as a length mismatch in `fit`).
    let upto = upto_row_limit(args, obs_path).unwrap_or_else(|e| err_exit(e));
    let obs = match upto {
        Some(end_row) => dataset::read_rows_upto(obs_path, expert.obs_dim(), end_row),
        None => dataset::read_rows(obs_path, expert.obs_dim()),
    }
    .unwrap_or_else(|e| err_exit(format!("reading '--obs {obs_path}': {e}")));
    let targets = dataset::read_rows_upto(targets_path, expert.action_dim(), obs.len())
        .unwrap_or_else(|e| err_exit(format!("reading '--targets {targets_path}': {e}")));

    let (student, report) =
        distill::fit(&obs, &targets, &expert, &cfg).unwrap_or_else(|e: DistillError| err_exit(e));

    student
        .save_json(out_path)
        .unwrap_or_else(|e| err_exit(format!("writing '--out {out_path}': {e}")));

    write_report(report_path, &report);

    println!("distill fit: {}", report.one_line());
    println!(
        "distill fit: wrote {} (obs {} / act {}, hidden {:?})",
        out_path,
        student.obs_dim(),
        student.action_dim(),
        &student.actor.dims[1..student.actor.dims.len().saturating_sub(1)]
    );
    println!("distill fit: report -> {report_path}");
    if report.pass {
        println!(
            "distill fit: PASS (final val mse {:.6} < {})",
            report.final_mse, report.val_threshold
        );
    } else {
        println!("distill fit: val mse {:.6} >= threshold {}; the BEHAVIOURAL gate in walker_qa decides promotion, not this number",
            report.final_mse, report.val_threshold);
    }
}

fn write_report(report_path: &str, report: &walker_nn::FitReport) {
    let mut csv = String::new();
    csv.push_str(&format!(
        "# rows: {} train: {} val: {} action: {} epochs: {} lr: {} jitter: {:?} threshold: {}\n",
        report.rows,
        report.train_rows,
        report.val_rows,
        report.action_dim,
        report.epochs,
        report.lr,
        report.jitter,
        report.val_threshold
    ));
    csv.push_str("epoch,train_mse,val_mse\n");
    for (i, (t, v)) in report
        .train_mse
        .iter()
        .zip(report.val_mse.iter())
        .enumerate()
    {
        csv.push_str(&format!(
            "{i},{:.8},{}\n",
            t,
            if v.is_nan() {
                "nan".into()
            } else {
                format!("{v:.8}")
            }
        ));
    }
    if let Some(dir) = Path::new(report_path).parent() {
        std::fs::create_dir_all(dir).unwrap_or_else(|e| err_exit(format!("report dir: {e}")));
    }
    std::fs::write(report_path, csv)
        .unwrap_or_else(|e| err_exit(format!("writing report '{report_path}': {e}")));
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        usage();
    }
    match args[0].as_str() {
        "gen" => cmd_gen(&args[1..]),
        "fit" => cmd_fit(&args[1..]),
        "help" | "--help" | "-h" => usage(),
        other => err_exit(format!("unknown subcommand '{other}' (expected gen|fit)")),
    }
}
