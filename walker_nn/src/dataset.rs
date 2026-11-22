//! Dataset IO for distillation — the **collect inputs only** format
//! ([`plans/distill-single-model.md`](../../plans/distill-single-model.md) §2):
//!
//! - `<tag>.obs` — little-endian `f32`, `ROW = obs_dim` (22) per record,
//!   append-only. The raw normalized observation vectors `build_obs` produced
//!   (the exact vectors the controller fed the policy that tick).
//! - `<tag>.target` — little-endian `f32`, `ROW = action_dim` (5) per record,
//!   the generated `μ_B(s) + μ_A(s)` targets. **Rewritable** — because targets
//!   are a pure function of `(obs, B, A)`, regeneration never touches physics.
//! - `<tag>.manifest.json` — a [`DatasetMeta`] sidecar describing the collection.
//!   A dataset is never mutated in place; superseded tags point `replaced_by`
//!   at the newer tag (archive discipline).
//!
//! The reader/writer is row-width-parameterized so the same code serves `.obs`
//! and `.target`; the callers (the `distill` binary at
//! `walker_nn/src/bin/distill.rs`, the `walker_qa` DUMP_OBS hook) pass the width
//! they know from the loaded bundles. No Bevy, no physics — pure byte IO.

use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

/// Metadata sidecar for one `.obs` collection (`<tag>.manifest.json`). The
/// collector writes it; `distill gen`/`fit` read it to assert the shape
/// consistency invariant (§7) before touching the rows.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct DatasetMeta {
    /// Row width of `<tag>.obs` — the policy's input (`obs_dim`, 22 for the
    /// jump residual).
    pub obs_dim: usize,
    /// Generated target width (`action_dim`, 5 for the jump residual).
    pub action_dim: usize,
    /// Hidden widths of the producing actor (`dims[1..len-1]`). Records the
    /// network shape so a later `gen`/`fit` can assert the student stays
    /// identical across generations ("model consistent from start").
    pub hidden: Vec<usize>,
    /// Number of rows in `<tag>.obs` at last manifest write.
    pub rows: usize,
    /// Collection tag, e.g. `jump/0087`.
    pub tag: String,
    /// Where this collection came from (`walker_qa DUMP_OBS`, future
    /// `walker_ppo` rollout drain).
    pub source: String,
    /// Unix millis at first write.
    pub created_ts: u64,
    /// If this tag was superseded, the newer tag that replaced it (`None`
    /// otherwise). Archive discipline: the `.obs` file itself is never mutated.
    #[serde(default)]
    pub replaced_by: Option<String>,
}

impl DatasetMeta {
    /// Build a fresh metadata record from the policy shape.
    pub fn derive(
        tag: &str,
        source: &str,
        obs_dim: usize,
        action_dim: usize,
        hidden: &[usize],
        rows: usize,
    ) -> Self {
        Self {
            obs_dim,
            action_dim,
            hidden: hidden.to_vec(),
            rows,
            tag: tag.to_owned(),
            source: source.to_owned(),
            created_ts: now_millis(),
            replaced_by: None,
        }
    }

    /// Human-readable one-liner for logs / the report header.
    pub fn describe(&self) -> String {
        format!(
            "tag={} obs={} act={} hidden={:?} rows={} source={}",
            self.tag, self.obs_dim, self.action_dim, self.hidden, self.rows, self.source
        )
    }
}

/// The `.manifest.json` sidecar that describes `<tag>.obs` (same stem, the
/// extension swapped for `.manifest.json`).
pub fn manifest_path_for(obs_path: &Path) -> PathBuf {
    let stem = obs_path
        .file_stem()
        .map(|s| s.to_os_string())
        .unwrap_or_default();
    let mut name = stem;
    name.push(".manifest.json");
    obs_path.with_file_name(name)
}

/// Sidecar mapping **drain update → last row index** written into the `.obs`
/// file up to and including that update (`<tag>.windows.csv`). This is what
/// makes the dataset *checkpoint-consistent* (§7): rows drained at update N
/// are exactly the obs the trainer consumed to produce `policy_N`, so picking
/// checkpoint K means using rows `[0, end_row_of_update_K)`. Without this
/// slice a late-training reward-hacked tail would leak into the student.
///
/// Format: header `update,end_row` then one CSV row per drain, append-only.
pub fn windows_path_for(obs_path: &Path) -> PathBuf {
    obs_path.with_extension("windows.csv")
}

/// Append one window record (`update, end_row`) to the sidecar, creating the
/// file with a header on first use.
pub fn append_window(path: &str, update: usize, end_row: usize) -> io::Result<()> {
    if let Some(dir) = Path::new(path).parent() {
        std::fs::create_dir_all(dir)?;
    }
    let exists = Path::new(path).exists();
    let mut w = BufWriter::new(
        std::fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open(path)?,
    );
    if !exists {
        w.write_all(b"update,end_row\n")?;
    }
    writeln!(w, "{update},{end_row}")?;
    w.flush()
}

/// Read the `(update, end_row)` window records. The windows are strictly
/// increasing in `end_row`; returns `[]` for a missing file.
pub fn read_windows(path: &str) -> io::Result<Vec<(usize, usize)>> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(e) => return Err(e),
    };
    let mut out = Vec::new();
    for (i, line) in text.lines().enumerate() {
        if i == 0 {
            continue; // header
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let mut parts = line.split(',');
        let (update, end_row) = match (parts.next(), parts.next()) {
            (Some(u), Some(e)) => match (u.trim().parse::<usize>(), e.trim().parse::<usize>()) {
                (Ok(u), Ok(e)) => (u, e),
                _ => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("bad window row: '{line}'"),
                    ))
                }
            },
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("bad window row: '{line}'"),
                ))
            }
        };
        out.push((update, end_row));
    }
    Ok(out)
}

/// The `end_row` recorded for `update` (the largest recorded update `<=`
/// `update`), or `None` if the windows file has no record at or before it.
/// Passing the exact checkpoint update you promote gives the exact consistent
/// slice; passing a later update would admit the degenerate tail.
pub fn end_row_for_update(windows: &[(usize, usize)], update: usize) -> Option<usize> {
    windows
        .iter()
        .filter(|(u, _)| *u <= update)
        .map(|(_, e)| *e)
        .max()
}

/// Read a little-endian f32 matrix file: `ROW = row_width` f32 per record, in
/// row-major order. Errors if the byte length is not a whole number of rows.
pub fn read_rows(path: &str, row_width: usize) -> io::Result<Vec<Vec<f32>>> {
    if row_width == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "read_rows: row_width must be > 0",
        ));
    }
    let f = File::open(path)?;
    let mut r = BufReader::new(f);
    let mut raw = Vec::new();
    r.read_to_end(&mut raw)?;
    to_rows(&raw, row_width).map_err(|msg| io::Error::new(io::ErrorKind::InvalidData, msg))
}

/// Read only the **first `upto_rows` rows** of a matrix file. Used by the
/// `--upto` checkpoint-consistent slice: without it the whole late-training
/// tail would be read and handed to `gen`/`fit`, defeating the slice's
/// purpose. Reads exactly `upto_rows` rows if the file has that many, else all
/// of them. `upto_rows == 0` returns `[]`.
pub fn read_rows_upto(path: &str, row_width: usize, upto_rows: usize) -> io::Result<Vec<Vec<f32>>> {
    if row_width == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "read_rows_upto: row_width must be > 0",
        ));
    }
    if upto_rows == 0 {
        return Ok(Vec::new());
    }
    let row_bytes = row_width * 4;
    let want = upto_rows * row_bytes;
    let file_len = std::fs::metadata(path)?.len() as usize;
    if file_len % row_bytes != 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("{file_len} bytes is not a whole number of {row_width}f32 rows"),
        ));
    }
    let n = want.min(file_len);
    let mut raw = vec![0u8; n];
    {
        let f = File::open(path)?;
        let mut r = BufReader::new(f);
        r.read_exact(&mut raw)?;
    }
    to_rows(&raw, row_width).map_err(|msg| io::Error::new(io::ErrorKind::InvalidData, msg))
}

/// Interpret a raw byte buffer as `row_width`-wide LE f32 rows (shared by
/// [`read_rows`] and the size-based `row_width_of` below).
fn to_rows(raw: &[u8], row_width: usize) -> Result<Vec<Vec<f32>>, String> {
    let n = raw.len();
    let row_bytes = row_width * 4;
    if row_bytes == 0 {
        return Err("to_rows: row_width must be > 0".into());
    }
    if n % row_bytes != 0 {
        return Err(format!(
            "{n} bytes is not a whole number of {row_width}f32 rows (expected multiples of {row_bytes})"
        ));
    }
    let rows = n / row_bytes;
    let mut out = Vec::with_capacity(rows);
    for i in 0..rows {
        let base = i * row_width;
        let mut row = Vec::with_capacity(row_width);
        for j in 0..row_width {
            let off = (base + j) * 4;
            row.push(f32::from_le_bytes([
                raw[off],
                raw[off + 1],
                raw[off + 2],
                raw[off + 3],
            ]));
        }
        out.push(row);
    }
    Ok(out)
}

/// Append (or create) rows at `path` in the little-endian layout. The file is
/// only ever appended to — a collection can span multiple probe runs sharing a
/// tag, and `.target` regeneration overwrites via [`write_rows`] instead.
pub fn append_rows(path: &str, row_width: usize, rows: &[Vec<f32>]) -> io::Result<()> {
    if let Some(dir) = Path::new(path).parent() {
        std::fs::create_dir_all(dir)?;
    }
    let f = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open(path)?;
    let mut w = BufWriter::new(f);
    for row in rows {
        if row.len() != row_width {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("row length {} != {row_width}", row.len()),
            ));
        }
        for v in row {
            w.write_all(&v.to_le_bytes())?;
        }
    }
    w.flush()
}

/// Overwrite a whole matrix file with `rows` (used by `distill gen` to emit
/// the rewritable `.target`). Creates parent directories.
pub fn write_rows(path: &str, row_width: usize, rows: &[Vec<f32>]) -> io::Result<()> {
    if let Some(dir) = Path::new(path).parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut w = BufWriter::new(File::create(path)?);
    for row in rows {
        if row.len() != row_width {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("row length {} != {row_width}", row.len()),
            ));
        }
        for v in row {
            w.write_all(&v.to_le_bytes())?;
        }
    }
    w.flush()
}

/// Existing row count of a matrix file (`0` for a missing/empty file).
pub fn row_count(path: &str, row_width: usize) -> io::Result<usize> {
    let raw = match std::fs::read(path) {
        Ok(raw) => raw,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(0),
        Err(e) => return Err(e),
    };
    Ok(to_rows(&raw, row_width)
        .map_err(|msg| io::Error::new(io::ErrorKind::InvalidData, msg))?
        .len())
}

/// Write (create/truncate) the JSON manifest sidecar.
pub fn write_manifest(path: &str, meta: &DatasetMeta) -> io::Result<()> {
    if let Some(dir) = Path::new(path).parent() {
        std::fs::create_dir_all(dir)?;
    }
    let s = serde_json::to_string_pretty(meta).map_err(io::Error::other)?;
    std::fs::write(path, s)
}

/// Read a JSON manifest sidecar.
pub fn read_manifest(path: &str) -> Result<DatasetMeta, Box<dyn std::error::Error>> {
    let s = std::fs::read_to_string(path)?;
    Ok(serde_json::from_str(&s)?)
}

fn now_millis() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rows_roundtrip_in_little_endian() {
        let path = "/tmp/walker_nn_dataset_roundtrip.obs";
        let _ = std::fs::remove_file(path);
        let rows = vec![
            vec![1.0f32, -2.0, 3.5],
            vec![0.0, 0.25, -0.125],
            vec![-1e4, 1e-4, 42.0],
        ];
        write_rows(path, 3, &rows).expect("write");
        // Verify the on-disk layout is exactly row-major LE f32: byte 4 must
        // be the low byte of the second row's first value.
        let raw = std::fs::read(path).unwrap();
        assert_eq!(raw.len(), 3 * 3 * 4);
        assert_eq!(
            f32::from_le_bytes([raw[12], raw[13], raw[14], raw[15]]),
            0.0
        );
        let got = read_rows(path, 3).expect("read");
        assert_eq!(got, rows, "round trip must preserve every value");
        assert_eq!(row_count(path, 3).unwrap(), 3);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn append_grows_without_touching_existing_rows() {
        let path = "/tmp/walker_nn_dataset_append.obs";
        let _ = std::fs::remove_file(path);
        append_rows(path, 2, &[vec![1.0, 2.0]]).expect("first append");
        append_rows(path, 2, &[vec![3.0, 4.0]]).expect("second append");
        let got = read_rows(path, 2).unwrap();
        assert_eq!(got, vec![vec![1.0, 2.0], vec![3.0, 4.0]]);
        assert_eq!(row_count(path, 2).unwrap(), 2);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn ragged_width_is_an_error() {
        let path = "/tmp/walker_nn_dataset_ragged.obs";
        let _ = std::fs::remove_file(path);
        // 5 bytes is not a multiple of a 2-f32 row.
        std::fs::write(path, [0u8, 0, 0, 0, 1]).unwrap();
        assert!(read_rows(path, 2).is_err(), "partial row must be rejected");
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn manifest_roundtrips_and_derives() {
        let meta = DatasetMeta::derive("jump/0001", "walker_qa DUMP_OBS", 22, 5, &[64, 64], 4096);
        let path = "/tmp/walker_nn_dataset.manifest.json";
        write_manifest(path, &meta).unwrap();
        let got = read_manifest(path).unwrap();
        assert_eq!(got, meta);
        assert_eq!(got.describe().contains("obs=22"), true);
        assert_eq!(got.replaced_by, None);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn manifest_path_keeps_the_stem() {
        let p = manifest_path_for(Path::new("/tmp/distill/jump02.obs"));
        assert_eq!(p, PathBuf::from("/tmp/distill/jump02.manifest.json"));
        let p2 = manifest_path_for(Path::new("v8.character.json"));
        assert_eq!(p2, PathBuf::from("v8.character.manifest.json"));
    }

    #[test]
    fn windows_roundtrip_and_end_row_slice() {
        let path = "/tmp/walker_nn_dataset_windows.csv";
        let _ = std::fs::remove_file(path);
        append_window(path, 1, 500).unwrap();
        append_window(path, 2, 1050).unwrap();
        append_window(path, 3, 1700).unwrap();
        let windows = read_windows(path).unwrap();
        assert_eq!(windows, vec![(1, 500), (2, 1050), (3, 1700)]);
        // Exact checkpoint update -> its recorded end_row.
        assert_eq!(end_row_for_update(&windows, 2), Some(1050));
        // Requesting a later update (degenerate tail) is allowed but admits
        // more rows; requesting an earlier one yields the largest <= it.
        assert_eq!(end_row_for_update(&windows, 99), Some(1700));
        assert_eq!(end_row_for_update(&windows, 0), None);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn read_rows_upto_slices_and_caps() {
        let path = "/tmp/walker_nn_dataset_upto.obs";
        let _ = std::fs::remove_file(path);
        let rows = vec![
            vec![1.0, 2.0],
            vec![3.0, 4.0],
            vec![5.0, 6.0],
            vec![7.0, 8.0],
        ];
        write_rows(path, 2, &rows).unwrap();
        assert_eq!(
            read_rows_upto(path, 2, 3).unwrap(),
            vec![vec![1.0, 2.0], vec![3.0, 4.0], vec![5.0, 6.0]]
        );
        // Asking for more rows than the file holds caps at the file length.
        assert_eq!(read_rows_upto(path, 2, 99).unwrap(), rows);
        assert_eq!(read_rows_upto(path, 2, 0).unwrap(), Vec::<Vec<f32>>::new());
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn windows_path_replaces_extension_not_stem() {
        let p = windows_path_for(Path::new("/tmp/distill/jump02.obs"));
        assert_eq!(p, PathBuf::from("/tmp/distill/jump02.windows.csv"));
    }
}
