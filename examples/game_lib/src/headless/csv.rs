//! The single CSV writer used by every headless harness.
//!
//! This is the exact operation the harnesses used to duplicate inline:
//! `create_dir_all(parent)` → `fs::write` → `info!/error!`. Pure (no ECS), so it
//! is reusable from both the main schedule and any cleanup/drop path, and is
//! unit-testable against a temp path.

use bevy::log::{error, info};

use super::channels::TelemetryBuffer;

/// Write `header` + `rows` (one already-formatted string per line) to `path`,
/// creating parent directories as needed. Logs success/failure; never panics.
pub fn flush_rows(path: &str, header: &str, rows: &[String]) {
    if let Some(dir) = std::path::Path::new(path).parent() {
        if !dir.as_os_str().is_empty() {
            let _ = std::fs::create_dir_all(dir);
        }
    }
    let mut out =
        String::with_capacity(header.len() + 1 + rows.iter().map(|r| r.len() + 1).sum::<usize>());
    out.push_str(header);
    out.push('\n');
    for r in rows {
        out.push_str(r);
        out.push('\n');
    }
    match std::fs::write(path, out) {
        Ok(_) => info!("wrote {path} ({} rows)", rows.len()),
        Err(e) => error!("write failed ({path}): {e}"),
    }
}

/// [`flush_rows`] for an assembled [`TelemetryBuffer`].
pub fn flush_csv(path: &str, buf: &TelemetryBuffer) {
    flush_rows(path, &buf.header, &buf.rows);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flush_rows_creates_parents_and_writes_header_plus_rows() {
        let dir = std::env::temp_dir().join(format!(
            "headless_telemetry_test_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        // Deliberately nested + nonexistent so the `mkdir -p` path is exercised.
        let path = dir.join("nested/deeper/log.csv");
        let rows = vec!["1,a".to_string(), "2,b".to_string()];
        flush_rows(&path.to_string_lossy(), "t,name", &rows);
        let got = std::fs::read_to_string(&path).expect("file written");
        assert_eq!(got, "t,name\n1,a\n2,b\n");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn flush_csv_uses_header_and_rows() {
        let dir = std::env::temp_dir().join(format!(
            "headless_telemetry_test_csv_{}",
            std::process::id()
        ));
        let path = dir.join("log.csv");
        let buf = TelemetryBuffer {
            header: "t,P2x,P2y".to_string(),
            rows: vec!["0.5,10.00,20.00".to_string()],
        };
        flush_csv(&path.to_string_lossy(), &buf);
        let got = std::fs::read_to_string(&path).expect("file written");
        assert_eq!(got, "t,P2x,P2y\n0.5,10.00,20.00\n");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
