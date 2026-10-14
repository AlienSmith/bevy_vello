//! Reusable headless CSV telemetry.
//!
//! Every headless harness (`headless_ctrl`, `headless_arm_pd`) used to
//! re-implement the same plumbing: build a header, accumulate rows in a
//! `Vec<String>`, `mkdir -p`, `std::fs::write`, log. This module is the single
//! implementation of that.
//!
//! - [`flush_csv`] — the one writer (`mkdir -p` + write + log).
//! - [`HeadlessTelemetryPlugin`] — installs the row buffer + resolution state.
//! - [`capture_telemetry`] — appends one row per tick: harness scalars
//!   (via [`TelemetryRow`]) followed by the resolved particle channels in
//!   [`V10_PARTICLES`] order.
//!
//! ## Coordinate frame
//!
//! Particle positions/velocities are logged **raw in the Vello engine frame**
//! (x right, y **down**) exactly as held by `VelloParticle::particle`. No
//! transform is applied; convert in analysis if a y-up frame is wanted.
//!
//! ## What is *not* logged
//!
//! Static authored values — `inv_mass`, rest lengths — are deliberately
//! excluded: they live in the character JSON and never change at runtime.
//! Telemetry carries dynamic state only.

mod channels;
mod csv;
mod judge;
mod plugin;

pub use channels::{
    capture_telemetry, ParticleTelemetryConfig, ResolvedParticles, TelemetryBuffer, TelemetryRow,
    V10_PARTICLES,
};
pub use csv::{flush_csv, flush_rows};
pub use judge::{
    cell_verdict, check_bone_rigidity, check_invariants, diff_rows, rows_hash, tail_verdict, Bone,
    InvariantConfig, InvariantReport, RigidityReport, Verdict,
};
pub use plugin::HeadlessTelemetryPlugin;
