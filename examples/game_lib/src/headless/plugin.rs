//! The telemetry plugin: installs the shared resources.
//!
//! It deliberately does **not** schedule [`crate::headless::capture_telemetry`]
//! or pick an output cadence — those are harness concerns (some sample in
//! `Update`, others in `FixedPostUpdate`). The harness inserts a
//! [`crate::headless::ParticleTelemetryConfig`] once its character entity is
//! known and adds the capture system in its chosen schedule.

use bevy::prelude::*;

use super::channels::{ResolvedParticles, TelemetryBuffer, TelemetryRow};

/// Adds the headless-telemetry resources. Add once per harness app.
#[derive(Default)]
pub struct HeadlessTelemetryPlugin;

impl Plugin for HeadlessTelemetryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TelemetryRow>()
            .init_resource::<TelemetryBuffer>()
            .init_resource::<ResolvedParticles>();
    }
}
