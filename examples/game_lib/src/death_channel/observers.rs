//! General observers for the death-channel infrastructure.
//!
//! The custom per-body-part observers (`on_bodypart_first_hit`,
//! `on_bodypart_second_hit`, `on_bodypart_detach`, and the
//! `on_delayed_detach` timer callback) are located in
//! [`crate::character_factory::observers`] because they contain
//! game-specific logic (spawning delayed events, looking up
//! `Connectivity`, etc.).
//!
//! This file is intentionally minimal — only truly generic
//! death-channel observers would go here.
