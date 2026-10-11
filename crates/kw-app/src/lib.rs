//! The Kerfwright application: the welcome page, projects, and the workspaces, plus the Rayon ⇄ Tokio bridge,
//! cancellation, and command/event types the engines talk through. See docs/03-architecture.md.

pub mod bridge;
pub mod cam;
pub mod dialogs;
pub mod ecad;
pub mod fea;
pub mod job;
pub mod messages;
pub mod model;
pub mod project;
pub mod recent;
pub mod render;
pub mod sketch;
pub mod ui;
pub mod viewport;

pub use bridge::{spawn_compute, CancelToken};
pub use messages::{Command, Event, Priority};
