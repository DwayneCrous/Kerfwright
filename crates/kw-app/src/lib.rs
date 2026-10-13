//! The Kerfwright application: the welcome page, projects, and the workspaces, plus the Rayon ⇄ Tokio bridge,
//! cancellation, and command/event types the engines talk through. See docs/03-architecture.md.

pub mod bodies;
pub mod bridge;
pub mod cam;
pub mod chrome;
pub mod design;
pub mod dialogs;
pub mod drawing;
pub mod ecad;
pub mod exchange;
pub mod fea;
pub mod job;
pub mod messages;
pub mod model;
pub mod model_source;
pub mod project;
pub mod recent;
pub mod render;
pub mod sketch;
pub mod ui;
pub mod viewport;

pub use bridge::{spawn_compute, CancelToken};
pub use messages::{Command, Event, Priority};
