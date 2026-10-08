//! Application skeleton: Rayon (CPU-bound kernel work) ⇄ Tokio (I/O,
//! collaboration) bridge, cancellation, and command/event types.
//! See docs/03-architecture.md.

pub mod bridge;
pub mod messages;

pub use bridge::{spawn_compute, CancelToken};
pub use messages::{Command, Event, Priority};
