//! Solid-modelling operations that build B-reps from profiles: extrude and revolve (plus the
//! mass-property helpers re-exported from `kw-brep`). Every operation returns a
//! [`BuildResult`] with a [`ShapeHistory`](kw_brep::ShapeHistory).

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

mod extrude;
mod offset;
mod profile;
mod rebuild;
mod revolve;
mod skin;

pub use extrude::{extrude, ExtrudeOptions};
pub use offset::{draft_faces, offset_body, shell};
pub use profile::{ModelError, Profile, ProfileEdge, ProfileLoop};
pub use rebuild::{rebuild, Rebuilt};
pub use revolve::revolve;
pub use skin::{loft, loft_ring, sweep, SweepOptions};

pub use kw_brep::BuildResult;
