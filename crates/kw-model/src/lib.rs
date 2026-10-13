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
mod patch;
mod profile;
mod rebuild;
mod revolve;
mod sheetops;
mod skin;

pub use extrude::{extrude, extrude_sheet, ExtrudeOptions};
pub use offset::{draft_faces, offset_body, shell};
pub use patch::{loftable, patch_sheet, ruled_sheet, NormalFn, PatchEdge};
pub use profile::{Chain, ModelError, Profile, ProfileEdge, ProfileLoop};
pub use rebuild::{rebuild, Rebuilt};
pub use revolve::{revolve, revolve_sheet};
pub use sheetops::{offset_sheet, stitch, thicken, unstitch, ThickenSide};
pub use skin::{loft, loft_ring, loft_sheet, sweep, sweep_sheet, SweepOptions};

pub use kw_brep::BuildResult;
