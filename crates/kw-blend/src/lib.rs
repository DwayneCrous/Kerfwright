//! Fillets, variable-radius fillets, vertex blends and chamfers.
//!
//! Each edge blend is realised as a boolean with a swept "spandrel" tool: the region between the
//! sharp corner and the blend profile, extended by a margin so that no tool face is coincident with
//! a body face. Straight edges between planes (or a plane and a cylinder parallel to the edge) use an
//! extruded tool, circular rim edges between coaxial plane / cylinder / cone faces use a revolved
//! tool. Convex edges subtract the tool, concave edges add it.
//!
//! * **Variable radius** ([`fillet_law`], [`RadiusLaw`]): the section is built at stations along the
//!   edge and skinned (a lofted tool along a straight edge, a closed periodic loft around a rim).
//! * **Vertex blends** (in [`fillet`]): where three planar faces meet at a vertex whose three straight
//!   edges are all filleted with the same radius and convexity, a rolling-ball sphere patch replaces
//!   the mitre of the three edge blends.
//!
//! Limits (reported as [`BlendError`] naming the offending edge): edges whose end faces are not
//! planes, curved edges other than coaxial rim circles, faces other than plane / cylinder / cone,
//! radii larger than the available faces, vertex blends other than the three-plane case (those edges
//! meet in a mitre).

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

mod blend;
mod vertex;

pub use blend::{
    chamfer, fillet, fillet_law, fillet_variable, BlendError, BlendErrorKind, RadiusLaw,
};
