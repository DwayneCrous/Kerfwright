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
//! * **Tangent chains** ([`fillet`], [`chamfer`], [`tangent_chain`]): naming one edge of a run of
//!   tangent-continuous edges (lines and circular arcs, e.g. the outline of a rounded rectangle) blends
//!   the whole run. Each edge gets its own tool ending in an exact cap at the junction; the tools of a
//!   chain are united first (their caps coincide) and applied in one Boolean, which needs the exact
//!   cylinder / torus and plane / cone tangent contacts of `kw-ssi`.
//! * **Vertex blends** (in [`fillet`]): where three planar faces meet at a vertex whose three straight
//!   edges are all filleted with the same radius and convexity, a rolling-ball sphere patch replaces
//!   the mitre of the three edge blends.
//!
//! Limits (reported as [`BlendError`] naming the offending edge): edges whose end faces are not
//! planes (and are not a chain junction), curved edges other than coaxial circular arcs, faces other
//! than plane / cylinder / cone, radii larger than the available faces. Vertex blends exist for the
//! three-plane, equal-radius, equal-convexity corner; other corners (unequal radii, mixed convexity,
//! four or more edges, chamfers) meet in a mitre by design, setbacks are not provided. A radius law
//! is not applied along a tangent chain.

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

mod blend;
mod glue;
mod vertex;

pub use blend::{
    chamfer, fillet, fillet_law, fillet_variable, tangent_chain, BlendError, BlendErrorKind,
    RadiusLaw,
};
