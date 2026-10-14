//! Modelled threads. A single-start thread is a twisted extrusion of its section: a plane perpendicular to
//! the axis cuts the screw in a closed curve `r(θ) = f(P·θ/2π)` (`f` being the axial profile over one
//! pitch), and sweeping that curve straight along the axis while turning it by 2π per pitch makes the
//! screw. The tool that cuts a thread is built that way: a full prism whose section is the cavity of an
//! internal thread, or a ring whose hole is the section of an external thread. The tool meets the body
//! it cuts in planes and in the flanks crossing the old cylinder, so no helical curve has to be built.
//!
//! Section (ISO basic profile): a flat of P/8 at the large radius, a 30° flank, a flat at the small radius,
//! a second flank. The flanks are Archimedean spirals in the section (r linear in θ).

use kw_brep::{BRep, BuildResult};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::{sweep, Profile, ProfileEdge, ProfileLoop, SweepOptions};
use kw_nurbs::fit::hermite_fit;
use kw_nurbs::{CurveGeom, KnotVector, NurbsCurve};
use std::f64::consts::TAU;

/// The section of a thread tool: radii and the fractions of one turn taken by the large flat, a flank, the
/// small flat and the other flank.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreadSection {
    pub big: f64,
    pub small: f64,
    pub pitch: f64,
    /// Large flat, flank, small flat, flank (they add up to 1).
    pub w: [f64; 4],
}

impl ThreadSection {
    /// The section with a P/8 flat at radius `big`, 30° flanks, and the rest of the pitch at radius
    /// `small`. `Err` when the radii leave no room for the small flat.
    pub fn new(big: f64, small: f64, pitch: f64) -> Result<ThreadSection, String> {
        if !(big > small && small > 0.0 && pitch > 0.0) {
            return Err(
                "a thread needs 0 < small radius < large radius and a positive pitch".into(),
            );
        }
        let wc = 1.0 / 8.0;
        let wf = (big - small) * 30f64.to_radians().tan() / pitch;
        let wr = 1.0 - wc - 2.0 * wf;
        if wr < 0.02 {
            return Err(format!(
                "the thread is too deep for its pitch ({:.2} mm deep at a pitch of {:.2} mm)",
                big - small,
                pitch
            ));
        }
        Ok(ThreadSection {
            big,
            small,
            pitch,
            w: [wc, wf, wr, wf],
        })
    }

    /// Area of the section (the disc inside the curve), mm².
    pub fn area(&self) -> f64 {
        let [wc, wf, wr, _] = self.w;
        let (b, s) = (self.big, self.small);
        0.5 * TAU * (b * b * wc + s * s * wr + 2.0 * wf * (b * b + b * s + s * s) / 3.0)
    }

    /// Radius of the section at angle `t` from the start of the large flat.
    pub fn radius(&self, t: f64) -> f64 {
        let u = (t / TAU).rem_euclid(1.0);
        let [wc, wf, wr, _] = self.w;
        let (b, s) = (self.big, self.small);
        if u < wc {
            b
        } else if u < wc + wf {
            b + (s - b) * (u - wc) / wf
        } else if u < wc + wf + wr {
            s
        } else {
            s + (b - s) * (u - wc - wf - wr) / wf
        }
    }

    /// The four edges of the section in `plane`, counter-clockwise from the plane's x axis.
    pub fn edges(&self, plane: Frame3) -> Result<Vec<ProfileEdge>, String> {
        let [wc, wf, wr, _] = self.w;
        let (b, s) = (self.big, self.small);
        let a1 = TAU * wc;
        let a2 = a1 + TAU * wf;
        let a3 = a2 + TAU * wr;
        let a4 = a3 + TAU * wf;
        let spiral = |t0: f64, t1: f64, r0: f64, r1: f64| -> Result<ProfileEdge, String> {
            let k = (r1 - r0) / (t1 - t0);
            let mut f = |t: f64| {
                let r = r0 + k * (t - t0);
                let (c, sn) = (t.cos(), t.sin());
                let pt = plane.origin + plane.x * (r * c) + plane.y * (r * sn);
                let d = plane.x * (k * c - r * sn) + plane.y * (k * sn + r * c);
                Some((pt, d))
            };
            let c = hermite_fit(&mut f, t0, t1, 1e-5, 4, 256)
                .ok_or("the thread flank could not be fitted")?;
            let dom = c.domain();
            Ok(ProfileEdge::new(CurveGeom::Nurbs(c), dom))
        };
        Ok(vec![
            ProfileEdge::arc(&plane, b, 0.0, a1),
            spiral(a1, a2, b, s)?,
            ProfileEdge::arc(&plane, s, a2, a3),
            spiral(a3, a4, s, b)?,
        ])
    }

    /// The same loop run clockwise (for a hole in a profile).
    fn reversed_edges(&self, plane: Frame3) -> Result<Vec<ProfileEdge>, String> {
        Ok(self
            .edges(plane)?
            .into_iter()
            .rev()
            .map(|mut e| {
                e.reversed = !e.reversed;
                e
            })
            .collect())
    }
}

/// A thread tool: where it starts, how long it is and what it cuts.
#[derive(Debug, Clone, PartialEq)]
pub struct ThreadTool {
    /// The start plane: origin on the axis, `z` along the axis into the material.
    pub frame: Frame3,
    pub section: ThreadSection,
    pub length: f64,
    pub right_hand: bool,
    /// `Some(r)`: a ring of outer radius `r` whose hole is the section (external thread: the tool is
    /// the groove). `None`: the full section (internal thread: the tool is the cavity).
    pub outer_radius: Option<f64>,
}

/// Build the tool into `rep`.
pub fn build_thread_tool(rep: &mut BRep, t: &ThreadTool) -> Result<BuildResult, String> {
    if !(t.length > 0.0) {
        return Err("a thread needs a positive length".into());
    }
    let plane = t.frame;
    let profile = match t.outer_radius {
        None => Profile::new(plane, ProfileLoop::new(t.section.edges(plane)?)),
        Some(r) => {
            if r <= t.section.big {
                return Err("the ring is not wider than the thread".into());
            }
            Profile::new(plane, ProfileLoop::circle(&plane, r))
                .with_hole(ProfileLoop::new(t.section.reversed_edges(plane)?))
        }
    };
    let end = plane.origin + plane.z * t.length;
    let path = CurveGeom::Nurbs(
        NurbsCurve::new_polynomial(
            KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).map_err(|e| format!("{e:?}"))?,
            vec![Point3::from(plane.origin), Point3::from(end)],
        )
        .map_err(|e| format!("{e:?}"))?,
    );
    let turns = t.length / t.section.pitch;
    let opts = SweepOptions {
        twist: if t.right_hand {
            TAU * turns
        } else {
            -TAU * turns
        },
        max_sections: 1024,
        ..Default::default()
    };
    sweep(rep, &profile, &path, &opts).map_err(|e| format!("thread: {e}"))
}

/// The axis of a cylinder face as a frame: origin at the start, `z` pointing from `start` towards `end`.
pub fn axis_frame(origin: Point3<f64>, z: Vector3<f64>, x_hint: Vector3<f64>) -> Frame3 {
    let z = z.normalize();
    let mut x = x_hint - z * x_hint.dot(&z);
    if x.norm() < 1e-9 {
        x = if z.x.abs() < 0.9 {
            Vector3::x() - z * z.x
        } else {
            Vector3::y() - z * z.y
        };
    }
    let x = x.normalize();
    Frame3 {
        origin,
        x,
        y: z.cross(&x),
        z,
    }
}
