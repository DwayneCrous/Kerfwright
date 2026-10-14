//! A screw is the twisted extrusion of its plane section: volumes against `area × length`, and the cuts a
//! thread tool makes (planes and the flanks crossing an old cylinder) against closed forms.

use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::{BRep, BodyId};
use kw_math::{Frame3, Point3};
use kw_model::*;
use kw_nurbs::fit::hermite_fit;
use kw_nurbs::CurveGeom;
use std::f64::consts::TAU;

const PITCH: f64 = 1.25;

fn path(h: f64) -> CurveGeom {
    CurveGeom::Nurbs(
        kw_nurbs::NurbsCurve::new_polynomial(
            kw_nurbs::KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).unwrap(),
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, h)],
        )
        .unwrap(),
    )
}

fn at_z(z: f64) -> Frame3 {
    Frame3 {
        origin: Point3::new(0.0, 0.0, z),
        ..Frame3::world()
    }
}

/// The section of an ISO thread: a flat of P/8 at radius `big`, 30° flanks down to `small`, the rest flat.
struct Section {
    big: f64,
    small: f64,
    w: [f64; 4],
}

fn section(big: f64, small: f64) -> Section {
    let wf = (big - small) * 30f64.to_radians().tan() / PITCH;
    Section {
        big,
        small,
        w: [0.125, wf, 1.0 - 0.125 - 2.0 * wf, wf],
    }
}

impl Section {
    fn area(&self) -> f64 {
        let [wc, wf, wr, _] = self.w;
        let (b, s) = (self.big, self.small);
        0.5 * TAU * (b * b * wc + s * s * wr + 2.0 * wf * (b * b + b * s + s * s) / 3.0)
    }

    fn radius(&self, t: f64) -> f64 {
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

    /// Area of the section with its radius held to `lo..=hi`.
    fn clipped_area(&self, lo: f64, hi: f64) -> f64 {
        let n = 200_000;
        0.5 * TAU / n as f64
            * (0..n)
                .map(|k| {
                    self.radius(TAU * (k as f64 + 0.5) / n as f64)
                        .clamp(lo, hi)
                        .powi(2)
                })
                .sum::<f64>()
    }

    fn edges(&self, plane: Frame3) -> Vec<ProfileEdge> {
        let [wc, wf, wr, _] = self.w;
        let (b, s) = (self.big, self.small);
        let a1 = TAU * wc;
        let a2 = a1 + TAU * wf;
        let a3 = a2 + TAU * wr;
        let a4 = a3 + TAU * wf;
        let spiral = |t0: f64, t1: f64, r0: f64, r1: f64| -> ProfileEdge {
            let k = (r1 - r0) / (t1 - t0);
            let mut f = |t: f64| {
                let r = r0 + k * (t - t0);
                let (c, sn) = (t.cos(), t.sin());
                let pt = plane.origin + plane.x * (r * c) + plane.y * (r * sn);
                let d = plane.x * (k * c - r * sn) + plane.y * (k * sn + r * c);
                Some((pt, d))
            };
            let c = hermite_fit(&mut f, t0, t1, 1e-5, 4, 256).expect("spiral fit");
            let dom = c.domain();
            ProfileEdge::new(CurveGeom::Nurbs(c), dom)
        };
        vec![
            ProfileEdge::arc(&plane, b, 0.0, a1),
            spiral(a1, a2, b, s),
            ProfileEdge::arc(&plane, s, a2, a3),
            spiral(a3, a4, s, b),
        ]
    }

    fn profile(&self, plane: Frame3) -> Profile {
        Profile::new(plane, ProfileLoop::new(self.edges(plane)))
    }

    fn ring(&self, plane: Frame3, outer: f64) -> Profile {
        let hole: Vec<ProfileEdge> = self
            .edges(plane)
            .into_iter()
            .rev()
            .map(|mut e| {
                e.reversed = !e.reversed;
                e
            })
            .collect();
        Profile::new(plane, ProfileLoop::circle(&plane, outer)).with_hole(ProfileLoop::new(hole))
    }
}

fn tool(rep: &mut BRep, profile: &Profile, length: f64) -> BodyId {
    sweep(
        rep,
        profile,
        &path(length),
        &SweepOptions {
            twist: TAU * length / PITCH,
            ..Default::default()
        },
    )
    .expect("the thread tool")
    .body
}

fn mesh_volume(rep: &BRep, b: BodyId) -> f64 {
    kw_tess::tessellate_body(
        rep,
        b,
        &kw_tess::TessOptions {
            chord_tol: 0.003,
            angle_tol: 0.15,
        },
    )
    .expect("tessellation")
    .mesh
    .signed_volume()
}

fn cut(rep: &mut BRep, a: BodyId, b: BodyId) -> BodyId {
    boolean(rep, a, b, Op::Subtract, &BoolOptions::default())
        .unwrap_or_else(|e| panic!("the cut failed: {e}"))
        .body
}

fn box_body(rep: &mut BRep, half: f64, z0: f64, z1: f64) -> BodyId {
    let f = Frame3 {
        origin: Point3::new(0.0, 0.0, 0.5 * (z0 + z1)),
        ..Frame3::world()
    };
    kw_brep::builders::make_box(rep, &f, 2.0 * half, 2.0 * half, z1 - z0).body
}

fn cylinder(rep: &mut BRep, r: f64, z0: f64, z1: f64) -> BodyId {
    kw_brep::builders::make_cylinder(rep, &at_z(z0), r, z1 - z0).body
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// The turns the cuts are tried with: a few in a debug build.
fn turns() -> Vec<f64> {
    if cfg!(debug_assertions) {
        vec![1.0, 2.0]
    } else {
        vec![1.0, 5.0, 12.0]
    }
}

#[test]
fn a_twisted_section_is_a_screw_for_any_number_of_turns() {
    let s = section(4.0, 4.0 - 0.6134 * PITCH);
    for n in [0.5, 1.0, 2.0, 5.0, 12.0, 20.0] {
        if cfg!(debug_assertions) && n > 5.0 {
            continue;
        }
        let len = n * PITCH;
        let mut rep = BRep::new();
        let t0 = std::time::Instant::now();
        let b = tool(&mut rep, &s.profile(at_z(0.0)), len);
        let secs = t0.elapsed().as_secs_f64();
        assert!(
            rep.validate().is_ok(),
            "{n} turns: {:?}",
            rep.validate().errors
        );
        let v = rep.volume(b).unwrap();
        assert!(
            rel(v, s.area() * len) < 1e-5,
            "{n} turns: {v} vs {}",
            s.area() * len
        );
        // a face per turn and flank, not one that winds around many times
        let turns_ceil = n.ceil() as usize;
        assert_eq!(rep.body_faces(b).len(), 4 * turns_ceil + 2, "{n} turns");
        assert!(rel(mesh_volume(&rep, b), v) < 0.02, "{n} turns");
        eprintln!("{n} turns: {secs:.2} s");
    }
}

#[test]
fn a_thread_cavity_cut_from_a_block_removes_its_section() {
    let s = section(4.0, 4.0 - 0.5413 * PITCH);
    for n in turns() {
        let len = n * PITCH;
        let mut rep = BRep::new();
        let block = box_body(&mut rep, 7.0, 0.0, len);
        let t = tool(&mut rep, &s.profile(at_z(-1.0)), len + 2.0);
        let b = cut(&mut rep, block, t);
        assert!(rep.validate().is_ok(), "{n} turns");
        let want = 196.0 * len - s.area() * len;
        let v = rep.volume(b).unwrap();
        assert!(rel(v, want) < 1e-5, "{n} turns: {v} vs {want}");
    }
}

#[test]
fn a_groove_ring_cut_from_a_pin_leaves_the_external_thread() {
    // the groove's crest flats sit 0.02 mm outside the pin, so the pin's own surface stays as the crest
    let r_pin = 4.0;
    let s = section(r_pin + 0.02, 4.0 - 0.6134 * PITCH);
    for n in turns() {
        let len = n * PITCH;
        let mut rep = BRep::new();
        let pin = cylinder(&mut rep, r_pin, 0.0, len);
        let t = tool(&mut rep, &s.ring(at_z(-1.0), 6.0), len + 2.0);
        let b = cut(&mut rep, pin, t);
        assert!(rep.validate().is_ok(), "{n} turns");
        let want = s.clipped_area(0.0, r_pin) * len;
        let v = rep.volume(b).unwrap();
        assert!(rel(v, want) < 1e-5, "{n} turns: {v} vs {want}");
    }
}

#[test]
fn a_thread_is_cut_into_an_existing_drilled_hole() {
    // the cavity's root lands sit 0.02 mm inside the drilled wall, its flanks cross the wall along helices
    let r_hole = 3.35;
    let s = section(4.0, r_hole - 0.02);
    for n in turns() {
        let len = n * PITCH;
        let mut rep = BRep::new();
        let block = box_body(&mut rep, 7.0, 0.0, len);
        let drill = cylinder(&mut rep, r_hole, -1.0, len + 1.0);
        let drilled = cut(&mut rep, block, drill);
        let t = tool(&mut rep, &s.profile(at_z(-1.0)), len + 2.0);
        let b = cut(&mut rep, drilled, t);
        assert!(rep.validate().is_ok(), "{n} turns");
        let want = 196.0 * len - s.clipped_area(r_hole, 4.0) * len;
        let v = rep.volume(b).unwrap();
        assert!(rel(v, want) < 1e-5, "{n} turns: {v} vs {want}");
    }
}

#[test]
fn points_on_either_side_of_the_thread_surface_are_classified_correctly() {
    let s = section(4.0, 4.0 - 0.5413 * PITCH);
    let len = 3.0 * PITCH;
    let mut rep = BRep::new();
    let block = box_body(&mut rep, 7.0, 0.0, len);
    let t = tool(&mut rep, &s.profile(at_z(-1.0)), len + 2.0);
    let b = cut(&mut rep, block, t);
    // the tool starts at z = -1, so the section at height z is turned by 2π (z + 1) / P
    let (mut checked, mut ambiguous) = (0, 0);
    for k in 0..60 {
        let a = TAU * (k as f64 + 0.37) / 60.0;
        let z = 0.5 + 0.05 * k as f64;
        let rs = s.radius(a - TAU * (z + 1.0) / PITCH);
        // beyond the cavity is material, inside it is air
        for (dr, want) in [
            (0.05, kw_brep::PointClass::In),
            (-0.05, kw_brep::PointClass::Out),
        ] {
            let r = rs + dr;
            let p = Point3::new(r * a.cos(), r * a.sin(), z);
            match kw_brep::classify_point(&rep, b, &p, 1e-7) {
                Ok(c) => {
                    assert_eq!(c, want, "angle {a:.3}, z {z:.2}, section radius {rs:.3}");
                    checked += 1;
                }
                Err(_) => ambiguous += 1,
            }
        }
    }
    assert!(checked > 100, "{checked} classified, {ambiguous} ambiguous");
}
