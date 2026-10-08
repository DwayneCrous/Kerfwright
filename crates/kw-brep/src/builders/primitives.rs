use super::BuildResult;
use crate::history::{EntityRef, Origin, ShapeHistory};
use crate::ids::*;
use crate::store::{BRep, CoedgeSpec};
use crate::topology::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::geom::{Circle, Cone, Cylinder, Line, Plane, Sphere, Torus};
use kw_nurbs::{CurveGeom, SurfaceGeom};
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, TAU};

fn line2(brep: &mut BRep, o: (f64, f64), d: (f64, f64)) -> CurveId {
    brep.add_curve(CurveGeom::Line(Line {
        origin: Point3::new(o.0, o.1, 0.0),
        dir: Vector3::new(d.0, d.1, 0.0),
    }))
}

fn new_solid(brep: &mut BRep) -> (BodyId, ShellId) {
    let body = brep.add_body(BodyKind::Solid);
    let lump = brep.add_lump(body);
    let shell = brep.add_shell(lump);
    (body, shell)
}

fn note_face(h: &mut ShapeHistory, f: FaceId, role: &'static str) {
    h.record(EntityRef::Face(f), Origin::New { role });
}

/// Axis-aligned (in `frame`) box centred at `frame.origin` with the given
/// full extents along `frame.x/y/z`.
pub fn make_box(brep: &mut BRep, frame: &Frame3, dx: f64, dy: f64, dz: f64) -> BuildResult {
    assert!(
        dx > 0.0 && dy > 0.0 && dz > 0.0,
        "box extents must be positive"
    );
    let (body, shell) = new_solid(brep);
    let mut history = ShapeHistory::default();
    let half = [dx / 2.0, dy / 2.0, dz / 2.0];
    let axes = [frame.x, frame.y, frame.z];
    let corner = |idx: usize| {
        let mut p = frame.origin;
        for a in 0..3 {
            let s = if (idx >> a) & 1 == 1 { 1.0 } else { -1.0 };
            p += axes[a] * (s * half[a]);
        }
        p
    };
    let verts: Vec<VertexId> = (0..8).map(|i| brep.add_vertex(corner(i))).collect();
    let mut edge_of: HashMap<(usize, usize), EdgeId> = HashMap::new();
    let mut new_edges: Vec<EdgeId> = Vec::new();
    let mut get_edge = |brep: &mut BRep, a: usize, b: usize| -> (EdgeId, Sense) {
        let (lo, hi) = (a.min(b), a.max(b));
        let id = *edge_of.entry((lo, hi)).or_insert_with(|| {
            let (p0, p1) = (corner(lo), corner(hi));
            let c = brep.add_curve(CurveGeom::Line(Line {
                origin: p0,
                dir: p1 - p0,
            }));
            let e = brep.add_edge(verts[lo], verts[hi], Some(c), (0.0, 1.0));
            new_edges.push(e);
            e
        });
        (
            id,
            if a == lo {
                Sense::Forward
            } else {
                Sense::Reversed
            },
        )
    };

    let roles = [["-x", "+x"], ["-y", "+y"], ["-z", "+z"]];
    for axis in 0..3usize {
        for (si, sign) in [-1.0_f64, 1.0].iter().enumerate() {
            let (a1, a2) = ((axis + 1) % 3, (axis + 2) % 3);
            // right-handed (x, y, z = outward normal)
            let (ux, uy) = if *sign > 0.0 { (a1, a2) } else { (a2, a1) };
            let pf = Frame3 {
                origin: frame.origin + axes[axis] * (sign * half[axis]),
                x: axes[ux],
                y: axes[uy],
                z: axes[axis] * *sign,
            };
            let surf = brep.add_surface(SurfaceGeom::Plane(Plane { frame: pf }));
            let face = brep.add_face(shell, surf, Sense::Forward);
            note_face(&mut history, face, roles[axis][si]);
            // corners CCW in (u, v)
            let bits = |su: f64, sv: f64| {
                let mut idx = 0usize;
                idx |= ((*sign > 0.0) as usize) << axis;
                idx |= ((su > 0.0) as usize) << ux;
                idx |= ((sv > 0.0) as usize) << uy;
                idx
            };
            let cs = [
                bits(-1.0, -1.0),
                bits(1.0, -1.0),
                bits(1.0, 1.0),
                bits(-1.0, 1.0),
            ];
            let uv = |idx: usize| {
                let p = corner(idx) - pf.origin;
                (p.dot(&pf.x), p.dot(&pf.y))
            };
            let mut uses = Vec::new();
            for i in 0..4 {
                let (a, b) = (cs[i], cs[(i + 1) % 4]);
                let (e, sense) = get_edge(brep, a, b);
                let (lo, hi) = (a.min(b), a.max(b));
                let (p0, p1) = (uv(lo), uv(hi));
                let pc = line2(brep, p0, (p1.0 - p0.0, p1.1 - p0.1));
                uses.push(CoedgeSpec {
                    edge: e,
                    sense,
                    pcurve: Some(pc),
                });
            }
            brep.add_loop(face, LoopKind::Outer, &uses);
        }
    }
    for e in new_edges {
        history.record(EntityRef::Edge(e), Origin::New { role: "box-edge" });
    }
    BuildResult { body, history }
}

/// Cylinder with base centre at `frame.origin`, axis `frame.z`.
pub fn make_cylinder(brep: &mut BRep, frame: &Frame3, radius: f64, height: f64) -> BuildResult {
    make_frustum(brep, frame, radius, radius, height)
}

/// Cone / frustum: base radius `r_base` at `frame.origin`, top radius `r_top`
/// (`0` gives an apex) at height `height` along `frame.z`.
pub fn make_cone(
    brep: &mut BRep,
    frame: &Frame3,
    r_base: f64,
    r_top: f64,
    height: f64,
) -> BuildResult {
    make_frustum(brep, frame, r_base, r_top, height)
}

fn make_frustum(brep: &mut BRep, frame: &Frame3, r0: f64, r1: f64, h: f64) -> BuildResult {
    assert!(
        r0 > 0.0 && r1 >= 0.0 && h > 0.0,
        "need r_base > 0, r_top >= 0, height > 0"
    );
    let (body, shell) = new_solid(brep);
    let mut history = ShapeHistory::default();
    let top_origin = frame.origin + frame.z * h;
    let apex = r1 == 0.0;

    let side_surface = if (r0 - r1).abs() < 1e-15 {
        SurfaceGeom::Cylinder(Cylinder {
            frame: *frame,
            radius: r0,
        })
    } else {
        SurfaceGeom::Cone(Cone {
            frame: *frame,
            radius_at_origin: r0,
            half_angle: ((r1 - r0) / h).atan(),
        })
    };

    let vb = brep.add_vertex(frame.origin + frame.x * r0);
    let vt = brep.add_vertex(if apex {
        top_origin
    } else {
        top_origin + frame.x * r1
    });

    let bot_curve = brep.add_curve(CurveGeom::Circle(Circle {
        frame: *frame,
        radius: r0,
    }));
    let e_bot = brep.add_edge(vb, vb, Some(bot_curve), (0.0, TAU));
    let seam_curve = {
        let p0 = frame.origin + frame.x * r0;
        let p1 = if apex {
            top_origin
        } else {
            top_origin + frame.x * r1
        };
        brep.add_curve(CurveGeom::Line(Line {
            origin: p0,
            dir: p1 - p0,
        }))
    };
    let e_seam = brep.add_edge(vb, vt, Some(seam_curve), (0.0, 1.0));
    history.record(
        EntityRef::Edge(e_bot),
        Origin::New {
            role: "bottom-circle",
        },
    );
    history.record(EntityRef::Edge(e_seam), Origin::New { role: "seam" });

    // side face -------------------------------------------------------------
    let side_surf = brep.add_surface(side_surface);
    let side = brep.add_face(shell, side_surf, Sense::Forward);
    note_face(&mut history, side, "side");
    let pc_bot = line2(brep, (0.0, 0.0), (1.0, 0.0));
    let pc_seam_r = line2(brep, (TAU, 0.0), (0.0, h));
    let pc_seam_l = line2(brep, (0.0, 0.0), (0.0, h));
    let mut uses = vec![
        CoedgeSpec {
            edge: e_bot,
            sense: Sense::Forward,
            pcurve: Some(pc_bot),
        },
        CoedgeSpec {
            edge: e_seam,
            sense: Sense::Forward,
            pcurve: Some(pc_seam_r),
        },
    ];
    if apex {
        let e_apex = brep.add_edge(vt, vt, None, (0.0, 1.0));
        history.record(EntityRef::Edge(e_apex), Origin::New { role: "apex" });
        let pc_apex = line2(brep, (0.0, h), (TAU, 0.0));
        uses.push(CoedgeSpec {
            edge: e_apex,
            sense: Sense::Reversed,
            pcurve: Some(pc_apex),
        });
        uses.push(CoedgeSpec {
            edge: e_seam,
            sense: Sense::Reversed,
            pcurve: Some(pc_seam_l),
        });
        brep.add_loop(side, LoopKind::Outer, &uses);
    } else {
        let top_curve = brep.add_curve(CurveGeom::Circle(Circle {
            frame: Frame3 {
                origin: top_origin,
                ..*frame
            },
            radius: r1,
        }));
        let e_top = brep.add_edge(vt, vt, Some(top_curve), (0.0, TAU));
        history.record(EntityRef::Edge(e_top), Origin::New { role: "top-circle" });
        let pc_top = line2(brep, (0.0, h), (1.0, 0.0));
        uses.push(CoedgeSpec {
            edge: e_top,
            sense: Sense::Reversed,
            pcurve: Some(pc_top),
        });
        uses.push(CoedgeSpec {
            edge: e_seam,
            sense: Sense::Reversed,
            pcurve: Some(pc_seam_l),
        });
        brep.add_loop(side, LoopKind::Outer, &uses);

        // top cap (+z, plane frame = cylinder frame at top)
        let cap_frame = Frame3 {
            origin: top_origin,
            ..*frame
        };
        let cs = brep.add_surface(SurfaceGeom::Plane(Plane { frame: cap_frame }));
        let cap = brep.add_face(shell, cs, Sense::Forward);
        note_face(&mut history, cap, "top");
        let pc = brep.add_curve(CurveGeom::Circle(Circle {
            frame: Frame3::world(),
            radius: r1,
        }));
        brep.add_loop(
            cap,
            LoopKind::Outer,
            &[CoedgeSpec {
                edge: e_top,
                sense: Sense::Forward,
                pcurve: Some(pc),
            }],
        );
    }

    // bottom cap (−z): plane frame with flipped y and z so (x, y, z) stays right-handed
    let bf = Frame3 {
        origin: frame.origin,
        x: frame.x,
        y: -frame.y,
        z: -frame.z,
    };
    let bs = brep.add_surface(SurfaceGeom::Plane(Plane { frame: bf }));
    let bottom = brep.add_face(shell, bs, Sense::Forward);
    note_face(&mut history, bottom, "bottom");
    let flip = Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: -Vector3::y(),
        z: -Vector3::z(),
    };
    let pc = brep.add_curve(CurveGeom::Circle(Circle {
        frame: flip,
        radius: r0,
    }));
    brep.add_loop(
        bottom,
        LoopKind::Outer,
        &[CoedgeSpec {
            edge: e_bot,
            sense: Sense::Reversed,
            pcurve: Some(pc),
        }],
    );

    BuildResult { body, history }
}

/// Sphere centred at `frame.origin` with poles on `±frame.z`.
pub fn make_sphere(brep: &mut BRep, frame: &Frame3, radius: f64) -> BuildResult {
    assert!(radius > 0.0);
    let (body, shell) = new_solid(brep);
    let mut history = ShapeHistory::default();
    let south = brep.add_vertex(frame.origin - frame.z * radius);
    let north = brep.add_vertex(frame.origin + frame.z * radius);
    let meridian = brep.add_curve(CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: frame.origin,
            x: frame.x,
            y: frame.z,
            z: -frame.y,
        },
        radius,
    }));
    let e_seam = brep.add_edge(south, north, Some(meridian), (-FRAC_PI_2, FRAC_PI_2));
    let e_south = brep.add_edge(south, south, None, (0.0, 1.0));
    let e_north = brep.add_edge(north, north, None, (0.0, 1.0));
    history.record(EntityRef::Edge(e_seam), Origin::New { role: "meridian" });

    let surf = brep.add_surface(SurfaceGeom::Sphere(Sphere {
        frame: *frame,
        radius,
    }));
    let face = brep.add_face(shell, surf, Sense::Forward);
    note_face(&mut history, face, "sphere");
    let pc_s = line2(brep, (0.0, -FRAC_PI_2), (TAU, 0.0));
    let pc_n = line2(brep, (0.0, FRAC_PI_2), (TAU, 0.0));
    let pc_r = line2(brep, (TAU, 0.0), (0.0, 1.0));
    let pc_l = line2(brep, (0.0, 0.0), (0.0, 1.0));
    brep.add_loop(
        face,
        LoopKind::Outer,
        &[
            CoedgeSpec {
                edge: e_south,
                sense: Sense::Forward,
                pcurve: Some(pc_s),
            },
            CoedgeSpec {
                edge: e_seam,
                sense: Sense::Forward,
                pcurve: Some(pc_r),
            },
            CoedgeSpec {
                edge: e_north,
                sense: Sense::Reversed,
                pcurve: Some(pc_n),
            },
            CoedgeSpec {
                edge: e_seam,
                sense: Sense::Reversed,
                pcurve: Some(pc_l),
            },
        ],
    );
    BuildResult { body, history }
}

/// Torus centred at `frame.origin`, axis `frame.z`.
pub fn make_torus(brep: &mut BRep, frame: &Frame3, major: f64, minor: f64) -> BuildResult {
    assert!(major > minor && minor > 0.0, "need major > minor > 0");
    let (body, shell) = new_solid(brep);
    let mut history = ShapeHistory::default();
    let v = brep.add_vertex(frame.origin + frame.x * (major + minor));
    let cu = brep.add_curve(CurveGeom::Circle(Circle {
        frame: *frame,
        radius: major + minor,
    }));
    let cv = brep.add_curve(CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: frame.origin + frame.x * major,
            x: frame.x,
            y: frame.z,
            z: -frame.y,
        },
        radius: minor,
    }));
    let e_u = brep.add_edge(v, v, Some(cu), (0.0, TAU));
    let e_v = brep.add_edge(v, v, Some(cv), (0.0, TAU));
    history.record(EntityRef::Edge(e_u), Origin::New { role: "major-seam" });
    history.record(EntityRef::Edge(e_v), Origin::New { role: "minor-seam" });
    let surf = brep.add_surface(SurfaceGeom::Torus(Torus {
        frame: *frame,
        major_radius: major,
        minor_radius: minor,
    }));
    let face = brep.add_face(shell, surf, Sense::Forward);
    note_face(&mut history, face, "torus");
    let pc_bottom = line2(brep, (0.0, 0.0), (1.0, 0.0));
    let pc_right = line2(brep, (TAU, 0.0), (0.0, 1.0));
    let pc_top = line2(brep, (0.0, TAU), (1.0, 0.0));
    let pc_left = line2(brep, (0.0, 0.0), (0.0, 1.0));
    brep.add_loop(
        face,
        LoopKind::Outer,
        &[
            CoedgeSpec {
                edge: e_u,
                sense: Sense::Forward,
                pcurve: Some(pc_bottom),
            },
            CoedgeSpec {
                edge: e_v,
                sense: Sense::Forward,
                pcurve: Some(pc_right),
            },
            CoedgeSpec {
                edge: e_u,
                sense: Sense::Reversed,
                pcurve: Some(pc_top),
            },
            CoedgeSpec {
                edge: e_v,
                sense: Sense::Reversed,
                pcurve: Some(pc_left),
            },
        ],
    );
    BuildResult { body, history }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stats(b: &BRep) -> (usize, usize, usize) {
        let e = b.edges.values().filter(|e| !e.is_degenerate()).count();
        (b.vertices.len(), e, b.faces.len())
    }

    #[test]
    #[allow(clippy::type_complexity)]
    fn all_primitives_validate_with_expected_counts() {
        let f =
            Frame3::from_origin_normal(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.3, -0.2, 1.0));
        let cases: Vec<(&str, BRep, (usize, usize, usize), i64)> = {
            let mut v = Vec::new();
            let mut b = BRep::new();
            make_box(&mut b, &f, 2.0, 3.0, 4.0);
            v.push(("box", b, (8, 12, 6), 0));
            let mut b = BRep::new();
            make_cylinder(&mut b, &f, 1.5, 4.0);
            v.push(("cylinder", b, (2, 3, 3), 0));
            let mut b = BRep::new();
            make_cone(&mut b, &f, 2.0, 1.0, 3.0);
            v.push(("frustum", b, (2, 3, 3), 0));
            let mut b = BRep::new();
            make_cone(&mut b, &f, 2.0, 0.0, 3.0);
            v.push(("cone", b, (2, 2, 2), 0));
            let mut b = BRep::new();
            make_sphere(&mut b, &f, 2.0);
            v.push(("sphere", b, (2, 1, 1), 0));
            let mut b = BRep::new();
            make_torus(&mut b, &f, 5.0, 1.0);
            v.push(("torus", b, (1, 2, 1), 1));
            v
        };
        for (name, b, counts, genus) in cases {
            assert_eq!(stats(&b), counts, "{name}");
            let rep = b.validate();
            assert!(rep.is_ok(), "{name}: {:?}", rep.errors);
            assert_eq!(rep.shell_genus.len(), 1, "{name}");
            assert_eq!(rep.shell_genus[0].1, genus, "{name}");
        }
    }

    #[test]
    fn box_edges_have_two_opposite_coedges() {
        let mut b = BRep::new();
        make_box(&mut b, &Frame3::world(), 1.0, 1.0, 1.0);
        for e in b.edges.keys() {
            let ring = b.radial_ring(e);
            assert_eq!(ring.len(), 2);
            assert_ne!(b.coedges[ring[0]].sense, b.coedges[ring[1]].sense);
            assert_eq!(b.faces_around_edge(e).len(), 2);
        }
    }

    #[test]
    fn history_records_face_roles() {
        let mut b = BRep::new();
        let r = make_cylinder(&mut b, &Frame3::world(), 1.0, 2.0);
        let faces = b.body_faces(r.body);
        assert_eq!(faces.len(), 3);
        for f in faces {
            assert!(matches!(
                r.history.origins.get(&EntityRef::Face(f)),
                Some(Origin::New { .. })
            ));
        }
    }
}
