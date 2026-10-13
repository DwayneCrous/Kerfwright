#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_step::*;
use proptest::prelude::*;
use std::f64::consts::{PI, TAU};

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

fn counts(rep: &BRep, b: BodyId) -> (usize, usize, usize) {
    let faces = rep.body_faces(b);
    let mut edges = std::collections::HashSet::new();
    let mut verts = std::collections::HashSet::new();
    for f in &faces {
        for lp in &rep.faces[*f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if edges.insert(e) && rep.edges[e].curve.is_some() {
                    verts.insert(rep.edges[e].start);
                    verts.insert(rep.edges[e].end);
                }
            }
        }
    }
    (
        verts.len(),
        edges
            .iter()
            .filter(|e| rep.edges[**e].curve.is_some())
            .count(),
        faces.len(),
    )
}

/// Export `body`, import it again and compare volume, area, centroid and (non-degenerate) topology counts.
fn roundtrip(rep: &BRep, body: BodyId, what: &str) {
    let text = export(rep, &[body], &ExportOptions::default());
    let mut back = BRep::new();
    let report = import(
        &mut back,
        &text,
        &ImportOptions {
            to_millimetres: false,
            ..Default::default()
        },
    )
    .unwrap_or_else(|e| {
        panic!(
            "{what}: import failed: {e}\n{}",
            &text[..text.len().min(3000)]
        )
    });
    assert!(report.failed.is_empty(), "{what}: {:?}", report.failed);
    assert_eq!(report.bodies.len(), 1, "{what}");
    let b2 = report.bodies[0];
    let (m1, m2) = (
        rep.mass_properties(body).unwrap(),
        back.mass_properties(b2).unwrap(),
    );
    assert!(
        rel(m2.volume, m1.volume) < 1e-9,
        "{what}: volume {} vs {}",
        m2.volume,
        m1.volume
    );
    assert!(
        rel(m2.area, m1.area) < 1e-9,
        "{what}: area {} vs {}",
        m2.area,
        m1.area
    );
    assert!(
        (m2.centroid - m1.centroid).norm() < 1e-7 * (1.0 + m1.centroid.coords.amax()),
        "{what}: centroid"
    );
    assert_eq!(
        counts(rep, body),
        counts(&back, b2),
        "{what}: topology counts (V,E,F)"
    );
    assert!(back.validate().is_ok(), "{what}: {:?}", back.validate());
}

#[test]
fn primitives_round_trip() {
    let f = Frame3::from_origin_normal(Point3::new(1.0, -2.0, 0.5), Vector3::new(0.3, 0.2, 1.0));
    let mut rep = BRep::new();
    let ids = [
        ("box", make_box(&mut rep, &f, 2.0, 3.0, 4.0).body),
        ("cylinder", make_cylinder(&mut rep, &f, 1.5, 2.0).body),
        ("frustum", make_cone(&mut rep, &f, 2.0, 1.0, 3.0).body),
        ("cone", make_cone(&mut rep, &f, 2.0, 0.0, 3.0).body),
        ("sphere", make_sphere(&mut rep, &f, 1.7).body),
        ("torus", make_torus(&mut rep, &f, 3.0, 0.8).body),
    ];
    for (n, b) in ids {
        roundtrip(&rep, b, n);
    }
}

#[test]
fn modelled_and_boolean_bodies_round_trip() {
    // extruded plate with a hole, a revolved bottle, a boolean (box with a cross hole) and a spline prism
    let mut rep = BRep::new();
    let hole_frame = Frame3 {
        origin: Point3::new(0.3, 0.0, 0.0),
        x: Vector3::x(),
        y: -Vector3::y(),
        z: -Vector3::z(),
    };
    let plate =
        Profile::rectangle(world(), 4.0, 3.0).with_hole(ProfileLoop::circle(&hole_frame, 0.4));
    let p = extrude(
        &mut rep,
        &plate,
        Vector3::new(0.0, 0.0, 0.5),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    roundtrip(&rep, p, "plate with hole");

    let fxz = Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::z(),
        z: -Vector3::y(),
    };
    let shoulder = Frame3 {
        origin: Point3::new(0.9, 0.0, 1.5),
        ..fxz
    };
    let edges = vec![
        ProfileEdge::line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.2, 0.0, 0.0)),
        ProfileEdge::line(Point3::new(1.2, 0.0, 0.0), Point3::new(1.2, 0.0, 1.5)),
        ProfileEdge::arc(&shoulder, 0.3, 0.0, PI / 2.0),
        ProfileEdge::line(Point3::new(0.9, 0.0, 1.8), Point3::new(0.4, 0.0, 1.8)),
        ProfileEdge::line(Point3::new(0.4, 0.0, 1.8), Point3::new(0.4, 0.0, 2.6)),
        ProfileEdge::line(Point3::new(0.4, 0.0, 2.6), Point3::new(0.0, 0.0, 2.6)),
        ProfileEdge::line(Point3::new(0.0, 0.0, 2.6), Point3::new(0.0, 0.0, 0.0)),
    ];
    let bottle = revolve(
        &mut rep,
        &Profile::new(fxz, ProfileLoop::new(edges)),
        Point3::origin(),
        Vector3::z(),
        TAU,
    )
    .unwrap()
    .body;
    roundtrip(&rep, bottle, "bottle");

    let bx = make_box(&mut rep, &world(), 2.0, 2.0, 2.0).body;
    let cyl = make_cylinder(
        &mut rep,
        &Frame3 {
            origin: Point3::new(0.3, 0.2, -2.0),
            ..world()
        },
        0.5,
        4.0,
    )
    .body;
    let holed = boolean(&mut rep, bx, cyl, Op::Subtract, &BoolOptions::default())
        .unwrap()
        .body;
    roundtrip(&rep, holed, "box with hole");
    // hollow: sphere inside a box
    let big = make_box(&mut rep, &world(), 4.0, 4.0, 4.0).body;
    let ball = make_sphere(&mut rep, &world(), 1.0).body;
    let hollow = boolean(&mut rep, big, ball, Op::Subtract, &BoolOptions::default())
        .unwrap()
        .body;
    roundtrip(&rep, hollow, "hollow box");
}

#[test]
fn poles_seams_and_rational_surfaces_round_trip() {
    use kw_boolean::{boolean, BoolOptions, Op};
    // a sphere whose face is bounded by two pole runs and a window straddling the seam
    let mut rep = BRep::new();
    let ball = make_sphere(&mut rep, &world(), 1.9).body;
    let bx = make_box(
        &mut rep,
        &Frame3 {
            origin: Point3::new(1.5, -0.5, -0.5),
            ..world()
        },
        1.0,
        1.0,
        1.0,
    )
    .body;
    let u = boolean(&mut rep, ball, bx, Op::Union, &BoolOptions::default())
        .unwrap()
        .body;
    roundtrip(&rep, u, "sphere union box across the seam");

    // rational B-spline surfaces are written as complex instances
    let mut rep = BRep::new();
    let pf = Frame3 {
        origin: Point3::new(4.0, 0.0, 0.0),
        x: Vector3::z(),
        y: Vector3::x(),
        z: Vector3::y(),
    };
    let arc = kw_nurbs::build::make_arc(&Frame3::world(), 4.0, 0.0, PI / 2.0).unwrap();
    let bend = sweep(
        &mut rep,
        &Profile::disc(pf, 0.5),
        &kw_nurbs::CurveGeom::Nurbs(arc),
        &SweepOptions::default(),
    )
    .unwrap()
    .body;
    roundtrip(&rep, bend, "swept bend");
}

const STEP_HEAD: &str = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('t','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n";

#[test]
fn a_whole_sphere_bounded_by_a_vertex_loop_imports() {
    let src = format!(
        "{STEP_HEAD}#1=CARTESIAN_POINT('',(0.,0.,0.));\n#2=DIRECTION('',(0.,0.,1.));\n#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n#5=SPHERICAL_SURFACE('',#4,2.);\n#6=CARTESIAN_POINT('',(2.,0.,0.));\n\
#7=VERTEX_POINT('',#6);\n#8=VERTEX_LOOP('',#7);\n#9=FACE_BOUND('',#8,.T.);\n#10=ADVANCED_FACE('',(#9),#5,.T.);\n\
#11=CLOSED_SHELL('',(#10));\n#12=MANIFOLD_SOLID_BREP('',#11);\nENDSEC;\nEND-ISO-10303-21;\n"
    );
    let mut rep = BRep::new();
    let r = import(&mut rep, &src, &ImportOptions::default()).unwrap();
    assert_eq!(r.bodies.len(), 1);
    let v = rep.volume(r.bodies[0]).unwrap();
    assert!(rel(v, 4.0 / 3.0 * PI * 8.0) < 1e-9, "{v}");
}

#[test]
fn the_widest_bound_is_the_outer_one_when_none_is_marked() {
    // a plate with a hole, rewritten the way some exporters write it: plain FACE_BOUNDs, hole first
    let mut rep = BRep::new();
    let hole_frame = Frame3 {
        origin: Point3::new(0.3, 0.0, 0.0),
        x: Vector3::x(),
        y: -Vector3::y(),
        z: -Vector3::z(),
    };
    let plate =
        Profile::rectangle(world(), 4.0, 3.0).with_hole(ProfileLoop::circle(&hole_frame, 0.4));
    let body = extrude(
        &mut rep,
        &plate,
        Vector3::new(0.0, 0.0, 0.5),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    let text = export(&rep, &[body], &ExportOptions::default());
    let mut out = String::new();
    let mut reordered = 0;
    for line in text.lines() {
        let line = line.replace("FACE_OUTER_BOUND(", "FACE_BOUND(");
        if let Some(i) = line.find("ADVANCED_FACE('',(") {
            let start = i + "ADVANCED_FACE('',(".len();
            let end = start + line[start..].find(')').unwrap();
            let mut ids: Vec<&str> = line[start..end].split(',').collect();
            if ids.len() == 2 {
                ids.reverse();
                reordered += 1;
                out.push_str(&format!(
                    "{}{}{}\n",
                    &line[..start],
                    ids.join(","),
                    &line[end..]
                ));
                continue;
            }
        }
        out.push_str(&line);
        out.push('\n');
    }
    assert!(reordered > 0);
    let mut back = BRep::new();
    let r = import(&mut back, &out, &ImportOptions::default()).unwrap();
    assert_eq!(r.bodies.len(), 1);
    let (v0, v1) = (rep.volume(body).unwrap(), back.volume(r.bodies[0]).unwrap());
    assert!(rel(v0, v1) < 1e-9, "{v0} vs {v1}");
}

#[test]
fn units_are_converted_to_millimetres() {
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &world(), 2.0, 3.0, 4.0).body;
    let text = export(&rep, &[b], &ExportOptions::default())
        .replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT($,.METRE.)");
    let mut back = BRep::new();
    let r = import(&mut back, &text, &ImportOptions::default()).unwrap();
    assert_eq!(r.unit_scale, 1000.0);
    assert!(rel(back.volume(r.bodies[0]).unwrap(), 24.0 * 1e9) < 1e-9);
}

#[test]
fn parser_handles_the_grammar() {
    let src = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('a'),'2;1');\nENDSEC;\nDATA;\n/* c */ #1 = CARTESIAN_POINT('it''s',(1.,-2.5E1,3));\n#2 = (A(1) B('x',.T.,$,*,#1));\n#3=LINE('',#1,#4);\nENDSEC;\nEND-ISO-10303-21;\n";
    let f = parse(src).unwrap();
    assert_eq!(f.instances.len(), 3);
    let i = &f.instances[&1];
    assert_eq!(i.params()[0], Param::Str("it's".into()));
    assert_eq!(
        i.params()[1],
        Param::List(vec![Param::Real(1.0), Param::Real(-25.0), Param::Int(3)])
    );
    assert_eq!(f.instances[&2].parts.len(), 2);
    assert!(parse("garbage").is_err());
    assert!(parse("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = X(\n").is_err());
}

#[test]
fn no_solids_and_unsupported_input_are_errors() {
    let src = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = CARTESIAN_POINT('',(0.,0.,0.));\nENDSEC;\nEND-ISO-10303-21;\n";
    let mut rep = BRep::new();
    assert_eq!(
        import(&mut rep, src, &ImportOptions::default()).unwrap_err(),
        StepError::NoSolids
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(std::env::var("PROPTEST_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(256)))]

    /// Arbitrary text never panics the parser.
    #[test]
    fn arbitrary_text_never_panics(s in ".{0,300}") {
        let _ = parse(&s);
    }

    /// Corrupting a valid file (byte edits, truncation) never panics the parser or the importer.
    #[test]
    fn mutated_files_never_panic(
        edits in prop::collection::vec((any::<prop::sample::Index>(), any::<u8>()), 1..16),
        cut in any::<prop::sample::Index>(),
    ) {
        let mut rep = BRep::new();
        let b = make_cylinder(&mut rep, &world(), 1.0, 2.0).body;
        let mut bytes = export(&rep, &[b], &ExportOptions::default()).into_bytes();
        for (i, v) in &edits {
            let k = i.index(bytes.len());
            bytes[k] = *v;
        }
        let keep = cut.index(bytes.len() + 1);
        bytes.truncate(keep);
        let text = String::from_utf8_lossy(&bytes).into_owned();
        let mut out = BRep::new();
        let _ = import(&mut out, &text, &ImportOptions::default());
    }
}
