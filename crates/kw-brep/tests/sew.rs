//! Sewing faces into sheets and solids.
use kw_brep::builders::make_plane_sheet;
use kw_brep::sew::{BuildOptions, Soup};
use kw_brep::{make_box, make_cylinder, make_sphere, BRep, BodyId, BodyKind, EntityRef, Origin};
use kw_math::{Frame3, Point3, Vector3};

fn opts() -> BuildOptions {
    BuildOptions {
        vertex_tol: 1e-6,
        one_body: false,
    }
}

/// The six faces of a cube of side 2 centred on the origin, as separate sheets; faces with an odd
/// index are made to face inwards so that the sewing has to turn them.
fn cube_sheets(rep: &mut BRep) -> Vec<BodyId> {
    let mut bodies = Vec::new();
    let dirs = [
        (Vector3::x(), Vector3::y()),
        (Vector3::y(), Vector3::z()),
        (Vector3::z(), Vector3::x()),
    ];
    let mut k = 0;
    for (a, b) in dirs {
        let n = a.cross(&b);
        for s in [1.0, -1.0] {
            let inward = k % 2 == 1;
            k += 1;
            let nn = if inward { -n * s } else { n * s };
            let frame = Frame3 {
                origin: Point3::from(n * s),
                x: a,
                y: nn.cross(&a),
                z: nn,
            };
            bodies.push(make_plane_sheet(rep, &frame, 2.0, 2.0).body);
        }
    }
    bodies
}

fn sew(rep: &mut BRep, bodies: &[BodyId], tol: f64) -> kw_brep::sew::SewResult {
    let mut soup = Soup::default();
    for b in bodies {
        soup.add_body(rep, *b, true);
    }
    soup.weld(tol);
    soup.build(rep, &opts())
}

#[test]
fn six_planar_sheets_sew_into_a_cube() {
    let mut rep = BRep::new();
    let sheets = cube_sheets(&mut rep);
    let r = sew(&mut rep, &sheets, 1e-6);
    assert!(r.rejected.is_empty(), "{:?}", r.rejected);
    assert_eq!(r.bodies.len(), 1);
    let b = r.bodies[0];
    assert_eq!(rep.bodies[b].kind, BodyKind::Solid);
    assert!((rep.volume(b).unwrap() - 8.0).abs() < 1e-9);
    assert!((rep.area(b).unwrap() - 24.0).abs() < 1e-9);
    assert!(rep.free_edges(b).is_empty());
    // all twelve edges are shared by two faces
    assert_eq!(rep.body_faces(b).len(), 6);
    // every new face remembers the face it came from
    let preserved = r
        .history
        .origins
        .values()
        .filter(|o| matches!(o, Origin::Preserved(EntityRef::Face(_))))
        .count();
    assert_eq!(preserved, 6);
}

#[test]
fn five_sheets_stay_a_sheet_and_two_far_apart_make_two_groups() {
    let mut rep = BRep::new();
    let sheets = cube_sheets(&mut rep);
    let r = sew(&mut rep, &sheets[..5], 1e-6);
    assert_eq!(r.bodies.len(), 1);
    let b = r.bodies[0];
    assert_eq!(rep.bodies[b].kind, BodyKind::Sheet);
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    // the open top has four free edges
    assert_eq!(rep.free_edges(b).len(), 4);
    assert!((rep.area(b).unwrap() - 20.0).abs() < 1e-9);
    // opposite faces never touch
    let r = sew(&mut rep, &[sheets[0], sheets[1]], 1e-6);
    assert_eq!(r.bodies.len(), 2);
}

#[test]
fn a_gap_wider_than_the_tolerance_is_not_closed() {
    let mut rep = BRep::new();
    let sheets = cube_sheets(&mut rep);
    // the same cube, but one face is 1 mm too small to reach its neighbours
    let mut soup = Soup::default();
    for b in &sheets {
        soup.add_body(&rep, *b, true);
    }
    let r = {
        let before = soup.weld(1e-9);
        assert!(before.0 >= 12, "exact edges weld: {}", before.0);
        soup.build(&mut rep, &opts())
    };
    assert_eq!(r.bodies.len(), 1);
    // now a cube whose faces are shifted by 0.01: a tolerance of 1e-6 leaves it open, 0.1 sews it
    let mut rep2 = BRep::new();
    let mut loose = cube_sheets(&mut rep2);
    let frame = Frame3 {
        origin: Point3::new(0.0, 0.0, 1.01),
        ..Frame3::world()
    };
    loose[4] = make_plane_sheet(&mut rep2, &frame, 2.0, 2.0).body;
    let r = sew(&mut rep2, &loose, 1e-6);
    assert!(r
        .bodies
        .iter()
        .all(|b| rep2.bodies[*b].kind == BodyKind::Sheet));
    let r = sew(&mut rep2, &loose, 0.1);
    assert_eq!(r.bodies.len(), 1, "{:?}", r.rejected);
}

/// Take a solid apart into faces with edges of their own and sew it again.
fn round_trip(rep: &mut BRep, body: BodyId) -> BodyId {
    let mut soup = Soup::default();
    soup.add_body(rep, body, false);
    soup.weld(1e-7);
    let r = soup.build(rep, &opts());
    assert!(r.rejected.is_empty(), "{:?}", r.rejected);
    assert_eq!(r.bodies.len(), 1);
    r.bodies[0]
}

#[test]
fn solids_come_back_whole_from_their_faces() {
    let frame = Frame3::world();
    for (name, build) in [
        (
            "box",
            Box::new(|r: &mut BRep| make_box(r, &frame, 3.0, 2.0, 1.0).body)
                as Box<dyn Fn(&mut BRep) -> BodyId>,
        ),
        (
            "cylinder",
            Box::new(|r: &mut BRep| make_cylinder(r, &frame, 1.5, 4.0).body),
        ),
        (
            "sphere",
            Box::new(|r: &mut BRep| make_sphere(r, &frame, 2.0).body),
        ),
    ] {
        let mut rep = BRep::new();
        let b = build(&mut rep);
        let (v, a) = (rep.volume(b).unwrap(), rep.area(b).unwrap());
        let c = round_trip(&mut rep, b);
        assert_eq!(rep.bodies[c].kind, BodyKind::Solid, "{name}");
        assert!(
            rep.validate().is_ok(),
            "{name}: {:?}",
            rep.validate().errors
        );
        assert!((rep.volume(c).unwrap() - v).abs() < 1e-6 * v, "{name}");
        assert!((rep.area(c).unwrap() - a).abs() < 1e-6 * a, "{name}");
    }
}

#[test]
fn unstitching_gives_a_sheet_of_loose_faces() {
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &Frame3::world(), 2.0, 2.0, 2.0).body;
    let mut soup = Soup::default();
    soup.add_body(&rep, b, false);
    let r = soup.build(
        &mut rep,
        &BuildOptions {
            vertex_tol: 1e-6,
            one_body: true,
        },
    );
    assert_eq!(r.bodies.len(), 1, "{:?}", r.rejected);
    let s = r.bodies[0];
    assert_eq!(rep.bodies[s].kind, BodyKind::Sheet);
    assert_eq!(rep.body_faces(s).len(), 6);
    assert_eq!(rep.free_edges(s).len(), 24);
    assert!((rep.area(s).unwrap() - 24.0).abs() < 1e-9);
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
}
