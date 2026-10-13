use kw_brep::builders::{make_box, make_plane_sheet};
use kw_brep::{BRep, BodyKind, CopyOptions};
use kw_math::{Frame3, Point3};

#[test]
fn a_plane_sheet_is_a_valid_body_with_area_free_edges_and_no_volume() {
    let mut rep = BRep::new();
    let s = make_plane_sheet(&mut rep, &Frame3::world(), 6.0, 4.0).body;
    assert_eq!(rep.bodies[s].kind, BodyKind::Sheet);
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    assert!((rep.area(s).unwrap() - 24.0).abs() < 1e-9);
    // it encloses nothing
    assert_eq!(rep.volume(s), None);
    assert!(rep.mass_properties(s).is_none());
    assert_eq!(rep.free_edges(s).len(), 4);
    assert!(!rep.is_closed(s));
    assert_eq!(rep.body_faces(s).len(), 1);
}

#[test]
fn a_solid_has_no_free_edges_and_keeps_its_volume() {
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &Frame3::world(), 2.0, 3.0, 4.0).body;
    assert!(rep.free_edges(b).is_empty());
    assert!(rep.is_closed(b));
    assert!((rep.volume(b).unwrap() - 24.0).abs() < 1e-9);
}

#[test]
fn a_sheet_beside_a_solid_does_not_loosen_the_checks_of_the_solid() {
    let mut rep = BRep::new();
    make_box(&mut rep, &Frame3::world(), 2.0, 2.0, 2.0);
    let frame = Frame3 {
        origin: Point3::new(10.0, 0.0, 0.0),
        ..Frame3::world()
    };
    let sheet = make_plane_sheet(&mut rep, &frame, 3.0, 3.0).body;
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    // a second solid that is really an open sheet is caught, even though other bodies are fine
    let open = make_plane_sheet(
        &mut rep,
        &Frame3 {
            origin: Point3::new(0.0, 10.0, 0.0),
            ..Frame3::world()
        },
        1.0,
        1.0,
    )
    .body;
    rep.bodies[open].kind = BodyKind::Solid;
    assert!(!rep.validate().is_ok(), "an open shell cannot be a solid");
    rep.bodies[open].kind = BodyKind::Sheet;
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    let _ = sheet;
}

#[test]
fn copying_a_sheet_keeps_it_a_sheet() {
    let mut rep = BRep::new();
    let s = make_plane_sheet(&mut rep, &Frame3::world(), 6.0, 4.0).body;
    let src = rep.clone();
    let (copy, _map) = rep.copy_body_from(&src, s, &CopyOptions::default());
    assert_eq!(rep.bodies[copy].kind, BodyKind::Sheet);
    assert_eq!(rep.free_edges(copy).len(), 4);
}
