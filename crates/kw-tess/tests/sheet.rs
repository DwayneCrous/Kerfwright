use kw_brep::builders::{make_box, make_plane_sheet};
use kw_brep::BRep;
use kw_math::{Frame3, Vector3};

#[test]
fn a_sheet_tessellates_and_shows_its_boundary_as_unmatched_mesh_edges() {
    let mut rep = BRep::new();
    let s = make_plane_sheet(&mut rep, &Frame3::world(), 6.0, 4.0).body;
    let t = kw_tess::tessellate_body(&rep, s, &kw_tess::TessOptions::default()).unwrap();
    assert!(t.mesh.triangle_count() >= 2);
    assert!(
        t.mesh
            .normals
            .iter()
            .all(|n| (n - Vector3::z()).norm() < 1e-9),
        "faces +z"
    );
    assert!(t.mesh.first_unmatched_edge().is_some(), "the free boundary");
    assert!((t.mesh.surface_area() - 24.0).abs() < 1e-9);
    // a closed solid has none
    let b = make_box(&mut rep, &Frame3::world(), 2.0, 2.0, 2.0).body;
    let tb = kw_tess::tessellate_body(&rep, b, &kw_tess::TessOptions::default()).unwrap();
    assert!(tb.mesh.first_unmatched_edge().is_none());
}
