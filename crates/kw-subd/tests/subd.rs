use kw_subd::*;

fn run(name: &str, mesh: &SubdMesh, tol: f64) -> Finished {
    let t = std::time::Instant::now();
    let f = finish(
        mesh,
        &FinishOptions {
            tolerance: tol,
            ..Default::default()
        },
    )
    .unwrap_or_else(|e| panic!("{name}: {e}"));
    let v = f.rep.validate();
    eprintln!(
        "{name}: level {} patches {} deviation {:.2e} mm; G1 max {:.3}° mean {:.4}° over {} smooth edges ({} creases); {:.0} ms; valid {}",
        f.level, f.patches, f.deviation, f.g1.max_angle_deg, f.g1.mean_angle_deg, f.g1.edges_checked, f.g1.crease_edges,
        t.elapsed().as_secs_f64() * 1e3, v.is_ok()
    );
    assert!(v.is_ok(), "{name}: {:?}", v);
    f
}

#[test]
fn shapes_finish_within_tolerance() {
    let shapes: Vec<(&str, SubdMesh)> = vec![
        ("cube 20", SubdMesh::cube(20.0)),
        ("box 30x20x10", SubdMesh::box_(30.0, 20.0, 10.0)),
        ("torus 20/6", SubdMesh::torus(20.0, 6.0, 8, 6)),
        (
            "pentagonal prism (valence 3, pentagon faces)",
            SubdMesh::prism(5, 10.0, 15.0),
        ),
        (
            "hexagonal prism (valence 3, hexagon faces)",
            SubdMesh::prism(6, 10.0, 15.0),
        ),
        (
            "hexagonal bipyramid (valence 6 apexes)",
            SubdMesh::bipyramid(6, 10.0, 12.0),
        ),
        (
            "pentagonal bipyramid (valence 5 apexes)",
            SubdMesh::bipyramid(5, 10.0, 12.0),
        ),
        (
            "triangular bipyramid (valence 3)",
            SubdMesh::bipyramid(3, 10.0, 12.0),
        ),
    ];
    for (name, m) in shapes {
        let f = run(name, &m, 0.01);
        assert!(f.deviation <= 0.01);
        // tangent planes of neighbouring patches agree: exactly where the surface is regular, and to a
        // fraction of a degree on average with a few degrees at the worst extraordinary vertex
        assert!(
            f.g1.edges_checked > 0 && f.g1.mean_angle_deg < 0.2 && f.g1.max_angle_deg < 4.0,
            "{:?}",
            f.g1
        );
    }
}

#[test]
fn creased_shapes_keep_their_sharp_edges() {
    // a cube with all edges sharp stays a cube
    let sharp = SubdMesh::cube(20.0).crease_all(100.0);
    let f = run("fully creased cube", &sharp, 0.01);
    let v = f.rep.volume(f.body).unwrap();
    assert!((v - 8000.0).abs() < 1e-3, "{v}");
    assert_eq!(f.g1.crease_edges, 24);
    // two creased edges: a partly sharp box
    let part = SubdMesh::cube(20.0)
        .with_crease(0, 1, 100.0)
        .with_crease(1, 2, 100.0);
    run("two creased edges", &part, 0.01);
    // semi-sharp: the creases soften after two subdivisions
    let semi = SubdMesh::cube(20.0).crease_all(2.0);
    let f = run("semi-sharp cube", &semi, 0.01);
    assert!(f.deviation <= 0.01);
}

#[test]
fn invalid_cages_are_explained() {
    let mut open = SubdMesh::cube(1.0);
    open.faces.pop();
    assert!(matches!(
        finish(&open, &FinishOptions::default()),
        Err(SubdError::NotClosed)
    ));
    let mut bad = SubdMesh::cube(1.0);
    bad.faces[0] = vec![0, 1, 99, 2];
    assert!(matches!(bad.validate(), Err(SubdError::Invalid(m)) if m.contains("missing vertex")));
    let mut twice = SubdMesh::cube(1.0);
    twice.faces[1] = twice.faces[0].clone();
    assert!(twice.validate().is_err());
    // a cage given inside out is flipped, not rejected
    let inside_out = SubdMesh::cube(10.0).flipped();
    let f = finish(&inside_out, &FinishOptions::default()).unwrap();
    assert!(f.rep.volume(f.body).unwrap() > 0.0);
}

#[test]
fn regular_surfaces_are_reproduced_exactly_and_are_g1() {
    // a torus cage has only valence-4 vertices: the limit surface is a uniform bicubic B-spline
    let f = run("torus", &SubdMesh::torus(20.0, 6.0, 12, 8), 0.01);
    assert!(f.deviation < 1e-9, "{}", f.deviation);
    assert!(f.g1.max_angle_deg < 1e-4, "{:?}", f.g1);
    // the solid it encloses has the volume of the limit surface (≈ the smooth torus; the cage is coarse)
    let v = f.rep.volume(f.body).unwrap();
    assert!(v > 0.0 && v < 2.0 * std::f64::consts::PI.powi(2) * 20.0 * 36.0);
}
