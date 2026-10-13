use kw_analysis::ramp;
use kw_analysis::zebra::{reflect, stripe, zebra};
use kw_analysis::{curvature_at, draft_degrees, mesh_curvature, mesh_draft, CurvKind};
use kw_brep::builders::{make_box, make_cylinder, make_sphere, make_torus};
use kw_brep::BRep;
use kw_math::{Frame3, Point3};
use kw_nurbs::{KnotVector, NurbsSurface, SurfaceGeom};
use kw_tess::{tessellate_body, TessOptions};

fn tess(rep: &BRep, b: kw_brep::BodyId) -> kw_tess::Tessellation {
    tessellate_body(
        rep,
        b,
        &TessOptions {
            chord_tol: 0.01,
            angle_tol: 0.15,
        },
    )
    .unwrap()
}

fn finite(v: Vec<f32>) -> Vec<f32> {
    v.into_iter().filter(|x| x.is_finite()).collect()
}

#[test]
fn a_sphere_has_gaussian_curvature_one_over_r_squared_and_mean_one_over_r() {
    let mut rep = BRep::new();
    let b = make_sphere(&mut rep, &Frame3::world(), 5.0).body;
    let t = tess(&rep, b);
    let k = finite(mesh_curvature(&rep, &t, CurvKind::Gaussian));
    let h = finite(mesh_curvature(&rep, &t, CurvKind::Mean));
    assert!(k.len() > 100);
    assert!(
        k.iter().all(|v| (*v - 0.04).abs() < 1e-4),
        "K: {:?}",
        k.iter()
            .cloned()
            .fold((f32::MAX, f32::MIN), |a, v| (a.0.min(v), a.1.max(v)))
    );
    // convex from outside: positive
    assert!(h.iter().all(|v| (*v - 0.2).abs() < 1e-4));
    // principal curvatures are equal on a sphere
    let (mx, mn) = (
        finite(mesh_curvature(&rep, &t, CurvKind::Max)),
        finite(mesh_curvature(&rep, &t, CurvKind::Min)),
    );
    assert!(mx.iter().zip(&mn).all(|(a, b)| (a - b).abs() < 1e-4));
}

#[test]
fn a_cylinder_is_flat_one_way_and_a_plane_is_flat_both() {
    let mut rep = BRep::new();
    let b = make_cylinder(&mut rep, &Frame3::world(), 4.0, 10.0).body;
    let t = tess(&rep, b);
    let k = finite(mesh_curvature(&rep, &t, CurvKind::Gaussian));
    let h = finite(mesh_curvature(&rep, &t, CurvKind::Mean));
    assert!(k.iter().all(|v| v.abs() < 1e-4));
    // the wall: 1/(2r); the two caps: 0
    assert!(
        h.iter()
            .all(|v| v.abs() < 1e-4 || (*v - 0.125).abs() < 1e-4),
        "{:?}",
        h.iter()
            .cloned()
            .fold((f32::MAX, f32::MIN), |a, v| (a.0.min(v), a.1.max(v)))
    );
    assert!(h.iter().any(|v| (*v - 0.125).abs() < 1e-4));
    let (mx, mn) = (
        finite(mesh_curvature(&rep, &t, CurvKind::Max)),
        finite(mesh_curvature(&rep, &t, CurvKind::Min)),
    );
    assert!(mx.iter().any(|v| (*v - 0.25).abs() < 1e-4) && mn.iter().all(|v| v.abs() < 1e-4));
    // a box is flat everywhere
    let bx = make_box(&mut rep, &Frame3::world(), 2.0, 3.0, 4.0).body;
    let tb = tess(&rep, bx);
    assert!(finite(mesh_curvature(&rep, &tb, CurvKind::Mean))
        .iter()
        .all(|v| v.abs() < 1e-9));
}

#[test]
fn a_hole_is_concave_so_its_curvature_is_negative() {
    // a plate with a drilled hole, made as the app's Hole feature does: subtract a cylinder
    let mut rep = BRep::new();
    let plate = make_box(
        &mut rep,
        &Frame3 {
            origin: Point3::new(0.0, 0.0, 3.0),
            ..Frame3::world()
        },
        20.0,
        20.0,
        6.0,
    )
    .body;
    let drill = make_cylinder(
        &mut rep,
        &Frame3 {
            origin: Point3::new(0.0, 0.0, -1.0),
            ..Frame3::world()
        },
        3.0,
        8.0,
    )
    .body;
    let b = kw_boolean::subtract(&mut rep, plate, drill).unwrap().body;
    let vol = rep.volume(b).unwrap();
    assert!(
        (vol - (2400.0 - std::f64::consts::PI * 9.0 * 6.0)).abs() < 1e-3,
        "volume {vol}"
    );
    let t = tess(&rep, b);
    let h = finite(mesh_curvature(&rep, &t, CurvKind::Mean));
    assert!(
        h.iter().any(|v| (*v + 1.0 / 6.0).abs() < 1e-4),
        "the hole wall: -1/(2 r)"
    );
    assert!(h
        .iter()
        .all(|v| v.abs() < 1e-4 || (*v + 1.0 / 6.0).abs() < 1e-4));
    // and Gaussian curvature of the wall is still zero: orientation does not change it
    assert!(finite(mesh_curvature(&rep, &t, CurvKind::Gaussian))
        .iter()
        .all(|v| v.abs() < 1e-4));
}

#[test]
fn a_torus_matches_its_closed_forms_on_the_outer_equator() {
    let (big, small) = (10.0, 2.0);
    let mut rep = BRep::new();
    let b = make_torus(&mut rep, &Frame3::world(), big, small).body;
    let t = tess(&rep, b);
    // the outermost ring of vertices: radius R + r from the axis, at z = 0
    let (mut k_seen, mut h_seen) = (0, 0);
    let ks = mesh_curvature(&rep, &t, CurvKind::Gaussian);
    let hs = mesh_curvature(&rep, &t, CurvKind::Mean);
    for (i, p) in t.mesh.positions.iter().enumerate() {
        if p.z.abs() < 1e-6 && ((p.x * p.x + p.y * p.y).sqrt() - (big + small)).abs() < 1e-6 {
            assert!(
                (f64::from(ks[i]) - 1.0 / (small * (big + small))).abs() < 1e-5,
                "{}",
                ks[i]
            );
            assert!(
                (f64::from(hs[i]) - 0.5 * (1.0 / small + 1.0 / (big + small))).abs() < 1e-5,
                "{}",
                hs[i]
            );
            k_seen += 1;
            h_seen += 1;
        }
    }
    assert!(k_seen > 8 && h_seen > 8);
    // the inner equator is a saddle: negative Gaussian curvature
    let mut saddle = 0;
    for (i, p) in t.mesh.positions.iter().enumerate() {
        if p.z.abs() < 1e-6 && ((p.x * p.x + p.y * p.y).sqrt() - (big - small)).abs() < 1e-6 {
            assert!(f64::from(ks[i]) < 0.0);
            assert!((f64::from(ks[i]) + 1.0 / (small * (big - small))).abs() < 1e-5);
            saddle += 1;
        }
    }
    assert!(saddle > 8);
}

#[test]
fn a_nurbs_paraboloid_has_exact_curvature_and_the_sign_of_its_normal() {
    // z = (x^2 + y^2) / 2 over [-1, 1]^2 as a biquadratic Bezier patch: curvature -1 at the apex
    // with the normal Su x Sv pointing up into the bowl
    let ku = KnotVector::new(2, vec![0., 0., 0., 1., 1., 1.]).unwrap();
    let c = [0.5, -0.5, 0.5];
    let mut pts = Vec::new();
    for i in 0..3 {
        for j in 0..3 {
            pts.push(Point3::new(i as f64 - 1.0, j as f64 - 1.0, c[i] + c[j]));
        }
    }
    let s = NurbsSurface::new(ku.clone(), ku, pts, vec![1.0; 9]).unwrap();
    let g = SurfaceGeom::Nurbs(s);
    let apex = curvature_at(&g, 0.5, 0.5).unwrap();
    assert!(
        (apex.k1 + 1.0).abs() < 1e-9 && (apex.k2 + 1.0).abs() < 1e-9,
        "{apex:?}"
    );
    // away from the apex the surface z = r^2/2 has K = 1/(1 + r^2)^2
    let (u, v) = (0.75, 0.5); // x = 0.5, y = 0
    let c2 = curvature_at(&g, u, v).unwrap();
    let r2 = 0.25;
    assert!(
        (c2.gaussian() - 1.0 / (1.0 + r2) / (1.0 + r2)).abs() < 1e-9,
        "{}",
        c2.gaussian()
    );
    assert!((c2.flipped().mean() + c2.mean()).abs() < 1e-12);
}

#[test]
fn draft_angle_is_zero_for_walls_and_ninety_for_floors() {
    let pull = [0.0, 0.0, 1.0];
    assert!((draft_degrees([1.0, 0.0, 0.0], pull)).abs() < 1e-12);
    assert!((draft_degrees([0.0, 0.0, 1.0], pull) - 90.0).abs() < 1e-12);
    assert!((draft_degrees([0.0, 0.0, -1.0], pull) + 90.0).abs() < 1e-12);
    // a wall leaning 5 degrees toward the pull
    let a = 5.0_f64.to_radians();
    assert!((draft_degrees([a.cos(), 0.0, a.sin()], pull) - 5.0).abs() < 1e-9);
    // the pull need not be a unit vector
    assert!((draft_degrees([0.0, 0.0, 1.0], [0.0, 0.0, 7.0]) - 90.0).abs() < 1e-12);
    // a box meshed: its mesh has exactly the values 0 and +-90
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &Frame3::world(), 2.0, 3.0, 4.0).body;
    let d = mesh_draft(&tess(&rep, b), pull);
    assert!(d
        .iter()
        .all(|v| v.abs() < 1e-3 || (v.abs() - 90.0).abs() < 1e-3));
    assert!(
        d.iter().any(|v| (*v - 90.0).abs() < 1e-3) && d.iter().any(|v| (*v + 90.0).abs() < 1e-3)
    );
}

#[test]
fn zebra_stripes_are_constant_on_a_plane_and_alternate_on_a_sphere() {
    let view = [0.0, 0.0, -1.0];
    // a flat face reflects every ray the same way: one colour
    let flat = [zebra(view, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 3.0, 0.5); 4];
    assert!(flat.iter().all(|v| *v == flat[0]));
    // a sphere seen from the front: normals sweep through all directions, the stripes alternate
    let mut last = -1.0;
    let mut changes = 0;
    for i in 0..400 {
        let x = -0.99 + 1.98 * i as f32 / 399.0;
        let n = [x, 0.0, (1.0 - x * x).sqrt()];
        let z = zebra(view, n, [1.0, 0.0, 0.0], 3.0, 0.5);
        if z != last {
            changes += 1;
            last = z;
        }
    }
    assert!(changes >= 6, "{changes}");
    // the stripe function itself: period 1/freq, duty fraction white
    assert_eq!(stripe([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 2.0, 0.5), 1.0);
    assert_eq!(stripe([0.3, 0.0, 0.0], [1.0, 0.0, 0.0], 2.0, 0.5), 0.0);
    assert_eq!(stripe([0.5, 0.0, 0.0], [1.0, 0.0, 0.0], 2.0, 0.5), 1.0);
    let r = reflect([0.0, 0.0, -1.0], [0.0, 0.6, 0.8]);
    assert!((r[1] - 0.96).abs() < 1e-6 && (r[2] - (-1.0 + 2.0 * 0.64)).abs() < 1e-6);
}

#[test]
fn ramps_hit_their_end_stops_and_stay_in_range() {
    let same = |a: [f32; 3], b: [f32; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-6);
    assert!(same(ramp::sequential(0.0), ramp::SEQUENTIAL[0]));
    assert!(same(ramp::sequential(1.0), ramp::SEQUENTIAL[4]));
    assert!(same(ramp::sequential(-5.0), ramp::SEQUENTIAL[0]));
    assert!(same(ramp::diverging(0.0), ramp::DIVERGING[1]));
    assert!(same(ramp::diverging(-1.0), ramp::DIVERGING[0]));
    assert!(same(ramp::diverging(1.0), ramp::DIVERGING[2]));
    for i in 0..=100 {
        let c = ramp::sequential(i as f32 / 100.0);
        assert!(c.iter().all(|v| (0.0..=1.0).contains(v)));
    }
    // draft colours: too little draft red, right way green-ish, wrong way blue-ish
    let red = ramp::draft(0.5, 3.0);
    let ok = ramp::draft(30.0, 3.0);
    let wrong = ramp::draft(-30.0, 3.0);
    assert!(red[0] > 0.7 && red[1] < 0.3);
    assert!(ok[1] > ok[0] && ok[1] > ok[2]);
    assert!(wrong[2] > wrong[0] && wrong[2] > wrong[1]);
}
