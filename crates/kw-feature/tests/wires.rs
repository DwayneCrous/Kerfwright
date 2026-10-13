use kw_feature::sketch_wires;
use kw_math::{Frame3, Point3};
use kw_sketch::{EntityId, Sketch};

fn ents(sk: &Sketch) -> Vec<EntityId> {
    sk.entities.keys().collect()
}

fn near(a: Point3<f64>, b: Point3<f64>) -> bool {
    (a - b).norm() < 1e-9
}

fn consecutive(w: &kw_feature::Wire) -> bool {
    w.edges.windows(2).all(|p| near(p[0].end(), p[1].start()))
}

#[test]
fn an_open_polyline_is_one_chain_whose_edges_follow_each_other() {
    let mut sk = Sketch::new();
    // drawn in a scrambled order and directions
    sk.add_line_xy(10.0, 0.0, 5.0, 0.0);
    sk.add_line_xy(0.0, 0.0, 5.0, 0.0);
    sk.add_line_xy(10.0, 0.0, 10.0, 6.0);
    let w = sketch_wires(&sk, &ents(&sk), Frame3::world());
    assert_eq!(w.len(), 1);
    let w = &w[0];
    assert!(!w.closed);
    assert_eq!(w.edges.len(), 3);
    assert!(consecutive(w));
    // it runs from one free end to the other
    let ends = [w.start(), w.end()];
    assert!(ends.iter().any(|p| near(*p, Point3::new(0.0, 0.0, 0.0))));
    assert!(ends.iter().any(|p| near(*p, Point3::new(10.0, 6.0, 0.0))));
    assert_eq!(w.labels.len(), 3);
}

#[test]
fn a_closed_outline_is_a_closed_chain_and_a_circle_is_one_edge() {
    let mut sk = Sketch::new();
    let pts: Vec<_> = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]
        .iter()
        .map(|p| sk.add_point(p.0, p.1))
        .collect();
    for i in 0..4 {
        sk.add_line(pts[i], pts[(i + 1) % 4]);
    }
    sk.add_circle_xy(20.0, 20.0, 2.0);
    let w = sketch_wires(&sk, &ents(&sk), Frame3::world());
    assert_eq!(w.len(), 2);
    let rect = w.iter().find(|x| x.edges.len() == 4).unwrap();
    assert!(rect.closed && consecutive(rect) && near(rect.end(), rect.start()));
    let circle = w.iter().find(|x| x.edges.len() == 1).unwrap();
    assert!(circle.closed);
}

#[test]
fn separate_curves_are_separate_chains_and_a_junction_splits_a_chain() {
    let mut sk = Sketch::new();
    // a T: the bar passes the foot's end, so three curves meet there
    sk.add_line_xy(0.0, 0.0, 5.0, 0.0);
    sk.add_line_xy(5.0, 0.0, 10.0, 0.0);
    sk.add_line_xy(5.0, 0.0, 5.0, 4.0);
    // and an unrelated line
    sk.add_line_xy(20.0, 0.0, 30.0, 0.0);
    let w = sketch_wires(&sk, &ents(&sk), Frame3::world());
    assert_eq!(w.len(), 4, "three branches of the T and the loose line");
    assert!(w.iter().all(|c| !c.closed && c.edges.len() == 1));
}

#[test]
fn construction_geometry_is_not_a_chain_and_the_order_is_stable() {
    let mut sk = Sketch::new();
    let a = sk.add_line_xy(0.0, 0.0, 5.0, 0.0);
    sk.add_line_xy(5.0, 0.0, 5.0, 5.0);
    let guide = sk.add_line_xy(0.0, 9.0, 9.0, 9.0);
    sk.set_construction(guide, true);
    let _ = a;
    let e = ents(&sk);
    let w1 = sketch_wires(&sk, &e, Frame3::world());
    assert_eq!(w1.len(), 1, "the construction line is left out");
    assert_eq!(w1[0].edges.len(), 2);
    // the same sketch again: the same chain, edge for edge
    let w2 = sketch_wires(&sk, &e, Frame3::world());
    assert_eq!(w1[0].labels, w2[0].labels);
    assert!(near(w1[0].start(), w2[0].start()) && near(w1[0].end(), w2[0].end()));
}

#[test]
fn an_arc_chain_lifts_onto_the_sketch_plane() {
    let mut sk = Sketch::new();
    sk.add_arc_xy(0.0, 0.0, 5.0, 0.0, std::f64::consts::FRAC_PI_2);
    sk.add_line_xy(0.0, 5.0, 0.0, 12.0);
    let plane = Frame3 {
        origin: Point3::new(0.0, 0.0, 7.0),
        ..Frame3::world()
    };
    let w = sketch_wires(&sk, &ents(&sk), plane);
    assert_eq!(w.len(), 1);
    assert!(consecutive(&w[0]));
    let ends = [w[0].start(), w[0].end()];
    assert!(
        ends.iter().all(|p| (p.z - 7.0).abs() < 1e-9),
        "on the plane"
    );
    assert!(ends.iter().any(|p| near(*p, Point3::new(5.0, 0.0, 7.0))));
    assert!(ends.iter().any(|p| near(*p, Point3::new(0.0, 12.0, 7.0))));
}
