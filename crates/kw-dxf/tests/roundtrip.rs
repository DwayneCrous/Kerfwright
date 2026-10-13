use kw_dxf::{export_sketch, import_sketch, parse, write, DxfDoc, DxfEntity, ImportOptions};
use kw_sketch::geom::arc_params;
use kw_sketch::{EntityId, EntityKind, Sketch};
use proptest::prelude::*;
use std::collections::HashSet;
use std::io::Cursor;

const TOL: f64 = 1e-6;

// ---------------------------------------------------------------------------
// test-side geometry descriptors (independent of the exporter)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct Desc {
    tag: &'static str,
    construction: bool,
    v: Vec<f64>,
}

fn describe(sk: &Sketch) -> Vec<Desc> {
    let mut referenced: HashSet<EntityId> = HashSet::new();
    for id in sk.entities.keys() {
        referenced.extend(sk.children(id));
    }
    let mut out = Vec::new();
    for (id, e) in &sk.entities {
        let p = |q: &EntityId| sk.point(*q).unwrap();
        let (tag, v): (&'static str, Vec<f64>) = match &e.kind {
            EntityKind::Point(_) => {
                if referenced.contains(&id) {
                    continue;
                }
                ("point", vec![p(&id).0, p(&id).1])
            }
            EntityKind::Line { p1, p2 } => {
                let (a, b) = (p(p1), p(p2));
                // end-point order is not significant
                let (a, b) = if (a.0, a.1) <= (b.0, b.1) {
                    (a, b)
                } else {
                    (b, a)
                };
                ("line", vec![a.0, a.1, b.0, b.1])
            }
            EntityKind::Circle { center, radius } => {
                ("circle", vec![p(center).0, p(center).1, sk.value(*radius)])
            }
            EntityKind::Arc { center, start, end } => (
                "arc",
                vec![
                    p(center).0,
                    p(center).1,
                    p(start).0,
                    p(start).1,
                    p(end).0,
                    p(end).1,
                ],
            ),
            EntityKind::Ellipse {
                center,
                major_end,
                minor_radius,
            } => (
                "ellipse",
                vec![
                    p(center).0,
                    p(center).1,
                    p(major_end).0,
                    p(major_end).1,
                    sk.value(*minor_radius),
                ],
            ),
            EntityKind::Spline { control, .. } => (
                "spline",
                control.iter().flat_map(|c| [p(c).0, p(c).1]).collect(),
            ),
            EntityKind::FitSpline { points } => (
                "fit",
                points.iter().flat_map(|c| [p(c).0, p(c).1]).collect(),
            ),
        };
        out.push(Desc {
            tag,
            construction: e.construction,
            v,
        });
    }
    out
}

/// Every descriptor in `a` has a distinct partner in `b` within `TOL`, and vice versa.
fn assert_same(a: &[Desc], b: &[Desc]) {
    assert_eq!(a.len(), b.len(), "entity count {a:#?} vs {b:#?}");
    let mut used = vec![false; b.len()];
    for d in a {
        let hit = b.iter().enumerate().find(|(i, e)| {
            !used[*i]
                && e.tag == d.tag
                && e.construction == d.construction
                && e.v.len() == d.v.len()
                && e.v.iter().zip(&d.v).all(|(x, y)| (x - y).abs() <= TOL)
        });
        match hit {
            Some((i, _)) => used[i] = true,
            None => panic!("no match for {d:?} in {b:#?}"),
        }
    }
}

fn round_trip(sk: &Sketch) -> Sketch {
    let text = write(&export_sketch(sk));
    let doc = parse(&text).expect("own output parses");
    import_sketch(&doc, &ImportOptions::default())
}

// ---------------------------------------------------------------------------
// random sketches
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
enum Spec {
    Point(f64, f64),
    Line([f64; 4]),
    Circle(f64, f64, f64),
    Arc(f64, f64, f64, f64, f64),
    Ellipse(f64, f64, f64, f64, f64),
    Spline(usize, Vec<(f64, f64)>),
    Fit(Vec<(f64, f64)>),
}

fn coord() -> impl Strategy<Value = f64> {
    -100.0f64..100.0
}

fn spec() -> impl Strategy<Value = (Spec, bool)> {
    let s = prop_oneof![
        (coord(), coord()).prop_map(|(x, y)| Spec::Point(x, y)),
        (
            coord(),
            coord(),
            0.5f64..50.0,
            0.0f64..std::f64::consts::TAU
        )
            .prop_map(|(x, y, l, a)| Spec::Line([x, y, x + l * a.cos(), y + l * a.sin()])),
        (coord(), coord(), 0.1f64..80.0).prop_map(|(x, y, r)| Spec::Circle(x, y, r)),
        (coord(), coord(), 0.5f64..60.0, -3.0f64..3.0, 0.2f64..6.0)
            .prop_map(|(x, y, r, a0, d)| Spec::Arc(x, y, r, a0, a0 + d)),
        (
            coord(),
            coord(),
            1.0f64..40.0,
            0.1f64..0.99,
            0.0f64..std::f64::consts::PI
        )
            .prop_map(|(x, y, a, k, t)| Spec::Ellipse(
                x,
                y,
                a * t.cos(),
                a * t.sin(),
                a * k
            )),
        (1usize..4, prop::collection::vec((coord(), coord()), 4..9))
            .prop_map(|(d, p)| Spec::Spline(d, p)),
        prop::collection::vec((coord(), coord()), 3..8).prop_map(Spec::Fit),
    ];
    (s, any::<bool>())
}

fn build(specs: &[(Spec, bool)]) -> Sketch {
    let mut sk = Sketch::new();
    for (s, cons) in specs {
        let id = match s {
            Spec::Point(x, y) => sk.add_point(*x, *y),
            Spec::Line([a, b, c, d]) => sk.add_line_xy(*a, *b, *c, *d),
            Spec::Circle(x, y, r) => sk.add_circle_xy(*x, *y, *r),
            Spec::Arc(x, y, r, a0, a1) => sk.add_arc_xy(*x, *y, *r, *a0, *a1),
            Spec::Ellipse(x, y, mx, my, b) => {
                let c = sk.add_point(*x, *y);
                let m = sk.add_point(x + mx, y + my);
                sk.add_ellipse(c, m, *b)
            }
            Spec::Spline(d, pts) => {
                let d = (*d).min(pts.len() - 1);
                let ids = pts.iter().map(|p| sk.add_point(p.0, p.1)).collect();
                sk.add_spline(d, ids).unwrap()
            }
            Spec::Fit(pts) => {
                let ids = pts.iter().map(|p| sk.add_point(p.0, p.1)).collect();
                sk.add_fit_spline(ids)
            }
        };
        sk.set_construction(id, *cons);
    }
    sk
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    /// sketch → DXF text → parse → sketch preserves geometry to 1e-6.
    #[test]
    fn sketch_dxf_sketch(specs in prop::collection::vec(spec(), 1..14)) {
        let sk = build(&specs);
        let back = round_trip(&sk);
        assert_same(&describe(&sk), &describe(&back));
    }

    /// The raw document is bit-exact through write → parse.
    #[test]
    fn document_is_bit_exact(specs in prop::collection::vec(spec(), 1..14)) {
        let doc = export_sketch(&build(&specs));
        let again = parse(&write(&doc)).unwrap();
        prop_assert_eq!(doc, again);
    }

    /// Arbitrary text never panics the reader.
    #[test]
    fn arbitrary_text_never_panics(s in ".{0,400}") {
        let _ = parse(&s);
    }

    /// Corrupting valid files never panics the reader or the importer.
    #[test]
    fn mutated_files_never_panic(
        specs in prop::collection::vec(spec(), 1..6),
        edits in prop::collection::vec((any::<prop::sample::Index>(), any::<u8>()), 1..12),
        cut in any::<prop::sample::Index>(),
    ) {
        let mut bytes = write(&export_sketch(&build(&specs))).into_bytes();
        for (i, b) in &edits {
            let k = i.index(bytes.len());
            bytes[k] = *b;
        }
        let keep = cut.index(bytes.len() + 1);
        bytes.truncate(keep);
        let text = String::from_utf8_lossy(&bytes);
        if let Ok(doc) = parse(&text) {
            let _ = import_sketch(&doc, &ImportOptions::default());
        }
    }
}

// ---------------------------------------------------------------------------
// deterministic cases
// ---------------------------------------------------------------------------

#[test]
fn shared_endpoints_are_welded_back() {
    let mut sk = Sketch::new();
    let pts: Vec<_> = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)]
        .iter()
        .map(|p| sk.add_point(p.0, p.1))
        .collect();
    for i in 0..4 {
        sk.add_line(pts[i], pts[(i + 1) % 4]);
    }
    let back = round_trip(&sk);
    let n_points = back
        .entities
        .values()
        .filter(|e| matches!(e.kind, EntityKind::Point(_)))
        .count();
    assert_eq!(n_points, 4, "4 corners, not 8 end points");
    assert_same(&describe(&sk), &describe(&back));
}

#[test]
fn near_coincident_ends_weld_within_tolerance_only() {
    let doc = DxfDoc {
        entities: vec![
            DxfEntity::Line {
                layer: "0".into(),
                a: (0.0, 0.0),
                b: (1.0, 0.0),
            },
            DxfEntity::Line {
                layer: "0".into(),
                a: (1.0 + 5e-7, 0.0),
                b: (1.0, 1.0),
            },
            DxfEntity::Line {
                layer: "0".into(),
                a: (1.0, 1.0 + 1e-3),
                b: (0.0, 1.0),
            },
        ],
    };
    let sk = import_sketch(&doc, &ImportOptions::default());
    let n_points = sk
        .entities
        .values()
        .filter(|e| matches!(e.kind, EntityKind::Point(_)))
        .count();
    // (0,0) (1,0) (1,1) (1,1.001) (0,1) = 5 distinct points
    assert_eq!(n_points, 5);
}

#[test]
fn lwpolyline_bulges_become_arcs() {
    // a "stadium": two lines and two semicircles (bulge = 1)
    let doc = DxfDoc {
        entities: vec![DxfEntity::LwPolyline {
            layer: "0".into(),
            closed: true,
            verts: vec![
                (0.0, 0.0, 0.0),
                (10.0, 0.0, 1.0),
                (10.0, 4.0, 0.0),
                (0.0, 4.0, 1.0),
            ],
        }],
    };
    let sk = import_sketch(&doc, &ImportOptions::default());
    let arcs: Vec<_> = sk
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
        .collect();
    let lines = sk
        .entities
        .values()
        .filter(|e| matches!(e.kind, EntityKind::Line { .. }))
        .count();
    assert_eq!((arcs.len(), lines), (2, 2));
    for (id, _) in arcs {
        let (c, r, a0, a1) = arc_params(&sk, id).unwrap();
        assert!(
            (r - 2.0).abs() < TOL,
            "semicircle over a chord of 4 has r=2, got {r}"
        );
        assert!(
            (a1 - a0 - std::f64::consts::PI).abs() < 1e-9,
            "sweeps 180 degrees"
        );
        assert!((c.1 - 2.0).abs() < TOL);
    }
    // negative bulge goes clockwise: arc bulges to the other side
    let doc = DxfDoc {
        entities: vec![DxfEntity::LwPolyline {
            layer: "0".into(),
            closed: false,
            verts: vec![(0.0, 0.0, -1.0), (4.0, 0.0, 0.0)],
        }],
    };
    let sk = import_sketch(&doc, &ImportOptions::default());
    let (id, _) = sk
        .entities
        .iter()
        .find(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
        .unwrap();
    let (c, r, a0, a1) = arc_params(&sk, id).unwrap();
    assert!((r - 2.0).abs() < TOL && (c.0 - 2.0).abs() < TOL && c.1.abs() < TOL);
    // DXF: bulge > 0 is counter-clockwise from vertex to next vertex, < 0 clockwise.
    // Going left to right, CW passes over the top.
    let mid = 0.5 * (a0 + a1);
    let m = (c.0 + r * mid.cos(), c.1 + r * mid.sin());
    assert!(
        m.1 > 0.0,
        "negative bulge on a left-to-right chord bulges upward, got {m:?}"
    );

    let doc = DxfDoc {
        entities: vec![DxfEntity::LwPolyline {
            layer: "0".into(),
            closed: false,
            verts: vec![(0.0, 0.0, 1.0), (4.0, 0.0, 0.0)],
        }],
    };
    let sk = import_sketch(&doc, &ImportOptions::default());
    let (id, _) = sk
        .entities
        .iter()
        .find(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
        .unwrap();
    let (c, r, a0, a1) = arc_params(&sk, id).unwrap();
    let mid = 0.5 * (a0 + a1);
    let m = (c.0 + r * mid.cos(), c.1 + r * mid.sin());
    assert!(
        m.1 < 0.0,
        "positive bulge on a left-to-right chord bulges downward, got {m:?}"
    );
}

#[test]
fn rational_and_non_uniform_splines_import_as_fit_splines() {
    let doc = DxfDoc {
        entities: vec![DxfEntity::Spline {
            layer: "0".into(),
            degree: 2,
            knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            ctrl: vec![(0.0, 0.0), (1.0, 2.0), (2.0, 0.0)],
            weights: vec![1.0, 2.0, 1.0],
            fit: vec![],
            closed: false,
        }],
    };
    let sk = import_sketch(&doc, &ImportOptions::default());
    let d = describe(&sk);
    assert!(d.iter().any(|x| x.tag == "fit"), "{d:?}");
}

#[test]
fn partial_ellipse_imports_as_fit_spline_through_true_points() {
    let doc = DxfDoc {
        entities: vec![DxfEntity::Ellipse {
            layer: "0".into(),
            c: (1.0, 2.0),
            major: (0.0, 5.0),
            ratio: 0.4,
            t0: 0.0,
            t1: std::f64::consts::FRAC_PI_2,
        }],
    };
    let sk = import_sketch(&doc, &ImportOptions::default());
    let d = describe(&sk);
    let fit = d.iter().find(|x| x.tag == "fit").expect("fit spline");
    // first point = centre + major, last = centre + minor-axis end (rotated 90 deg => -x)
    assert!((fit.v[0] - 1.0).abs() < TOL && (fit.v[1] - 7.0).abs() < TOL);
    let n = fit.v.len();
    assert!((fit.v[n - 2] - (1.0 - 2.0)).abs() < TOL && (fit.v[n - 1] - 2.0).abs() < TOL);
}

#[test]
fn construction_geometry_uses_its_own_layer() {
    let mut sk = Sketch::new();
    let a = sk.add_line_xy(0.0, 0.0, 1.0, 0.0);
    sk.add_line_xy(0.0, 1.0, 1.0, 1.0);
    sk.set_construction(a, true);
    let doc = export_sketch(&sk);
    let layers: HashSet<&str> = doc.entities.iter().map(|e| e.layer()).collect();
    assert_eq!(layers, HashSet::from(["0", "CONSTRUCTION"]));
    assert_same(&describe(&sk), &describe(&round_trip(&sk)));
}

#[test]
fn malformed_input_is_an_error_not_a_panic() {
    assert!(parse("0\nSECTION\n2\nENTITIES\n0\nLINE\n10\nabc\n").is_err());
    assert!(parse("x\ny\n").is_err());
    assert!(parse("0\nSECTION\n2\nENTITIES\n0\nCIRCLE\n10\n1\n20\n2\n").is_err()); // missing radius
    assert!(parse("0\nSECTION\n2\nENTITIES\n0\nLINE\n10\nNaN\n20\n0\n11\n1\n21\n1\n").is_err());
    // unknown entities and empty documents are fine
    assert_eq!(parse("").unwrap(), DxfDoc::default());
    assert!(
        parse("0\nSECTION\n2\nENTITIES\n0\nHATCH\n8\n0\n0\nENDSEC\n0\nEOF\n")
            .unwrap()
            .entities
            .is_empty()
    );
}

// ---------------------------------------------------------------------------
// independent parser (`dxf` crate)
// ---------------------------------------------------------------------------

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= TOL
}

#[test]
fn output_is_read_identically_by_the_dxf_crate() {
    use dxf::entities::EntityType as E;
    let mut sk = Sketch::new();
    sk.add_point(3.0, 4.0);
    sk.add_line_xy(-1.0, 2.0, 5.5, 7.25);
    sk.add_circle_xy(1.0, 1.0, 2.5);
    sk.add_arc_xy(0.0, 0.0, 3.0, 0.3, 2.0);
    let c = sk.add_point(2.0, 2.0);
    let m = sk.add_point(6.0, 2.0);
    sk.add_ellipse(c, m, 1.5);
    let ids: Vec<_> = [(0.0, 0.0), (1.0, 3.0), (3.0, -1.0), (5.0, 2.0)]
        .iter()
        .map(|p| sk.add_point(p.0, p.1))
        .collect();
    sk.add_spline(3, ids).unwrap();
    let cl = sk.add_line_xy(0.0, 0.0, 9.0, 9.0);
    sk.set_construction(cl, true);

    let ours = export_sketch(&sk);
    let text = write(&ours);
    let drawing =
        dxf::Drawing::load(&mut Cursor::new(text.as_bytes())).expect("dxf crate parses our output");
    let theirs: Vec<_> = drawing.entities().collect();
    assert_eq!(theirs.len(), ours.entities.len());

    let mut seen_construction = 0;
    for e in &theirs {
        let hit = ours.entities.iter().any(|o| match (&e.specific, o) {
            (E::ModelPoint(p), DxfEntity::Point { p: q, .. }) => {
                close(p.location.x, q.0) && close(p.location.y, q.1)
            }
            (E::Line(l), DxfEntity::Line { a, b, .. }) => {
                close(l.p1.x, a.0) && close(l.p1.y, a.1) && close(l.p2.x, b.0) && close(l.p2.y, b.1)
            }
            (E::Circle(c), DxfEntity::Circle { c: q, r, .. }) => {
                close(c.center.x, q.0) && close(c.center.y, q.1) && close(c.radius, *r)
            }
            (E::Arc(a), DxfEntity::Arc { c, r, a0, a1, .. }) => {
                close(a.center.x, c.0)
                    && close(a.center.y, c.1)
                    && close(a.radius, *r)
                    && close(a.start_angle, *a0)
                    && close(a.end_angle, *a1)
            }
            (
                E::Ellipse(el),
                DxfEntity::Ellipse {
                    c, major, ratio, ..
                },
            ) => {
                close(el.center.x, c.0)
                    && close(el.major_axis.x, major.0)
                    && close(el.major_axis.y, major.1)
                    && close(el.minor_axis_ratio, *ratio)
            }
            (
                E::Spline(s),
                DxfEntity::Spline {
                    degree,
                    knots,
                    ctrl,
                    ..
                },
            ) => {
                s.degree_of_curve as usize == *degree
                    && s.knot_values.len() == knots.len()
                    && s.knot_values.iter().zip(knots).all(|(x, y)| close(*x, *y))
                    && s.control_points.len() == ctrl.len()
                    && s.control_points
                        .iter()
                        .zip(ctrl)
                        .all(|(p, q)| close(p.x, q.0) && close(p.y, q.1))
            }
            _ => false,
        });
        assert!(
            hit,
            "the dxf crate read an entity we did not write: {:?}",
            e.specific
        );
        if e.common.layer == "CONSTRUCTION" {
            seen_construction += 1;
        }
    }
    assert_eq!(seen_construction, 1);
}

#[test]
fn dxf_crate_reads_our_lwpolyline_with_bulges() {
    use dxf::entities::EntityType as E;
    let doc = DxfDoc {
        entities: vec![DxfEntity::LwPolyline {
            layer: "0".into(),
            closed: true,
            verts: vec![
                (0.0, 0.0, 0.0),
                (10.0, 0.0, 0.5),
                (10.0, 4.0, 0.0),
                (0.0, 4.0, -0.25),
            ],
        }],
    };
    let text = write(&doc);
    let d = dxf::Drawing::load(&mut Cursor::new(text.as_bytes())).unwrap();
    let e = d.entities().next().unwrap();
    let E::LwPolyline(p) = &e.specific else {
        panic!("{:?}", e.specific)
    };
    assert_eq!(p.vertices.len(), 4);
    assert!(p.is_closed());
    let want = [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.5),
        (10.0, 4.0, 0.0),
        (0.0, 4.0, -0.25),
    ];
    for (v, w) in p.vertices.iter().zip(want) {
        assert!(
            close(v.x, w.0) && close(v.y, w.1) && close(v.bulge, w.2),
            "{v:?} vs {w:?}"
        );
    }
    // and we read what we wrote
    assert_eq!(parse(&text).unwrap(), doc);
}

#[test]
fn we_read_files_written_by_the_dxf_crate() {
    use dxf::entities::{Circle, Entity, EntityType, Line};
    use dxf::{Drawing, Point};
    let mut d = Drawing::new();
    d.add_entity(Entity::new(EntityType::Line(Line::new(
        Point::new(1.0, 2.0, 0.0),
        Point::new(4.0, 6.0, 0.0),
    ))));
    d.add_entity(Entity::new(EntityType::Circle(Circle::new(
        Point::new(0.5, 0.25, 0.0),
        3.0,
    ))));
    let mut buf = Vec::new();
    d.save(&mut buf).unwrap();
    let doc = parse(&String::from_utf8(buf).unwrap()).expect("our reader handles a foreign file");
    assert_eq!(doc.entities.len(), 2);
    assert!(
        matches!(&doc.entities[0], DxfEntity::Line { a, b, .. } if close(a.0, 1.0) && close(b.1, 6.0))
    );
    assert!(matches!(&doc.entities[1], DxfEntity::Circle { r, .. } if close(*r, 3.0)));
}

#[test]
fn text_round_trips_and_the_dxf_crate_reads_it() {
    use dxf::entities::EntityType as E;
    let doc = DxfDoc {
        entities: vec![DxfEntity::Text {
            layer: "NOTES".into(),
            p: (12.5, -3.0),
            height: 3.5,
            rotation: 90.0,
            halign: 1,
            text: "SCALE 1:2".into(),
        }],
    };
    let text = write(&doc);
    assert_eq!(parse(&text).unwrap(), doc);
    let d = dxf::Drawing::load(&mut Cursor::new(text.as_bytes())).unwrap();
    let E::Text(t) = &d.entities().next().unwrap().specific else {
        panic!("not text")
    };
    assert_eq!(t.value, "SCALE 1:2");
    assert!(close(t.text_height, 3.5) && close(t.rotation, 90.0));
}

#[test]
fn layer_styles_linetypes_and_extents_are_written() {
    use kw_dxf::{write_with_layers, LayerDef};
    let doc = DxfDoc {
        entities: vec![
            DxfEntity::Line {
                layer: "HIDDEN".into(),
                a: (0.0, 0.0),
                b: (30.0, 10.0),
            },
            DxfEntity::Circle {
                layer: "OTHER".into(),
                c: (50.0, 50.0),
                r: 5.0,
            },
        ],
    };
    let text = write_with_layers(&doc, &[LayerDef::new("HIDDEN", 8, "HIDDEN", 25)]);
    let d = dxf::Drawing::load(&mut Cursor::new(text.as_bytes())).unwrap();
    let l = d.layers().find(|l| l.name == "HIDDEN").unwrap();
    assert_eq!(l.line_type_name, "HIDDEN");
    assert!(d
        .line_types()
        .any(|t| t.name == "CENTER" && t.dash_dot_space_lengths.len() == 4));
    assert!(close(d.header.minimum_drawing_extents.x, 0.0));
    assert!(close(d.header.maximum_drawing_extents.x, 55.0));
    assert!(close(d.header.maximum_drawing_extents.y, 55.0));
    // layers the caller did not describe are still defined
    assert!(d.layers().any(|l| l.name == "OTHER"));
    assert_eq!(parse(&text).unwrap(), doc);
}
