//! Sketch → extrude → fillet → shell through the DAG node types and `KernelEvaluator`, with
//! persistent names that survive parameter edits.

use kw_boolean::Op;
use kw_brep::EntityRef;
use kw_feature::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_sketch::Sketch;
use std::sync::Arc;

fn node(kind: NodeKind, name: &str) -> DagNode {
    DagNode {
        kind,
        name: name.into(),
        suppressed: false,
        state: EvalState::Dirty,
    }
}

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

fn rectangle_sketch(w: f64, h: f64) -> Sketch {
    let mut sk = Sketch::new();
    let p: Vec<_> = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]
        .iter()
        .map(|c| sk.add_point(c.0, c.1))
        .collect();
    for i in 0..4 {
        sk.add_line(p[i], p[(i + 1) % 4]);
    }
    sk
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

struct Doc {
    dag: FeatureDag,
    eval: KernelEvaluator,
}

impl Doc {
    fn run(&mut self, id: NodeId, inputs: &[Arc<EvalOutput>]) -> Arc<EvalOutput> {
        let n = self.dag.node(id).unwrap().clone();
        Arc::new(
            self.eval
                .evaluate(&n, id, inputs)
                .unwrap_or_else(|e| panic!("{e:?}")),
        )
    }
}

fn name_of(out: &EvalOutput, role: &str) -> TopoName {
    out.names
        .by_name
        .keys()
        .find(|n| matches!(&n.origin, NameOrigin::Primitive { role: r } if r == role))
        .unwrap_or_else(|| {
            panic!(
                "no face named {role}: {:?}",
                out.names.by_name.keys().collect::<Vec<_>>()
            )
        })
        .clone()
}

#[test]
fn sketch_extrude_fillet_shell_with_stable_names() {
    let mut doc = Doc {
        dag: FeatureDag::new(),
        eval: KernelEvaluator::new(),
    };
    let ext = doc.dag.add_node(node(NodeKind::Extrude, "Extrude1"));
    let fil = doc.dag.add_node(node(NodeKind::Fillet, "Fillet1"));
    let shl = doc.dag.add_node(node(NodeKind::Shell, "Shell1"));
    doc.dag.add_dependency(ext, fil).unwrap();
    doc.dag.add_dependency(fil, shl).unwrap();

    let profile = sketch_profile(&rectangle_sketch(4.0, 3.0), world()).unwrap();
    doc.eval.register(
        ext,
        FeatureOp::Extrude {
            profile: profile.clone(),
            vector: Vector3::new(0.0, 0.0, 2.0),
            symmetric: false,
        },
    );
    let e1 = doc.run(ext, &[]);
    let vol = |o: &EvalOutput| o.brep.volume(o.brep.bodies.keys().next().unwrap()).unwrap();
    assert!(rel(vol(&e1), 24.0) < 1e-10);
    // every output face is named, with roles from the extrude
    assert_eq!(e1.names.by_name.len(), 6);
    let (s0, s1) = (name_of(&e1, "side-0-0"), name_of(&e1, "side-0-1"));
    let cap_top = name_of(&e1, "end-cap");

    // fillet the edge between side 0 and side 1 (selected by the persistent names of its faces)
    doc.eval.register(
        fil,
        FeatureOp::Fillet {
            edges: vec![EdgeSel(s0.clone(), s1.clone())],
            radius: 0.5,
        },
    );
    let f1 = doc.run(fil, std::slice::from_ref(&e1));
    assert!(
        rel(
            vol(&f1),
            24.0 - 0.25 * (1.0 - std::f64::consts::FRAC_PI_4) * 2.0 * 2.0 / 2.0 * 1.0
        ) < 1.0,
        "volume sanity"
    );
    assert!(vol(&f1) < vol(&e1));
    assert_eq!(
        f1.names.by_name.len(),
        7,
        "six original faces keep their names, plus the fillet face"
    );
    // untouched faces keep the very same persistent names
    for n in e1.names.by_name.keys() {
        assert!(
            f1.names.by_name.contains_key(n),
            "name {n:?} was lost by the fillet"
        );
    }
    assert!(f1.names.by_name.keys().any(
        |n| matches!(&n.origin, NameOrigin::Generated { role, .. } if role.starts_with("fillet"))
    ));

    // shell with the top cap open
    doc.eval.register(
        shl,
        FeatureOp::Shell {
            thickness: 0.2,
            open: vec![cap_top.clone()],
        },
    );
    let s = doc.run(shl, std::slice::from_ref(&f1));
    assert!(vol(&s) < vol(&f1));
    assert!(s.brep.validate().is_ok(), "{:?}", s.brep.validate());

    // parameter edit: the extrusion gets taller; names and downstream features still resolve
    doc.eval.register(
        ext,
        FeatureOp::Extrude {
            profile: profile.clone(),
            vector: Vector3::new(0.0, 0.0, 3.5),
            symmetric: false,
        },
    );
    let e2 = doc.run(ext, &[]);
    assert!(rel(vol(&e2), 42.0) < 1e-10);
    let names1: std::collections::BTreeSet<String> =
        e1.names.by_name.keys().map(|n| format!("{n:?}")).collect();
    let names2: std::collections::BTreeSet<String> =
        e2.names.by_name.keys().map(|n| format!("{n:?}")).collect();
    assert_eq!(
        names1, names2,
        "a dimension change must not rename anything"
    );
    let f2 = doc.run(fil, std::slice::from_ref(&e2));
    let s2 = doc.run(shl, std::slice::from_ref(&f2));
    assert!(vol(&s2) < vol(&f2) && s2.brep.validate().is_ok());

    // determinism: re-evaluating gives bit-identical hashes
    let again = doc.run(fil, std::slice::from_ref(&e2));
    assert_eq!(again.output_hash, f2.output_hash);
    assert_ne!(f2.output_hash, f1.output_hash);
}

#[test]
fn boolean_between_two_features_and_unresolvable_references() {
    let mut doc = Doc {
        dag: FeatureDag::new(),
        eval: KernelEvaluator::new(),
    };
    let a = doc.dag.add_node(node(NodeKind::Extrude, "A"));
    let b = doc.dag.add_node(node(NodeKind::Extrude, "B"));
    let c = doc.dag.add_node(node(NodeKind::Boolean, "Cut"));
    doc.eval.register(
        a,
        FeatureOp::Extrude {
            profile: sketch_profile(&rectangle_sketch(4.0, 4.0), world()).unwrap(),
            vector: Vector3::new(0.0, 0.0, 2.0),
            symmetric: false,
        },
    );
    let shifted = Frame3 {
        origin: Point3::new(1.0, 1.0, -1.0),
        ..world()
    };
    doc.eval.register(
        b,
        FeatureOp::Extrude {
            profile: sketch_profile(&rectangle_sketch(1.0, 1.0), shifted).unwrap(),
            vector: Vector3::new(0.0, 0.0, 4.0),
            symmetric: false,
        },
    );
    let (oa, ob) = (doc.run(a, &[]), doc.run(b, &[]));
    doc.eval
        .register(c, FeatureOp::Boolean { op: Op::Subtract });
    let cut = doc.run(c, &[oa.clone(), ob.clone()]);
    let body = cut.brep.bodies.keys().next().unwrap();
    assert!(rel(cut.brep.volume(body).unwrap(), 32.0 - 2.0) < 1e-9);
    assert!(cut.brep.validate().is_ok());
    // faces from A keep A's names, the hole walls descend from B's names (through the Boolean history)
    for n in oa.names.by_name.keys() {
        assert!(
            cut.names.by_name.contains_key(n)
                || cut.names.by_name.keys().any(
                    |m| matches!(&m.origin, NameOrigin::Split { parent, .. } if **parent == *n)
                ),
            "A's face {n:?} vanished without a trace"
        );
    }
    // a fillet with a name that does not exist is a reference error, not a panic
    let f = doc.dag.add_node(node(NodeKind::Fillet, "BadFillet"));
    let bogus = TopoName {
        feature: f,
        origin: NameOrigin::Primitive {
            role: "nope".into(),
        },
        ordinal: 0,
    };
    doc.eval.register(
        f,
        FeatureOp::Fillet {
            edges: vec![EdgeSel(bogus.clone(), bogus)],
            radius: 0.1,
        },
    );
    let n = doc.dag.node(f).unwrap().clone();
    assert!(matches!(
        doc.eval.evaluate(&n, f, &[cut]),
        Err(EvalError::Reference(_))
    ));
    let _ = EntityRef::Face;
}

#[test]
fn sketch_with_a_hole_and_open_loops() {
    // rectangle with a circular hole
    let mut sk = rectangle_sketch(6.0, 4.0);
    let c = sk.add_point(3.0, 2.0);
    sk.add_circle(c, 0.8);
    let profile = sketch_profile(&sk, world()).unwrap();
    assert_eq!(profile.holes.len(), 1);
    let mut doc = Doc {
        dag: FeatureDag::new(),
        eval: KernelEvaluator::new(),
    };
    let e = doc.dag.add_node(node(NodeKind::Extrude, "E"));
    doc.eval.register(
        e,
        FeatureOp::Extrude {
            profile,
            vector: Vector3::new(0.0, 0.0, 1.0),
            symmetric: false,
        },
    );
    let o = doc.run(e, &[]);
    let body = o.brep.bodies.keys().next().unwrap();
    assert!(
        rel(
            o.brep.volume(body).unwrap(),
            (24.0 - std::f64::consts::PI * 0.64) * 1.0
        ) < 1e-9
    );
    // open loop is an error
    let mut open = Sketch::new();
    open.add_line_xy(0.0, 0.0, 1.0, 0.0);
    open.add_line_xy(1.0, 0.0, 1.0, 1.0);
    assert!(sketch_profile(&open, world()).is_err());
}

#[test]
fn variable_radius_fillet_feature_names_its_face() {
    use kw_blend::RadiusLaw;
    let mut doc = Doc {
        dag: FeatureDag::new(),
        eval: KernelEvaluator::new(),
    };
    let ext = doc.dag.add_node(node(NodeKind::Extrude, "Extrude1"));
    let fil = doc.dag.add_node(node(NodeKind::Fillet, "VarFillet1"));
    doc.dag.add_dependency(ext, fil).unwrap();
    let profile = sketch_profile(&rectangle_sketch(4.0, 3.0), world()).unwrap();
    doc.eval.register(
        ext,
        FeatureOp::Extrude {
            profile,
            vector: Vector3::new(0.0, 0.0, 2.0),
            symmetric: false,
        },
    );
    let e1 = doc.run(ext, &[]);
    let (s0, s1) = (name_of(&e1, "side-0-0"), name_of(&e1, "side-0-1"));
    doc.eval.register(
        fil,
        FeatureOp::VariableFillet {
            edge: EdgeSel(s0, s1),
            law: RadiusLaw::Table(vec![(0.0, 0.2), (0.5, 0.6), (1.0, 0.3)]),
        },
    );
    let f1 = doc.run(fil, std::slice::from_ref(&e1));
    let vol = |o: &EvalOutput| o.brep.volume(o.brep.bodies.keys().next().unwrap()).unwrap();
    assert!(vol(&f1) < vol(&e1) && vol(&f1) > vol(&e1) - 0.2);
    assert_eq!(f1.names.by_name.len(), 7);
}
