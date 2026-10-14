//! Studies on several bodies: bonded contact, per-body materials and every kind of study.
use kw_app::fea::FeaResult;
use kw_app::fea_run::{run_study, study_material, StudyModel};
use kw_app::model_source::{Combined, ModelSource};
use kw_app::project::Project;
use kw_job::{CancelFlag, Progress};
use kw_store::{FeaLoad, FeaStudy, SourceKind, StudyKind, Support, ThermalBc};
use kw_tess::{tessellate_body, TessOptions};

fn mm(v: f64) -> kw_expr::Param {
    kw_expr::Param::length(v)
}

/// A design with one block per entry of `x_ranges` (10 mm in y, 5 mm in z), as separate bodies.
fn blocks(x_ranges: &[(f64, f64)]) -> Project {
    let mut p = Project::empty();
    p.history
        .edit(|d| {
            for (k, (x0, x1)) in x_ranges.iter().enumerate() {
                let s = d.add_rect_sketch(
                    &format!("S{k}"),
                    kw_feature::PlaneSpec::Xy(mm(0.0)),
                    mm(*x0),
                    mm(0.0),
                    mm(x1 - x0),
                    mm(10.0),
                )?;
                d.add_extrude(&format!("Block{k}"), s, mm(5.0), kw_feature::BodyOp::New)?;
            }
            Ok(())
        })
        .unwrap();
    p
}

fn combined(p: &mut Project) -> Combined {
    let src = ModelSource::of(p, SourceKind::Design, &Default::default()).unwrap();
    src.combined(&|_| true)
}

/// The global tag of the face of body `body` whose centre is the furthest in direction `sign` along `axis`.
fn end_face(c: &Combined, body: usize, axis: usize, sign: f64) -> u32 {
    let mut offset = 0u32;
    for (i, (_, id)) in c.bodies.iter().enumerate() {
        let t = tessellate_body(
            &c.brep,
            *id,
            &TessOptions {
                chord_tol: 0.005,
                angle_tol: 12f64.to_radians(),
            },
        )
        .unwrap();
        if i == body {
            let n = t.face_keys.len();
            let mut sum = vec![(0.0f64, 0usize); n];
            for (k, tri) in t.mesh.indices.chunks(3).enumerate() {
                let f = t.mesh.tri_face[k] as usize;
                for v in tri {
                    sum[f].0 += t.mesh.positions[*v as usize][axis];
                    sum[f].1 += 1;
                }
            }
            let best = (0..n)
                .max_by(|a, b| {
                    (sign * sum[*a].0 / sum[*a].1 as f64)
                        .total_cmp(&(sign * sum[*b].0 / sum[*b].1 as f64))
                })
                .unwrap();
            return offset + best as u32;
        }
        offset += t.face_keys.len() as u32;
    }
    panic!("no body {body}");
}

fn study_with(c: &Combined, materials: &[&str]) -> (FeaStudy, Vec<kw_store::StudyMaterial>) {
    let lib = kw_materials::global();
    let mats: Vec<kw_store::StudyMaterial> = (0..c.bodies.len())
        .map(|i| study_material(lib.find(materials[i % materials.len()]).unwrap()))
        .collect();
    (
        FeaStudy {
            max_size: 2.5,
            ..FeaStudy::default()
        },
        mats,
    )
}

fn run(
    c: &Combined,
    study: &FeaStudy,
    mats: Vec<kw_store::StudyMaterial>,
) -> Result<FeaResult, String> {
    let m = StudyModel {
        brep: &c.brep,
        bodies: c.bodies.clone(),
        materials: mats,
    };
    run_study(&m, study, &CancelFlag::new(), &Progress::new())
}

fn field<'a>(r: &'a FeaResult, name: &str) -> &'a [f64] {
    &r.fields
        .iter()
        .find(|f| f.name.starts_with(name))
        .unwrap_or_else(|| {
            panic!(
                "no field {name}: {:?}",
                r.fields.iter().map(|f| &f.name).collect::<Vec<_>>()
            )
        })
        .values
}

#[test]
fn two_touching_blocks_are_bonded_and_stretch_like_one_bar() {
    let mut p = blocks(&[(0.0, 10.0), (10.0, 20.0)]);
    let c = combined(&mut p);
    assert_eq!(c.bodies.len(), 2);
    let (mut study, mats) = study_with(&c, &["Steel"]);
    study.fixed_tags = vec![end_face(&c, 0, 0, -1.0)];
    study.loads = vec![FeaLoad::Force {
        tag: end_face(&c, 1, 0, 1.0),
        force: [1000.0, 0.0, 0.0],
    }];
    let r = run(&c, &study, mats).unwrap();
    assert!(r.ties > 10, "the touching faces are tied: {}", r.ties);
    // F L / (E A) = 1000 · 20 / (210e3 · 50)
    let want = 1000.0 * 20.0 / (210e3 * 50.0);
    let u = field(&r, "Displacement");
    let top = u.iter().cloned().fold(0.0, f64::max);
    assert!((top - want).abs() < 0.04 * want, "{top} vs {want}");
    assert!(r.body_of_node.contains(&0) && r.body_of_node.contains(&1));
    assert!(field(&r, "Safety factor").iter().all(|s| *s > 1.0));
    // the stress is F / A = 20 MPa in the middle of both blocks
    let vm = field(&r, "Von Mises");
    let mid: Vec<f64> = r
        .nodes
        .iter()
        .enumerate()
        .filter(|(_, p)| (p[0] - 5.0).abs() < 1e-6 || (p[0] - 15.0).abs() < 1e-6)
        .map(|(i, _)| vm[i])
        .collect();
    assert!(mid.len() > 10);
    assert!(mid.iter().all(|s| (s - 20.0).abs() < 2.0), "{mid:?}");
}

#[test]
fn blocks_that_overlap_are_refused_and_blocks_that_are_apart_are_not_bonded() {
    let mut p = blocks(&[(0.0, 10.0), (8.0, 18.0)]);
    let c = combined(&mut p);
    let (mut study, mats) = study_with(&c, &["Steel"]);
    study.fixed_tags = vec![end_face(&c, 0, 0, -1.0)];
    study.loads = vec![FeaLoad::Gravity {
        g: [0.0, 0.0, -9810.0],
    }];
    let err = run(&c, &study, mats).unwrap_err();
    assert!(err.contains("overlap"), "{err}");
    // a gap: two free-standing bodies; the second is held by its own support and finds no neighbour
    let mut p = blocks(&[(0.0, 10.0), (12.0, 22.0)]);
    let c = combined(&mut p);
    let (mut study, mats) = study_with(&c, &["Steel"]);
    study.fixed_tags = vec![end_face(&c, 0, 0, -1.0), end_face(&c, 1, 0, 1.0)];
    study.loads = vec![FeaLoad::Gravity {
        g: [0.0, 0.0, -9810.0],
    }];
    let r = run(&c, &study, mats).unwrap();
    assert_eq!(r.ties, 0);
}

#[test]
fn heat_crosses_the_interface_between_two_materials() {
    let mut p = blocks(&[(0.0, 10.0), (10.0, 20.0)]);
    let c = combined(&mut p);
    let (mut study, mats) = study_with(&c, &["Steel", "Aluminium"]);
    study.kind = StudyKind::Thermal;
    study.thermal = vec![
        ThermalBc::Temperature {
            tag: end_face(&c, 0, 0, -1.0),
            t: 100.0,
        },
        ThermalBc::Temperature {
            tag: end_face(&c, 1, 0, 1.0),
            t: 20.0,
        },
    ];
    let r = run(&c, &study, mats).unwrap();
    let t = field(&r, "Temperature");
    // series resistances L/k: steel 10/50, aluminium 10/167
    let (r1, r2) = (10.0 / 50.0, 10.0 / 167.0);
    let interface = 100.0 - 80.0 * r1 / (r1 + r2);
    let at: Vec<f64> = r
        .nodes
        .iter()
        .enumerate()
        .filter(|(_, p)| (p[0] - 10.0).abs() < 1e-6)
        .map(|(i, _)| t[i])
        .collect();
    assert!(at.len() > 10);
    assert!(
        at.iter().all(|v| (v - interface).abs() < 0.012 * interface),
        "{at:?} vs {interface}"
    );
    assert!((t.iter().cloned().fold(f64::MAX, f64::min) - 20.0).abs() < 1e-6);
    assert!((t.iter().cloned().fold(f64::MIN, f64::max) - 100.0).abs() < 1e-6);
    // a thermal study without a condition says what is missing
    study.thermal.clear();
    let (_, mats) = study_with(&c, &["Steel"]);
    assert!(run(&c, &study, mats)
        .unwrap_err()
        .contains("thermal condition"));
}

#[test]
fn a_heated_bar_expands_by_alpha_dt_l() {
    let mut p = blocks(&[(0.0, 10.0), (10.0, 20.0)]);
    let c = combined(&mut p);
    let (mut study, mats) = study_with(&c, &["Steel"]);
    study.kind = StudyKind::ThermalStress;
    // held on three symmetry planes only, so that the bar can grow freely from them
    study.supports = (0..3)
        .map(|axis| Support {
            tag: end_face(&c, 0, axis, -1.0),
            axes: [axis == 0, axis == 1, axis == 2],
            value: [0.0; 3],
        })
        .collect();
    // both ends held at 120 °C: the whole bar is at 120 (reference 20)
    study.thermal = vec![
        ThermalBc::Temperature {
            tag: end_face(&c, 0, 0, -1.0),
            t: 120.0,
        },
        ThermalBc::Temperature {
            tag: end_face(&c, 1, 0, 1.0),
            t: 120.0,
        },
    ];
    let r = run(&c, &study, mats).unwrap();
    let grow = 12e-6 * 100.0 * 20.0;
    // the x displacement of the far end (the magnitude would include the sideways growth)
    let far: Vec<f64> = (0..r.nodes.len())
        .filter(|i| (r.nodes[*i][0] - 20.0).abs() < 1e-6)
        .map(|i| r.u[3 * i])
        .collect();
    let top = far.iter().sum::<f64>() / far.len() as f64;
    assert!((top - grow).abs() < 0.03 * grow, "{top} vs {grow}");
    assert!(field(&r, "Temperature")
        .iter()
        .all(|t| (t - 120.0).abs() < 1e-6));
}

#[test]
fn bonded_blocks_vibrate_like_the_same_body_in_one_piece() {
    let freq = |ranges: &[(f64, f64)]| {
        let mut p = blocks(ranges);
        let c = combined(&mut p);
        let (mut study, mats) = study_with(&c, &["Steel"]);
        study.kind = StudyKind::Modal;
        study.modes = 3;
        study.max_size = 3.5;
        study.fixed_tags = vec![end_face(&c, 0, 0, -1.0)];
        let r = run(&c, &study, mats).unwrap();
        assert_eq!(r.modes.len(), 3);
        assert!(r.modes.windows(2).all(|w| w[0].frequency <= w[1].frequency));
        r.modes[0].frequency
    };
    let one = freq(&[(0.0, 20.0)]);
    let two = freq(&[(0.0, 10.0), (10.0, 20.0)]);
    assert!(one > 1000.0, "{one} Hz");
    assert!(
        (two - one).abs() < 0.04 * one,
        "{two} Hz vs {one} Hz in one piece"
    );
}

#[test]
fn a_nonlinear_study_yields_and_a_cancelled_one_stops() {
    let mut p = blocks(&[(0.0, 10.0), (10.0, 20.0)]);
    let c = combined(&mut p);
    let (mut study, mats) = study_with(&c, &["Steel"]);
    study.kind = StudyKind::Nonlinear;
    study.nl_geometric = false;
    study.nl_steps = 6;
    study.max_size = 5.0;
    study.fixed_tags = vec![end_face(&c, 0, 0, -1.0)];
    // 15 kN on 50 mm²: 300 MPa, above the 250 MPa yield strength
    study.loads = vec![FeaLoad::Force {
        tag: end_face(&c, 1, 0, 1.0),
        force: [15000.0, 0.0, 0.0],
    }];
    let r = run(&c, &study, mats.clone()).expect("nonlinear study runs");
    assert!(!r.curve.is_empty() && (r.curve.last().unwrap()[1] - 1.0).abs() < 1e-9);
    let plastic = field(&r, "Plastic strain")
        .iter()
        .cloned()
        .fold(0.0, f64::max);
    assert!(plastic > 1e-3, "{plastic}");
    let elastic = 15000.0 * 20.0 / (210e3 * 50.0);
    assert!(
        r.max_displacement > 3.0 * elastic,
        "{} vs {elastic}",
        r.max_displacement
    );
    // the load-displacement curve is monotone and bends over after yield
    assert!(r.curve.windows(2).all(|w| w[1][0] >= w[0][0] - 1e-12));
    // cancelled
    let m = StudyModel {
        brep: &c.brep,
        bodies: c.bodies.clone(),
        materials: mats,
    };
    let cancel = CancelFlag::new();
    cancel.cancel();
    assert_eq!(
        run_study(&m, &study, &cancel, &Progress::new()).unwrap_err(),
        "cancelled"
    );
}

#[test]
fn a_roller_support_and_an_empty_study_are_handled() {
    let mut p = blocks(&[(0.0, 10.0)]);
    let c = combined(&mut p);
    let (mut study, mats) = study_with(&c, &["Steel"]);
    assert!(run(&c, &study, mats.clone())
        .unwrap_err()
        .contains("fix at least one face"));
    // supported only against x on the left, and against y and z at a point's worth of faces below
    study.supports = vec![
        Support {
            tag: end_face(&c, 0, 0, -1.0),
            axes: [true, false, false],
            value: [0.0; 3],
        },
        Support {
            tag: end_face(&c, 0, 1, -1.0),
            axes: [false, true, false],
            value: [0.0; 3],
        },
        Support {
            tag: end_face(&c, 0, 2, -1.0),
            axes: [false, false, true],
            value: [0.0; 3],
        },
    ];
    assert!(run(&c, &study, mats.clone())
        .unwrap_err()
        .contains("add a load"));
    study.loads = vec![FeaLoad::Force {
        tag: end_face(&c, 0, 0, 1.0),
        force: [500.0, 0.0, 0.0],
    }];
    let r = run(&c, &study, mats).unwrap();
    // uniform strain F / (E A): the free end moves 500 · 10 / (210e3 · 50)
    let want = 500.0 * 10.0 / (210e3 * 50.0);
    let top = (0..r.nodes.len()).map(|i| r.u[3 * i]).fold(0.0, f64::max);
    assert!((top - want).abs() < 0.03 * want, "{top} vs {want}");
}
