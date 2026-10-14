//! Supports and loads that keep their faces when the model changes, and studies in a list.

use kw_app::fea::FeaWorkspace;
use kw_app::fea_faces::{face_signatures, find_face, is_missing};
use kw_app::project::Project;
use kw_store::{FeaLoad, StudyKind};
use kw_tess::{tessellate_body, TessOptions};

fn mm(v: f64) -> kw_expr::Param {
    kw_expr::Param::length(v)
}

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

/// The tag, among the faces of the workspace's bodies, of the face of body `body` furthest along `axis`.
fn end_face(ws: &FeaWorkspace, body: usize, axis: usize, sign: f64) -> u32 {
    let c = ws.combined().expect("bodies");
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

#[test]
fn a_support_follows_its_face_when_the_numbering_changes() {
    let mut p = blocks(&[(0.0, 10.0), (10.0, 20.0)]);
    let mut ws = FeaWorkspace::default();
    ws.sync_model(&mut p);
    let far = end_face(&ws, 1, 0, 1.0);
    ws.fix_face(&mut p, far);
    ws.add_force(&mut p, far, [0.0, 0.0, -50.0]);
    assert_eq!(ws.study.face_sigs.len(), 1, "one face, one signature");
    // leave the first body out: the faces of the second body are now numbered from zero
    ws.study.bodies[0].included = false;
    ws.sync_model(&mut p);
    let now = end_face(&ws, 0, 0, 1.0);
    assert_ne!(now, far);
    assert_eq!(ws.study.fixed_tags, vec![now]);
    assert!(matches!(ws.study.loads[0], FeaLoad::Force { tag, .. } if tag == now));
    assert!(ws.missing_faces().is_empty());
    // and back again
    ws.study.bodies[0].included = true;
    ws.sync_model(&mut p);
    assert_eq!(ws.study.fixed_tags, vec![far]);
    // the project holds the study as it is
    assert_eq!(p.fea[0].fixed_tags, vec![far]);
}

#[test]
fn a_face_that_is_gone_is_reported_and_blocks_the_study() {
    let mut p = blocks(&[(0.0, 10.0), (10.0, 20.0)]);
    let mut ws = FeaWorkspace::default();
    ws.sync_model(&mut p);
    let a = end_face(&ws, 0, 0, -1.0);
    let b = end_face(&ws, 1, 0, 1.0);
    ws.fix_face(&mut p, a);
    ws.add_pressure(&mut p, b, 2.0);
    // the second body leaves the study: its pressure face is gone, the support stays
    ws.study.bodies[1].included = false;
    ws.sync_model(&mut p);
    assert_eq!(ws.missing_faces().len(), 1);
    assert_eq!(ws.study.fixed_tags, vec![a]);
    let tags = ws.study.face_tags();
    assert!(tags.iter().any(|t| is_missing(*t)), "{tags:?}");
    assert!(ws.message.contains("no longer exist"), "{}", ws.message);
    ws.requests.push("solve");
    // (the solve request is carried out when the workspace is drawn; the check is the same call)
    ws.start_study();
    assert!(ws.message.contains("no longer exists"), "{}", ws.message);
    // it is not guessed at: no other face took the number
    assert!(ws.study.loads.iter().all(|l| match l {
        FeaLoad::Pressure { tag, .. } => is_missing(*tag),
        _ => true,
    }));
    // the body comes back: so does the face, with its old tag
    ws.study.bodies[1].included = true;
    ws.sync_model(&mut p);
    assert!(ws.missing_faces().is_empty());
    assert!(matches!(ws.study.loads[0], FeaLoad::Pressure { tag, .. } if tag == b));
    // or the face is dropped for good
    ws.study.bodies[1].included = false;
    ws.sync_model(&mut p);
    let gone: Vec<u32> = ws
        .study
        .face_tags()
        .into_iter()
        .filter(|t| is_missing(*t))
        .collect();
    for t in gone {
        ws.study.drop_face(t);
    }
    assert!(ws.study.loads.is_empty() && ws.study.face_sigs.len() == 1);
}

#[test]
fn signatures_tolerate_small_edits_and_refuse_large_ones() {
    let mut p = blocks(&[(0.0, 20.0)]);
    let mut ws = FeaWorkspace::default();
    ws.sync_model(&mut p);
    let c = ws.combined().unwrap();
    let all = face_signatures(&c);
    assert_eq!(all.len(), 6);
    let (tag, sig) = all
        .iter()
        .find(|(_, s)| s.normal[0] > 1.0)
        .cloned()
        .expect("the +x face");
    assert_eq!(find_face(&all, &sig), Some(tag));
    // the face moved by 1 % of the body's size: still the same face
    let mut near = sig.clone();
    near.centre[0] += 0.01 * sig.scale;
    assert_eq!(find_face(&all, &near), Some(tag));
    // 10 % along: no face is there
    let mut far = sig.clone();
    far.centre[0] -= 0.10 * sig.scale;
    assert_eq!(find_face(&all, &far), None);
    // a different area is a different face
    let mut big = sig.clone();
    big.area *= 1.3;
    assert_eq!(find_face(&all, &big), None);
    // another body's face with the same numbers is not it
    let mut other = sig;
    other.body = kw_store::BodyKey::Assembly("x".into());
    assert_eq!(find_face(&all, &other), None);
}

#[test]
fn studies_are_a_list_that_is_saved_with_the_project() {
    let mut p = blocks(&[(0.0, 10.0), (10.0, 20.0)]);
    let mut ws = FeaWorkspace::default();
    ws.sync_model(&mut p);
    let end = end_face(&ws, 0, 0, -1.0);
    ws.fix_face(&mut p, end);
    ws.study.name = "Static".into();
    ws.new_study(&mut p);
    assert_eq!(p.fea.len(), 2);
    assert_eq!(ws.current, 1);
    assert!(
        ws.study.fixed_tags.is_empty(),
        "a new study has no supports"
    );
    assert_eq!(ws.study.bodies.len(), 2, "but the same bodies");
    ws.study.kind = StudyKind::Modal;
    ws.study.name = "Modes".into();
    ws.fix_face(&mut p, end);
    ws.select_study(&mut p, 0);
    assert_eq!(ws.study.name, "Static");
    assert_eq!(ws.study.kind, StudyKind::Static);
    assert_eq!(p.fea[1].kind, StudyKind::Modal);
    assert_eq!(p.fea[1].fixed_tags, vec![end]);
    // saved and reopened
    let dir = std::env::temp_dir().join(format!("kw_fea_list_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    p.save_as(&dir.join("s.kwf")).unwrap();
    let back = Project::open(&dir.join("s.kwf")).unwrap();
    assert_eq!(back.fea, p.fea);
    assert_eq!(back.fea[1].face_sigs.len(), 1);
    // a workspace on the reopened project shows its first study, and deleting one keeps the other
    let mut back = back;
    let mut ws2 = FeaWorkspace::default();
    ws2.sync_model(&mut back);
    ws2.select_study(&mut back, 1);
    assert_eq!(ws2.study.name, "Modes");
    ws2.delete_study(&mut back, 0);
    assert_eq!(back.fea.len(), 1);
    assert_eq!(ws2.current, 0);
    assert_eq!(ws2.study.name, "Modes");
    ws2.delete_study(&mut back, 0);
    assert_eq!(back.fea.len(), 1, "the last study stays");
    std::fs::remove_dir_all(&dir).ok();
}
