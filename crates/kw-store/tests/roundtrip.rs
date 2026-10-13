//! Phase 5 entries survive a save / load of the `.kwf` container next to the model.

use kw_cam::path::Event;
use kw_cam::{Tool, Toolpath};
use kw_ecad::bench_gen::{benchmark_specs, generate};
use kw_feature::kwf::KwfFile;
use kw_feature::Document;
use kw_store::*;

#[test]
fn cam_fea_and_board_entries_round_trip_through_a_kwf_file() {
    let mut file = KwfFile::new(Document::new());
    let mut path = Toolpath::new("rough");
    path.push(Event::Rapid { p: [0.0, 0.0, 5.0] });
    path.push(Event::Feed {
        p: [10.0, 0.0, -2.0],
        feed: 600.0,
    });
    path.push(Event::Arc {
        p: [10.0, 10.0, -2.0],
        centre: [5.0, 5.0],
        ccw: true,
        feed: 600.0,
    });
    let cam = CamSetup {
        machine: "AC trunnion table".into(),
        stock_min: [-5.0, -5.0, -10.0],
        stock_max: [55.0, 45.0, 0.0],
        tools: vec![Tool::flat(1, 6.0, 20.0), Tool::ball(2, 4.0, 12.0)],
        operations: vec![CamOperation {
            name: "Rough".into(),
            tool: 1,
            path,
        }],
    };
    let fea = FeaStudy {
        name: "Bracket load".into(),
        body: "Bracket".into(),
        material: kw_fem::Material {
            e: 70e3,
            nu: 0.33,
            rho: 2.7e-9,
        },
        max_size: 2.0,
        radius_edge: 2.0,
        fixed_tags: vec![3],
        loads: vec![
            FeaLoad::Force {
                tag: 5,
                force: [0.0, -100.0, 0.0],
            },
            FeaLoad::Gravity {
                g: [0.0, 0.0, -9810.0],
            },
        ],
    };
    let (board, _) = generate(&benchmark_specs()[0]);
    attach_cam(&mut file, &cam);
    attach_fea(&mut file, &fea);
    attach_board(&mut file, &board);
    let bytes = file.to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap();
    assert_eq!(cam_from_file(&back).unwrap().unwrap(), cam);
    assert_eq!(fea_from_file(&back).unwrap().unwrap(), fea);
    assert_eq!(board_from_file(&back).unwrap().unwrap(), board);
    // a file without the entries has none
    let empty = KwfFile::new(Document::new());
    assert!(
        cam_from_file(&empty).is_none()
            && fea_from_file(&empty).is_none()
            && board_from_file(&empty).is_none()
    );
}

#[test]
fn drawings_round_trip_through_a_kwf_file_and_old_files_have_none() {
    use kw_drawing::{DrawingSet, PaperSize, SheetFormat};
    let mut file = KwfFile::new(Document::new());
    assert!(drawings_from_file(&file).is_none());
    let mut set = DrawingSet::default();
    let i = set.add_sheet("Sheet 1", SheetFormat::iso(PaperSize::A3));
    set.sheets[i].title.title = "Bracket".into();
    attach_drawings(&mut file, &set);
    let bytes = file.to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap();
    assert_eq!(drawings_from_file(&back).unwrap().unwrap(), set);
    // saving is deterministic
    assert_eq!(bytes, back.to_bytes().unwrap());
}
