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

/// The first versions stored one bare struct; these are their shapes, written here so that the files they
/// made can be rebuilt without the old code.
mod first_version {
    use serde::Serialize;

    #[derive(Serialize)]
    pub struct Operation {
        pub name: String,
        pub tool: u32,
        pub path: kw_cam::Toolpath,
    }

    #[derive(Serialize)]
    pub struct Setup {
        pub machine: String,
        pub stock_min: [f64; 3],
        pub stock_max: [f64; 3],
        pub tools: Vec<kw_cam::Tool>,
        pub operations: Vec<Operation>,
    }

    #[derive(Serialize)]
    #[allow(dead_code)]
    pub enum Load {
        Force { tag: u32, force: [f64; 3] },
        Pressure { tag: u32, pressure: f64 },
        Gravity { g: [f64; 3] },
    }

    #[derive(Serialize)]
    pub struct Study {
        pub name: String,
        pub body: String,
        pub material: kw_fem::Material,
        pub max_size: f64,
        pub radius_edge: f64,
        pub fixed_tags: Vec<u32>,
        pub loads: Vec<Load>,
    }

    pub fn bytes<T: Serialize>(v: &T) -> Vec<u8> {
        let mut out = Vec::new();
        ciborium::into_writer(v, &mut out).unwrap();
        out
    }
}

#[test]
fn entries_of_the_first_version_are_read_as_a_list_of_one() {
    let mut file = KwfFile::new(Document::new());
    let setup = first_version::Setup {
        machine: "3-axis mill".into(),
        stock_min: [0.0; 3],
        stock_max: [10.0, 10.0, 5.0],
        tools: vec![Tool::flat(1, 6.0, 20.0)],
        operations: vec![first_version::Operation {
            name: "Pocket".into(),
            tool: 1,
            path: Toolpath::new("p"),
        }],
    };
    let study = first_version::Study {
        name: "Load".into(),
        body: "Block".into(),
        material: kw_fem::Material {
            e: 200e3,
            nu: 0.3,
            rho: 7.85e-9,
        },
        max_size: 2.0,
        radius_edge: 2.0,
        fixed_tags: vec![1, 2],
        loads: vec![first_version::Load::Pressure {
            tag: 4,
            pressure: 1.5,
        }],
    };
    file.extra_entries
        .insert(CAM_ENTRY.into(), first_version::bytes(&setup));
    file.extra_entries
        .insert(FEA_ENTRY.into(), first_version::bytes(&study));
    let back = KwfFile::from_bytes(&file.to_bytes().unwrap()).unwrap();
    let cam = cam_file_from_file(&back).unwrap().unwrap();
    assert_eq!((cam.version, cam.setups.len()), (STORE_VERSION, 1));
    assert_eq!(cam.setups[0].operations[0].name, "Pocket");
    assert_eq!(cam_from_file(&back).unwrap().unwrap(), cam.setups[0]);
    let fea = fea_file_from_file(&back).unwrap().unwrap();
    assert_eq!((fea.version, fea.studies.len()), (STORE_VERSION, 1));
    assert_eq!(fea.studies[0].fixed_tags, vec![1, 2]);
    // saving writes the current form, which reads back the same
    let mut again = KwfFile::new(Document::new());
    attach_cam_file(&mut again, &cam);
    attach_fea_file(&mut again, &fea);
    let back = KwfFile::from_bytes(&again.to_bytes().unwrap()).unwrap();
    assert_eq!(cam_file_from_file(&back).unwrap().unwrap(), cam);
    assert_eq!(fea_file_from_file(&back).unwrap().unwrap(), fea);
}

#[test]
fn a_damaged_entry_is_an_error_to_report_not_a_panic() {
    let mut file = KwfFile::new(Document::new());
    file.extra_entries
        .insert(CAM_ENTRY.into(), b"not cbor at all".to_vec());
    file.extra_entries.insert(FEA_ENTRY.into(), vec![0xa0]); // an empty map: neither form
    assert!(cam_file_from_file(&file).unwrap().is_err());
    assert!(fea_file_from_file(&file).unwrap().is_err());
    assert!(cam_from_file(&file).unwrap().is_err());
}
