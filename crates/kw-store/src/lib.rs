//! Phase 5 data inside a `.kwf` file, as extra entries next to the model (like `assembly.cbor`):
//! `cam.cbor` (stock, tools and the toolpaths of the operations), `fea.cbor` (material, meshing parameters
//! and boundary conditions by geometry-face tag) and `board.cbor` (the whole printed circuit board).

use kw_cam::{Tool, Toolpath};
use kw_ecad::Board;
use kw_feature::kwf::KwfFile;
use serde::{Deserialize, Serialize};

pub const CAM_ENTRY: &str = "cam.cbor";
pub const FEA_ENTRY: &str = "fea.cbor";
pub const BOARD_ENTRY: &str = "board.cbor";

fn encode<T: Serialize>(v: &T) -> Vec<u8> {
    let mut out = Vec::new();
    ciborium::into_writer(v, &mut out).expect("serialises");
    out
}

fn decode<T: for<'a> Deserialize<'a>>(b: &[u8]) -> Result<T, String> {
    ciborium::from_reader(b).map_err(|e| e.to_string())
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CamOperation {
    pub name: String,
    pub tool: u32,
    pub path: Toolpath,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CamSetup {
    pub machine: String,
    pub stock_min: [f64; 3],
    pub stock_max: [f64; 3],
    pub tools: Vec<Tool>,
    pub operations: Vec<CamOperation>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FeaLoad {
    /// Total force on the faces with this tag.
    Force {
        tag: u32,
        force: [f64; 3],
    },
    Pressure {
        tag: u32,
        pressure: f64,
    },
    Gravity {
        g: [f64; 3],
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeaStudy {
    pub name: String,
    /// Name of the body in the model this study is run on.
    pub body: String,
    pub material: kw_fem::Material,
    pub max_size: f64,
    pub radius_edge: f64,
    /// Faces held fixed (all components).
    pub fixed_tags: Vec<u32>,
    pub loads: Vec<FeaLoad>,
}

macro_rules! entry {
    ($ty:ty, $name:expr, $attach:ident, $get:ident) => {
        pub fn $attach(file: &mut KwfFile, v: &$ty) {
            file.extra_entries.insert($name.into(), encode(v));
        }
        pub fn $get(file: &KwfFile) -> Option<Result<$ty, String>> {
            file.extra_entries.get($name).map(|b| decode(b))
        }
    };
}

entry!(CamSetup, CAM_ENTRY, attach_cam, cam_from_file);
entry!(FeaStudy, FEA_ENTRY, attach_fea, fea_from_file);
entry!(Board, BOARD_ENTRY, attach_board, board_from_file);
