//! Phase 5 data inside a `.kwf` file, as extra entries next to the model (like `assembly.cbor`):
//! `cam.cbor` (stock, tools and the toolpaths of the operations), `fea.cbor` (material, meshing parameters
//! and boundary conditions by geometry-face tag) and `board.cbor` (the whole printed circuit board).
//!
//! `cam.cbor` and `fea.cbor` are versioned: the current form is a [`CamFile`] / [`FeaFile`] holding a list.
//! The first versions stored one bare [`CamSetup`] / [`FeaStudy`]; they are still read (and become a list of
//! one). Every field added since has a serde default, so older files keep loading, and a damaged entry is
//! reported by the caller instead of stopping the project from opening.

use kw_cam::{Tool, Toolpath};
use kw_drawing::DrawingSet;
use kw_ecad::Board;
use kw_feature::kwf::KwfFile;
use serde::{Deserialize, Serialize};

pub const CAM_ENTRY: &str = "cam.cbor";
pub const FEA_ENTRY: &str = "fea.cbor";
pub const BOARD_ENTRY: &str = "board.cbor";
pub const DRAWING_ENTRY: &str = "drawings.cbor";

fn encode<T: Serialize>(v: &T) -> Vec<u8> {
    let mut out = Vec::new();
    ciborium::into_writer(v, &mut out).expect("serialises");
    out
}

fn decode<T: for<'a> Deserialize<'a>>(b: &[u8]) -> Result<T, String> {
    ciborium::from_reader(b).map_err(|e| e.to_string())
}

/// Which body of the model a study or setup means. Unlike an index it survives edits that add, remove or
/// reorder bodies: a design body is named by the feature that started it, an assembly body by the path of its
/// component (`Instance`, `Outer/Inner`, with `/Body` appended for a part that has several).
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BodyKey {
    Design(kw_feature::FeatureId),
    Assembly(String),
}

/// Where the bodies of a study or setup come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum SourceKind {
    /// The solid bodies of the design.
    #[default]
    Design,
    /// The posed assembly.
    Assembly,
}

/// The version written to `cam.cbor` and `fea.cbor` now.
pub const STORE_VERSION: u32 = 2;

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

/// What a study computes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum StudyKind {
    /// Linear static stress.
    #[default]
    Static,
    /// Natural frequencies and mode shapes.
    Modal,
    /// Steady-state temperature.
    Thermal,
    /// Steady-state temperature, then the stress its thermal expansion causes.
    ThermalStress,
    /// Large displacements and plasticity.
    Nonlinear,
}

impl StudyKind {
    pub const ALL: [StudyKind; 5] = [
        StudyKind::Static,
        StudyKind::Modal,
        StudyKind::Thermal,
        StudyKind::ThermalStress,
        StudyKind::Nonlinear,
    ];

    pub fn name(self) -> &'static str {
        match self {
            StudyKind::Static => "Static stress",
            StudyKind::Modal => "Modal frequencies",
            StudyKind::Thermal => "Thermal",
            StudyKind::ThermalStress => "Thermal stress",
            StudyKind::Nonlinear => "Nonlinear static",
        }
    }
}

/// The constants of a material as a study uses them, kept in the study so that it solves the same
/// everywhere. Catalogue units (MPa, kg/m³, W/(m·K), 1/K, °C).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StudyMaterial {
    pub name: String,
    pub e: f64,
    pub nu: f64,
    /// kg/m³
    pub density: f64,
    #[serde(default)]
    pub yield_strength: f64,
    /// Tangent modulus after yield, MPa.
    #[serde(default)]
    pub hardening_modulus: f64,
    #[serde(default)]
    pub conductivity: f64,
    #[serde(default)]
    pub expansion: f64,
    #[serde(default)]
    pub ref_temperature: f64,
}

/// One body of the model in a study.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StudyBody {
    pub key: BodyKey,
    pub included: bool,
    pub material: StudyMaterial,
}

/// A thermal condition (catalogue units: °C, W/m², W/(m²·K), W/m³).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ThermalBc {
    /// Held at a temperature on the faces with this tag.
    Temperature { tag: u32, t: f64 },
    /// Heat flowing into the body through the faces.
    Flux { tag: u32, q: f64 },
    /// Convection to the ambient temperature `t_inf`.
    Convection { tag: u32, h: f64, t_inf: f64 },
    /// Heat generated in all the bodies of the study.
    Generation { q: f64 },
}

/// A support on the faces with this tag: the ticked components are held at `value` (0: fixed).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Support {
    pub tag: u32,
    pub axes: [bool; 3],
    #[serde(default)]
    pub value: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeaStudy {
    pub name: String,
    /// Name of the body in the model this study is run on (first versions; see `bodies`).
    pub body: String,
    pub material: kw_fem::Material,
    pub max_size: f64,
    pub radius_edge: f64,
    /// Faces held fixed (all components). Tags count the faces of the included bodies in order: the faces
    /// of the first body, then those of the second, and so on.
    pub fixed_tags: Vec<u32>,
    pub loads: Vec<FeaLoad>,
    #[serde(default)]
    pub kind: StudyKind,
    #[serde(default)]
    pub source: SourceKind,
    /// The bodies of the study, with a material each. Empty: every body, all of `material`.
    #[serde(default)]
    pub bodies: Vec<StudyBody>,
    /// Largest gap that still counts as touching (mm); 0 takes a small default.
    #[serde(default)]
    pub bond_tolerance: f64,
    /// Pairs of bodies that touch but are not bonded.
    #[serde(default)]
    pub unbonded: Vec<(BodyKey, BodyKey)>,
    /// How many modes a modal study finds (0: six).
    #[serde(default)]
    pub modes: usize,
    #[serde(default)]
    pub thermal: Vec<ThermalBc>,
    /// Supports with chosen components or prescribed displacements (besides `fixed_tags`).
    #[serde(default)]
    pub supports: Vec<Support>,
    /// Nonlinear: load steps to start with, and large displacements on or off.
    #[serde(default)]
    pub nl_steps: usize,
    #[serde(default = "yes")]
    pub nl_geometric: bool,
}

fn yes() -> bool {
    true
}

impl Default for FeaStudy {
    fn default() -> Self {
        FeaStudy {
            name: "Static study".into(),
            body: String::new(),
            material: kw_fem::Material {
                e: 210e3,
                nu: 0.30,
                rho: 7.85e-9,
            },
            max_size: 5.0,
            radius_edge: 2.0,
            fixed_tags: Vec::new(),
            loads: Vec::new(),
            kind: StudyKind::Static,
            source: SourceKind::Design,
            bodies: Vec::new(),
            bond_tolerance: 0.0,
            unbonded: Vec::new(),
            modes: 0,
            thermal: Vec::new(),
            supports: Vec::new(),
            nl_steps: 0,
            nl_geometric: true,
        }
    }
}

/// `cam.cbor` as written now: the setups of the project.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CamFile {
    pub version: u32,
    pub setups: Vec<CamSetup>,
}

/// `fea.cbor` as written now: the studies of the project.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeaFile {
    pub version: u32,
    pub studies: Vec<FeaStudy>,
}

/// Read a versioned entry, or the bare struct of the first versions (wrapped by `legacy`).
fn read_versioned<N, L>(bytes: &[u8], wrap_legacy: impl FnOnce(L) -> N) -> Result<N, String>
where
    N: for<'a> Deserialize<'a>,
    L: for<'a> Deserialize<'a>,
{
    match decode::<N>(bytes) {
        Ok(n) => Ok(n),
        Err(new_err) => decode::<L>(bytes).map(wrap_legacy).map_err(|_| new_err),
    }
}

/// Store all the setups of the project in `cam.cbor`.
pub fn attach_cam_file(file: &mut KwfFile, v: &CamFile) {
    file.extra_entries.insert(CAM_ENTRY.into(), encode(v));
}

/// The setups stored in the file (a file of the first version has its single setup).
pub fn cam_file_from_file(file: &KwfFile) -> Option<Result<CamFile, String>> {
    file.extra_entries.get(CAM_ENTRY).map(|b| {
        read_versioned(b, |s: CamSetup| CamFile {
            version: STORE_VERSION,
            setups: vec![s],
        })
    })
}

/// Store all the studies of the project in `fea.cbor`.
pub fn attach_fea_file(file: &mut KwfFile, v: &FeaFile) {
    file.extra_entries.insert(FEA_ENTRY.into(), encode(v));
}

/// The studies stored in the file (a file of the first version has its single study).
pub fn fea_file_from_file(file: &KwfFile) -> Option<Result<FeaFile, String>> {
    file.extra_entries.get(FEA_ENTRY).map(|b| {
        read_versioned(b, |s: FeaStudy| FeaFile {
            version: STORE_VERSION,
            studies: vec![s],
        })
    })
}

/// Store one setup (as a file of one).
pub fn attach_cam(file: &mut KwfFile, v: &CamSetup) {
    attach_cam_file(
        file,
        &CamFile {
            version: STORE_VERSION,
            setups: vec![v.clone()],
        },
    );
}

/// The first setup stored in the file.
pub fn cam_from_file(file: &KwfFile) -> Option<Result<CamSetup, String>> {
    cam_file_from_file(file).map(|r| {
        r.and_then(|f| {
            f.setups
                .into_iter()
                .next()
                .ok_or_else(|| "the file holds no CAM setup".to_string())
        })
    })
}

/// Store one study (as a file of one).
pub fn attach_fea(file: &mut KwfFile, v: &FeaStudy) {
    attach_fea_file(
        file,
        &FeaFile {
            version: STORE_VERSION,
            studies: vec![v.clone()],
        },
    );
}

/// The first study stored in the file.
pub fn fea_from_file(file: &KwfFile) -> Option<Result<FeaStudy, String>> {
    fea_file_from_file(file).map(|r| {
        r.and_then(|f| {
            f.studies
                .into_iter()
                .next()
                .ok_or_else(|| "the file holds no FEA study".to_string())
        })
    })
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

entry!(Board, BOARD_ENTRY, attach_board, board_from_file);
entry!(
    DrawingSet,
    DRAWING_ENTRY,
    attach_drawings,
    drawings_from_file
);
