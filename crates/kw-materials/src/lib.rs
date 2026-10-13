//! The material library shared by mass properties, FEA and CAM.
//!
//! A [`MaterialDef`] is one material with everything the other crates need by name: density (mass,
//! bill of materials, gravity), elastic and plastic constants and thermal constants (FEA), and a machining
//! group with cutting data (CAM). Bodies and parts only store the *name*; the definition is looked up in a
//! [`Library`]: the built-in table, then the user's own `materials.json` (which may add materials or
//! replace a built-in one of the same name). A project embeds a copy of every definition it uses
//! (`kw-store`), so a file opened on another machine keeps its numbers.
//!
//! Units are the usual catalogue ones: kg/m³, MPa, W/(m·K), J/(kg·K), 1/K, °C. The `to_fem` helpers give
//! the millimetre / newton / tonne / second system the FEA works in.

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

/// What a material machines like.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MaterialGroup {
    Aluminium,
    Steel,
    StainlessSteel,
    Titanium,
    /// Brass, bronze and copper.
    Copper,
    Plastic,
    Wood,
}

impl MaterialGroup {
    pub const ALL: [MaterialGroup; 7] = [
        MaterialGroup::Aluminium,
        MaterialGroup::Steel,
        MaterialGroup::StainlessSteel,
        MaterialGroup::Titanium,
        MaterialGroup::Copper,
        MaterialGroup::Plastic,
        MaterialGroup::Wood,
    ];

    pub fn name(self) -> &'static str {
        match self {
            MaterialGroup::Aluminium => "Aluminium",
            MaterialGroup::Steel => "Steel",
            MaterialGroup::StainlessSteel => "Stainless steel",
            MaterialGroup::Titanium => "Titanium",
            MaterialGroup::Copper => "Brass / copper",
            MaterialGroup::Plastic => "Plastic",
            MaterialGroup::Wood => "Wood",
        }
    }
}

/// Cutting data for a material (carbide tools): the surface speed range and the feed per tooth as a
/// fraction of the tool diameter.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CamData {
    pub group: MaterialGroup,
    /// Surface speed range, m/min.
    pub surface_speed: (f64, f64),
    /// Feed per tooth divided by the tool diameter.
    pub chip_load_factor: f64,
}

impl CamData {
    pub fn for_group(group: MaterialGroup) -> CamData {
        let (surface_speed, chip_load_factor) = match group {
            MaterialGroup::Aluminium => ((300.0, 600.0), 0.015),
            MaterialGroup::Steel => ((100.0, 200.0), 0.006),
            MaterialGroup::StainlessSteel => ((60.0, 120.0), 0.005),
            MaterialGroup::Titanium => ((40.0, 80.0), 0.004),
            MaterialGroup::Copper => ((150.0, 300.0), 0.010),
            MaterialGroup::Plastic => ((200.0, 500.0), 0.020),
            MaterialGroup::Wood => ((400.0, 1000.0), 0.020),
        };
        CamData {
            group,
            surface_speed,
            chip_load_factor,
        }
    }

    /// The surface speed to use: the middle of the range.
    pub fn nominal_speed(&self) -> f64 {
        0.5 * (self.surface_speed.0 + self.surface_speed.1)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MaterialDef {
    pub name: String,
    /// kg/m³
    pub density: f64,
    /// Young's modulus, MPa.
    pub e: f64,
    pub nu: f64,
    /// Yield strength, MPa.
    #[serde(default)]
    pub yield_strength: f64,
    /// Tangent modulus after yield (linear isotropic hardening), MPa.
    #[serde(default)]
    pub hardening_modulus: f64,
    /// Thermal conductivity, W/(m·K).
    #[serde(default)]
    pub conductivity: f64,
    /// Specific heat, J/(kg·K).
    #[serde(default)]
    pub specific_heat: f64,
    /// Coefficient of thermal expansion, 1/K.
    #[serde(default)]
    pub expansion: f64,
    /// Temperature at which the material is stress free, °C.
    #[serde(default = "room_temperature")]
    pub ref_temperature: f64,
    #[serde(default = "default_cam")]
    pub cam: CamData,
}

fn room_temperature() -> f64 {
    20.0
}

fn default_cam() -> CamData {
    CamData::for_group(MaterialGroup::Steel)
}

impl MaterialDef {
    /// Density in tonne/mm³, the FEA's mass unit.
    pub fn rho_fem(&self) -> f64 {
        self.density * 1e-12
    }

    /// Thermal conductivity in the FEA's units (mW/(mm·K), numerically W/(m·K)).
    pub fn conductivity_fem(&self) -> f64 {
        self.conductivity
    }

    /// A heat transfer coefficient given in W/(m²·K) in mW/(mm²·K).
    pub fn convection_fem(h_w_m2k: f64) -> f64 {
        h_w_m2k * 1e-3
    }

    /// A heat flux given in W/m² in mW/mm².
    pub fn flux_fem(q_w_m2: f64) -> f64 {
        q_w_m2 * 1e-3
    }

    /// Volumetric heat generation given in W/m³ in mW/mm³.
    pub fn generation_fem(q_w_m3: f64) -> f64 {
        q_w_m3 * 1e-6
    }
}

#[allow(clippy::too_many_arguments)]
fn def(
    name: &str,
    density: f64,
    e: f64,
    nu: f64,
    yield_strength: f64,
    hardening_modulus: f64,
    conductivity: f64,
    specific_heat: f64,
    expansion: f64,
    group: MaterialGroup,
) -> MaterialDef {
    MaterialDef {
        name: name.into(),
        density,
        e,
        nu,
        yield_strength,
        hardening_modulus,
        conductivity,
        specific_heat,
        expansion,
        ref_temperature: 20.0,
        cam: CamData::for_group(group),
    }
}

/// The built-in materials, in the order they are offered.
pub fn builtin() -> Vec<MaterialDef> {
    use MaterialGroup as G;
    vec![
        def(
            "Steel",
            7850.0,
            210e3,
            0.30,
            250.0,
            2.1e3,
            50.0,
            490.0,
            12e-6,
            G::Steel,
        ),
        def(
            "Stainless steel",
            8000.0,
            193e3,
            0.29,
            215.0,
            2.0e3,
            16.0,
            500.0,
            17e-6,
            G::StainlessSteel,
        ),
        def(
            "Aluminium",
            2700.0,
            70e3,
            0.33,
            276.0,
            0.7e3,
            167.0,
            896.0,
            23.6e-6,
            G::Aluminium,
        ),
        def(
            "Brass",
            8500.0,
            100e3,
            0.34,
            200.0,
            1.0e3,
            120.0,
            380.0,
            19e-6,
            G::Copper,
        ),
        def(
            "Copper",
            8960.0,
            117e3,
            0.34,
            70.0,
            1.0e3,
            390.0,
            385.0,
            17e-6,
            G::Copper,
        ),
        def(
            "Titanium",
            4500.0,
            110e3,
            0.34,
            880.0,
            1.1e3,
            7.0,
            520.0,
            8.6e-6,
            G::Titanium,
        ),
        def(
            "ABS plastic",
            1050.0,
            2.3e3,
            0.35,
            40.0,
            0.2e3,
            0.17,
            1400.0,
            90e-6,
            G::Plastic,
        ),
        def(
            "PLA",
            1240.0,
            3.5e3,
            0.36,
            50.0,
            0.3e3,
            0.13,
            1800.0,
            68e-6,
            G::Plastic,
        ),
        def(
            "Nylon",
            1150.0,
            2.7e3,
            0.39,
            70.0,
            0.3e3,
            0.25,
            1700.0,
            80e-6,
            G::Plastic,
        ),
        def(
            "Oak wood",
            750.0,
            11e3,
            0.35,
            40.0,
            0.5e3,
            0.17,
            2400.0,
            5e-6,
            G::Wood,
        ),
    ]
}

/// The materials by name: the built-in table with the user's additions and replacements.
#[derive(Debug, Clone, PartialEq)]
pub struct Library {
    materials: Vec<MaterialDef>,
}

impl Library {
    /// Only the built-in materials.
    pub fn builtin() -> Library {
        Library {
            materials: builtin(),
        }
    }

    /// The built-in materials with those of a user file (JSON: a list of materials) laid over them: a
    /// material of a built-in name replaces it, the others are added.
    pub fn with_user_json(json: &str) -> Result<Library, String> {
        let user: Vec<MaterialDef> = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let mut lib = Library::builtin();
        for m in user {
            lib.set(m);
        }
        Ok(lib)
    }

    /// Add a material, or replace the one of the same name.
    pub fn set(&mut self, m: MaterialDef) {
        match self.materials.iter_mut().find(|x| x.name == m.name) {
            Some(slot) => *slot = m,
            None => self.materials.push(m),
        }
    }

    pub fn all(&self) -> &[MaterialDef] {
        &self.materials
    }

    pub fn find(&self, name: &str) -> Option<&MaterialDef> {
        self.materials.iter().find(|m| m.name == name)
    }

    pub fn names(&self) -> Vec<&str> {
        self.materials.iter().map(|m| m.name.as_str()).collect()
    }

    /// Density (kg/m³) of a material by name; 0 when there is none (its mass is then not known).
    pub fn density_of(&self, material: Option<&str>) -> f64 {
        material
            .and_then(|m| self.find(m))
            .map_or(0.0, |m| m.density)
    }

    /// Where the user's materials are kept: `$XDG_CONFIG_HOME/kerfwright/materials.json`, or
    /// `~/.config/kerfwright/materials.json`.
    pub fn user_path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("kerfwright").join("materials.json"))
    }

    /// The built-in materials with the user's file laid over them. A missing file is not an error; a
    /// damaged one is reported and the built-in table is used.
    pub fn load(path: Option<&Path>) -> (Library, Option<String>) {
        let Some(path) = path else {
            return (Library::builtin(), None);
        };
        match std::fs::read_to_string(path) {
            Err(_) => (Library::builtin(), None),
            Ok(text) => match Library::with_user_json(&text) {
                Ok(l) => (l, None),
                Err(e) => (Library::builtin(), Some(format!("{}: {e}", path.display()))),
            },
        }
    }

    /// The materials that differ from the built-in table, as JSON for the user's file.
    pub fn user_json(&self) -> String {
        let base = builtin();
        let extra: Vec<&MaterialDef> = self
            .materials
            .iter()
            .filter(|m| base.iter().find(|b| b.name == m.name) != Some(*m))
            .collect();
        serde_json::to_string_pretty(&extra).expect("materials serialise")
    }
}

/// The library of this run: built-in plus the user's file, read once.
pub fn global() -> &'static Library {
    static LIB: OnceLock<Library> = OnceLock::new();
    LIB.get_or_init(|| Library::load(Library::user_path().as_deref()).0)
}
