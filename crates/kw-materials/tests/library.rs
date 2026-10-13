use kw_materials::*;

#[test]
fn the_built_in_table_has_what_every_user_of_it_needs() {
    let lib = Library::builtin();
    assert_eq!(lib.all().len(), 10);
    for m in lib.all() {
        assert!(
            m.density > 0.0 && m.e > 0.0 && m.nu > 0.0 && m.nu < 0.5,
            "{}",
            m.name
        );
        assert!(m.yield_strength > 0.0 && m.conductivity > 0.0 && m.specific_heat > 0.0);
        assert!(
            m.hardening_modulus < m.e,
            "{}: the tangent modulus is below E",
            m.name
        );
        assert!(m.cam.surface_speed.0 < m.cam.surface_speed.1);
    }
    // names are unique
    let mut names = lib.names();
    names.sort_unstable();
    names.dedup();
    assert_eq!(names.len(), 10);
    // the mass-property density and the FEA density are the same number
    let steel = lib.find("Steel").unwrap();
    assert_eq!(steel.density, 7850.0);
    assert!((steel.rho_fem() - 7.85e-9).abs() < 1e-15);
    assert_eq!(lib.density_of(Some("Aluminium")), 2700.0);
    assert_eq!(lib.density_of(Some("Unobtainium")), 0.0);
    assert_eq!(lib.density_of(None), 0.0);
}

#[test]
fn a_user_file_adds_and_replaces_materials() {
    let json = r#"[
        {"name": "Steel", "density": 7900.0, "e": 205000.0, "nu": 0.29},
        {"name": "Delrin", "density": 1410.0, "e": 3100.0, "nu": 0.35, "yield_strength": 66.0,
         "cam": {"group": "Plastic", "surface_speed": [200.0, 400.0], "chip_load_factor": 0.02}}
    ]"#;
    let lib = Library::with_user_json(json).unwrap();
    assert_eq!(lib.all().len(), 11);
    // replaced in place (same position), the fields the file leaves out take their defaults
    assert_eq!(lib.all()[0].name, "Steel");
    assert_eq!(lib.find("Steel").unwrap().density, 7900.0);
    assert_eq!(lib.find("Steel").unwrap().yield_strength, 0.0);
    assert_eq!(lib.find("Steel").unwrap().ref_temperature, 20.0);
    assert_eq!(
        lib.find("Delrin").unwrap().cam.group,
        MaterialGroup::Plastic
    );
    // only what differs from the built-in table is written back
    let back = Library::with_user_json(&lib.user_json()).unwrap();
    assert_eq!(back, lib);
    let written: Vec<MaterialDef> = serde_json::from_str(&lib.user_json()).unwrap();
    assert_eq!(written.len(), 2);
    // damage is an error, and `load` falls back to the built-in table with the reason
    assert!(Library::with_user_json("[{").is_err());
}

#[test]
fn a_missing_or_damaged_user_file_leaves_the_built_in_table() {
    let dir = std::env::temp_dir().join(format!("kw-materials-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let missing = dir.join("none.json");
    let (lib, problem) = Library::load(Some(&missing));
    assert!(problem.is_none() && lib == Library::builtin());
    let bad = dir.join("bad.json");
    std::fs::write(&bad, "this is not json").unwrap();
    let (lib, problem) = Library::load(Some(&bad));
    assert!(problem.is_some() && lib == Library::builtin());
    let good = dir.join("good.json");
    std::fs::write(
        &good,
        r#"[{"name":"Foam","density":30.0,"e":10.0,"nu":0.3}]"#,
    )
    .unwrap();
    let (lib, problem) = Library::load(Some(&good));
    assert!(problem.is_none() && lib.find("Foam").is_some());
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn units_convert_to_the_fem_system() {
    // 1 W/m² is 1e-3 mW/mm², 1 W/(m²K) is 1e-3 mW/(mm²K), 1 W/m³ is 1e-6 mW/mm³
    assert_eq!(MaterialDef::flux_fem(1000.0), 1.0);
    assert_eq!(MaterialDef::convection_fem(10.0), 0.01);
    assert!((MaterialDef::generation_fem(1e6) - 1.0).abs() < 1e-12);
    let cu = Library::builtin().find("Copper").unwrap().clone();
    assert_eq!(cu.conductivity_fem(), 390.0);
    // the nominal surface speed sits inside the range
    let c = CamData::for_group(MaterialGroup::Aluminium);
    assert!(c.nominal_speed() > c.surface_speed.0 && c.nominal_speed() < c.surface_speed.1);
}
