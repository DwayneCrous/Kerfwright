//! The `.kwf` container: round trips, three schema versions both ways, damaged input.

mod common;
use common::*;
use kw_feature::kwf::{legacy, KwfError, KwfFile, SCHEMA_VERSION};
use kw_feature::*;
use proptest::prelude::*;
use std::path::PathBuf;

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn hashes(doc: &Document) -> (u128, u128, u128) {
    let ev = Engine::with_threads(2).evaluate(doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    let (g, n) = ev.final_hashes().unwrap();
    (g, n, ev.document_hash())
}

fn hex3(h: (u128, u128, u128)) -> String {
    format!("{:032x} {:032x} {:032x}", h.0, h.1, h.2)
}

/// Rewrite the checked-in fixtures: `KW_REGEN_FIXTURES=1 cargo test -p kw-feature --test kwf regenerate`.
#[test]
fn regenerate_fixtures() {
    if std::env::var("KW_REGEN_FIXTURES").is_err() {
        return;
    }
    let doc = sample_document();
    std::fs::create_dir_all(fixture_dir()).unwrap();
    for v in 1..SCHEMA_VERSION {
        std::fs::write(
            fixture_dir().join(format!("sample_v{v}.kwf")),
            legacy::write(&doc, v).unwrap(),
        )
        .unwrap();
    }
    KwfFile::new(doc.clone())
        .save(&fixture_dir().join(format!("sample_v{SCHEMA_VERSION}.kwf")))
        .unwrap();
    std::fs::write(
        fixture_dir().join("sample_future.kwf"),
        legacy::write_future(&doc, SCHEMA_VERSION).unwrap(),
    )
    .unwrap();
    std::fs::write(
        fixture_dir().join("sample_too_new.kwf"),
        legacy::write_future(&doc, SCHEMA_VERSION + 5).unwrap(),
    )
    .unwrap();
    std::fs::write(fixture_dir().join("sample.hash"), hex3(hashes(&doc))).unwrap();
}

fn expected() -> String {
    std::fs::read_to_string(fixture_dir().join("sample.hash"))
        .expect("fixtures missing: run with KW_REGEN_FIXTURES=1")
}

#[test]
fn every_schema_version_reads_to_the_same_model() {
    for v in 1..=SCHEMA_VERSION {
        let f = KwfFile::load(&fixture_dir().join(format!("sample_v{v}.kwf"))).unwrap();
        assert_eq!(f.manifest.schema_version, v);
        assert_eq!(
            f.notes.iter().filter(|n| n.starts_with("migrated")).count(),
            (SCHEMA_VERSION - v) as usize,
            "{:?}",
            f.notes
        );
        assert_eq!(hex3(hashes(&f.document)), expected(), "schema {v}");
    }
}

#[test]
fn a_newer_writers_file_is_read_and_its_unknown_fields_survive() {
    let f = KwfFile::load(&fixture_dir().join("sample_future.kwf")).unwrap();
    assert!(
        f.notes.iter().any(|n| n.contains("newer version")),
        "{:?}",
        f.notes
    );
    assert!(f.extra_entries.contains_key("cache/outputs.bin"));
    assert_eq!(hex3(hashes(&f.document)), expected());
    // saved again by this version: the unknown fields and the cache entry are still there
    let again = KwfFile::from_bytes(&f.to_bytes().unwrap()).unwrap();
    let text = format!("{:?}", again.raw_recipe().unwrap());
    assert!(text.contains("future_document_field"));
    assert!(text.contains("future_feature_field"));
    assert!(again.extra_entries.contains_key("cache/outputs.bin"));
    assert_eq!(again.document, f.document);
}

#[test]
fn a_file_that_needs_a_newer_reader_is_refused_cleanly() {
    let err = KwfFile::load(&fixture_dir().join("sample_too_new.kwf")).unwrap_err();
    assert!(matches!(err, KwfError::TooNew { .. }), "{err}");
    assert!(err.to_string().contains("newer"));
}

#[test]
fn round_trip_is_exact_and_deterministic() {
    let doc = sample_document();
    let a = KwfFile::new(doc.clone()).to_bytes().unwrap();
    let b = KwfFile::new(doc.clone()).to_bytes().unwrap();
    assert_eq!(a, b, "the same document must give the same bytes");
    let back = KwfFile::from_bytes(&a).unwrap();
    assert_eq!(back.document, doc);
    assert_eq!(hashes(&back.document), hashes(&doc));
    // and again through the file system
    let dir = std::env::temp_dir().join(format!("kwf-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let p = dir.join("m.kwf");
    KwfFile::new(doc.clone()).save(&p).unwrap();
    assert_eq!(KwfFile::load(&p).unwrap().document, doc);
    let _ = std::fs::remove_file(&p);
    let _ = std::fs::remove_dir(&dir);
}

#[test]
fn damage_is_detected() {
    let bytes = KwfFile::new(sample_document()).to_bytes().unwrap();
    // not a zip
    assert!(matches!(
        KwfFile::from_bytes(b"hello"),
        Err(KwfError::Zip(_))
    ));
    assert!(KwfFile::from_bytes(&[]).is_err());
    // truncated
    assert!(KwfFile::from_bytes(&bytes[..bytes.len() / 2]).is_err());
    // a recipe altered without updating the manifest hash is reported as damage
    let mut zr = zip::ZipArchive::new(std::io::Cursor::new(bytes.clone())).unwrap();
    let mut manifest = String::new();
    std::io::Read::read_to_string(&mut zr.by_name("manifest.json").unwrap(), &mut manifest)
        .unwrap();
    let mut recipe = Vec::new();
    std::io::Read::read_to_end(&mut zr.by_name("recipe.cbor").unwrap(), &mut recipe).unwrap();
    let mid = recipe.len() / 2;
    recipe[mid] ^= 0x55;
    let mut zw = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    let o = zip::write::SimpleFileOptions::default();
    std::io::Write::write_all(
        &mut {
            zw.start_file("manifest.json", o).unwrap();
            &mut zw
        },
        manifest.as_bytes(),
    )
    .unwrap();
    zw.start_file("recipe.cbor", o).unwrap();
    std::io::Write::write_all(&mut zw, &recipe).unwrap();
    let broken = zw.finish().unwrap().into_inner();
    let err = KwfFile::from_bytes(&broken).unwrap_err();
    assert!(matches!(err, KwfError::Corrupt(_)), "{err}");
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(400))]

    /// Arbitrary bytes and mutated real files never panic the reader.
    #[test]
    fn reader_never_panics(noise in proptest::collection::vec(any::<u8>(), 0..400),
                           flips in proptest::collection::vec((any::<prop::sample::Index>(), any::<u8>()), 0..12),
                           cut in any::<prop::sample::Index>()) {
        let _ = KwfFile::from_bytes(&noise);
        let mut bytes = KwfFile::new(sample_small()).to_bytes().unwrap();
        for (i, b) in &flips {
            let k = i.index(bytes.len());
            bytes[k] ^= *b;
        }
        let end = cut.index(bytes.len() + 1);
        let _ = KwfFile::from_bytes(&bytes[..end]);
    }
}

fn sample_small() -> Document {
    let mut d = Document::new();
    d.add_parameter("a", mm(1.0)).unwrap();
    let s = d
        .add_rect_sketch(
            "S",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(2.0),
            mm(3.0),
        )
        .unwrap();
    d.add_extrude("E", s, mm(1.0), BodyOp::New).unwrap();
    d
}
