//! Determinism: a model rebuilt from its file, without any cache, gives identical geometry and
//! identical names on 1, 4 and 32 threads.

mod common;
use common::*;
use kw_feature::kwf::KwfFile;
use kw_feature::*;

fn run(doc: &Document, threads: usize) -> (u128, Vec<TopoName>, u128, usize) {
    let mut e = Engine::with_threads(threads);
    let ev = e.evaluate(doc);
    assert!(ev.is_clean(), "{threads} threads: {:?}", ev.failures());
    let (g, _) = ev.final_hashes().unwrap();
    let mut names = ev.final_names();
    names.sort();
    (g, names, ev.document_hash(), ev.stats.executed.len())
}

#[test]
fn rebuild_from_file_is_identical_on_1_4_32_threads() {
    let docs = vec![
        ("sample", sample_document()),
        ("12 chains", parallel_document(12)),
        ("30 chains", parallel_document(30)),
    ];
    for (label, doc) in docs {
        let reference = run(&doc, 1);
        // through a file, with no cache of any kind
        let file = KwfFile::new(doc.clone()).to_bytes().unwrap();
        let rebuilt = KwfFile::from_bytes(&file).unwrap().document;
        assert_eq!(rebuilt, doc, "{label}");
        for threads in [1usize, 4, 32] {
            for pass in 0..2 {
                let r = run(&rebuilt, threads);
                assert_eq!(
                    r.0, reference.0,
                    "{label}: geometry hash differs ({threads} threads, pass {pass})"
                );
                assert_eq!(
                    r.1, reference.1,
                    "{label}: name set differs ({threads} threads, pass {pass})"
                );
                assert_eq!(
                    r.2, reference.2,
                    "{label}: document hash differs ({threads} threads)"
                );
            }
        }
        eprintln!(
            "{label}: {} features, {} names, geometry hash {:032x} identical on 1/4/32 threads",
            doc.len(),
            reference.1.len(),
            reference.0
        );
    }
}
