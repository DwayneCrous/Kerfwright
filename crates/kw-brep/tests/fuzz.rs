//! Phase 1 acceptance: "fuzzed random topology mutations never produce an
//! unreported invalid body".
//!
//! Property: for any primitive and any applicable single mutation, either the
//! body's semantic signature is unchanged, or `validate()` reports an error.
//! Sequences of mutations are covered too.

use kw_brep::mutate::{apply, signature, Mutation};
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use proptest::prelude::*;

fn cases() -> u32 {
    std::env::var("PROPTEST_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(5_000)
}

fn primitive(kind: u8) -> BRep {
    let f = Frame3::from_origin_normal(Point3::new(0.5, -1.0, 2.0), Vector3::new(0.1, 0.2, 1.0));
    let mut b = BRep::new();
    match kind % 6 {
        0 => {
            make_box(&mut b, &f, 2.0, 3.0, 4.0);
        }
        1 => {
            make_cylinder(&mut b, &f, 1.5, 4.0);
        }
        2 => {
            make_cone(&mut b, &f, 2.0, 1.0, 3.0);
        }
        3 => {
            make_cone(&mut b, &f, 2.0, 0.0, 3.0);
        }
        4 => {
            make_sphere(&mut b, &f, 2.0);
        }
        _ => {
            make_torus(&mut b, &f, 5.0, 1.0);
        }
    }
    b
}

fn mutation() -> impl Strategy<Value = Mutation> {
    let i = 0usize..64;
    let j = 0usize..64;
    prop_oneof![
        i.clone().prop_map(Mutation::FlipCoedgeSense),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SetCoedgeNext(a, b)),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SetCoedgePrev(a, b)),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SetCoedgeRadialNext(a, b)),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SetCoedgeEdge(a, b)),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SetCoedgeLoop(a, b)),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SwapPcurves(a, b)),
        i.clone().prop_map(Mutation::RemoveCoedge),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::RetargetEdgeStart(a, b)),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::RetargetEdgeEnd(a, b)),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SetEdgeCurve(a, b)),
        i.clone().prop_map(Mutation::ScaleEdgeRange),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SetEdgeFirstCoedge(a, b)),
        i.clone().prop_map(Mutation::RemoveEdge),
        i.clone().prop_map(Mutation::RemoveVertex),
        (i.clone(), prop::array::uniform3(-1i8..=1)).prop_map(|(a, d)| Mutation::MoveVertex(a, d)),
        i.clone().prop_map(Mutation::FlipFaceSense),
        (i.clone(), j.clone()).prop_map(|(a, b)| Mutation::SetFaceSurface(a, b)),
        i.clone().prop_map(Mutation::RemoveFace),
        i.clone().prop_map(Mutation::DropFaceFromShell),
        i.clone().prop_map(Mutation::DuplicateFaceInShell),
        i.clone().prop_map(Mutation::FlipLoopKind),
        (i, j).prop_map(|(a, b)| Mutation::AddBogusLoop(a, b)),
    ]
}

#[test]
fn unmutated_primitives_are_valid_and_signature_is_stable() {
    for k in 0..6 {
        let b = primitive(k);
        assert!(
            b.validate().is_ok(),
            "primitive {k}: {:?}",
            b.validate().errors
        );
        assert_eq!(signature(&b), signature(&b.clone()));
    }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: cases(), ..ProptestConfig::default() })]

    #[test]
    fn single_mutation_is_never_silently_valid(kind in 0u8..6, m in mutation()) {
        let original = primitive(kind);
        let before = signature(&original);
        let mut b = original.clone();
        if apply(&mut b, m) {
            let after = signature(&b);
            if after != before {
                let rep = b.validate();
                prop_assert!(!rep.is_ok(), "silently valid after {m:?} on primitive {kind}");
            }
        }
    }

    #[test]
    fn mutation_sequences_are_never_silently_valid(kind in 0u8..6, ms in prop::collection::vec(mutation(), 1..4)) {
        let mut b = primitive(kind);
        let before = signature(&b);
        for m in ms {
            apply(&mut b, m);
        }
        if signature(&b) != before {
            prop_assert!(!b.validate().is_ok());
        }
    }
}
