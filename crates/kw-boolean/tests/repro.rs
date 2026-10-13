#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Reproduce one fuzz seed: `KW_REPRO_SEED=123 cargo test --release -p kw-boolean --test repro -- --nocapture`
mod common;
use common::*;
use kw_boolean::*;
use kw_brep::*;

#[test]
fn repro() {
    let Some(seed) = std::env::var("KW_REPRO_SEED")
        .ok()
        .and_then(|v| v.parse::<u64>().ok())
    else {
        return;
    };
    let case = gen_case(seed);
    println!("case: {case:?}");
    for op in [Op::Union, Op::Intersect, Op::Subtract] {
        let mut rep = BRep::new();
        let a = build(&mut rep, case.a.0, &case.a.1);
        let b = build(&mut rep, case.b.0, &case.b.1);
        let r = boolean(&mut rep, a, b, op, &BoolOptions::default());
        match r {
            Ok(r) => println!("{op:?}: ok volume {:?}", rep.volume(r.body)),
            Err(e) => println!("{op:?}: ERR {e}"),
        }
        if op != Op::Subtract {
            let r = boolean(&mut rep, b, a, op, &BoolOptions::default());
            match r {
                Ok(r) => println!(
                    "{op:?} swapped: ok volume {:?} faces {}",
                    rep.volume(r.body),
                    rep.body_faces(r.body).len()
                ),
                Err(e) => println!("{op:?} swapped: ERR {e}"),
            }
        }
    }
}
