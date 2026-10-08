//! Phase 2 acceptance: classification (under / fully / over / conflicting)
//! matches ground truth on a ≥ 300-case suite including special positions;
//! the conflicting set is minimal on ≥ 95 % of conflict cases and always a
//! superset of a minimal set.

use kw_sketch::testgen::{classification_suite, Truth};
use kw_sketch::{SketchStatus, Solver};

#[test]
fn classification_matches_ground_truth() {
    let suite = classification_suite();
    assert!(suite.len() >= 300, "suite has only {} cases", suite.len());
    let mut failures = Vec::new();
    let (mut conflicts, mut minimal_exact, mut superset_ok) = (0usize, 0usize, 0usize);
    let mut counts = [0usize; 4];
    for mut case in suite {
        let mut solver = Solver::default();
        let rep = solver.analyze(&mut case.sketch).unwrap();
        match &case.truth {
            Truth::Under { dof } => {
                counts[0] += 1;
                if rep.status != SketchStatus::UnderConstrained || rep.dof != *dof {
                    failures.push(format!(
                        "{}: expected Under(dof {dof}), got {:?} dof {} (residual {:e})",
                        case.name, rep.status, rep.dof, rep.residual
                    ));
                }
            }
            Truth::Fully => {
                counts[1] += 1;
                if rep.status != SketchStatus::FullyConstrained || rep.dof != 0 {
                    failures.push(format!(
                        "{}: expected Fully, got {:?} dof {} (residual {:e})",
                        case.name, rep.status, rep.dof, rep.residual
                    ));
                }
            }
            Truth::Over { flag } => {
                counts[2] += 1;
                if rep.status != SketchStatus::OverConstrained
                    || !flag.iter().any(|f| rep.redundant.contains(f))
                {
                    failures.push(format!("{}: expected Over containing {flag:?}, got {:?} redundant {:?} (residual {:e})", case.name, rep.status, rep.redundant, rep.residual));
                }
            }
            Truth::Conflict { minimal } => {
                counts[3] += 1;
                conflicts += 1;
                if rep.status != SketchStatus::Conflicting {
                    failures.push(format!(
                        "{}: expected Conflicting, got {:?}",
                        case.name, rep.status
                    ));
                    continue;
                }
                let mut got = rep.conflicting.clone();
                got.sort();
                if minimal.iter().all(|c| got.contains(c)) {
                    superset_ok += 1;
                } else {
                    failures.push(format!(
                        "{}: conflicting set {got:?} is not a superset of planted {minimal:?}",
                        case.name
                    ));
                }
                if got == *minimal {
                    minimal_exact += 1;
                } else {
                    println!(
                        "NONMINIMAL {}: got {} constraints, planted {}",
                        case.name,
                        got.len(),
                        minimal.len()
                    );
                }
            }
        }
    }
    println!(
        "cases: under {} fully {} over {} conflict {}",
        counts[0], counts[1], counts[2], counts[3]
    );
    println!("conflicts: {conflicts}, superset of planted minimal: {superset_ok}, exactly minimal: {minimal_exact}");
    for f in failures.iter().take(40) {
        println!("FAIL {f}");
    }
    assert!(
        failures.is_empty(),
        "{} classification failures",
        failures.len()
    );
    assert!(
        superset_ok == conflicts,
        "conflicting sets must always contain a minimal conflict"
    );
    assert!(
        minimal_exact as f64 >= 0.95 * conflicts as f64,
        "minimal on {minimal_exact}/{conflicts}"
    );
}
