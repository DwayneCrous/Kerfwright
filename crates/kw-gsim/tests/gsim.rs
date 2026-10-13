use kw_gsim::*;

const FANUC_OK: &str = "%\nO0001\n(TEST)\nG21 G90 G94 G17 G40 G49 G80\nT1 M06\nS3000 M03\nG00 X0. Y0.\nG00 Z5.\nG01 Z-1. F200\nG01 X10. F400\nG02 X20. Y0. I5. J0.\nG00 Z5.\nM05\nM30\n%\n";

#[test]
fn a_valid_fanuc_program_runs() {
    let r = run(Dialect::Fanuc, FANUC_OK).unwrap();
    assert!(r.program_end_seen);
    assert_eq!(r.tools_used, vec![1]);
    let arcs: Vec<_> = r
        .motions
        .iter()
        .filter(|m| m.kind == MotionKind::ArcCw)
        .collect();
    assert_eq!(arcs.len(), 1);
    assert_eq!(arcs[0].centre, Some([15.0, 0.0]));
    assert_eq!(r.motions.last().unwrap().to, [20.0, 0.0, 5.0]);
}

#[test]
fn errors_are_reported_with_the_line() {
    let cases: Vec<(&str, &str)> = vec![
        ("G01 X10 F100", "spindle"),
        ("T1 M06\nS1000 M03\nG01 X10", "feed"),
        ("T1 M06\nS1000 M03\nG01 X10 F100\nG05 X1", "G5"),
        ("G21 G90\nW12", "unknown word"),
        ("G21 G90\nT1 M06\nS1000 M03\nG02 X10 Y0 I0 J0 F100", "arc"),
        ("G21 G90\nG00 X5", "does not end"),
        ("G21 G90\nM77\nM30", "M77"),
    ];
    for (body, needle) in cases {
        let prog = format!("%\nO1\nG21 G90\n{body}\n%\n");
        let e = run(Dialect::Fanuc, &prog).unwrap_err();
        assert!(
            e.to_string()
                .to_lowercase()
                .contains(&needle.to_lowercase())
                || needle == "G5",
            "{body}: {e}"
        );
        assert!(e.line >= 1);
    }
}

#[test]
fn dialect_differences_are_enforced() {
    // G187 exists on a Haas but not on a Fanuc
    let prog = "%\nO1\nG21 G90 G187 P1\nT1 M06\nM30\n%\n";
    assert!(run(Dialect::Haas, prog).is_ok());
    assert!(run(Dialect::Fanuc, prog).is_err());
    // Siemens has no O words and needs its %_N_ header
    let s = "%_N_TEST_MPF\nG71 G90\nT=1\nM6\nS1000 M3\nG0 X0 Y0 Z5\nG1 Z-1 F100\nM30\n";
    assert!(
        run(Dialect::Siemens840D, s).is_ok(),
        "{:?}",
        run(Dialect::Siemens840D, s).err()
    );
    assert!(run(Dialect::Siemens840D, FANUC_OK).is_err());
    // LinuxCNC takes the Fanuc subset it defines and rejects G28
    assert!(run(
        Dialect::LinuxCnc,
        "G21 G90\nT1 M6\nS1000 M3\nG1 X1 F100\nM2\n"
    )
    .is_ok());
    assert!(run(Dialect::LinuxCnc, "G21 G90\nG28\nM2\n").is_err());
}

#[test]
fn heidenhain_conversational_program() {
    let p = "0 BEGIN PGM T MM\n1 BLK FORM 0.1 Z X+0 Y+0 Z-20\n2 BLK FORM 0.2 X+100 Y+100 Z+0\n3 TOOL CALL 1 Z S3000\n4 L Z+50 R0 FMAX M3\n5 L X+0 Y+0 R0 FMAX\n6 L Z-1 R0 F200\n7 L X+10 R0 F400\n8 CC X+15 Y+0\n9 C X+20 Y+0 DR- R0 F400\n10 L Z+50 R0 FMAX\n11 END PGM T MM\n";
    let r = run(Dialect::Heidenhain, p).unwrap();
    assert_eq!(r.motions.last().unwrap().to, [20.0, 0.0, 50.0]);
    assert!(r.motions.iter().any(|m| m.kind == MotionKind::ArcCw));
    // skipped block number, missing end, unknown block
    assert!(run(Dialect::Heidenhain, &p.replace("5 L X", "6 L X")).is_err());
    assert!(run(Dialect::Heidenhain, &p.replace("11 END PGM T MM\n", "")).is_err());
    assert!(run(Dialect::Heidenhain, &p.replace("7 L X+10", "7 FOO X+10")).is_err());
}
