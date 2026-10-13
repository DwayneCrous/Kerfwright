use kw_brep::BRep;
use kw_sheet::*;
use std::f64::consts::{FRAC_PI_2, PI};

fn spec(t: f64, k: f64, r: f64) -> SheetSpec {
    SheetSpec {
        thickness: t,
        k_factor: k,
        default_radius: r,
    }
}

fn volume(strip: &SheetStrip) -> (BRep, f64) {
    let mut rep = BRep::new();
    let r = build(&mut rep, strip).unwrap_or_else(|e| panic!("{e}"));
    assert!(rep.validate().is_ok(), "{:?}", rep.validate());
    let v = rep.volume(r.body).unwrap();
    (rep, v)
}

#[test]
fn l_bracket_has_the_analytic_volume_and_flat_length() {
    let s = SheetStrip::new(spec(2.0, 0.44, 3.0), 40.0)
        .flat(30.0)
        .flange(FRAC_PI_2, 20.0);
    let (_, v) = volume(&s);
    assert!(
        (v - analytic_volume(&s)).abs() / v < 1e-10,
        "{v} vs {}",
        analytic_volume(&s)
    );
    let flat = unfold(&s).unwrap();
    let ba = FRAC_PI_2 * (3.0 + 0.44 * 2.0);
    assert!((flat.length() - (30.0 + ba + 20.0)).abs() < 1e-12);
}

#[test]
fn channels_z_brackets_and_hems_build_valid_solids() {
    let cases = vec![
        // U channel
        SheetStrip::new(spec(1.5, 0.5, 2.0), 25.0)
            .flange(FRAC_PI_2, 15.0)
            .flat(40.0)
            .flange(FRAC_PI_2, 15.0),
        // Z bracket (bends in opposite directions)
        SheetStrip::new(spec(2.0, 0.4, 2.0), 30.0)
            .flat(20.0)
            .flange(FRAC_PI_2, 25.0)
            .flange(-FRAC_PI_2, 20.0),
        // an acute flange
        SheetStrip::new(spec(1.0, 0.33, 1.0), 10.0)
            .flat(15.0)
            .flange(1.0, 12.0),
        // a hem
        SheetStrip::new(spec(1.0, 0.45, 1.0), 50.0)
            .flat(30.0)
            .hem(Hem::Open {
                gap: 0.6,
                return_length: 8.0,
            }),
        // a full omega
        SheetStrip::new(spec(1.0, 0.5, 1.5), 20.0)
            .flat(10.0)
            .flange(FRAC_PI_2, 8.0)
            .flange(-FRAC_PI_2, 14.0)
            .flange(-FRAC_PI_2, 8.0)
            .flange(FRAC_PI_2, 10.0),
    ];
    for s in cases {
        let (_rep, v) = volume(&s);
        assert!(
            (v - analytic_volume(&s)).abs() / v < 1e-9,
            "{s:?}: {v} vs {}",
            analytic_volume(&s)
        );
    }
}

#[test]
fn a_part_that_folds_through_itself_is_refused() {
    // a long flange bent back over the base crosses it
    let s = SheetStrip::new(spec(1.0, 0.44, 1.0), 20.0)
        .flat(20.0)
        .flange(FRAC_PI_2, 20.0)
        .flange(FRAC_PI_2, 15.0)
        .flange(FRAC_PI_2, 25.0);
    let mut rep = BRep::new();
    assert!(matches!(
        build(&mut rep, &s),
        Err(SheetError::SelfIntersecting(_))
    ));
    // bad dimensions are explained
    let bad = SheetStrip::new(spec(0.0, 0.5, 1.0), 10.0).flat(5.0);
    assert!(
        matches!(build(&mut rep, &bad), Err(SheetError::Invalid(m)) if m.contains("thickness"))
    );
}

#[test]
fn flat_pattern_matches_the_bend_allowance_in_the_dxf() {
    let mut worst: f64 = 0.0;
    let mut n = 0;
    for &t in &[0.8, 1.5, 3.0] {
        for &k in &[0.3, 0.4, 0.5] {
            for &r in &[0.5 * t, t, 2.5 * t] {
                for &a in &[0.4, FRAC_PI_2, 2.0, -1.2, -FRAC_PI_2] {
                    let s = SheetStrip::new(spec(t, k, r), 20.0)
                        .flat(25.0 + t)
                        .flange(a, 17.0)
                        .flange(-a * 0.7, 9.0);
                    let flat = unfold(&s).unwrap();
                    let text = kw_dxf::write(&flat.to_dxf());
                    let doc = kw_dxf::parse(&text).unwrap();
                    let (len, w, zones) = measure_dxf(&doc);
                    assert!((w - 20.0).abs() < 1e-9);
                    let expect = [(a, r), (-a * 0.7, r)];
                    assert_eq!(zones.len(), 2);
                    let mut total = 25.0 + t + 17.0 + 9.0;
                    for ((s0, s1, up), (ang, rad)) in zones.iter().zip(expect) {
                        let ba = ang.abs() * (rad + k * t);
                        worst = worst.max(((s1 - s0) - ba).abs());
                        assert_eq!(*up, ang > 0.0);
                        total += ba;
                    }
                    worst = worst.max((len - total).abs());
                    n += 1;
                }
            }
        }
    }
    eprintln!("{n} flat patterns, worst DXF length error {worst:e} mm");
    assert!(worst < 1e-6);
}

#[test]
fn unfold_then_refold_keeps_the_volume() {
    let mut worst: f64 = 0.0;
    let mut n = 0;
    for &t in &[0.8, 2.0] {
        for &r in &[t, 3.0 * t] {
            for &a in &[FRAC_PI_2, 1.0, -2.0, PI * 0.9] {
                let s = SheetStrip::new(spec(t, 0.44, r), 15.0)
                    .flat(22.0)
                    .flange(a, 14.0)
                    .flange(-FRAC_PI_2 * 0.8, 11.0);
                if build(&mut BRep::new(), &s).is_err() {
                    continue; // folds through itself
                }
                let (rep, v0) = volume(&s);
                // recognise the folded body without its recipe, unfold, refold
                let body = rep.bodies.keys().next().unwrap();
                let rec = recognize(&rep, body, 0.44).unwrap();
                let flat = unfold(&rec).unwrap();
                let (_, v1) = volume(&refold(&flat));
                if (v1 - v0).abs() / v0 > 1e-6 {
                    eprintln!("MISMATCH original {:?}\n recognised {:?}", s.segs, rec.segs);
                }
                worst = worst.max((v1 - v0).abs() / v0);
                // and the recognised flat pattern equals the original's
                let flat0 = unfold(&s).unwrap();
                worst = worst.max((flat.length() - flat0.length()).abs());
                n += 1;
            }
        }
    }
    eprintln!("{n} unfold/refold round trips, worst error {worst:e}");
    assert!(n > 10 && worst < 1e-6);
}
