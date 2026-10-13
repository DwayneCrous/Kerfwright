#![allow(clippy::needless_range_loop)]
//! SIMP topology optimisation of the MBB beam (see `kw_fem::simp`).
//!
//! Reference value. The paper that defines the algorithm (Andreassen et al. 2011, "Efficient topology
//! optimization in MATLAB using 88 lines of code") prints no compliance in its text or captions; it shows
//! the optimised MBB beams of its figure 3 as pictures. The number c = 216.81 for the 60 × 20 MBB beam with
//! the sensitivity filter at r_min = 2.4 (0.04 × the width of the domain, the paper's choice) is quoted by
//! a third-party line-by-line port of the MATLAB code that cites that paper's figure 3
//! (github.com/mustafaobaidd1/strut-topology: 216.8137 after 106 iterations; density filter 233.71 after
//! 144 iterations). `kw_fem::simp` is an independent transcription of the published MATLAB listing and
//! reproduces those figures — the original MATLAB could not be run here, so the third-party value is the
//! only published-by-proxy reference.

use kw_fem::simp::*;

const REFERENCE: f64 = 216.81;

#[test]
fn mbb_beam_compliance_matches_the_published_value_within_five_percent() {
    let r = simp(&SimpParams::mbb(60, 20, 0.5, 3.0, 2.4, Filter::Sensitivity)).unwrap();
    eprintln!(
        "MBB 60x20, volume fraction 0.5, p = 3, r_min = 2.4, sensitivity filter: compliance {:.4} after {} iterations (reference {REFERENCE}, {:+.3} %), volume {:.4}",
        r.compliance,
        r.iterations,
        100.0 * (r.compliance / REFERENCE - 1.0),
        r.volume_fraction
    );
    assert!(
        (r.compliance / REFERENCE - 1.0).abs() < 0.05,
        "{}",
        r.compliance
    );
    // regression guard on the transcription itself: the figures the third-party port reports
    assert!((r.compliance - 216.8137).abs() < 1e-3, "{}", r.compliance);
    assert_eq!(r.iterations, 106);
    assert!((r.volume_fraction - 0.5).abs() < 1e-3);
    // the design is mostly black and white
    let grey =
        r.density.iter().filter(|&&x| x > 0.1 && x < 0.9).count() as f64 / r.density.len() as f64;
    let solid = r.density.iter().filter(|&&x| x >= 0.9).count() as f64 / r.density.len() as f64;
    eprintln!(
        "  {:.1} % solid, {:.1} % grey, {:.1} % void",
        100.0 * solid,
        100.0 * grey,
        100.0 * (1.0 - solid - grey)
    );
    // a filtered (non-projected) design keeps a grey band along the member edges, but is far from uniform
    assert!(solid > 0.2 && solid < 0.5 && grey < 0.5, "{solid} {grey}");
}

#[test]
fn the_other_variants_of_the_paper() {
    // density filter (ft = 2) at the same radius: 233.71 after 144 iterations in the third-party port
    let d = simp(&SimpParams::mbb(60, 20, 0.5, 3.0, 2.4, Filter::Density)).unwrap();
    eprintln!(
        "MBB density filter: compliance {:.4} after {} iterations",
        d.compliance, d.iterations
    );
    assert!((d.compliance - 233.7146).abs() < 1e-3, "{}", d.compliance);
    assert_eq!(d.iterations, 144);
    // the example call of the paper, top88(60,20,0.5,3,1.5,1): a smaller radius gives a stiffer design
    let s = simp(&SimpParams::mbb(60, 20, 0.5, 3.0, 1.5, Filter::Sensitivity)).unwrap();
    eprintln!(
        "MBB top88(60,20,0.5,3,1.5,1): compliance {:.4} after {} iterations",
        s.compliance, s.iterations
    );
    assert!(s.compliance < 216.0 && s.compliance > 190.0);
}
