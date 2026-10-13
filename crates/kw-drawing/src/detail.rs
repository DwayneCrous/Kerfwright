//! Detail views: the part of a view inside a circle, enlarged.

use crate::scene::{flatten, Layer, Prim, Scene2d, Shape, P};

/// The part of segment `a`..`b` inside the disc, if any.
fn seg_in_disc(a: P, b: P, c: P, r: f64) -> Option<(P, P)> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let (fx, fy) = (a[0] - c[0], a[1] - c[1]);
    let qa = dx * dx + dy * dy;
    if qa < 1e-24 {
        return None;
    }
    let qb = 2.0 * (fx * dx + fy * dy);
    let qc = fx * fx + fy * fy - r * r;
    let disc = qb * qb - 4.0 * qa * qc;
    if disc <= 0.0 {
        return None;
    }
    let s = disc.sqrt();
    let (t0, t1) = (
        ((-qb - s) / (2.0 * qa)).max(0.0),
        ((-qb + s) / (2.0 * qa)).min(1.0),
    );
    if t1 - t0 < 1e-9 {
        return None;
    }
    let at = |t: f64| [a[0] + dx * t, a[1] + dy * t];
    Some((at(t0), at(t1)))
}

fn inside(p: P, c: P, r: f64) -> bool {
    (p[0] - c[0]).hypot(p[1] - c[1]) <= r + 1e-9
}

/// Keep only what lies inside the disc `(c, r)`. Lines stay lines; curves that cross the circle
/// become polylines; curves wholly inside stay as they are. Text and fills are dropped.
pub fn clip_to_disc(scene: &Scene2d, c: P, r: f64) -> Scene2d {
    let mut out = Scene2d::new();
    for pr in &scene.prims {
        match &pr.shape {
            Shape::Line { a, b } => {
                if let Some((a, b)) = seg_in_disc(*a, *b, c, r) {
                    out.push(pr.layer, Shape::Line { a, b });
                }
            }
            Shape::Text { .. } | Shape::Fill { .. } => {}
            shape => {
                let pts = flatten(shape, 0.02);
                if pts.iter().all(|p| inside(*p, c, r)) {
                    out.prims.push(pr.clone());
                    continue;
                }
                let mut run: Vec<P> = Vec::new();
                let flush = |run: &mut Vec<P>, out: &mut Scene2d| {
                    if run.len() >= 2 {
                        out.push(
                            pr.layer,
                            Shape::Polyline {
                                pts: std::mem::take(run),
                                closed: false,
                            },
                        );
                    }
                    run.clear();
                };
                for w in pts.windows(2) {
                    match seg_in_disc(w[0], w[1], c, r) {
                        Some((a, b)) => {
                            if run
                                .last()
                                .is_none_or(|l| (l[0] - a[0]).hypot(l[1] - a[1]) > 1e-9)
                            {
                                flush(&mut run, &mut out);
                                run.push(a);
                            }
                            run.push(b);
                        }
                        None => flush(&mut run, &mut out),
                    }
                }
                flush(&mut run, &mut out);
            }
        }
    }
    out
}

/// The enlarged detail: the scene clipped to the disc, moved so the disc is centred on the
/// origin, with the circle drawn around it.
pub fn detail_scene(scene: &Scene2d, c: P, r: f64) -> Scene2d {
    let mut s = clip_to_disc(scene, c, r).placed(1.0, [-c[0], -c[1]]);
    s.prims.push(Prim {
        layer: Layer::Symbol,
        shape: Shape::Circle { c: [0.0, 0.0], r },
    });
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lines_are_cut_at_the_circle() {
        let mut s = Scene2d::new();
        s.line(Layer::Visible, [-10.0, 0.0], [10.0, 0.0]);
        s.line(Layer::Visible, [-10.0, 20.0], [10.0, 20.0]);
        let c = clip_to_disc(&s, [0.0, 0.0], 5.0);
        assert_eq!(c.prims.len(), 1);
        match &c.prims[0].shape {
            Shape::Line { a, b } => {
                assert!((a[0] + 5.0).abs() < 1e-9 && (b[0] - 5.0).abs() < 1e-9);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_circle_crossing_the_boundary_becomes_an_arc_polyline() {
        let mut s = Scene2d::new();
        s.circle(Layer::Visible, [6.0, 0.0], 3.0);
        let c = clip_to_disc(&s, [0.0, 0.0], 5.0);
        assert_eq!(c.prims.len(), 1);
        let Shape::Polyline { pts, .. } = &c.prims[0].shape else {
            panic!()
        };
        assert!(pts.iter().all(|p| inside(*p, [0.0, 0.0], 5.0)));
        assert!(pts.len() > 4);
    }
}
