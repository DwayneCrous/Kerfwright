//! Section views: where the cutting plane meets the model, and the hatching of the cut faces.
//! The cut is found by slicing the tessellation, not with the kernel's boolean, so it works on
//! any part the viewer can draw (including faces with holes in them).

use crate::projection::{dot, ViewFrame, V3};
use crate::scene::{Layer, Prim, Shape, P};
use kw_brep::{BRep, BodyId};
use kw_tess::{tessellate_body, TessOptions};
use std::collections::BTreeMap;

/// The frame a section looks with and a point of the cutting plane, for a cut along `a`..`b`
/// (view coordinates of the parent, model units). The cutting plane holds the parent's line of
/// sight and the cut line. The viewer looks toward the kept material: to the right of `a`→`b`,
/// or to the left when `flip`.
pub fn section_frame(
    parent: &ViewFrame,
    a: P,
    b: P,
    flip: bool,
) -> Result<(ViewFrame, V3), String> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len = dx.hypot(dy);
    if len < 1e-9 {
        return Err("the cutting line has no length".into());
    }
    let (dx, dy) = (dx / len, dy / len);
    // right of the direction a→b, in the view plane: where the arrows point
    let sign = if flip { -1.0 } else { 1.0 };
    let (nu, nv) = (sign * dy, -sign * dx);
    let look = [
        parent.right[0] * nu + parent.up[0] * nv,
        parent.right[1] * nu + parent.up[1] * nv,
        parent.right[2] * nu + parent.up[2] * nv,
    ];
    // keep the parent's up on the sheet unless that is the direction we look in
    let along_up = dot(look, parent.up);
    let up = if along_up.abs() > 0.99 {
        let k = if along_up > 0.0 { -1.0 } else { 1.0 };
        [k * parent.dir[0], k * parent.dir[1], k * parent.dir[2]]
    } else {
        parent.up
    };
    let p0 = [
        parent.right[0] * a[0] + parent.up[0] * a[1],
        parent.right[1] * a[0] + parent.up[1] * a[1],
        parent.right[2] * a[0] + parent.up[2] * a[1],
    ];
    Ok((ViewFrame::new(look, up), p0))
}

/// The outline of what the plane through `p0` cuts out of each body: for every body its loops
/// (3-D points lying in the plane, a loop not repeating its first point). Found by slicing the
/// tessellation, so curved cut edges are polylines at the tessellation's tolerance. A cut that
/// meets nothing is an error.
pub fn slice_loops(
    brep: &BRep,
    bodies: &[BodyId],
    p0: V3,
    keep: V3,
) -> Result<Vec<Vec<Vec<V3>>>, String> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for b in bodies {
        let bb = brep.body_bbox(*b);
        for k in 0..3 {
            lo[k] = lo[k].min(bb.min[k]);
            hi[k] = hi[k].max(bb.max[k]);
        }
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    let opts = TessOptions {
        chord_tol: (diag * 1e-4).max(1e-6),
        angle_tol: 0.1,
    };
    let side = |p: V3| dot([p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]], keep);
    let mut out: Vec<Vec<Vec<V3>>> = Vec::new();
    for b in bodies {
        let t = tessellate_body(brep, *b, &opts).map_err(|e| e.to_string())?;
        // one segment per triangle the plane crosses; a vertex on the plane counts as kept, so
        // neighbouring triangles agree on where the crossing is
        let mut segs: Vec<(V3, V3)> = Vec::new();
        for tri in 0..t.mesh.triangle_count() {
            let c = t.mesh.triangle(tri);
            let p = [
                [c[0].x, c[0].y, c[0].z],
                [c[1].x, c[1].y, c[1].z],
                [c[2].x, c[2].y, c[2].z],
            ];
            let d = [side(p[0]), side(p[1]), side(p[2])];
            let kept = [d[0] >= 0.0, d[1] >= 0.0, d[2] >= 0.0];
            if kept[0] == kept[1] && kept[1] == kept[2] {
                continue;
            }
            let mut hits: Vec<V3> = Vec::new();
            for i in 0..3 {
                let j = (i + 1) % 3;
                if kept[i] != kept[j] {
                    // the same arithmetic whichever triangle (and end) it is reached from
                    let (a, b, da, db) = if key(p[i]) < key(p[j]) {
                        (p[i], p[j], d[i], d[j])
                    } else {
                        (p[j], p[i], d[j], d[i])
                    };
                    let f = da / (da - db);
                    hits.push([
                        a[0] + (b[0] - a[0]) * f,
                        a[1] + (b[1] - a[1]) * f,
                        a[2] + (b[2] - a[2]) * f,
                    ]);
                }
            }
            if hits.len() == 2 && key(hits[0]) != key(hits[1]) {
                segs.push((hits[0], hits[1]));
            }
        }
        let mut loops = chain_loops(&segs);
        for l in &mut loops {
            drop_collinear(l, diag * 1e-7);
        }
        out.push(loops);
    }
    if out.iter().all(|l| l.is_empty()) {
        return Err("the cutting plane does not pass through the model".into());
    }
    Ok(out)
}

/// Remove the points of a closed loop that lie on the line between their neighbours.
fn drop_collinear(l: &mut Vec<V3>, tol: f64) {
    let mut i = 0;
    while l.len() > 3 && i < l.len() {
        let n = l.len();
        let (a, p, c) = (l[(i + n - 1) % n], l[i], l[(i + 1) % n]);
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
        let len = dot(ac, ac).sqrt();
        let cr = [
            ac[1] * ap[2] - ac[2] * ap[1],
            ac[2] * ap[0] - ac[0] * ap[2],
            ac[0] * ap[1] - ac[1] * ap[0],
        ];
        // on the line, and between the ends
        if len > 0.0
            && dot(cr, cr).sqrt() / len < tol
            && dot(ap, ac) > 0.0
            && dot(ap, ap) < dot(ac, ac)
        {
            l.remove(i);
        } else {
            i += 1;
        }
    }
}

fn key(p: V3) -> [u64; 3] {
    [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()]
}

/// Join segments end to end into loops (or open chains, if the surface is not closed).
fn chain_loops(segs: &[(V3, V3)]) -> Vec<Vec<V3>> {
    let mut at: BTreeMap<[u64; 3], Vec<usize>> = BTreeMap::new();
    for (i, (a, b)) in segs.iter().enumerate() {
        at.entry(key(*a)).or_default().push(i);
        at.entry(key(*b)).or_default().push(i);
    }
    let mut used = vec![false; segs.len()];
    let mut loops = Vec::new();
    for s in 0..segs.len() {
        if used[s] {
            continue;
        }
        used[s] = true;
        let mut pts = vec![segs[s].0, segs[s].1];
        loop {
            let end = *pts.last().unwrap();
            let next = at
                .get(&key(end))
                .and_then(|v| v.iter().find(|i| !used[**i]).copied());
            let Some(n) = next else { break };
            used[n] = true;
            let (a, b) = segs[n];
            pts.push(if key(a) == key(end) { b } else { a });
        }
        if key(pts[0]) == key(*pts.last().unwrap()) {
            pts.pop();
        }
        if pts.len() >= 3 {
            loops.push(pts);
        }
    }
    loops
}

/// Hatch lines (sheet scale, relative to the view origin) over the regions the loops enclose,
/// by the even-odd rule so holes stay open. Lines are parallel at `spacing` mm on a common
/// lattice; each body is hatched at its own angle (45 and 135 degrees alternately).
pub fn hatch_loops(
    caps: &[Vec<Vec<V3>>],
    frame: &ViewFrame,
    scale: f64,
    centre: P,
    spacing: f64,
) -> Vec<Prim> {
    let mut out = Vec::new();
    for (bi, loops) in caps.iter().enumerate() {
        let angle = if bi % 2 == 0 { 45.0_f64 } else { 135.0 }.to_radians();
        let (d, m) = ([angle.cos(), angle.sin()], [-angle.sin(), angle.cos()]);
        let q: Vec<Vec<P>> = loops
            .iter()
            .map(|l| {
                l.iter()
                    .map(|p| {
                        let (uv, _) = frame.project(*p);
                        [uv[0] * scale - centre[0], uv[1] * scale - centre[1]]
                    })
                    .collect()
            })
            .collect();
        let ms: Vec<f64> = q
            .iter()
            .flatten()
            .map(|p| m[0] * p[0] + m[1] * p[1])
            .collect();
        if ms.is_empty() {
            continue;
        }
        let mlo = ms.iter().cloned().fold(f64::INFINITY, f64::min);
        let mhi = ms.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        for k in (mlo / spacing).ceil() as i64..=(mhi / spacing).floor() as i64 {
            let c = k as f64 * spacing;
            let mut ts: Vec<f64> = Vec::new();
            for l in &q {
                for i in 0..l.len() {
                    let (a, b) = (l[i], l[(i + 1) % l.len()]);
                    let (ma, mb) = (m[0] * a[0] + m[1] * a[1] - c, m[0] * b[0] + m[1] * b[1] - c);
                    // half-open: a vertex on the line belongs to one side only
                    if (ma < 0.0) != (mb < 0.0) {
                        let f = ma / (ma - mb);
                        let p = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
                        ts.push(d[0] * p[0] + d[1] * p[1]);
                    }
                }
            }
            ts.sort_by(|a, b| a.total_cmp(b));
            let base = [m[0] * c, m[1] * c];
            for pair in ts.as_chunks::<2>().0 {
                if pair[1] - pair[0] > 1e-9 {
                    out.push(Prim {
                        layer: Layer::Hatch,
                        shape: Shape::Line {
                            a: [base[0] + d[0] * pair[0], base[1] + d[1] * pair[0]],
                            b: [base[0] + d[0] * pair[1], base[1] + d[1] * pair[1]],
                        },
                    });
                }
            }
        }
    }
    out
}

/// The point where `a`..`b` of a view (model units) sits on the sheet, for a view at `pos` that
/// draws a model point `(u, v)` at `(u, v) * scale - centre` from its origin.
pub fn to_sheet(pos: P, scale: f64, centre: P, uv: P) -> P {
    [
        pos[0] + uv[0] * scale - centre[0],
        pos[1] + uv[1] * scale - centre[1],
    ]
}

/// Direction `a`→`b` and the direction the arrows point (unit, view coordinates).
pub fn cut_directions(a: P, b: P, flip: bool) -> Option<(P, P)> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l = dx.hypot(dy);
    if l < 1e-12 {
        return None;
    }
    let (dx, dy) = (dx / l, dy / l);
    let s = if flip { -1.0 } else { 1.0 };
    Some(([dx, dy], [s * dy, -s * dx]))
}
