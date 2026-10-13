use crate::edges::{circle_step, EdgeContext};
use crate::{TessError, TessOptions};
use kw_brep::{BRep, FaceId, Sense};
use kw_nurbs::{Curve3, Surface, SurfaceGeom};
use nalgebra::{Point3, Vector3};
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

/// Placeholder weld id for vertices that exist only inside one face.
pub(crate) const LOCAL: u32 = u32::MAX;

pub(crate) struct FaceMesh {
    /// Surface parameters of each vertex.
    pub uv: Vec<(f64, f64)>,
    pub positions: Vec<Point3<f64>>,
    pub normals: Vec<Vector3<f64>>,
    pub weld: Vec<u32>,
    pub tris: Vec<[u32; 3]>,
}

struct BPoint {
    uv: (f64, f64),
    pos: Point3<f64>,
    weld: u32,
}

/// Sampling steps for a surface: angular step used along periodic `u`
/// boundaries/poles, and the interior grid spacing (parameter units).
struct Steps {
    ang_u: Option<f64>,
    grid_u: Option<f64>,
    grid_v: Option<f64>,
}

fn steps(
    surface: &SurfaceGeom,
    bbox: ((f64, f64), (f64, f64)),
    rmax: f64,
    opts: &TessOptions,
) -> Steps {
    const SAFETY: f64 = 2.0; // triangle circumradius can exceed half a chord (measured: >1.5x)
    let ((u0, u1), (v0, v1)) = bbox;
    match surface {
        SurfaceGeom::Plane(_) => Steps {
            ang_u: None,
            grid_u: None,
            grid_v: None,
        },
        // interior points are only used for non-rectangular (trimmed) faces; see `mesh_face`
        SurfaceGeom::Cylinder(c) => Steps {
            ang_u: Some(circle_step(c.radius, opts)),
            grid_u: Some(circle_step(c.radius, opts) / SAFETY),
            grid_v: Some(((v1 - v0) / 6.0).max(1e-9)),
        },
        SurfaceGeom::Cone(c) => {
            let t = c.half_angle.tan();
            let r = (c.radius_at_origin + v0 * t)
                .abs()
                .max((c.radius_at_origin + v1 * t).abs())
                .max(rmax);
            Steps {
                ang_u: Some(circle_step(r, opts)),
                grid_u: Some(circle_step(r, opts) / SAFETY),
                grid_v: Some(((v1 - v0) / 6.0).max(1e-9)),
            }
        }
        SurfaceGeom::Sphere(s) => {
            let a = circle_step(s.radius, opts) / SAFETY;
            Steps {
                ang_u: Some(a),
                grid_u: Some(a),
                grid_v: Some(a),
            }
        }
        SurfaceGeom::Torus(t) => {
            let a = circle_step(t.major_radius + t.minor_radius, opts) / SAFETY;
            let b = circle_step(t.minor_radius, opts) / SAFETY;
            Steps {
                ang_u: Some(a),
                grid_u: Some(a),
                grid_v: Some(b),
            }
        }
        SurfaceGeom::Nurbs(s) => {
            let nu = (4 * s.num_u()).clamp(8, 128) as f64;
            let nv = (4 * s.num_v()).clamp(8, 128) as f64;
            Steps {
                ang_u: None,
                grid_u: Some((u1 - u0) / nu),
                grid_v: Some((v1 - v0) / nv),
            }
        }
        SurfaceGeom::Procedural(p) => {
            let nu = (4 * p.approximation.num_u()).clamp(8, 128) as f64;
            let nv = (4 * p.approximation.num_v()).clamp(8, 128) as f64;
            Steps {
                ang_u: None,
                grid_u: Some((u1 - u0) / nu),
                grid_v: Some((v1 - v0) / nv),
            }
        }
        SurfaceGeom::Extrusion { .. } => Steps {
            ang_u: None,
            grid_u: Some(((u1 - u0) / 16.0).max(1e-9)),
            grid_v: Some(((v1 - v0) / 8.0).max(1e-9)),
        },
        SurfaceGeom::Revolution { profile, .. } => {
            let straight = matches!(**profile, kw_nurbs::CurveGeom::Line(_));
            let a = circle_step(rmax.max(1e-9), opts) / SAFETY;
            Steps {
                ang_u: Some(a),
                grid_u: if straight {
                    None
                } else {
                    Some((u1 - u0) / 16.0)
                },
                grid_v: Some(a),
            }
        }
    }
}

fn lerp_params(a: f64, b: f64, n: usize) -> impl Iterator<Item = f64> {
    (0..=n).map(move |i| a + (b - a) * i as f64 / n as f64)
}

pub(crate) fn mesh_face(
    brep: &BRep,
    ctx: &EdgeContext,
    fid: FaceId,
    opts: &TessOptions,
) -> Result<FaceMesh, TessError> {
    let face = &brep.faces[fid];
    let surface = brep
        .surfaces
        .get(face.surface)
        .ok_or_else(|| TessError::BadTopology(format!("face {fid:?} has dangling surface")))?;
    let flip = face.sense == Sense::Reversed;

    // ---- boundary loops --------------------------------------------------
    // first pass collects uv ranges so degenerate edges can be sized
    struct RawCoedge {
        cid: kw_brep::CoedgeId,
    }
    let mut raw: Vec<Vec<RawCoedge>> = Vec::new();
    for l in &face.loops {
        raw.push(
            brep.loop_coedges(*l)
                .into_iter()
                .map(|cid| RawCoedge { cid })
                .collect(),
        );
    }
    // uv bbox and radius hint from non-degenerate edges
    let mut bb = (
        (f64::INFINITY, f64::NEG_INFINITY),
        (f64::INFINITY, f64::NEG_INFINITY),
    );
    let mut rmax = 0.0f64;
    for loop_ in &raw {
        for rc in loop_ {
            let co = &brep.coedges[rc.cid];
            let pc = co
                .pcurve
                .and_then(|p| brep.curves.get(p))
                .ok_or_else(|| TessError::MissingPcurve(format!("{:?}", rc.cid)))?;
            let (t0, t1) = brep.coedge_param_range(rc.cid).unwrap();
            for t in lerp_params(t0, t1, 8) {
                let p = pc.point_at(t);
                bb.0 .0 = bb.0 .0.min(p.x);
                bb.0 .1 = bb.0 .1.max(p.x);
                bb.1 .0 = bb.1 .0.min(p.y);
                bb.1 .1 = bb.1 .1.max(p.y);
            }
            if let Some(s) = ctx.samples.get(&co.edge) {
                for p in &s.points {
                    rmax = rmax.max(p.coords.norm());
                }
            }
        }
    }
    // distance-from-axis hint for cones/revolutions: use bounding radius of boundary
    let st = {
        let mut axis_r = 0.0f64;
        for loop_ in &raw {
            for rc in loop_ {
                if let Some(s) = ctx.samples.get(&brep.coedges[rc.cid].edge) {
                    for p in &s.points {
                        axis_r = axis_r.max(radial_distance(surface, p));
                    }
                }
            }
        }
        steps(surface, bb, axis_r, opts)
    };

    let mut loops: Vec<Vec<BPoint>> = Vec::new();
    for loop_ in &raw {
        let mut pts: Vec<BPoint> = Vec::new();
        for rc in loop_ {
            let co = &brep.coedges[rc.cid];
            let edge = &brep.edges[co.edge];
            let pc = brep.curves.get(co.pcurve.unwrap()).unwrap();
            // snap parameter noise (~1e-16) so exactly collinear boundary points stay collinear
            let snap = |x: f64| (x * 1e12).round() / 1e12;
            let uv = |t: f64| {
                let p = pc.point_at(t);
                (snap(p.x), snap(p.y))
            };
            if edge.is_degenerate() {
                let vtx = brep
                    .vertices
                    .get(edge.start)
                    .ok_or_else(|| TessError::BadTopology("degenerate edge vertex".into()))?;
                let (t0, t1) = brep.coedge_param_range(rc.cid).unwrap();
                let (a, b) = (uv(t0), uv(t1));
                let du = (b.0 - a.0).abs();
                let dv = (b.1 - a.1).abs();
                let step = st.grid_u.or(st.ang_u).unwrap_or(f64::INFINITY);
                let n = if du > 1e-12 {
                    (du / step).ceil().max(1.0) as usize
                } else if dv > 1e-12 {
                    8
                } else {
                    1
                };
                let w = ctx.vertex_weld[&edge.start];
                for t in lerp_params(t0, t1, n).take(n) {
                    pts.push(BPoint {
                        uv: uv(t),
                        pos: vtx.point,
                        weld: w,
                    });
                }
            } else {
                let s = ctx
                    .samples
                    .get(&co.edge)
                    .ok_or_else(|| TessError::BadTopology("edge not discretised".into()))?;
                let n = s.params.len();
                let order: Vec<usize> = match co.sense {
                    Sense::Forward => (0..n - 1).collect(),
                    Sense::Reversed => (1..n).rev().collect(),
                };
                for k in order {
                    pts.push(BPoint {
                        uv: uv(s.params[k]),
                        pos: s.points[k],
                        weld: s.weld[k],
                    });
                }
            }
        }
        loops.push(pts);
    }

    // ---- scaling so Delaunay sees roughly isotropic uv --------------------
    let ((u0, u1), (v0, v1)) = {
        let (mut u0, mut u1, mut v0, mut v1) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for l in &loops {
            for p in l {
                u0 = u0.min(p.uv.0);
                u1 = u1.max(p.uv.0);
                v0 = v0.min(p.uv.1);
                v1 = v1.max(p.uv.1);
            }
        }
        ((u0, u1), (v0, v1))
    };
    let (mut sx, mut sy, mut cnt) = (0.0, 0.0, 0.0);
    for fu in [0.25, 0.5, 0.75] {
        for fv in [0.25, 0.5, 0.75] {
            let (u, v) = (u0 + (u1 - u0) * fu, v0 + (v1 - v0) * fv);
            let (su, sv) = surface.derivatives(u, v);
            sx += su.norm();
            sy += sv.norm();
            cnt += 1.0;
        }
    }
    let (sx, sy) = {
        let (a, b) = (sx / cnt, sy / cnt);
        (
            if a > 1e-12 { a } else { 1.0 },
            if b > 1e-12 { b } else { 1.0 },
        )
    };
    let scaled = |uv: (f64, f64)| (uv.0 * sx, uv.1 * sy);

    // ---- rectangle detection (fast paths) ---------------------------------
    let is_rect = loops.len() == 1 && {
        let eps = 1e-9 * (1.0 + (u1 - u0).abs().max((v1 - v0).abs()));
        let on_border = loops[0].iter().all(|p| {
            (p.uv.0 - u0).abs() < eps
                || (p.uv.0 - u1).abs() < eps
                || (p.uv.1 - v0).abs() < eps
                || (p.uv.1 - v1).abs() < eps
        });
        let mut area = 0.0;
        let l = &loops[0];
        for i in 0..l.len() {
            let (a, b) = (l[i].uv, l[(i + 1) % l.len()].uv);
            area += a.0 * b.1 - b.0 * a.1;
        }
        on_border
            && (0.5 * area.abs() - (u1 - u0) * (v1 - v0)).abs()
                < 1e-9 * (1.0 + (u1 - u0) * (v1 - v0))
    };

    // ---- gather all points -------------------------------------------------
    let mut uvs: Vec<(f64, f64)> = Vec::new();
    let mut positions: Vec<Point3<f64>> = Vec::new();
    let mut welds: Vec<u32> = Vec::new();
    let mut loop_ranges: Vec<(usize, usize)> = Vec::new();
    for l in &loops {
        let start = uvs.len();
        for p in l {
            uvs.push(p.uv);
            positions.push(p.pos);
            welds.push(p.weld);
        }
        loop_ranges.push((start, uvs.len()));
    }
    let boundary_count = uvs.len();

    // ---- interior grid -------------------------------------------------------
    // cylinders, cones and extrusions are rectangular in parameter space when untrimmed: no interior points
    let grid = if is_rect
        && matches!(
            surface,
            SurfaceGeom::Cylinder(_) | SurfaceGeom::Cone(_) | SurfaceGeom::Extrusion { .. }
        ) {
        None
    } else {
        st.grid_u.zip(st.grid_v)
    };
    if let Some((du, dv)) = grid {
        let nu = (((u1 - u0) / du).ceil() as usize).max(1);
        let nv = (((v1 - v0) / dv).ceil() as usize).max(1);
        if nu * nv > 8_000_000 {
            return Err(TessError::Triangulation(format!(
                "interior grid too large ({nu}x{nv})"
            )));
        }
        let margin = 0.35 * (sx * (u1 - u0) / nu as f64).min(sy * (v1 - v0) / nv as f64);
        let poly_all: Vec<Vec<(f64, f64)>> = if is_rect {
            Vec::new()
        } else {
            loop_ranges
                .iter()
                .map(|(a, b)| uvs[*a..*b].iter().map(|p| scaled(*p)).collect())
                .collect()
        };
        for i in 1..nu {
            let u = u0 + (u1 - u0) * i as f64 / nu as f64;
            for j in 1..nv {
                let v = v0 + (v1 - v0) * j as f64 / nv as f64;
                if !is_rect {
                    let q = scaled((u, v));
                    if !point_in_polygons(&poly_all, q)
                        || min_dist_to_polygons(&poly_all, q) < margin
                    {
                        continue;
                    }
                }
                uvs.push((u, v));
                positions.push(surface.point_at(u, v));
                welds.push(LOCAL);
            }
        }
    }

    // ---- constrained Delaunay in scaled uv ---------------------------------------
    let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f64>>::new();
    let mut handles = Vec::with_capacity(uvs.len());
    for uv in &uvs {
        let (x, y) = scaled(*uv);
        let h = cdt
            .insert(Point2::new(x, y))
            .map_err(|e| TessError::Triangulation(format!("insert failed: {e:?}")))?;
        handles.push(h);
    }
    let mut index_of_handle = vec![usize::MAX; cdt.num_vertices() + 8];
    for (i, h) in handles.iter().enumerate() {
        let k = h.index();
        if index_of_handle[k] == usize::MAX {
            index_of_handle[k] = i;
        }
    }
    for (a, b) in &loop_ranges {
        let n = b - a;
        for i in 0..n {
            let (p, q) = (handles[a + i], handles[a + (i + 1) % n]);
            if p != q {
                cdt.add_constraint(p, q);
            }
        }
    }

    let poly: Vec<Vec<(f64, f64)>> = if is_rect {
        Vec::new()
    } else {
        loop_ranges
            .iter()
            .map(|(a, b)| uvs[*a..*b].iter().map(|p| scaled(*p)).collect())
            .collect()
    };
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for f in cdt.inner_faces() {
        let vs = f.vertices();
        let idx = [
            index_of_handle[vs[0].index()],
            index_of_handle[vs[1].index()],
            index_of_handle[vs[2].index()],
        ];
        {
            let q: Vec<(f64, f64)> = vs
                .iter()
                .map(|v| (v.position().x, v.position().y))
                .collect();
            let a2 = (q[1].0 - q[0].0) * (q[2].1 - q[0].1) - (q[2].0 - q[0].0) * (q[1].1 - q[0].1);
            let sc = q
                .iter()
                .fold(1.0f64, |m, p| m.max(p.0.abs()).max(p.1.abs()));
            if a2.abs() < 1e-14 * sc * sc {
                continue;
            }
        }
        if !is_rect {
            let p: Vec<(f64, f64)> = vs
                .iter()
                .map(|v| (v.position().x, v.position().y))
                .collect();
            let c = (
                (p[0].0 + p[1].0 + p[2].0) / 3.0,
                (p[0].1 + p[1].1 + p[2].1) / 3.0,
            );
            if !point_in_polygons(&poly, c) {
                continue;
            }
        }
        // drop triangles collapsed onto a shared point (pole / apex rows)
        let w = [
            welds_of(&welds, idx[0], &positions),
            welds_of(&welds, idx[1], &positions),
            welds_of(&welds, idx[2], &positions),
        ];
        if w[0] == w[1] || w[1] == w[2] || w[0] == w[2] {
            continue;
        }
        let t = if flip { [idx[0], idx[2], idx[1]] } else { idx };
        tris.push([t[0] as u32, t[1] as u32, t[2] as u32]);
    }

    // ---- attributes --------------------------------------------------------------
    let sign = if flip { -1.0 } else { 1.0 };
    let normals: Vec<Vector3<f64>> = uvs
        .iter()
        .map(|(u, v)| surface.normal_at(*u, *v) * sign)
        .collect();
    let _ = boundary_count;
    Ok(FaceMesh {
        uv: uvs,
        positions,
        normals,
        weld: welds,
        tris,
    })
}

/// Identity used to detect collapsed triangles: the weld id for shared
/// points; for face-local points the point's own index (never collides).
fn welds_of(welds: &[u32], i: usize, _positions: &[Point3<f64>]) -> u64 {
    if welds[i] == LOCAL {
        (1u64 << 40) + i as u64
    } else {
        welds[i] as u64
    }
}

fn radial_distance(surface: &SurfaceGeom, p: &Point3<f64>) -> f64 {
    match surface {
        SurfaceGeom::Cone(c) => {
            let d = p - c.frame.origin;
            (d - c.frame.z * d.dot(&c.frame.z)).norm()
        }
        SurfaceGeom::Revolution {
            axis_origin,
            axis_dir,
            ..
        } => {
            let a = axis_dir.normalize();
            let d = p - axis_origin;
            (d - a * d.dot(&a)).norm()
        }
        _ => 0.0,
    }
}

fn point_in_polygons(polys: &[Vec<(f64, f64)>], p: (f64, f64)) -> bool {
    let mut inside = false;
    for poly in polys {
        let n = poly.len();
        let mut j = n - 1;
        for i in 0..n {
            let (a, b) = (poly[i], poly[j]);
            if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
                inside = !inside;
            }
            j = i;
        }
    }
    inside
}

fn min_dist_to_polygons(polys: &[Vec<(f64, f64)>], p: (f64, f64)) -> f64 {
    let mut best = f64::INFINITY;
    for poly in polys {
        let n = poly.len();
        for i in 0..n {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let l2 = dx * dx + dy * dy;
            let t = if l2 > 0.0 {
                (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (qx, qy) = (a.0 + t * dx, a.1 + t * dy);
            best = best.min(((p.0 - qx).powi(2) + (p.1 - qy).powi(2)).sqrt());
        }
    }
    best
}
