//! The geometry of a setup: its work coordinate system, the workpiece and fixture bodies in it, the faces
//! that can serve as machining regions, the program the setup posts, and the hash that tells whether an
//! operation was made for the setup as it is now.

use crate::model_source::{ModelSource, SourceBody};
use kw_cam::multiaxis::Frame;
use kw_cam::path::{Event, Toolpath};
use kw_cam::{OpParams, Tool};
use kw_store::{BodyRole, CamSetup, Wcs};

pub type P3 = [f64; 3];
pub type Tris = Vec<[P3; 3]>;

/// Rapid traverse used to estimate cycle times (mm/min).
pub const RAPID_RATE: f64 = 10_000.0;

fn unit(a: P3) -> Option<P3> {
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    (n > 1e-12).then(|| [a[0] / n, a[1] / n, a[2] / n])
}

fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The frame of a work coordinate system: z along `z_axis`, x as near to `x_axis` as perpendicularity
/// allows. A degenerate axis falls back to the world axis.
pub fn frame_of(w: &Wcs) -> Frame {
    let z = unit(w.z_axis).unwrap_or([0.0, 0.0, 1.0]);
    let x = unit(w.x_axis).unwrap_or([1.0, 0.0, 0.0]);
    let d = dot(x, z);
    let xp = unit([x[0] - d * z[0], x[1] - d * z[1], x[2] - d * z[2]]).unwrap_or_else(|| {
        // x was along z: take the world axis least aligned with z
        let helper = if z[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let d = dot(helper, z);
        unit([
            helper[0] - d * z[0],
            helper[1] - d * z[1],
            helper[2] - d * z[2],
        ])
        .expect("a frame")
    });
    let v = [
        z[1] * xp[2] - z[2] * xp[1],
        z[2] * xp[0] - z[0] * xp[2],
        z[0] * xp[1] - z[1] * xp[0],
    ];
    Frame {
        origin: w.origin,
        u: xp,
        v,
        w: z,
    }
}

/// The work coordinate system for looking at a face from outside it: origin at `origin`, z along `normal`.
pub fn wcs_looking_at(origin: P3, normal: P3, offset: u8) -> Wcs {
    let frame = Frame::from_axis(origin, normal);
    Wcs {
        origin,
        x_axis: frame.u,
        z_axis: frame.w,
        offset,
    }
}

pub fn map_tris(tris: &Tris, f: impl Fn(P3) -> P3) -> Tris {
    tris.iter().map(|t| [f(t[0]), f(t[1]), f(t[2])]).collect()
}

/// A body tessellated: triangles in world coordinates and the face of each (index among the body's faces).
pub struct BodyMesh {
    pub tris: Tris,
    pub tri_face: Vec<u32>,
}

pub fn tessellate(b: &SourceBody) -> Option<BodyMesh> {
    let t = kw_tess::tessellate_body(
        &b.brep,
        b.body,
        &kw_tess::TessOptions {
            chord_tol: 0.01,
            angle_tol: 0.15,
        },
    )
    .ok()?;
    let p = |i: u32| {
        let q = t.mesh.positions[i as usize];
        [q.x, q.y, q.z]
    };
    let tris: Tris = t
        .mesh
        .indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|c| [p(c[0]), p(c[1]), p(c[2])])
        .collect();
    (!tris.is_empty()).then(|| BodyMesh {
        tris,
        tri_face: t.mesh.tri_face.clone(),
    })
}

/// A flat face perpendicular to the work z axis: a candidate floor, top or outline for an operation.
#[derive(Debug, Clone)]
pub struct FlatFace {
    pub body: String,
    /// Height of the face in the work coordinate system.
    pub z: f64,
    pub area: f64,
    /// True when the face looks up (towards +z): a floor or a top. A face looking down is a bottom.
    pub up: bool,
    pub outer: Vec<[f64; 2]>,
    pub islands: Vec<Vec<[f64; 2]>>,
}

/// A round hole along the work z axis, found from the cylindrical faces of the workpiece.
#[derive(Debug, Clone, PartialEq)]
pub struct Hole {
    pub centre: [f64; 2],
    pub diameter: f64,
    /// Heights of the ends of the cylinder.
    pub top: f64,
    pub bottom: f64,
    /// The hole goes through the workpiece (its bottom is the workpiece's lowest point).
    pub through: bool,
}

/// The bodies of a setup in its work coordinate system.
pub struct SetupGeometry {
    pub frame: Frame,
    pub workpiece: Tris,
    pub fixtures: Tris,
    pub faces: Vec<FlatFace>,
    pub holes: Vec<Hole>,
}

/// A piece of a cylinder found on a face: its circle, its extent in z and the angles it covers.
struct Arc {
    centre: [f64; 2],
    r: f64,
    z: (f64, f64),
    angles: Vec<f64>,
}

/// If the triangles of a face are a piece of a cylinder about a z-parallel axis with its surface facing the
/// axis (a hole, not a boss), the piece.
fn hole_arc(tris: &Tris) -> Option<Arc> {
    // every triangle stands upright
    let mut pts: Vec<[f64; 2]> = Vec::new();
    let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for t in tris {
        let (u, v) = (
            [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]],
            [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]],
        );
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let l = dot(n, n).sqrt();
        if l < 1e-12 {
            continue;
        }
        if (n[2] / l).abs() > 0.02 {
            return None;
        }
        for p in t {
            pts.push([p[0], p[1]]);
            zmin = zmin.min(p[2]);
            zmax = zmax.max(p[2]);
        }
    }
    if pts.len() < 6 {
        return None;
    }
    // a circle through the points: the algebraic least-squares fit
    let n = pts.len() as f64;
    let (mx, my) = (
        pts.iter().map(|p| p[0]).sum::<f64>() / n,
        pts.iter().map(|p| p[1]).sum::<f64>() / n,
    );
    let (mut suu, mut suv, mut svv, mut suuu, mut svvv, mut suvv, mut svuu) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for p in &pts {
        let (u, v) = (p[0] - mx, p[1] - my);
        suu += u * u;
        suv += u * v;
        svv += v * v;
        suuu += u * u * u;
        svvv += v * v * v;
        suvv += u * v * v;
        svuu += v * u * u;
    }
    let det = suu * svv - suv * suv;
    if det.abs() < 1e-12 {
        return None;
    }
    let (b1, b2) = (0.5 * (suuu + suvv), 0.5 * (svvv + svuu));
    let (uc, vc) = ((b1 * svv - b2 * suv) / det, (b2 * suu - b1 * suv) / det);
    let centre = [mx + uc, my + vc];
    let r = (uc * uc + vc * vc + (suu + svv) / n).sqrt();
    if r < 0.05
        || pts
            .iter()
            .any(|p| ((p[0] - centre[0]).hypot(p[1] - centre[1]) - r).abs() > 0.02 * r + 0.01)
    {
        return None;
    }
    // facing the axis: the normal of a triangle points to the centre
    let mut inward = 0;
    let mut total = 0;
    for t in tris {
        let c = [
            (t[0][0] + t[1][0] + t[2][0]) / 3.0 - centre[0],
            (t[0][1] + t[1][1] + t[2][1]) / 3.0 - centre[1],
        ];
        let (u, v) = (
            [t[1][0] - t[0][0], t[1][1] - t[0][1]],
            [t[2][0] - t[0][0], t[2][1] - t[0][1]],
        );
        // the horizontal part of the normal of an upright triangle
        let nx = u[1] * (t[2][2] - t[0][2]) - (t[1][2] - t[0][2]) * v[1];
        let ny = (t[1][2] - t[0][2]) * v[0] - u[0] * (t[2][2] - t[0][2]);
        if nx.hypot(ny) < 1e-12 {
            continue;
        }
        total += 1;
        if nx * c[0] + ny * c[1] < 0.0 {
            inward += 1;
        }
    }
    if total == 0 || inward * 2 < total {
        return None;
    }
    Some(Arc {
        centre,
        r,
        z: (zmin, zmax),
        angles: pts
            .iter()
            .map(|p| (p[1] - centre[1]).atan2(p[0] - centre[0]))
            .collect(),
    })
}

/// Pieces of cylinder that lie on the same circle and height range are one hole when together they go all
/// the way round.
fn holes_from(arcs: Vec<Arc>, floor: f64) -> Vec<Hole> {
    let mut groups: Vec<Arc> = Vec::new();
    for a in arcs {
        match groups.iter_mut().find(|g| {
            (g.centre[0] - a.centre[0]).abs() < 0.02
                && (g.centre[1] - a.centre[1]).abs() < 0.02
                && (g.r - a.r).abs() < 0.02
                && (g.z.0 - a.z.0).abs() < 0.02
                && (g.z.1 - a.z.1).abs() < 0.02
        }) {
            Some(g) => g.angles.extend(a.angles),
            None => groups.push(a),
        }
    }
    groups
        .into_iter()
        .filter(|g| {
            let mut a = g.angles.clone();
            a.sort_by(f64::total_cmp);
            let mut gap = 0.0f64;
            for w in a.windows(2) {
                gap = gap.max(w[1] - w[0]);
            }
            gap = gap.max(a[0] + std::f64::consts::TAU - a[a.len() - 1]);
            gap < 70f64.to_radians()
        })
        .map(|g| Hole {
            centre: g.centre,
            diameter: 2.0 * g.r,
            top: g.z.1,
            bottom: g.z.0,
            through: (g.z.0 - floor).abs() < 1e-3,
        })
        .collect()
}

fn area_of(p: &[[f64; 2]]) -> f64 {
    let mut a = 0.0;
    for i in 0..p.len() {
        let (q, r) = (p[i], p[(i + 1) % p.len()]);
        a += q[0] * r[1] - r[0] * q[1];
    }
    0.5 * a.abs()
}

/// Tessellate the bodies of `source` as `setup` uses them and move them into its work coordinate system.
pub fn geometry(source: &ModelSource, setup: &CamSetup) -> Result<SetupGeometry, String> {
    let frame = frame_of(&setup.wcs);
    let to = |p: P3| frame.to_frame(p);
    let (mut workpiece, mut fixtures, mut faces) = (Vec::new(), Vec::new(), Vec::new());
    let mut arcs: Vec<Arc> = Vec::new();
    for b in &source.bodies {
        let role = setup.role_of(&b.key);
        if role == BodyRole::Ignore {
            continue;
        }
        let Some(m) = tessellate(b) else {
            continue;
        };
        let local = map_tris(&m.tris, to);
        if role == BodyRole::Fixture {
            fixtures.extend(local);
            continue;
        }
        // flat faces
        let nfaces = m.tri_face.iter().copied().max().map_or(0, |m| m + 1) as usize;
        let mut by_face: Vec<Tris> = vec![Vec::new(); nfaces];
        for (t, f) in local.iter().zip(&m.tri_face) {
            by_face[*f as usize].push(*t);
        }
        for tris in by_face {
            if let Some(a) = hole_arc(&tris) {
                arcs.push(a);
                continue;
            }
            let mut up = 0usize;
            let mut flat = true;
            let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
            for t in &tris {
                let (u, v) = (
                    [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]],
                    [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]],
                );
                let n = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                let len = dot(n, n).sqrt();
                if len < 1e-12 {
                    continue;
                }
                if (n[2] / len).abs() < 0.9999 {
                    flat = false;
                    break;
                }
                if n[2] > 0.0 {
                    up += 1;
                }
                for p in t {
                    zmin = zmin.min(p[2]);
                    zmax = zmax.max(p[2]);
                }
            }
            if !flat || tris.is_empty() || zmax - zmin > 1e-4 {
                continue;
            }
            let Some((outer, islands)) = kw_cam::regions::face_loops(&tris) else {
                continue;
            };
            let tol = 0.002;
            let outer = kw_cam::regions::simplify(&outer, tol);
            let islands: Vec<_> = islands
                .iter()
                .map(|i| kw_cam::regions::simplify(i, tol))
                .collect();
            let area = area_of(&outer) - islands.iter().map(|i| area_of(i)).sum::<f64>();
            faces.push(FlatFace {
                body: b.name.clone(),
                z: 0.5 * (zmin + zmax),
                area,
                up: up * 2 >= tris.len(),
                outer,
                islands,
            });
        }
        workpiece.extend(local);
    }
    if workpiece.is_empty() {
        return Err("the setup has no workpiece body".into());
    }
    faces.sort_by(|a, b| b.z.total_cmp(&a.z).then(b.area.total_cmp(&a.area)));
    let floor = workpiece
        .iter()
        .flat_map(|t| t.iter().map(|p| p[2]))
        .fold(f64::INFINITY, f64::min);
    let mut holes = holes_from(arcs, floor);
    holes.sort_by(|a, b| {
        a.diameter
            .total_cmp(&b.diameter)
            .then(a.centre[0].total_cmp(&b.centre[0]))
            .then(a.centre[1].total_cmp(&b.centre[1]))
    });
    Ok(SetupGeometry {
        frame,
        workpiece,
        fixtures,
        faces,
        holes,
    })
}

/// Hash of what a setup's operations depend on: the model, the roles of its bodies, the work coordinate
/// system and the stock.
pub fn geometry_key(source: &ModelSource, setup: &CamSetup) -> u128 {
    let mut h = kw_feature::Hasher128::new();
    h.u128(source.key);
    h.bytes(format!("{:?}|{:?}|{:?}", setup.bodies, setup.wcs, setup.stock).as_bytes());
    h.finish()
}

/// What an operation was generated from: its parameters, its tool and the geometry key.
pub fn op_hash(params: &OpParams, tool: &Tool, geometry: u128) -> u64 {
    let mut h = kw_feature::Hasher128::new();
    h.u128(geometry);
    h.bytes(format!("{params:?}|{tool:?}").as_bytes());
    let v = h.finish();
    (v ^ (v >> 64)) as u64
}

/// The program of a setup: its operations in order under one work offset, with the program end only once.
pub fn setup_program(setup: &CamSetup) -> Toolpath {
    let mut tp = Toolpath::new(&setup.name);
    tp.push(Event::Comment(format!("setup {}", setup.name)));
    tp.push(Event::WorkOffset {
        number: setup.wcs.offset,
    });
    for op in &setup.operations {
        tp.push(Event::Comment(op.name.clone()));
        tp.events.extend(
            op.path
                .events
                .iter()
                .filter(|e| !matches!(e, Event::ProgramEnd))
                .cloned(),
        );
    }
    tp.push(Event::ProgramEnd);
    tp
}

/// Seconds the toolpath takes: cutting length at the feed of each move plus the rapids.
pub fn estimate_seconds(tp: &Toolpath) -> f64 {
    let mut pos: Option<P3> = None;
    let mut seconds = 0.0;
    for e in &tp.events {
        match e {
            Event::Rapid { p } => {
                if let Some(q) = pos {
                    seconds += dist(q, *p) / RAPID_RATE * 60.0;
                }
                pos = Some(*p);
            }
            Event::Feed { p, feed } => {
                if let Some(q) = pos {
                    seconds += dist(q, *p) / feed.max(1.0) * 60.0;
                }
                pos = Some(*p);
            }
            Event::Arc { p, feed, .. } => {
                if let Some(q) = pos {
                    let mut mini = Toolpath::new("arc");
                    mini.push(Event::Rapid { p: q });
                    mini.push(e.clone());
                    seconds += mini.cutting_length() / feed.max(1.0) * 60.0;
                }
                pos = Some(*p);
            }
            _ => {}
        }
    }
    seconds
}

fn dist(a: P3, b: P3) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The tool of the setup with this number.
pub fn tool_of(setup: &CamSetup, number: u32) -> Option<&Tool> {
    setup.tools.iter().find(|t| t.number == number)
}

/// The number of a tool equal to `tool` (ignoring its number) in setup `index`, adding it with a number when
/// the setup lacks it. Tools are numbered across the whole project, so one physical tool has one number in
/// every setup and two different tools never share one.
pub fn tool_number(setups: &mut [CamSetup], index: usize, tool: &Tool) -> u32 {
    let plain = |t: &Tool| Tool {
        number: 0,
        ..t.clone()
    };
    let want = plain(tool);
    if let Some(t) = setups[index].tools.iter().find(|t| plain(t) == want) {
        return t.number;
    }
    let number = setups
        .iter()
        .flat_map(|s| s.tools.iter())
        .find(|t| plain(t) == want)
        .map(|t| t.number)
        .unwrap_or_else(|| {
            setups
                .iter()
                .flat_map(|s| s.tools.iter())
                .map(|t| t.number)
                .max()
                .unwrap_or(0)
                + 1
        });
    setups[index].tools.push(Tool {
        number,
        ..tool.clone()
    });
    number
}
