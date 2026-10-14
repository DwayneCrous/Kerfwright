//! Stock simulation of a setup: the stock is cut by every operation with its own tool, compared with the
//! workpiece, and the tools are checked against the fixtures. A setup may start from what the previous
//! setup left.

use super::setup::{map_tris, tool_of, SetupGeometry, Tris, P3};
use kw_cam::bvh::TriMesh;
use kw_cam::dexel::{Facet, TriDexel};
use kw_cam::finish3d::{check_poses, moves, required_stickout, sample_poses};
use kw_job::{CancelFlag, Progress};
use kw_store::CamSetup;

/// What one operation did to the stock and what it came near.
#[derive(Debug, Clone, Default)]
pub struct OpSim {
    pub name: String,
    pub tool: u32,
    /// mm³ taken from the stock.
    pub removed: f64,
    /// mm³ taken by rapid moves (a rapid through material is a collision).
    pub rapid_cut: f64,
    /// Poses at which the tool tip is inside a fixture.
    pub fixture_gouges: usize,
    pub shank_collisions: usize,
    pub holder_collisions: usize,
    /// The stick-out (mm from the tip to the holder) that keeps the holder clear of the fixtures, if any.
    pub required_stickout: Option<f64>,
    pub seconds: f64,
    /// Why the operation was not simulated.
    pub skipped: Option<String>,
    /// The tool is tilted in this operation: the fixtures were not checked (the check assumes an upright
    /// tool).
    pub fixture_unchecked: bool,
}

impl OpSim {
    pub fn collides(&self) -> bool {
        self.fixture_gouges + self.shank_collisions + self.holder_collisions > 0
            || self.rapid_cut > 1e-3
    }
}

/// The result for one setup.
#[derive(Debug, Clone)]
pub struct SimResult {
    pub setup: String,
    /// The edge of a dexel cell (mm): the sideways accuracy of the surface.
    pub cell: f64,
    pub stock_volume: f64,
    pub final_volume: f64,
    /// mm³ of stock still outside the workpiece.
    pub remaining: f64,
    /// mm³ of workpiece cut away.
    pub overcut: f64,
    pub ops: Vec<OpSim>,
    /// The surface of the stock in work coordinates, each triangle with its distance from the workpiece
    /// along the normal (positive: material left; negative: cut too deep).
    pub surface: Vec<(Facet, f32)>,
    /// The same surface as triangles in world coordinates, for the next setup and for display.
    pub world: Tris,
}

/// The stock of a setup: its mesh in work coordinates and its box.
pub struct StockMesh {
    pub mesh: TriMesh,
    pub min: P3,
    pub max: P3,
}

/// The stock `setup` starts from. `previous` is the surface the previous setup left, in world coordinates.
pub fn stock_of(
    setup: &CamSetup,
    geo: &SetupGeometry,
    previous: Option<&Tris>,
) -> Result<StockMesh, String> {
    use kw_cam::StockSpec;
    if setup.stock == StockSpec::Previous {
        let Some(prev) = previous else {
            return Err("this is the first setup: there is no earlier stock to start from".into());
        };
        let local = map_tris(prev, |p| geo.frame.to_frame(p));
        let b = kw_cam::stock::bounds_of(&local);
        return Ok(StockMesh {
            mesh: TriMesh::new(local),
            min: b.min,
            max: b.max,
        });
    }
    let part = TriMesh::new(geo.workpiece.clone());
    let s = setup
        .stock
        .shape(&part)
        .ok_or("the workpiece has no extent")?;
    Ok(StockMesh {
        mesh: s.mesh,
        min: s.min,
        max: s.max,
    })
}

/// The cell size that keeps a stock of this size near two million cells, at least 0.25 mm.
pub fn cell_for(min: P3, max: P3) -> f64 {
    let vol = (max[0] - min[0]).max(1.0) * (max[1] - min[1]).max(1.0) * (max[2] - min[2]).max(1.0);
    (vol / 2.0e6).cbrt().max(0.25)
}

/// The stock of `setup` as a dexel grid and the workpiece on the same grid.
pub fn grids(
    setup: &CamSetup,
    geo: &SetupGeometry,
    previous: Option<&Tris>,
) -> Result<(TriDexel, TriDexel), String> {
    let stock = stock_of(setup, geo, previous)?;
    let h = cell_for(stock.min, stock.max);
    let d = TriDexel::from_mesh(&stock.mesh, stock.min, stock.max, h);
    let part = TriDexel::from_mesh(&TriMesh::new(geo.workpiece.clone()), d.min, d.max, h);
    Ok((d, part))
}

/// The stock of `setup` after its first `count` operations.
pub fn stock_after(
    setup: &CamSetup,
    geo: &SetupGeometry,
    previous: Option<&Tris>,
    count: usize,
) -> Result<TriDexel, String> {
    let (mut d, _) = grids(setup, geo, previous)?;
    for op in setup.operations.iter().take(count) {
        if let Some(tool) = tool_of(setup, op.tool) {
            d.lift_tool();
            d.run(tool, &op.path);
        }
    }
    Ok(d)
}

/// What setup `index` starts from when its stock is "what the previous setup left": the surface the
/// previous setups' operations leave, in world coordinates (simulated from the first setup of the chain).
pub fn previous_surface(
    source: &crate::model_source::ModelSource,
    setups: &[CamSetup],
    index: usize,
    cancel: &CancelFlag,
) -> Result<Option<Tris>, String> {
    if index == 0 || setups[index].stock != kw_cam::StockSpec::Previous {
        return Ok(None);
    }
    let mut first = index - 1;
    while first > 0 && setups[first].stock == kw_cam::StockSpec::Previous {
        first -= 1;
    }
    let mut previous: Option<Tris> = None;
    for setup in &setups[first..index] {
        let geo = super::setup::geometry(source, setup)?;
        let r = simulate_setup(setup, &geo, previous.as_ref(), cancel, &Progress::new())?;
        previous = Some(r.world);
    }
    Ok(previous)
}

/// Run all operations of `setup` on its stock. Poses along the paths are checked against the fixtures.
pub fn simulate_setup(
    setup: &CamSetup,
    geo: &SetupGeometry,
    previous: Option<&Tris>,
    cancel: &CancelFlag,
    progress: &Progress,
) -> Result<SimResult, String> {
    let (mut d, part) = grids(setup, geo, previous)?;
    let h = d.h;
    let stock_volume = d.volume()[0];
    let fixtures = (!geo.fixtures.is_empty()).then(|| TriMesh::new(geo.fixtures.clone()));
    let n = setup.operations.len().max(1);
    let mut ops = Vec::new();
    for (k, op) in setup.operations.iter().enumerate() {
        if cancel.is_cancelled() {
            return Err("cancelled".into());
        }
        progress.set(k as f64 / n as f64);
        let mut o = OpSim {
            name: op.name.clone(),
            tool: op.tool,
            seconds: super::setup::estimate_seconds(&op.path),
            ..OpSim::default()
        };
        let Some(tool) = tool_of(setup, op.tool) else {
            o.skipped = Some(format!("tool T{} is not in the setup", op.tool));
            ops.push(o);
            continue;
        };
        d.lift_tool();
        let before = d.rapid_removed;
        let before_v = d.volume()[0];
        d.run(tool, &op.path);
        o.rapid_cut = d.rapid_removed - before;
        o.removed = before_v - d.volume()[0];
        let tilted = op
            .path
            .events
            .iter()
            .any(|e| matches!(e, kw_cam::Event::Pose { axis, .. } if axis[0].abs() > 1e-6 || axis[1].abs() > 1e-6));
        o.fixture_unchecked = tilted && fixtures.is_some();
        if let (Some(fix), false) = (&fixtures, tilted) {
            let mv = moves(&op.path);
            let len: f64 = mv
                .iter()
                .map(|(a, b)| {
                    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
                })
                .sum();
            // at most about 20 000 poses per operation
            let spacing = (len / 20_000.0).max(1.0);
            let poses = sample_poses(&mv, spacing);
            let rep = check_poses(fix, tool, &poses, 0.01);
            o.fixture_gouges = rep.gouges;
            o.shank_collisions = rep.shank_collisions;
            o.holder_collisions = rep.holder_collisions;
            let holder_r = tool
                .holder
                .sections
                .first()
                .map_or(tool.shank_radius, |s| s.0);
            let need = required_stickout(fix, holder_r, &poses);
            o.required_stickout = need.is_finite().then_some(need.max(0.0));
        }
        ops.push(o);
    }
    progress.set(0.9);
    let final_volume = d.volume()[0];
    let cmp = d.compare(&part);
    let facets = d.facets();
    let surface: Vec<(Facet, f32)> = facets
        .iter()
        .map(|f| {
            let dev = d.facet_deviation(&part, f);
            (*f, if dev.is_finite() { dev as f32 } else { 1e6 })
        })
        .collect();
    let world = surface
        .iter()
        .map(|(f, _)| {
            [
                geo.frame.to_work(f.tri[0]),
                geo.frame.to_work(f.tri[1]),
                geo.frame.to_work(f.tri[2]),
            ]
        })
        .collect();
    progress.set(1.0);
    Ok(SimResult {
        setup: setup.name.clone(),
        cell: h,
        stock_volume,
        final_volume,
        remaining: cmp.remaining,
        overcut: cmp.overcut,
        ops,
        surface,
        world,
    })
}
