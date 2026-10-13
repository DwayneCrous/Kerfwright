#![allow(clippy::needless_range_loop)]
//! The 3-D verification suite: raster finishing of 21 seeded parts with fixtures, checked for gouges and
//! shank/holder collisions by the library (drop-cutter, exact cylinder test) and by an independent point
//! sampler (tool body points tested for being inside the solid by ray parity over every triangle).

use kw_cam::bvh::Tri;
use kw_cam::dropcutter::Profile;
use kw_cam::finish3d::{
    check_poses, moves, raster_finish, required_stickout, sample_poses, sections, RasterParams,
};
use kw_cam::suite3d::{suite, SIZE};
use kw_cam::Tool;

/// Is `p` inside the closed triangle soup? Ray parity along +z with a tiny xy offset against edge hits.
fn inside(tris: &[Tri], p: [f64; 3]) -> bool {
    let (x, y) = (p[0] + 1.234_567e-7, p[1] + 2.345_678e-7);
    let mut crossings = 0;
    for t in tris {
        let (a, b, c) = (t[0], t[1], t[2]);
        let d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
        if d.abs() < 1e-14 {
            continue;
        }
        let l1 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / d;
        let l2 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / d;
        let l3 = 1.0 - l1 - l2;
        if l1 >= 0.0 && l2 >= 0.0 && l3 >= 0.0 {
            let z = l1 * a[2] + l2 * b[2] + l3 * c[2];
            if z > p[2] {
                crossings += 1;
            }
        }
    }
    crossings % 2 == 1
}

/// Points inside the tool body, `eps` inside its surface: the lower surface moved up, the side wall moved in.
fn tool_points(tool: &Tool, tip: [f64; 3], eps: f64) -> Vec<[f64; 3]> {
    let prof = Profile::of(tool);
    let mut v = Vec::new();
    for ring in 0..=6 {
        let d = prof.big * ring as f64 / 6.0 * 0.999;
        let nth = if ring == 0 { 1 } else { 6 * ring };
        for k in 0..nth {
            let th = std::f64::consts::TAU * k as f64 / nth as f64;
            v.push([
                tip[0] + d * th.cos(),
                tip[1] + d * th.sin(),
                tip[2] + prof.lift(d) + eps,
            ]);
        }
    }
    for h in [0.3, 0.6, 0.9] {
        for k in 0..16 {
            let th = std::f64::consts::TAU * k as f64 / 16.0;
            let r = prof.big - eps;
            v.push([
                tip[0] + r * th.cos(),
                tip[1] + r * th.sin(),
                tip[2] + prof.r + h * (tool.flute_length - prof.r),
            ]);
        }
    }
    v
}

/// Points inside the shank and holder cylinders (a grid in radius, angle and height).
fn stack_points(tool: &Tool, tip: [f64; 3], eps: f64) -> Vec<[f64; 3]> {
    let mut v = Vec::new();
    for (r, a, b) in sections(tool) {
        for h in 0..=4 {
            let z = tip[2] + a + eps + (b - a - 2.0 * eps) * h as f64 / 4.0;
            for ring in 0..=3 {
                let d = (r - eps) * ring as f64 / 3.0;
                let nth = if ring == 0 { 1 } else { 12 * ring };
                for k in 0..nth {
                    let th = std::f64::consts::TAU * k as f64 / nth as f64;
                    v.push([tip[0] + d * th.cos(), tip[1] + d * th.sin(), z]);
                }
            }
        }
    }
    v
}

struct Counts {
    gouge: usize,
    stack: usize,
}

fn independent_check(tris: &[Tri], tool: &Tool, poses: &[[f64; 3]], skip: usize) -> Counts {
    let mut c = Counts { gouge: 0, stack: 0 };
    let reach = tool
        .radius()
        .max(sections(tool).iter().map(|s| s.0).fold(0.0, f64::max))
        + 0.1;
    for p in poses.iter().step_by(skip) {
        // only triangles whose xy box meets the tool's footprint can matter for a vertical ray
        let near: Vec<Tri> = tris
            .iter()
            .filter(|t| {
                let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
                for v in t.iter() {
                    for k in 0..2 {
                        lo[k] = lo[k].min(v[k]);
                        hi[k] = hi[k].max(v[k]);
                    }
                }
                lo[0] <= p[0] + reach
                    && hi[0] >= p[0] - reach
                    && lo[1] <= p[1] + reach
                    && hi[1] >= p[1] - reach
            })
            .copied()
            .collect();
        // the cutting part: points 5 µm inside the tool surface; in the solid means a cut deeper than 5 µm
        if tool_points(tool, *p, 5e-3)
            .iter()
            .any(|q| inside(&near, *q))
        {
            c.gouge += 1;
        }
        if stack_points(tool, *p, 5e-3)
            .iter()
            .any(|q| inside(&near, *q))
        {
            c.stack += 1;
        }
    }
    c
}

fn tool_with_stickout(stick: f64) -> Tool {
    let mut t = Tool::ball(1, 6.0, 5.0);
    t.shank_radius = 3.0;
    t.shank_length = (stick - t.flute_length).max(0.0);
    t.holder.sections = vec![(10.0, 40.0)];
    t
}

#[test]
fn the_three_dimensional_suite_has_no_gouges_and_no_collisions() {
    // the full suite of 21 parts takes minutes unoptimised; a debug build checks three
    let parts = suite(if cfg!(debug_assertions) { 3 } else { 21 }, 2024);
    let mut total = (0usize, 0usize, 0usize);
    let mut worst_gouge = 0.0f64;
    for part in &parts {
        let mesh = part.part_mesh();
        let scene = part.scene();
        let probe = Tool::ball(1, 6.0, 5.0);
        let par = RasterParams {
            region: [0.0, 0.0, SIZE, SIZE],
            stepover: 1.5,
            step: 1.0,
            tolerance: 0.002,
            min_segment: 0.01,
            floor: 0.0,
            safe_z: part.zmax + 60.0,
            feed: 1200.0,
            plunge_feed: 300.0,
        };
        let t0 = std::time::Instant::now();
        let tp = raster_finish(&scene, &probe, &par);
        let t_path = t0.elapsed().as_secs_f64();
        let mv = moves(&tp);
        let cut_poses = sample_poses(&mv, 0.25);
        // the assembly needs a holder start high enough to clear the part and the jaws along the whole path
        // (rapids at the safe height included)
        let t1 = std::time::Instant::now();
        let need = required_stickout(&scene, 10.0, &cut_poses);
        let t_need = t1.elapsed().as_secs_f64();
        let tool = tool_with_stickout(need + 0.5);
        let t2 = std::time::Instant::now();
        let rep = check_poses(&scene, &tool, &cut_poses, 5e-3);
        let t_check = t2.elapsed().as_secs_f64();
        let t3 = std::time::Instant::now();
        // the library and the independent sampler (every 12th pose: ray parity over all triangles)
        let ind = independent_check(
            &part
                .tris
                .iter()
                .chain(&part.fixture)
                .copied()
                .collect::<Vec<_>>(),
            &tool,
            &cut_poses,
            25,
        );
        eprintln!(
            "   times: path {t_path:.1} s, stick-out {t_need:.1} s, library check {t_check:.1} s, independent {:.1} s",
            t3.elapsed().as_secs_f64()
        );
        eprintln!(
            "{:<22} {:>6} triangles, {:>6} events, {:>6} poses, stick-out {:5.1} mm, max gouge {:.4} mm | lib: {} gouges, {} shank, {} holder | independent: {} gouges, {} stack hits",
            part.name,
            part.tris.len(),
            tp.events.len(),
            cut_poses.len(),
            need + 0.5,
            rep.max_gouge,
            rep.gouges,
            rep.shank_collisions,
            rep.holder_collisions,
            ind.gouge,
            ind.stack
        );
        total.0 += rep.gouges + ind.gouge;
        total.1 += rep.shank_collisions + rep.holder_collisions;
        total.2 += ind.stack;
        worst_gouge = worst_gouge.max(rep.max_gouge);
        let _ = &mesh;
    }
    eprintln!("suite totals: {} gouges, {} library collisions, {} independent stack hits; worst gouge {worst_gouge:.4} mm", total.0, total.1, total.2);
    assert_eq!(total.0, 0);
    assert_eq!(total.1, 0);
    assert_eq!(total.2, 0);
    assert!(worst_gouge <= 5e-3);
}

#[test]
fn both_checkers_catch_a_gouge_and_a_collision_when_one_is_planted() {
    let parts = suite(3, 77);
    for part in &parts {
        let scene = part.scene();
        let probe = Tool::ball(1, 6.0, 5.0);
        let par = RasterParams {
            region: [0.0, 0.0, SIZE, SIZE],
            stepover: 4.0,
            step: 1.0,
            tolerance: 0.002,
            min_segment: 0.05,
            floor: 0.0,
            safe_z: part.zmax + 60.0,
            feed: 1200.0,
            plunge_feed: 300.0,
        };
        let tp = raster_finish(&scene, &probe, &par);
        let poses = sample_poses(&moves(&tp), 0.5);
        let need = required_stickout(&scene, 10.0, &poses);
        let all: Vec<Tri> = part.tris.iter().chain(&part.fixture).copied().collect();

        // plant a gouge: lower every cutting pose by 0.2 mm
        let lowered: Vec<[f64; 3]> = poses.iter().map(|p| [p[0], p[1], p[2] - 0.2]).collect();
        let good = tool_with_stickout(need + 0.5);
        let lib = check_poses(&scene, &good, &lowered, 5e-3);
        let ind = independent_check(&all, &good, &lowered, 6);
        assert!(
            lib.gouges > 0 && ind.gouge > 0,
            "{}: lib {} independent {}",
            part.name,
            lib.gouges,
            ind.gouge
        );

        // plant a collision: a stick-out 3 mm shorter than required
        let short = tool_with_stickout(need - 3.0);
        let lib = check_poses(&scene, &short, &poses, 5e-3);
        let ind = independent_check(&all, &short, &poses, 6);
        assert!(
            lib.holder_collisions + lib.shank_collisions > 0,
            "{}: library missed the collision",
            part.name
        );
        assert!(
            ind.stack > 0,
            "{}: independent sampler missed the collision",
            part.name
        );
    }
}
