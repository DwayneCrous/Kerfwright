//! Tri-dexel simulation against five closed-form removed volumes (acceptance: within 0.5 %).

use kw_cam::dexel::TriDexel;
use kw_cam::path::{Event, Toolpath};
use kw_cam::Tool;
use std::f64::consts::PI;

fn feed(p: [f64; 3]) -> Event {
    Event::Feed { p, feed: 1000.0 }
}

fn report(name: &str, got: f64, want: f64, d: &TriDexel, removed_run: f64) -> f64 {
    let vols = d.volume();
    let v0 = d.initial_volume();
    let from_rays = [v0 - vols[0], v0 - vols[1], v0 - vols[2]];
    let err = (got - want).abs() / want;
    eprintln!(
        "{name}: expected {want:.3} mm³, z rays {got:.3} ({:+.3} %), x rays {:.3}, y rays {:.3}, accumulated {removed_run:.3}",
        100.0 * (got - want) / want,
        from_rays[1],
        from_rays[2]
    );
    err
}

#[test]
fn rectangular_pocket_with_a_flat_tool() {
    let r = 3.0;
    let tool = Tool::flat(1, 2.0 * r, 20.0);
    let mut d = TriDexel::block([-20.0, -20.0, -10.0], [20.0, 20.0, 0.0], 0.1);
    // tool-centre rectangle 12 x 8, depth 4: perimeter loop plus serpentine rows at stepover 4 (< 2R)
    let (x0, x1, y0, y1, z) = (-6.0, 6.0, -4.0, 4.0, -4.0);
    let mut p = Toolpath::new("pocket");
    p.push(Event::Rapid { p: [x0, y0, 5.0] });
    p.push(feed([x0, y0, z]));
    p.push(feed([x1, y0, z]));
    p.push(feed([x1, y1, z]));
    p.push(feed([x0, y1, z]));
    p.push(feed([x0, y0, z]));
    let mut y = y0;
    let mut left = true;
    while y < y1 - 1e-9 {
        y = (y + 4.0).min(y1);
        p.push(feed([if left { x1 } else { x0 }, y - 0.0, z]));
        p.push(feed([if left { x0 } else { x1 }, y, z]));
        left = !left;
    }
    let run = d.run(&tool, &p);
    let (w, h) = (x1 - x0, y1 - y0);
    let want = (w * h + 2.0 * r * (w + h) + PI * r * r) * 4.0;
    let removed = d.initial_volume() - d.volume()[0];
    let err = report("pocket", removed, want, &d, run);
    assert!(err < 0.005, "{err}");
}

#[test]
fn stadium_slot() {
    let r = 2.5;
    let tool = Tool::flat(1, 2.0 * r, 20.0);
    let mut d = TriDexel::block([-20.0, -10.0, -8.0], [20.0, 10.0, 0.0], 0.1);
    let mut p = Toolpath::new("slot");
    p.push(Event::Rapid {
        p: [-10.0, 1.0, 3.0],
    });
    p.push(feed([-10.0, 1.0, -3.0]));
    p.push(feed([9.0, 4.0, -3.0]));
    let run = d.run(&tool, &p);
    let len = (19.0f64).hypot(3.0);
    let want = (2.0 * r * len + PI * r * r) * 3.0;
    let removed = d.initial_volume() - d.volume()[0];
    let err = report("slot", removed, want, &d, run);
    assert!(err < 0.005, "{err}");
}

#[test]
fn ball_sweep() {
    let r = 4.0;
    let tool = Tool::ball(1, 2.0 * r, 20.0);
    let mut d = TriDexel::block([-20.0, -10.0, -8.0], [20.0, 10.0, 0.0], 0.1);
    let depth = 2.5;
    let mut p = Toolpath::new("ball");
    p.push(Event::Rapid {
        p: [-12.0, 0.0, 3.0],
    });
    p.push(feed([-12.0, 0.0, -depth]));
    p.push(feed([12.0, 0.0, -depth]));
    p.push(feed([12.0, 0.0, 3.0]));
    let run = d.run(&tool, &p);
    // a 24 mm sweep of a circular segment plus the two end caps (one full spherical cap of height `depth`)
    let seg =
        r * r * ((r - depth) / r).acos() - (r - depth) * (2.0 * r * depth - depth * depth).sqrt();
    let cap = PI * depth * depth * (3.0 * r - depth) / 3.0;
    let want = seg * 24.0 + cap;
    let removed = d.initial_volume() - d.volume()[0];
    let err = report("ball sweep", removed, want, &d, run);
    assert!(err < 0.005, "{err}");
}

#[test]
fn helical_ramp() {
    // flat tool, centre path a helix of radius a, ccw, from z = 0 down to −D in N turns
    let (r, a, depth, turns) = (2.0, 5.0, 3.0, 3.0);
    let tool = Tool::flat(1, 2.0 * r, 20.0);
    let mut d = TriDexel::block([-12.0, -12.0, -6.0], [12.0, 12.0, 0.0], 0.1);
    let mut p = Toolpath::new("helix");
    p.push(Event::Rapid { p: [a, 0.0, 2.0] });
    p.push(feed([a, 0.0, 0.0]));
    let per_turn = 8;
    for s in 1..=(per_turn * turns as usize) {
        let th = 2.0 * PI * s as f64 / per_turn as f64;
        let z = -depth * s as f64 / (per_turn as f64 * turns);
        p.push(Event::Arc {
            p: [a * th.cos(), a * th.sin(), z],
            centre: [0.0, 0.0],
            ccw: true,
            feed: 500.0,
        });
    }
    let run = d.run(&tool, &p);
    // reference: for each footprint point P, the lowest tip height is that of the last pose (largest angle)
    // covering it; integrate the depth over the plane in polar coordinates with a fine mesh
    let th_end = 2.0 * PI * turns;
    let z_of = |th: f64| -depth * th / th_end;
    let (nr, nph) = (1500, 3600);
    let (r0, r1) = (a - r, a + r);
    let mut vol = 0.0;
    for i in 0..nr {
        let rho = r0 + (i as f64 + 0.5) / nr as f64 * (r1 - r0);
        let drho = (r1 - r0) / nr as f64;
        let k = (rho * rho + a * a - r * r) / (2.0 * rho * a);
        let alpha = k.clamp(-1.0, 1.0).acos();
        if k > 1.0 {
            continue;
        }
        for j in 0..nph {
            let phi = 2.0 * PI * (j as f64 + 0.5) / nph as f64;
            // covered for θ ∈ [φ − α, φ + α] + 2π m, θ ∈ [0, θ_end]; the lowest pose is the largest such θ
            let mut best: Option<f64> = None;
            let m_max = turns as i32 + 1;
            for m in 0..=m_max {
                let hi = phi + alpha + 2.0 * PI * m as f64;
                let lo = phi - alpha + 2.0 * PI * m as f64;
                if lo > th_end || hi < 0.0 {
                    continue;
                }
                let t = hi.min(th_end);
                if t >= lo.max(0.0) {
                    best = Some(best.map_or(t, |b: f64| b.max(t)));
                }
            }
            if let Some(t) = best {
                vol += -z_of(t) * rho * drho * 2.0 * PI / nph as f64;
            }
        }
    }
    let removed = d.initial_volume() - d.volume()[0];
    let err = report("helix", removed, vol, &d, run);
    assert!(err < 0.005, "{err}");
}

#[test]
fn drilling_with_flat_and_ball_tools() {
    for (tool, name) in [
        (Tool::flat(1, 8.0, 30.0), "flat drill"),
        (Tool::ball(2, 8.0, 30.0), "ball drill"),
    ] {
        let r = 4.0;
        let depth = 7.0;
        let mut d = TriDexel::block([-10.0, -10.0, -10.0], [10.0, 10.0, 0.0], 0.1);
        let mut p = Toolpath::new("drill");
        p.push(Event::Rapid {
            p: [0.3, -0.2, 4.0],
        });
        p.push(feed([0.3, -0.2, -depth]));
        p.push(feed([0.3, -0.2, 4.0]));
        let run = d.run(&tool, &p);
        let want = if name == "flat drill" {
            PI * r * r * depth
        } else {
            PI * r * r * (depth - r) + 2.0 / 3.0 * PI * r * r * r
        };
        let removed = d.initial_volume() - d.volume()[0];
        let err = report(name, removed, want, &d, run);
        assert!(err < 0.005, "{err}");
    }
}
