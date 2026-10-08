use clap::Parser;
use kw_viewer::{app, gpu::Gpu, headless};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "kw-viewer", about = "Kerfwright viewer")]
struct Args {
    /// Render offscreen (no window): benchmark, screenshot, pick test.
    #[arg(long)]
    headless: bool,
    /// `primitives` or `million`.
    #[arg(long, default_value = "primitives")]
    scene: String,
    #[arg(long, default_value_t = 1920)]
    width: u32,
    #[arg(long, default_value_t = 1080)]
    height: u32,
    /// Frames for the headless benchmark.
    #[arg(long, default_value_t = 300)]
    frames: usize,
    /// Triangle target for the `million` scene.
    #[arg(long, default_value_t = 1_000_000)]
    tris: usize,
    /// Save a PNG of the first frame (headless).
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// Run the scripted pick-buffer correctness test (headless).
    #[arg(long)]
    pick_test: bool,
    #[arg(long, default_value_t = 4000)]
    pick_samples: usize,
    /// Windowed: disable vsync to measure uncapped FPS.
    #[arg(long)]
    no_vsync: bool,
    /// Windowed: quit after this many frames (smoke test).
    #[arg(long)]
    exit_after_frames: Option<u64>,
}

fn main() {
    env_logger::init();
    let a = Args::parse();
    if a.headless {
        let gpu = Gpu::headless();
        println!("adapter: {}", gpu.describe());
        let scene = headless::make_scene(&a.scene, a.tris);
        println!(
            "scene `{}`: {} triangles, {} vertices, {} edge segments, {} pick targets",
            a.scene,
            scene.triangle_count(),
            scene.vertices.len(),
            scene.segments.len(),
            scene.pick.len()
        );
        let size = (a.width, a.height);
        if let Some(p) = &a.screenshot {
            headless::screenshot(&gpu, &scene, size, p);
            println!("wrote {}", p.display());
        }
        if a.pick_test {
            let rep = headless::pick_test(&gpu, &scene, size, a.pick_samples);
            println!(
                "pick test: faces {} unambiguous samples ({} on a surface; {} ambiguous skipped), {} mismatches; edges {} tested ({} skipped), {} mismatches",
                rep.face_samples, rep.face_surface_samples, rep.face_ambiguous, rep.face_mismatches.len(), rep.edge_samples, rep.edge_skipped, rep.edge_mismatches.len()
            );
            for m in rep
                .face_mismatches
                .iter()
                .chain(&rep.edge_mismatches)
                .take(10)
            {
                println!("  MISMATCH {m}");
            }
            println!("pick test: {}", if rep.passed() { "PASS" } else { "FAIL" });
            if !rep.passed() {
                std::process::exit(1);
            }
        }
        if a.frames > 0 && !a.pick_test {
            let s = headless::bench_frames(&gpu, &scene, size, a.frames);
            println!(
                "frame time over {} frames @ {}x{}: mean {:.3} ms ({:.1} FPS), p50 {:.3}, p95 {:.3}, p99 {:.3}, max {:.3} ms",
                s.frames, a.width, a.height, s.mean_ms, s.fps_mean, s.p50_ms, s.p95_ms, s.p99_ms, s.max_ms
            );
        }
    } else {
        app::run(
            &a.scene,
            a.tris,
            (a.width, a.height),
            !a.no_vsync,
            a.exit_after_frames,
        );
    }
}
