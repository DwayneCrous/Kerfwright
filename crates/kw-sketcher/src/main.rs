use eframe::egui;
use kw_sketcher::SketcherApp;

const USAGE: &str = "kw-sketcher [--open FILE.dxf] [--demo] [--exit-after-frames N]\n\
\n\
  --open FILE            import a DXF file at start-up\n\
  --demo                 start with a small constrained sketch\n\
  --exit-after-frames N  render N frames, then quit (smoke test)\n";

fn main() -> eframe::Result {
    let mut app = SketcherApp::default();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--open" => {
                let Some(path) = args.next() else {
                    return usage("--open needs a file");
                };
                if let Err(e) = app.open_dxf(&path) {
                    eprintln!("cannot open {path}: {e}");
                    std::process::exit(2);
                }
            }
            "--demo" => app.load_demo(),
            "--exit-after-frames" => match args.next().and_then(|n| n.parse().ok()) {
                Some(n) => app.exit_after_frames = Some(n),
                None => return usage("--exit-after-frames needs a number"),
            },
            "-h" | "--help" => {
                print!("{USAGE}");
                return Ok(());
            }
            other => return usage(&format!("unknown argument {other}")),
        }
    }
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Kerfwright Sketcher")
            .with_inner_size([1280.0, 800.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Kerfwright Sketcher",
        options,
        Box::new(|_cc| Ok(Box::new(app))),
    )
}

fn usage(msg: &str) -> eframe::Result {
    eprintln!("{msg}\n\n{USAGE}");
    std::process::exit(2)
}
