use eframe::egui;
use kw_app::ui::{KerfwrightApp, Workspace};

const USAGE: &str = "kerfwright [PROJECT.kwf] [--demo] [--workspace NAME] [--screenshot FILE.png] [--exit-after-frames N]\n\
\n\
  PROJECT.kwf            open this project instead of showing the welcome page\n\
  --demo                 open the demo part\n\
  --workspace NAME       start in sketch|model|cam|fea|ecad|render (with a project)
  --screenshot FILE      write a PNG of the window after a few frames, then quit
  --exit-after-frames N  render N frames, then quit (smoke test)\n";

fn main() -> eframe::Result {
    let mut app = KerfwrightApp::default();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--demo" => app.open_demo(),
            "--workspace" => {
                let w = match args.next().as_deref() {
                    Some("sketch") => Workspace::Sketch,
                    Some("model") => Workspace::Model,
                    Some("cam") => Workspace::Cam,
                    Some("fea") => Workspace::Fea,
                    Some("ecad") => Workspace::Ecad,
                    Some("render") => Workspace::Render,
                    _ => return usage("--workspace needs sketch|model|cam|fea|ecad|render"),
                };
                app.set_workspace(w);
            }
            "--edit-sketch" => {
                // start inside the first sketch of the project (for screenshots and demos)
                let first = app.project().and_then(|p| {
                    p.document()
                        .features
                        .iter()
                        .find(|f| matches!(f.spec, kw_feature::FeatureSpec::Sketch(_)))
                        .map(|f| f.id)
                });
                if let Some(id) = first {
                    let _ = app.edit_sketch(id);
                }
            }
            "--render-demo" => {
                app.open_demo();
                app.set_workspace(Workspace::Render);
            }
            "--render-pt" => {
                app.render.mode = kw_app::render::Mode::PathTrace;
                app.render.target_spp = 128;
            }
            "--screenshot-after" => match args.next().and_then(|n| n.parse::<u64>().ok()) {
                Some(n) => {
                    if let Some(s) = app.screenshot.as_mut() {
                        s.1 = n;
                    }
                }
                None => return usage("--screenshot-after needs a number"),
            },
            "--fea-demo" => app.fea_demo(),
            "--cam-demo" => app.cam_demo(),
            "--ecad-demo" => app.ecad_demo(5),
            "--screenshot" => match args.next() {
                Some(f) => app.screenshot = Some((f.into(), 30)),
                None => return usage("--screenshot needs a file name"),
            },
            "--exit-after-frames" => match args.next().and_then(|n| n.parse().ok()) {
                Some(n) => app.exit_after_frames = Some(n),
                None => return usage("--exit-after-frames needs a number"),
            },
            "-h" | "--help" => {
                print!("{USAGE}");
                return Ok(());
            }
            path if !path.starts_with('-') => {
                if let Err(e) = app.open_path(std::path::Path::new(path)) {
                    eprintln!("cannot open {path}: {e}");
                    std::process::exit(2);
                }
            }
            other => return usage(&format!("unknown argument {other}")),
        }
    }
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Kerfwright")
            .with_inner_size([1360.0, 860.0]),
        ..Default::default()
    };
    eframe::run_native("Kerfwright", options, Box::new(|_cc| Ok(Box::new(app))))
}

fn usage(msg: &str) -> eframe::Result {
    eprintln!("{msg}\n\n{USAGE}");
    std::process::exit(2)
}
