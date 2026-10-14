//! The setup sheet: what the machinist needs at the machine, as a web page and as a table.

use super::setup::{estimate_seconds, frame_of, tool_of};
use kw_cam::StockSpec;
use kw_store::{BodyKey, BodyRole, CamSetup};

fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

fn csv_cell(s: &str) -> String {
    if s.contains([',', '"', '\n']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

pub fn stock_text(s: &StockSpec) -> String {
    match s {
        StockSpec::Box { side, top, bottom } => format!(
            "block around the workpiece (+{side:.2} mm sides, +{top:.2} mm top, +{bottom:.2} mm bottom)"
        ),
        StockSpec::Cylinder { axis, margin, ends } => format!(
            "round bar along {} (+{margin:.2} mm radius, +{ends:.2} mm each end)",
            ["X", "Y", "Z"][(*axis as usize).min(2)]
        ),
        StockSpec::Explicit { min, max } => format!(
            "block {:.2} × {:.2} × {:.2} mm",
            max[0] - min[0],
            max[1] - min[1],
            max[2] - min[2]
        ),
        StockSpec::Previous => "what the previous setup left".into(),
    }
}

fn hms(seconds: f64) -> String {
    let s = seconds.round() as u64;
    format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
}

fn role_text(r: BodyRole) -> &'static str {
    match r {
        BodyRole::Workpiece => "workpiece",
        BodyRole::Fixture => "fixture",
        BodyRole::Ignore => "ignored",
    }
}

/// One row of the operation table.
struct OpRow {
    n: usize,
    name: String,
    tool: String,
    rpm: f64,
    feed: f64,
    cut_mm: f64,
    seconds: f64,
}

fn rows(s: &CamSetup) -> Vec<OpRow> {
    s.operations
        .iter()
        .enumerate()
        .map(|(i, o)| {
            let (rpm, feed) = o.params.as_ref().map_or_else(
                || {
                    let mut rpm = 0.0;
                    let mut feed = 0.0;
                    for e in &o.path.events {
                        match e {
                            kw_cam::Event::ToolChange { rpm: r, .. } if rpm == 0.0 => rpm = *r,
                            kw_cam::Event::Feed { feed: f, .. } if feed == 0.0 => feed = *f,
                            _ => {}
                        }
                    }
                    (rpm, feed)
                },
                |p| (p.rpm, p.feed),
            );
            OpRow {
                n: i + 1,
                name: o.name.clone(),
                tool: tool_of(s, o.tool).map_or(format!("T{}", o.tool), |t| {
                    format!("T{} {}", t.number, t.name)
                }),
                rpm,
                feed,
                cut_mm: o.path.cutting_length(),
                seconds: estimate_seconds(&o.path),
            }
        })
        .collect()
}

/// The sheet as a web page.
pub fn html(project: &str, setups: &[CamSetup], name_of: &dyn Fn(&BodyKey) -> String) -> String {
    let mut o = String::new();
    o.push_str(&format!(
        "<!doctype html><html><head><meta charset=\"utf-8\"><title>Setup sheet – {}</title><style>\
body{{font:14px sans-serif;margin:2em;color:#222}}h1{{font-size:1.4em}}h2{{margin-top:1.6em;border-bottom:2px solid #444}}\
table{{border-collapse:collapse;margin:.5em 0}}td,th{{border:1px solid #999;padding:3px 8px;text-align:left}}\
th{{background:#eee}}td.n{{text-align:right}}</style></head><body><h1>Setup sheet – {}</h1>",
        esc(project),
        esc(project)
    ));
    let mut total = 0.0;
    for (i, s) in setups.iter().enumerate() {
        let f = frame_of(&s.wcs);
        o.push_str(&format!(
            "<h2>Setup {} – {}</h2><table>",
            i + 1,
            esc(&s.name)
        ));
        let row = |k: &str, v: String| format!("<tr><th>{}</th><td>{}</td></tr>", k, esc(&v));
        o.push_str(&row("Machine", s.machine.clone()));
        o.push_str(&row(
            "Post-processor",
            s.post.as_ref().map_or("default".into(), |p| p.id.clone()),
        ));
        o.push_str(&row(
            "Work offset",
            format!("G{}", 53 + s.wcs.offset.clamp(1, 6) as u32),
        ));
        o.push_str(&row(
            "Origin (model)",
            format!(
                "X {:.3}  Y {:.3}  Z {:.3}",
                s.wcs.origin[0], s.wcs.origin[1], s.wcs.origin[2]
            ),
        ));
        o.push_str(&row(
            "Tool axis (Z)",
            format!("{:.3} {:.3} {:.3}", f.w[0], f.w[1], f.w[2]),
        ));
        o.push_str(&row(
            "X axis",
            format!("{:.3} {:.3} {:.3}", f.u[0], f.u[1], f.u[2]),
        ));
        o.push_str(&row("Stock", stock_text(&s.stock)));
        for b in &s.bodies {
            o.push_str(&row(
                &format!("Body {}", name_of(&b.key)),
                role_text(b.role).into(),
            ));
        }
        o.push_str("</table><h3>Tools</h3><table><tr><th>Tool</th><th>Name</th><th>Diameter</th><th>Flutes</th><th>Flute length</th><th>Stick-out</th><th>Holder</th></tr>");
        for t in &s.tools {
            let stick = t.flute_length + t.shank_length;
            o.push_str(&format!(
                "<tr><td>T{}</td><td>{}</td><td class=\"n\">{:.2}</td><td class=\"n\">{}</td><td class=\"n\">{:.1}</td><td class=\"n\">{:.1}</td><td>{}</td></tr>",
                t.number,
                esc(&t.name),
                t.diameter,
                t.flutes,
                t.flute_length,
                stick,
                esc(&t.holder_name)
            ));
        }
        o.push_str("</table><h3>Operations</h3><table><tr><th>#</th><th>Operation</th><th>Tool</th><th>Spindle</th><th>Feed</th><th>Cutting</th><th>Time</th></tr>");
        let mut seconds = 0.0;
        for r in rows(s) {
            seconds += r.seconds;
            o.push_str(&format!(
                "<tr><td class=\"n\">{}</td><td>{}</td><td>{}</td><td class=\"n\">{:.0} rpm</td><td class=\"n\">{:.0} mm/min</td><td class=\"n\">{:.0} mm</td><td class=\"n\">{}</td></tr>",
                r.n,
                esc(&r.name),
                esc(&r.tool),
                r.rpm,
                r.feed,
                r.cut_mm,
                hms(r.seconds)
            ));
        }
        o.push_str(&format!(
            "<tr><th colspan=\"6\">Setup time (estimate)</th><th class=\"n\">{}</th></tr></table>",
            hms(seconds)
        ));
        total += seconds;
    }
    o.push_str(&format!(
        "<p><b>Estimated machining time:</b> {} (rapids at {} mm/min)</p></body></html>",
        hms(total),
        super::setup::RAPID_RATE
    ));
    o
}

/// The operations of all setups as CSV.
pub fn csv(setups: &[CamSetup]) -> String {
    let mut o = String::from("setup,work offset,operation number,operation,tool,spindle rpm,feed mm/min,cutting mm,seconds\n");
    for s in setups {
        for r in rows(s) {
            o.push_str(&format!(
                "{},G{},{},{},{},{:.0},{:.0},{:.1},{:.1}\n",
                csv_cell(&s.name),
                53 + s.wcs.offset.clamp(1, 6) as u32,
                r.n,
                csv_cell(&r.name),
                csv_cell(&r.tool),
                r.rpm,
                r.feed,
                r.cut_mm,
                r.seconds
            ));
        }
    }
    o
}
