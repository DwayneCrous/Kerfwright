use kw_drawing::*;
use std::io::Cursor;

fn demo_set() -> DrawingSet {
    let mut set = DrawingSet::default();
    let a = set.add_sheet("Sheet 1", SheetFormat::iso(PaperSize::A3));
    set.sheets[a].title.title = "BRACKET".into();
    set.sheets[a].title.number = "KW-0001".into();
    set.sheets[a].title.author = "D. Crous".into();
    let id = set.fresh_id();
    let mut scene = Scene2d::new();
    scene.rect(Layer::Visible, [0.0, 0.0], [60.0, 40.0]);
    scene.circle(Layer::Visible, [30.0, 20.0], 8.0);
    scene.push(
        Layer::Visible,
        Shape::Arc {
            c: [0.0, 0.0],
            r: 10.0,
            a0: 0.0,
            a1: 1.5,
        },
    );
    scene.line(Layer::Hidden, [5.0, 5.0], [55.0, 5.0]);
    scene.line(Layer::Center, [30.0, -4.0], [30.0, 44.0]);
    scene.text(
        Layer::Dimension,
        [30.0, 48.0],
        3.5,
        Anchor::Center,
        "Ø16 ± 0.1 45°",
    );
    set.sheets[a].views.push(View {
        id,
        kind: ViewKind::Base,
        source: ViewSource::Design,
        orientation: Orientation::Front,
        scale: 1.0,
        style: ViewStyle::VisibleHidden,
        pos: [150.0, 150.0],
        label: "FRONT".into(),
        cache: Some(ViewCache {
            source_hash: 7,
            scene,
            centre: [0.0, 0.0],
            marks: vec![],
            axes: vec![],
        }),
        center_marks: true,
    });
    set.add_sheet("Sheet 2", SheetFormat::asme(PaperSize::AnsiB));
    set
}

#[test]
fn the_frame_lies_inside_the_paper_and_the_title_block_inside_the_border() {
    for size in PaperSize::STANDARD {
        for fmt in [SheetFormat::iso(size), SheetFormat::asme(size)] {
            let f = fmt.frame();
            let s = fmt.scene(&TitleBlock::default(), 1, 1);
            let (lo, hi) = s.bounds().unwrap();
            assert!(lo[0] >= -1e-9 && lo[1] >= -1e-9, "{size:?}");
            assert!(
                hi[0] <= f.paper[0] + 1e-9 && hi[1] <= f.paper[1] + 1e-9,
                "{size:?}"
            );
            assert!(f.title.0[0] >= f.border.0[0] && f.title.1[0] <= f.border.1[0]);
            assert!(f.title.0[1] >= f.border.0[1] && f.title.1[1] <= f.border.1[1]);
        }
    }
}

#[test]
fn title_block_carries_the_fields() {
    let set = demo_set();
    let s = set.sheets[0].scene(1, 2);
    let texts: Vec<&str> = s
        .prims
        .iter()
        .filter_map(|p| match &p.shape {
            Shape::Text { text, .. } => Some(text.as_str()),
            _ => None,
        })
        .collect();
    for want in [
        "BRACKET",
        "KW-0001",
        "D. Crous",
        "1 / 2",
        "ISO mm",
        "SCALE",
        "FRONT   SCALE 1:1",
    ] {
        assert!(texts.contains(&want), "missing {want:?} in {texts:?}");
    }
}

#[test]
fn views_are_placed_on_the_sheet() {
    let set = demo_set();
    let s = set.sheets[0].scene(1, 2);
    // the rectangle of the view, moved to (150, 150)
    assert!(s.prims.iter().any(|p| matches!(&p.shape,
        Shape::Polyline { pts, .. } if pts[0] == [150.0, 150.0] && pts[2] == [210.0, 190.0])));
}

#[test]
fn scale_labels() {
    assert_eq!(scale_label(1.0), "1:1");
    assert_eq!(scale_label(2.0), "2:1");
    assert_eq!(scale_label(0.5), "1:2");
    assert_eq!(scale_label(0.04), "1:25");
}

/// Offsets listed in the xref table must point at "N 0 obj".
#[test]
fn pdf_structure_is_valid() {
    let set = demo_set();
    let pdf = set_to_pdf("Demo", &set);
    assert!(pdf.starts_with(b"%PDF-1.4\n"));
    // everything after the binary comment line is ASCII
    let body_at = pdf.iter().position(|&b| b == b'\n').unwrap() + 1;
    let body_at = body_at + pdf[body_at..].iter().position(|&b| b == b'\n').unwrap() + 1;
    let tail = std::str::from_utf8(&pdf[body_at..]).unwrap();
    assert!(tail.trim_end().ends_with("%%EOF"));
    let sx = tail.rfind("startxref\n").unwrap();
    let xref_at: usize = tail[sx + 10..].lines().next().unwrap().parse().unwrap();
    assert!(pdf[xref_at..].starts_with(b"xref\n"));
    let mut lines = tail[xref_at - body_at..].lines();
    lines.next();
    let head: Vec<usize> = lines
        .next()
        .unwrap()
        .split(' ')
        .map(|v| v.parse().unwrap())
        .collect();
    assert_eq!(head[0], 0);
    let count = head[1];
    // 4 fixed objects + 2 per page + the free entry
    assert_eq!(count, 4 + 2 * 2 + 1);
    lines.next(); // free entry
    for obj in 1..count {
        let l = lines.next().unwrap();
        assert_eq!(l.len(), 19, "entries are 20 bytes with the newline: {l:?}");
        let off: usize = l[..10].parse().unwrap();
        assert!(
            pdf[off..].starts_with(format!("{obj} 0 obj\n").as_bytes()),
            "object {obj} not at offset {off}"
        );
    }
    let text = tail;
    // page sizes: A3 and ANSI B in points
    assert!(
        text.contains("/MediaBox [0 0 1190.5512 841.8898]"),
        "{text:.2000}"
    );
    assert!(text.contains("/MediaBox [0 0 1224 792]"));
    assert!(text.contains("/Count 2"));
    // /Length of every stream matches its bytes
    for part in text.split("/Length ").skip(1) {
        let len: usize = part.split(' ').next().unwrap().parse().unwrap();
        let body = part.split_once("stream\n").unwrap().1;
        assert_eq!(body.find("endstream").unwrap(), len);
    }
}

#[test]
fn pdf_is_deterministic_and_uses_winansi_for_drafting_symbols() {
    let set = demo_set();
    let a = set_to_pdf("Demo", &set);
    let b = set_to_pdf("Demo", &set);
    assert_eq!(a, b);
    let text = String::from_utf8_lossy(&a);
    // "Ø16 ± 0.1 45°" as hex WinAnsi
    assert!(
        text.contains("<D8313620B1203 02E3120343 5B0>".replace(' ', "").as_str()),
        "{text:.3000}"
    );
}

#[test]
fn dxf_is_read_by_an_independent_reader_with_layers_and_text() {
    let set = demo_set();
    let text = sheet_to_dxf(&set, 0).unwrap();
    let d = dxf::Drawing::load(&mut Cursor::new(text.as_bytes())).unwrap();
    let layer = |n: &str| d.layers().find(|l| l.name == n).unwrap();
    assert_eq!(layer("HIDDEN").line_type_name, "HIDDEN");
    assert_eq!(layer("CENTER").line_type_name, "CENTER");
    let n_text = d
        .entities()
        .filter(|e| matches!(e.specific, dxf::entities::EntityType::Text(_)))
        .count();
    assert!(n_text > 10, "title block text present: {n_text}");
    // drafting symbols become DXF %% codes
    assert!(d.entities().any(|e| matches!(&e.specific,
        dxf::entities::EntityType::Text(t) if t.value == "%%c16 %%p 0.1 45%%d")));
    // and our own reader reads it back to the same entities
    let ours = kw_dxf::parse(&text).unwrap();
    assert_eq!(ours.entities.len(), d.entities().count());
}

#[test]
fn drawing_set_round_trips_through_cbor() {
    let set = demo_set();
    let back = DrawingSet::from_bytes(&set.to_bytes()).unwrap();
    assert_eq!(set, back);
    // an empty set from an older file with no fields still reads
    let mut empty = Vec::new();
    ciborium::into_writer(&ciborium::value::Value::Map(vec![]), &mut empty).unwrap();
    assert_eq!(
        DrawingSet::from_bytes(&empty).unwrap(),
        DrawingSet::default()
    );
}
