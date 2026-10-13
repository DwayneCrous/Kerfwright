use kw_brep::BRep;
use kw_drawing::bom::{balloon_scene, bom_csv, table_scene, BALLOON_R, ROW_H};
use kw_drawing::pick::body_at;
use kw_drawing::projection::ViewFrame;
use kw_drawing::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::{extrude, ExtrudeOptions, Profile};

fn rows() -> Vec<BomRowData> {
    vec![
        BomRowData {
            part: "Base plate".into(),
            count: 1,
            material: "Steel".into(),
            volume: 16000.0,
            mass_g: 125.6,
            size: [40.0, 40.0, 10.0],
        },
        BomRowData {
            part: "Bolt, M6".into(),
            count: 4,
            material: String::new(),
            volume: 800.0,
            mass_g: 0.0,
            size: [6.0, 6.0, 20.0],
        },
    ]
}

fn texts(s: &Scene2d) -> Vec<(String, [f64; 2])> {
    s.prims
        .iter()
        .filter_map(|p| match &p.shape {
            Shape::Text { text, p, .. } => Some((text.clone(), *p)),
            _ => None,
        })
        .collect()
}

#[test]
fn the_table_has_its_header_at_the_bottom_and_items_counting_up() {
    let s = table_scene([400.0, 60.0], &BomColumn::defaults(), &rows());
    let t = texts(&s);
    let at = |name: &str| {
        t.iter()
            .find(|(x, _)| x == name)
            .unwrap_or_else(|| panic!("{name} in {t:?}"))
            .1
    };
    // header, then item 1, then item 2, going up
    assert!(at("ITEM")[1] < at("Base plate")[1] && at("Base plate")[1] < at("Bolt, M6")[1]);
    assert!(
        (at("ITEM")[1] - 60.0).abs() < ROW_H,
        "the header sits in the bottom row"
    );
    // cells: quantity, material (a dash when there is none), mass (a dash when unknown)
    for want in ["1", "2", "4", "Steel", "125.6", "-"] {
        assert!(t.iter().any(|(x, _)| x == want), "{want}");
    }
    // the table's lower-right corner is where it was put; it is as tall as its rows
    let (lo, hi) = s.bounds().unwrap();
    assert!((hi[0] - 400.0).abs() < 1e-9 && (lo[1] - 60.0).abs() < 1e-9);
    assert!((hi[1] - lo[1] - 3.0 * ROW_H).abs() < 1e-9);
    // a column that is not chosen is not there
    let s = table_scene([400.0, 60.0], &[BomColumn::Item, BomColumn::Part], &rows());
    assert!(!texts(&s).iter().any(|(x, _)| x == "Steel" || x == "QTY"));
    assert!(table_scene([0.0, 0.0], &[], &rows()).prims.is_empty());
    // wider columns for longer text: more columns, a wider table
    let wide = table_scene([400.0, 60.0], &BomColumn::ALL, &rows());
    assert!(wide.bounds().unwrap().0[0] < lo[0] - 50.0);
}

#[test]
fn the_csv_lists_every_part_and_the_totals() {
    let csv = bom_csv(&rows());
    let lines: Vec<&str> = csv.lines().collect();
    assert_eq!(lines.len(), 4);
    assert_eq!(
        lines[1],
        "1,Base plate,1,Steel,16000.000,125.600,125.600,40.000,40.000,10.000"
    );
    assert!(
        lines[2].starts_with("2,\"Bolt, M6\",4,,800.000,0.000,0.000"),
        "{}",
        lines[2]
    );
    assert_eq!(lines[3], ",Total,5,,,,125.600,,,");
}

fn view() -> (View, ViewCache) {
    let c = ViewCache {
        source_hash: 0,
        scene: Scene2d::new(),
        centre: [0.0, 0.0],
        marks: vec![],
        axes: vec![],
    };
    let v = View {
        id: 1,
        kind: ViewKind::Base,
        source: ViewSource::Assembly,
        orientation: Orientation::Front,
        scale: 2.0,
        style: ViewStyle::Visible,
        pos: [100.0, 100.0],
        label: String::new(),
        cache: Some(c.clone()),
        center_marks: true,
    };
    (v, c)
}

#[test]
fn a_balloon_shows_its_item_and_its_leader_ends_on_the_part() {
    let (v, c) = view();
    // the part's point is at x = 10, z = 5: on the sheet (120, 110)
    let s = balloon_scene([150.0, 140.0], 3, &DimStatus::Ok, [10.0, 7.0, 5.0], &v, &c);
    assert_eq!(
        texts(&s).iter().map(|t| t.0.as_str()).collect::<Vec<_>>(),
        ["3"]
    );
    assert!(s.prims.iter().any(
        |p| matches!(p.shape, Shape::Circle { c, r } if c == [150.0, 140.0] && r == BALLOON_R)
    ));
    let leader = s
        .prims
        .iter()
        .find_map(|p| match p.shape {
            Shape::Line { a, b } => Some((a, b)),
            _ => None,
        })
        .unwrap();
    assert_eq!(leader.1, [120.0, 110.0], "ends on the part");
    // it starts on the circle, not at its centre
    let d = (leader.0[0] - 150.0).hypot(leader.0[1] - 140.0);
    assert!((d - BALLOON_R).abs() < 1e-9);
    assert!(
        s.prims
            .iter()
            .any(|p| matches!(p.shape, Shape::Fill { .. })),
        "the dot"
    );
    // a part that is gone gives a question mark
    let s = balloon_scene(
        [150.0, 140.0],
        3,
        &DimStatus::Broken("gone".into()),
        [10.0, 7.0, 5.0],
        &v,
        &c,
    );
    assert_eq!(texts(&s)[0].0, "?");
}

#[test]
fn the_body_nearest_the_eye_at_a_point_is_the_one_picked() {
    let mut rep = BRep::new();
    let w = Frame3::world();
    let front = extrude(
        &mut rep,
        &Profile::rectangle(w, 20.0, 20.0),
        Vector3::new(0.0, 0.0, 10.0),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    let behind_at = Frame3 {
        origin: Point3::new(0.0, 30.0, 0.0),
        ..w
    };
    let behind = extrude(
        &mut rep,
        &Profile::rectangle(behind_at, 40.0, 20.0),
        Vector3::new(0.0, 0.0, 10.0),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    let f = ViewFrame::of(Orientation::Front); // looks along +y: x right, z up
                                               // in the middle both are there, the one at y = -10 is nearer
    let (i, p) = body_at(&rep, &[front, behind], &f, [0.0, 5.0]).unwrap();
    assert_eq!(i, 0);
    assert!(
        (p[0]).abs() < 1e-9 && (p[1] + 10.0).abs() < 1e-9 && (p[2] - 5.0).abs() < 1e-9,
        "{p:?}"
    );
    // to the side only the wider one
    let (i, p) = body_at(&rep, &[front, behind], &f, [15.0, 5.0]).unwrap();
    assert_eq!(i, 1);
    assert!((p[1] - 20.0).abs() < 1e-9);
    // nothing there
    assert!(body_at(&rep, &[front, behind], &f, [50.0, 5.0]).is_none());
    // from the back the order turns round
    let b = ViewFrame::of(Orientation::Back);
    let (i, _) = body_at(&rep, &[front, behind], &b, [0.0, 5.0]).unwrap();
    assert_eq!(i, 1);
}

#[test]
fn tables_and_balloons_survive_a_save() {
    let mut set = DrawingSet::default();
    let s = set.add_sheet("S", SheetFormat::default());
    let a = set.fresh_id();
    set.sheets[s].annots.push(Annotation {
        id: a,
        kind: AnnotKind::Bom {
            pos: [1.0, 2.0],
            columns: BomColumn::defaults(),
            rows: rows(),
        },
    });
    let b = set.fresh_id();
    set.sheets[s].annots.push(Annotation {
        id: b,
        kind: AnnotKind::Balloon {
            view: 1,
            part: "Base plate".into(),
            instance: 0,
            local: [1.0, 2.0, 3.0],
            world: [4.0, 5.0, 6.0],
            pos: [7.0, 8.0],
            item: 1,
            status: DimStatus::Ok,
        },
    });
    assert_eq!(DrawingSet::from_bytes(&set.to_bytes()).unwrap(), set);
}
