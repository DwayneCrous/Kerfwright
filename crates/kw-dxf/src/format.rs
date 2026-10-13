use std::fmt::Write as _;

pub type P2 = (f64, f64);

#[derive(Debug, Clone, PartialEq)]
pub enum DxfEntity {
    Point {
        layer: String,
        p: P2,
    },
    Line {
        layer: String,
        a: P2,
        b: P2,
    },
    Circle {
        layer: String,
        c: P2,
        r: f64,
    },
    /// Counter-clockwise from `a0` to `a1`, in **degrees** (as stored in DXF).
    Arc {
        layer: String,
        c: P2,
        r: f64,
        a0: f64,
        a1: f64,
    },
    /// `major` is the major-axis end point relative to the centre; `ratio` = minor/major;
    /// `t0`, `t1` are the start/end parameters in radians.
    Ellipse {
        layer: String,
        c: P2,
        major: P2,
        ratio: f64,
        t0: f64,
        t1: f64,
    },
    LwPolyline {
        layer: String,
        closed: bool,
        verts: Vec<(f64, f64, f64)>,
    },
    /// Single-line text. `rotation` is in degrees; `halign` is the DXF group-72 code
    /// (0 left, 1 centre, 2 right).
    Text {
        layer: String,
        p: P2,
        height: f64,
        rotation: f64,
        halign: u8,
        text: String,
    },
    /// `fit` holds the interpolation points (group 11) when the curve is a fit-point spline.
    Spline {
        layer: String,
        degree: usize,
        knots: Vec<f64>,
        ctrl: Vec<P2>,
        weights: Vec<f64>,
        fit: Vec<P2>,
        closed: bool,
    },
}

impl DxfEntity {
    pub fn layer(&self) -> &str {
        match self {
            DxfEntity::Point { layer, .. }
            | DxfEntity::Line { layer, .. }
            | DxfEntity::Circle { layer, .. }
            | DxfEntity::Arc { layer, .. }
            | DxfEntity::Ellipse { layer, .. }
            | DxfEntity::LwPolyline { layer, .. }
            | DxfEntity::Text { layer, .. }
            | DxfEntity::Spline { layer, .. } => layer,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct DxfDoc {
    pub entities: Vec<DxfEntity>,
}

/// Style of one layer in the `LAYER` table.
#[derive(Debug, Clone, PartialEq)]
pub struct LayerDef {
    pub name: String,
    /// AutoCAD colour index (7 is white / black).
    pub color: i32,
    /// `CONTINUOUS`, `HIDDEN`, `CENTER` or `PHANTOM`.
    pub linetype: String,
    /// Line weight in hundredths of a millimetre (0 = default).
    pub lineweight: i32,
}

impl LayerDef {
    pub fn new(name: &str, color: i32, linetype: &str, lineweight: i32) -> Self {
        LayerDef {
            name: name.into(),
            color,
            linetype: linetype.into(),
            lineweight,
        }
    }
}

/// Dash patterns (mm; positive = dash, negative = gap) of the linetypes the writer defines.
const LINETYPES: [(&str, &str, &[f64]); 4] = [
    ("CONTINUOUS", "Solid line", &[]),
    ("HIDDEN", "Hidden __ __ __", &[3.0, -1.5]),
    ("CENTER", "Center ____ _ ____ _", &[12.0, -2.0, 2.0, -2.0]),
    (
        "PHANTOM",
        "Phantom ____ _ _ ____",
        &[12.0, -2.0, 2.0, -2.0, 2.0, -2.0],
    ),
];

#[derive(Debug, Clone, PartialEq)]
pub enum DxfError {
    /// A group code line was not an integer, or a value was missing.
    Malformed {
        line: usize,
        reason: &'static str,
    },
    BadNumber {
        line: usize,
    },
    /// An entity is missing a required value.
    Incomplete {
        entity: &'static str,
    },
}

impl std::fmt::Display for DxfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for DxfError {}

// ---------------------------------------------------------------------------
// writer
// ---------------------------------------------------------------------------

struct W {
    out: String,
    handle: u32,
}

impl W {
    fn pair(&mut self, code: i32, v: impl std::fmt::Display) {
        let _ = writeln!(self.out, "{code}\n{v}");
    }
    fn start(&mut self, kind: &str, layer: &str) {
        self.pair(0, kind);
        self.handle += 1;
        let h = format!("{:X}", 0x100 + self.handle);
        self.pair(5, h);
        self.pair(100, "AcDbEntity");
        self.pair(8, layer);
    }
    fn pt(&mut self, c: i32, p: P2) {
        self.pair(c, p.0);
        self.pair(c + 10, p.1);
        self.pair(c + 20, 0.0);
    }
}

/// Serialise to ASCII DXF text.
pub fn write(doc: &DxfDoc) -> String {
    write_with_layers(doc, &[])
}

/// Conservative drawing extents of the entities (circles and arcs count as full circles).
fn extents(doc: &DxfDoc) -> Option<(P2, P2)> {
    let mut lo = (f64::INFINITY, f64::INFINITY);
    let mut hi = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut add = |p: P2, r: f64| {
        lo = (lo.0.min(p.0 - r), lo.1.min(p.1 - r));
        hi = (hi.0.max(p.0 + r), hi.1.max(p.1 + r));
    };
    for e in &doc.entities {
        match e {
            DxfEntity::Point { p, .. } | DxfEntity::Text { p, .. } => add(*p, 0.0),
            DxfEntity::Line { a, b, .. } => {
                add(*a, 0.0);
                add(*b, 0.0);
            }
            DxfEntity::Circle { c, r, .. } | DxfEntity::Arc { c, r, .. } => add(*c, *r),
            DxfEntity::Ellipse { c, major, .. } => add(*c, major.0.hypot(major.1)),
            DxfEntity::LwPolyline { verts, .. } => {
                for v in verts {
                    add((v.0, v.1), 0.0);
                }
            }
            DxfEntity::Spline { ctrl, fit, .. } => {
                for p in ctrl.iter().chain(fit) {
                    add(*p, 0.0);
                }
            }
        }
    }
    (lo.0.is_finite() && hi.0.is_finite()).then_some((lo, hi))
}

/// Serialise with explicit layer styles (colour, linetype, line weight). Layers used by
/// entities but not listed get colour 7 and `CONTINUOUS`.
pub fn write_with_layers(doc: &DxfDoc, defs: &[LayerDef]) -> String {
    let mut w = W {
        out: String::new(),
        handle: 0,
    };
    w.pair(0, "SECTION");
    w.pair(2, "HEADER");
    w.pair(9, "$ACADVER");
    w.pair(1, "AC1015");
    w.pair(9, "$INSUNITS");
    w.pair(70, 4);
    if let Some((lo, hi)) = extents(doc) {
        w.pair(9, "$EXTMIN");
        w.pt(10, lo);
        w.pair(9, "$EXTMAX");
        w.pt(10, hi);
    }
    w.pair(0, "ENDSEC");

    let mut layers: Vec<LayerDef> = vec![LayerDef::new("0", 7, "CONTINUOUS", 0)];
    for d in defs {
        if !layers.iter().any(|l| l.name == d.name) {
            layers.push(d.clone());
        }
    }
    for e in &doc.entities {
        if !layers.iter().any(|l| l.name == e.layer()) {
            layers.push(LayerDef::new(e.layer(), 7, "CONTINUOUS", 0));
        }
    }
    w.pair(0, "SECTION");
    w.pair(2, "TABLES");
    w.pair(0, "TABLE");
    w.pair(2, "LTYPE");
    w.pair(70, LINETYPES.len());
    for (name, desc, dashes) in LINETYPES {
        w.pair(0, "LTYPE");
        w.pair(2, name);
        w.pair(70, 0);
        w.pair(3, desc);
        w.pair(72, 65);
        w.pair(73, dashes.len());
        w.pair(40, dashes.iter().map(|d| d.abs()).sum::<f64>());
        for d in dashes {
            w.pair(49, d);
        }
    }
    w.pair(0, "ENDTAB");
    w.pair(0, "TABLE");
    w.pair(2, "LAYER");
    w.pair(70, layers.len());
    for l in &layers {
        w.pair(0, "LAYER");
        w.pair(2, &l.name);
        w.pair(70, 0);
        w.pair(62, l.color);
        w.pair(6, &l.linetype);
        if l.lineweight > 0 {
            w.pair(370, l.lineweight);
        }
    }
    w.pair(0, "ENDTAB");
    w.pair(0, "ENDSEC");

    w.pair(0, "SECTION");
    w.pair(2, "ENTITIES");
    for e in &doc.entities {
        match e {
            DxfEntity::Point { layer, p } => {
                w.start("POINT", layer);
                w.pt(10, *p);
            }
            DxfEntity::Line { layer, a, b } => {
                w.start("LINE", layer);
                w.pt(10, *a);
                w.pt(11, *b);
            }
            DxfEntity::Circle { layer, c, r } => {
                w.start("CIRCLE", layer);
                w.pt(10, *c);
                w.pair(40, r);
            }
            DxfEntity::Arc {
                layer,
                c,
                r,
                a0,
                a1,
            } => {
                w.start("ARC", layer);
                w.pt(10, *c);
                w.pair(40, r);
                w.pair(50, a0);
                w.pair(51, a1);
            }
            DxfEntity::Ellipse {
                layer,
                c,
                major,
                ratio,
                t0,
                t1,
            } => {
                w.start("ELLIPSE", layer);
                w.pt(10, *c);
                w.pt(11, *major);
                w.pair(210, 0.0);
                w.pair(220, 0.0);
                w.pair(230, 1.0);
                w.pair(40, ratio);
                w.pair(41, t0);
                w.pair(42, t1);
            }
            DxfEntity::LwPolyline {
                layer,
                closed,
                verts,
            } => {
                w.start("LWPOLYLINE", layer);
                w.pair(100, "AcDbPolyline");
                w.pair(90, verts.len());
                w.pair(70, if *closed { 1 } else { 0 });
                for (x, y, b) in verts {
                    w.pair(10, x);
                    w.pair(20, y);
                    if *b != 0.0 {
                        w.pair(42, b);
                    }
                }
            }
            DxfEntity::Text {
                layer,
                p,
                height,
                rotation,
                halign,
                text,
            } => {
                w.start("TEXT", layer);
                w.pt(10, *p);
                w.pair(40, height);
                w.pair(1, text);
                if *rotation != 0.0 {
                    w.pair(50, rotation);
                }
                if *halign != 0 {
                    w.pair(72, halign);
                    w.pt(11, *p);
                }
            }
            DxfEntity::Spline {
                layer,
                degree,
                knots,
                ctrl,
                weights,
                fit,
                closed,
            } => {
                w.start("SPLINE", layer);
                w.pair(100, "AcDbSpline");
                w.pair(210, 0.0);
                w.pair(220, 0.0);
                w.pair(230, 1.0);
                let rational = weights.iter().any(|x| *x != 1.0);
                w.pair(
                    70,
                    8 | if *closed { 1 } else { 0 } | if rational { 4 } else { 0 },
                );
                w.pair(71, degree);
                w.pair(72, knots.len());
                w.pair(73, ctrl.len());
                w.pair(74, fit.len());
                for k in knots {
                    w.pair(40, k);
                }
                if rational {
                    for x in weights {
                        w.pair(41, x);
                    }
                }
                for p in ctrl {
                    w.pt(10, *p);
                }
                for p in fit {
                    w.pt(11, *p);
                }
            }
        }
    }
    w.pair(0, "ENDSEC");
    w.pair(0, "EOF");
    w.out
}

// ---------------------------------------------------------------------------
// reader
// ---------------------------------------------------------------------------

type Pairs = Vec<(i32, String, usize)>;

fn tokenize(text: &str) -> Result<Pairs, DxfError> {
    let mut lines = text.lines().enumerate();
    let mut out = Vec::new();
    while let Some((i, code)) = lines.next() {
        let code = code.trim();
        if code.is_empty() {
            continue;
        }
        let c: i32 = code.parse().map_err(|_| DxfError::Malformed {
            line: i + 1,
            reason: "group code is not an integer",
        })?;
        let (_, value) = lines.next().ok_or(DxfError::Malformed {
            line: i + 1,
            reason: "missing value for group code",
        })?;
        out.push((c, value.trim().to_string(), i + 2));
    }
    Ok(out)
}

fn num(v: &(i32, String, usize)) -> Result<f64, DxfError> {
    v.1.parse::<f64>()
        .ok()
        .filter(|x| x.is_finite())
        .ok_or(DxfError::BadNumber { line: v.2 })
}

fn uint(v: &(i32, String, usize)) -> Result<usize, DxfError> {
    v.1.parse::<usize>()
        .map_err(|_| DxfError::BadNumber { line: v.2 })
}

/// Parse ASCII DXF text. Unknown entities and sections are ignored; malformed
/// input yields an error (never a panic).
pub fn parse(text: &str) -> Result<DxfDoc, DxfError> {
    let pairs = tokenize(text)?;
    let mut doc = DxfDoc::default();
    let mut i = 0;
    // find the ENTITIES section
    let mut in_entities = false;
    while i < pairs.len() {
        let (c, v, _) = &pairs[i];
        if *c == 0 && v == "SECTION" {
            in_entities = pairs
                .get(i + 1)
                .is_some_and(|p| p.0 == 2 && p.1 == "ENTITIES");
            i += 2;
            continue;
        }
        if *c == 0 && v == "ENDSEC" {
            in_entities = false;
            i += 1;
            continue;
        }
        if !in_entities || *c != 0 {
            i += 1;
            continue;
        }
        // an entity: gather its pairs up to the next group code 0
        let kind = v.clone();
        let start = i + 1;
        let mut end = start;
        while end < pairs.len() && pairs[end].0 != 0 {
            end += 1;
        }
        let body = &pairs[start..end];
        if let Some(e) = parse_entity(&kind, body)? {
            doc.entities.push(e);
        }
        i = end;
    }
    Ok(doc)
}

fn first(body: &[(i32, String, usize)], code: i32) -> Option<&(i32, String, usize)> {
    body.iter().find(|p| p.0 == code)
}

fn all(body: &[(i32, String, usize)], code: i32) -> impl Iterator<Item = &(i32, String, usize)> {
    body.iter().filter(move |p| p.0 == code)
}

fn get(body: &[(i32, String, usize)], code: i32, entity: &'static str) -> Result<f64, DxfError> {
    num(first(body, code).ok_or(DxfError::Incomplete { entity })?)
}

fn opt(body: &[(i32, String, usize)], code: i32, default: f64) -> Result<f64, DxfError> {
    first(body, code).map_or(Ok(default), num)
}

fn parse_entity(kind: &str, body: &[(i32, String, usize)]) -> Result<Option<DxfEntity>, DxfError> {
    let layer = first(body, 8).map_or_else(|| "0".to_string(), |p| p.1.clone());
    let pt = |c: i32, e: &'static str| -> Result<P2, DxfError> {
        Ok((get(body, c, e)?, get(body, c + 10, e)?))
    };
    Ok(Some(match kind {
        "POINT" => DxfEntity::Point {
            layer,
            p: pt(10, "POINT")?,
        },
        "LINE" => DxfEntity::Line {
            layer,
            a: pt(10, "LINE")?,
            b: pt(11, "LINE")?,
        },
        "CIRCLE" => DxfEntity::Circle {
            layer,
            c: pt(10, "CIRCLE")?,
            r: get(body, 40, "CIRCLE")?,
        },
        "ARC" => DxfEntity::Arc {
            layer,
            c: pt(10, "ARC")?,
            r: get(body, 40, "ARC")?,
            a0: get(body, 50, "ARC")?,
            a1: get(body, 51, "ARC")?,
        },
        "ELLIPSE" => DxfEntity::Ellipse {
            layer,
            c: pt(10, "ELLIPSE")?,
            major: pt(11, "ELLIPSE")?,
            ratio: get(body, 40, "ELLIPSE")?,
            t0: opt(body, 41, 0.0)?,
            t1: opt(body, 42, std::f64::consts::TAU)?,
        },
        "LWPOLYLINE" => {
            let flags = opt(body, 70, 0.0)? as i64;
            // vertices: each group-10 starts a vertex; a following 42 is its bulge
            let mut verts: Vec<(f64, f64, f64)> = Vec::new();
            let mut idx = 0;
            while idx < body.len() {
                if body[idx].0 == 10 {
                    let x = num(&body[idx])?;
                    let y = body
                        .get(idx + 1)
                        .filter(|p| p.0 == 20)
                        .map(num)
                        .transpose()?
                        .ok_or(DxfError::Incomplete {
                            entity: "LWPOLYLINE",
                        })?;
                    let mut bulge = 0.0;
                    let mut j = idx + 2;
                    while j < body.len() && body[j].0 != 10 {
                        if body[j].0 == 42 {
                            bulge = num(&body[j])?;
                        }
                        j += 1;
                    }
                    verts.push((x, y, bulge));
                    idx = j;
                } else {
                    idx += 1;
                }
            }
            DxfEntity::LwPolyline {
                layer,
                closed: flags & 1 == 1,
                verts,
            }
        }
        "TEXT" => DxfEntity::Text {
            layer,
            p: pt(10, "TEXT")?,
            height: get(body, 40, "TEXT")?,
            rotation: opt(body, 50, 0.0)?,
            halign: opt(body, 72, 0.0)? as u8,
            text: first(body, 1)
                .ok_or(DxfError::Incomplete { entity: "TEXT" })?
                .1
                .clone(),
        },
        "SPLINE" => {
            let flags = opt(body, 70, 0.0)? as i64;
            let degree = uint(first(body, 71).ok_or(DxfError::Incomplete { entity: "SPLINE" })?)?;
            let knots: Vec<f64> = all(body, 40).map(num).collect::<Result<_, _>>()?;
            let weights: Vec<f64> = all(body, 41).map(num).collect::<Result<_, _>>()?;
            let xs: Vec<f64> = all(body, 10).map(num).collect::<Result<_, _>>()?;
            let ys: Vec<f64> = all(body, 20).map(num).collect::<Result<_, _>>()?;
            let fx: Vec<f64> = all(body, 11).map(num).collect::<Result<_, _>>()?;
            let fy: Vec<f64> = all(body, 21).map(num).collect::<Result<_, _>>()?;
            if fx.len() != fy.len() {
                return Err(DxfError::Incomplete { entity: "SPLINE" });
            }
            if xs.len() != ys.len() || xs.is_empty() || degree == 0 {
                return Err(DxfError::Incomplete { entity: "SPLINE" });
            }
            let n = xs.len();
            let weights = if weights.len() == n {
                weights
            } else {
                vec![1.0; n]
            };
            DxfEntity::Spline {
                layer,
                degree,
                knots,
                ctrl: xs.into_iter().zip(ys).collect(),
                weights,
                fit: fx.into_iter().zip(fy).collect(),
                closed: flags & 1 == 1,
            }
        }
        _ => return Ok(None),
    }))
}
