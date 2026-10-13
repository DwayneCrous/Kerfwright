//! The naming corpus: hundreds of generated edit scenarios, each with ground truth computed from the
//! geometry of the edited model alone (never from names).
//!
//! A reference is made to a face of the original model, the model is edited, and the reference is
//! resolved against the edited model's faces. Outcomes:
//! * correct — resolved to the face the geometry says is meant (or flagged when the face is gone or
//!   genuinely ambiguous),
//! * flagged — reported as Ambiguous/Broken where a unique face exists,
//! * wrong — resolved silently to a face that is not the intended one.
//!
//! Acceptance: no wrong resolutions, at least 97 % correct.

use kw_brep::{BRep, FaceId, Sense};
use kw_expr::Param;
use kw_feature::*;
use kw_nurbs::SurfaceGeom;

// ------------------------------------------------------------------------------- random numbers

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn f(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * ((self.next() >> 11) as f64 / (1u64 << 53) as f64)
    }
    fn i(&mut self, lo: u32, hi: u32) -> u32 {
        lo + (self.next() % (hi - lo + 1) as u64) as u32
    }
    fn chance(&mut self, p: f64) -> bool {
        self.f(0.0, 1.0) < p
    }
}

// ----------------------------------------------------------------------------------------- model

#[derive(Clone, Debug)]
struct Boss {
    x: f64,
    y: f64,
    bw: f64,
    bh: f64,
    bd: f64,
    count: u32,
    dx: f64,
    pat: bool,
}
#[derive(Clone, Debug)]
struct HoleM {
    x: f64,
    y: f64,
    r: f64,
    count: u32,
    dx: f64,
    pat: bool,
}
#[derive(Clone, Debug)]
struct Pocket {
    x: f64,
    y: f64,
    pw: f64,
    ph: f64,
    pd: f64,
}
/// A slot across the whole depth (y) of the part, splitting the top face in two.
#[derive(Clone, Debug)]
struct Slot {
    x: f64,
    sw: f64,
    sd: f64,
}

#[derive(Clone, Debug)]
struct Model {
    w: f64,
    h: f64,
    d: f64,
    boss: Option<Boss>,
    hole: Option<HoleM>,
    pocket: Option<Pocket>,
    slot: Option<Slot>,
    slot_suppressed: bool,
    swap: bool,
    /// An extra boss added right after the base, away from everything else.
    extra: Option<Boss>,
    burn: u32,
}

struct Ids {
    base: FeatureId,
    boss: Option<FeatureId>,
    boss_pat: Option<FeatureId>,
    hole: Option<FeatureId>,
    hole_pat: Option<FeatureId>,
    pocket: Option<FeatureId>,
    slot: Option<FeatureId>,
    last: FeatureId,
}

fn mm(v: f64) -> Param {
    Param::length(v)
}

fn build(m: &Model) -> (Document, Ids) {
    let mut doc = Document::new();
    for k in 0..m.burn {
        let id = doc.add_parameter(&format!("burn{k}"), mm(1.0)).unwrap();
        doc.remove(id).unwrap();
    }
    let top = || PlaneSpec::Xy(mm(m.d));
    let s = doc
        .add_rect_sketch(
            "BaseS",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(m.w),
            mm(m.h),
        )
        .unwrap();
    let base = doc.add_extrude("Base", s, mm(m.d), BodyOp::New).unwrap();
    let mut ids = Ids {
        base,
        boss: None,
        boss_pat: None,
        hole: None,
        hole_pat: None,
        pocket: None,
        slot: None,
        last: base,
    };
    if let Some(e) = &m.extra {
        let s = doc
            .add_rect_sketch("ExtraS", top(), mm(e.x), mm(e.y), mm(e.bw), mm(e.bh))
            .unwrap();
        ids.last = doc.add_extrude("Extra", s, mm(e.bd), BodyOp::Join).unwrap();
    }
    let add_boss = |doc: &mut Document, ids: &mut Ids| {
        if let Some(b) = &m.boss {
            let s = doc
                .add_rect_sketch("BossS", top(), mm(b.x), mm(b.y), mm(b.bw), mm(b.bh))
                .unwrap();
            let e = doc.add_extrude("Boss", s, mm(b.bd), BodyOp::Join).unwrap();
            ids.boss = Some(e);
            ids.last = e;
            if b.pat {
                let p = doc
                    .add_linear_pattern(
                        "BossPat",
                        e,
                        [mm(b.dx), mm(0.0), mm(0.0)],
                        Param::scalar(b.count as f64),
                    )
                    .unwrap();
                ids.boss_pat = Some(p);
                ids.last = p;
            }
        }
    };
    let add_hole = |doc: &mut Document, ids: &mut Ids| {
        if let Some(h) = &m.hole {
            let s = doc
                .add_circle_sketch("HoleS", top(), mm(h.x), mm(h.y), mm(2.0 * h.r))
                .unwrap();
            let e = doc
                .add(
                    "Hole",
                    FeatureSpec::Extrude(ExtrudeSpec {
                        surface: false,
                        sketch: s,
                        depth: mm(m.d + 1.0),
                        symmetric: false,
                        flip: true,
                        op: BodyOp::Cut,
                        base: None,
                        extent: Default::default(),
                    }),
                )
                .unwrap();
            ids.hole = Some(e);
            ids.last = e;
            if h.pat {
                let p = doc
                    .add_linear_pattern(
                        "HolePat",
                        e,
                        [mm(h.dx), mm(0.0), mm(0.0)],
                        Param::scalar(h.count as f64),
                    )
                    .unwrap();
                ids.hole_pat = Some(p);
                ids.last = p;
            }
        }
    };
    if m.swap {
        add_hole(&mut doc, &mut ids);
        add_boss(&mut doc, &mut ids);
    } else {
        add_boss(&mut doc, &mut ids);
        add_hole(&mut doc, &mut ids);
    }
    if let Some(p) = &m.pocket {
        let s = doc
            .add_rect_sketch("PocketS", top(), mm(p.x), mm(p.y), mm(p.pw), mm(p.ph))
            .unwrap();
        let e = doc
            .add(
                "Pocket",
                FeatureSpec::Extrude(ExtrudeSpec {
                    surface: false,
                    sketch: s,
                    depth: mm(p.pd),
                    symmetric: false,
                    flip: true,
                    op: BodyOp::Cut,
                    base: None,
                    extent: Default::default(),
                }),
            )
            .unwrap();
        ids.pocket = Some(e);
        ids.last = e;
    }
    if let Some(sl) = &m.slot {
        let s = doc
            .add_rect_sketch("SlotS", top(), mm(sl.x), mm(-1.0), mm(sl.sw), mm(m.h + 2.0))
            .unwrap();
        let e = doc
            .add(
                "Slot",
                FeatureSpec::Extrude(ExtrudeSpec {
                    surface: false,
                    sketch: s,
                    depth: mm(sl.sd),
                    symmetric: false,
                    flip: true,
                    op: BodyOp::Cut,
                    base: None,
                    extent: Default::default(),
                }),
            )
            .unwrap();
        ids.slot = Some(e);
        ids.last = e;
        if m.slot_suppressed {
            doc.set_suppressed(e, true).unwrap();
        }
    }
    (doc, ids)
}

// ----------------------------------------------------------------------- independent geometry

fn plane_of(rep: &BRep, f: FaceId) -> Option<([f64; 3], f64)> {
    let face = &rep.faces[f];
    if let SurfaceGeom::Plane(p) = &rep.surfaces[face.surface] {
        let s = if face.sense == Sense::Forward {
            1.0
        } else {
            -1.0
        };
        let n = p.frame.z * s;
        Some(([n.x, n.y, n.z], n.dot(&p.frame.origin.coords)))
    } else {
        None
    }
}

fn cyl_of(rep: &BRep, f: FaceId) -> Option<([f64; 2], f64)> {
    let face = &rep.faces[f];
    if let SurfaceGeom::Cylinder(c) = &rep.surfaces[face.surface] {
        if c.frame.z.z.abs() > 0.999 {
            return Some(([c.frame.origin.x, c.frame.origin.y], c.radius));
        }
    }
    None
}

fn bbox(rep: &BRep, f: FaceId) -> ([f64; 3], [f64; 3]) {
    let b = rep.face_bbox(f).unwrap();
    ([b.min.x, b.min.y, b.min.z], [b.max.x, b.max.y, b.max.z])
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Side {
    XMin,
    XMax,
    YMin,
    YMax,
    ZMin,
    ZMax,
}
impl Side {
    fn normal(self) -> [f64; 3] {
        match self {
            Side::XMin => [-1.0, 0.0, 0.0],
            Side::XMax => [1.0, 0.0, 0.0],
            Side::YMin => [0.0, -1.0, 0.0],
            Side::YMax => [0.0, 1.0, 0.0],
            Side::ZMin => [0.0, 0.0, -1.0],
            Side::ZMax => [0.0, 0.0, 1.0],
        }
    }
}

#[derive(Clone, Debug)]
enum Target {
    Base(Side),
    BaseTopLarger,
    BaseTop,
    Boss(Side, u32),
    HoleWall(u32),
    PocketFloor,
    PocketWall(Side),
}

/// Geometry that a face must have to be the one meant by `t` in model `m`; `None` if no such face can
/// exist in `m`.
enum Want {
    Plane([f64; 3], f64),
    /// A plane, and the face must contain this point (x, y) in plan.
    PlaneContaining([f64; 3], f64, [f64; 2]),
    Cyl([f64; 2], f64),
}

fn want(m: &Model, t: &Target) -> Option<Want> {
    let n = |s: Side| s.normal();
    match t {
        Target::Base(s) => {
            let d = match s {
                Side::XMin | Side::YMin | Side::ZMin => 0.0,
                Side::XMax => m.w,
                Side::YMax => m.h,
                Side::ZMax => m.d,
            };
            Some(Want::Plane(n(*s), d))
        }
        Target::BaseTop => {
            // the top face of the base (unsplit)
            if m.slot.is_some() && !m.slot_suppressed {
                return None;
            }
            Some(Want::Plane([0.0, 0.0, 1.0], m.d))
        }
        Target::BaseTopLarger => {
            let sl = m.slot.as_ref()?;
            if m.slot_suppressed {
                return Some(Want::Plane([0.0, 0.0, 1.0], m.d));
            }
            let (left, right) = (sl.x, m.w - sl.x - sl.sw);
            let cx = if left > right {
                sl.x / 2.0
            } else {
                sl.x + sl.sw + right / 2.0
            };
            Some(Want::PlaneContaining([0.0, 0.0, 1.0], m.d, [cx, m.h / 2.0]))
        }
        Target::Boss(s, k) => {
            let b = m.boss.as_ref()?;
            if *k >= b.count {
                return None;
            }
            let x0 = b.x + *k as f64 * b.dx;
            let v = match s {
                Side::XMin => -x0,
                Side::XMax => x0 + b.bw,
                Side::YMin => -b.y,
                Side::YMax => b.y + b.bh,
                Side::ZMax => m.d + b.bd,
                Side::ZMin => return None,
            };
            Some(Want::PlaneContaining(
                n(*s),
                v,
                [x0 + b.bw / 2.0, b.y + b.bh / 2.0],
            ))
        }
        Target::HoleWall(k) => {
            let h = m.hole.as_ref()?;
            if *k >= h.count {
                return None;
            }
            Some(Want::Cyl([h.x + *k as f64 * h.dx, h.y], h.r))
        }
        Target::PocketFloor => {
            let p = m.pocket.as_ref()?;
            Some(Want::Plane([0.0, 0.0, 1.0], m.d - p.pd))
        }
        Target::PocketWall(s) => {
            let p = m.pocket.as_ref()?;
            let v = match s {
                Side::XMin => -p.x,
                Side::XMax => p.x + p.pw,
                Side::YMin => -p.y,
                Side::YMax => p.y + p.ph,
                _ => return None,
            };
            Some(Want::Plane(n(*s), v))
        }
    }
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-5
}

fn satisfies(rep: &BRep, f: FaceId, w: &Want) -> bool {
    match w {
        Want::Plane(n, d) => plane_of(rep, f)
            .is_some_and(|(pn, pd)| (0..3).all(|i| near(pn[i], n[i])) && near(pd, *d)),
        Want::PlaneContaining(n, d, p) => {
            plane_of(rep, f)
                .is_some_and(|(pn, pd)| (0..3).all(|i| near(pn[i], n[i])) && near(pd, *d))
                && {
                    let (lo, hi) = bbox(rep, f);
                    (0..2)
                        .all(|a| n[a].abs() > 0.5 || (lo[a] - 1e-6 <= p[a] && p[a] <= hi[a] + 1e-6))
                }
        }
        Want::Cyl(c, r) => cyl_of(rep, f)
            .is_some_and(|(cc, rr)| near(cc[0], c[0]) && near(cc[1], c[1]) && near(rr, *r)),
    }
}

/// The faces of `rep` that satisfy `w`.
fn faces_matching(rep: &BRep, body: kw_brep::BodyId, w: &Want) -> Vec<FaceId> {
    rep.body_faces(body)
        .into_iter()
        .filter(|f| satisfies(rep, *f, w))
        .collect()
}

// ------------------------------------------------------------------------------------ the corpus

#[derive(Default, Debug)]
struct Tally {
    correct: usize,
    correct_by_name: usize,
    correct_by_lineage: usize,
    correct_by_fingerprint: usize,
    correct_flagged_gone: usize,
    flagged_missed: usize,
    wrong: usize,
    skipped: usize,
    skip_reasons: std::collections::BTreeMap<&'static str, usize>,
    wrong_cases: Vec<String>,
    missed_cases: Vec<String>,
}

impl Tally {
    fn total(&self) -> usize {
        self.correct + self.flagged_missed + self.wrong
    }
}

struct Ctx {
    engine: Engine,
    tally: Tally,
    family: &'static str,
    by_family: Vec<(&'static str, usize, usize, usize)>,
}

fn final_out(engine: &mut Engine, doc: &Document) -> Option<std::sync::Arc<EvalOutput>> {
    let ev = engine.evaluate(doc);
    if !ev.is_clean() {
        if std::env::var("KW_CORPUS_VERBOSE").is_ok() {
            eprintln!("evaluation failed: {:?}", ev.failures());
        }
        return None;
    }
    ev.final_solid().map(|(_, o)| o.clone())
}

/// The document for the edited model: built with the same ids as the original, then edited the way a
/// user would (features moved, features inserted) so that existing features keep their ids.
fn edited_doc(m1: &Model) -> Document {
    if m1.burn > 0 {
        return build(m1).0;
    }
    let mut plain = m1.clone();
    plain.swap = false;
    plain.extra = None;
    let (mut d, ids) = build(&plain);
    if m1.swap {
        // hole group before boss group
        let mut boss_group: Vec<FeatureId> = Vec::new();
        if let Some(b) = ids.boss {
            let i = d.index_of(b).unwrap();
            boss_group.push(d.features[i - 1].id);
            boss_group.push(b);
        }
        if let Some(p) = ids.boss_pat {
            boss_group.push(p);
        }
        // moving the boss group to just after the hole group
        let end = ids.hole_pat.or(ids.hole).map(|h| d.index_of(h).unwrap());
        if let (Some(end), false) = (end, boss_group.is_empty()) {
            for _ in 0..boss_group.len() {
                let first = boss_group.remove(0);
                let _ = d.move_feature(first, end);
                boss_group.push(first);
            }
            // restore the group's internal order
            let mut order: Vec<FeatureId> = boss_group.clone();
            order.sort_by_key(|f| d.index_of(*f).unwrap());
            let _ = order;
        }
    }
    if let Some(e) = &m1.extra {
        let top = PlaneSpec::Xy(mm(m1.d));
        let at = d.index_of(ids.base).unwrap() + 1;
        let s = d.insert_at(at, "ExtraS", rect_sketch_spec(top, e)).unwrap();
        d.insert_at(
            at + 1,
            "Extra",
            FeatureSpec::Extrude(ExtrudeSpec {
                surface: false,
                sketch: s,
                depth: mm(e.bd),
                symmetric: false,
                flip: false,
                op: BodyOp::Join,
                base: None,
                extent: Default::default(),
            }),
        )
        .unwrap();
    }
    d
}

fn rect_sketch_spec(plane: PlaneSpec, e: &Boss) -> FeatureSpec {
    let mut tmp = Document::new();
    let id = tmp
        .add_rect_sketch("t", plane, mm(e.x), mm(e.y), mm(e.bw), mm(e.bh))
        .unwrap();
    tmp.get(id).unwrap().spec.clone()
}

fn skip(cx: &mut Ctx, why: &'static str) {
    cx.tally.skipped += 1;
    *cx.tally.skip_reasons.entry(why).or_insert(0) += 1;
}

/// Make a reference to the face `t` of `m0`, edit to `m1`, and judge the resolution.
fn judge(cx: &mut Ctx, label: &str, m0: &Model, t: &Target, m1: &Model, must_flag: bool) {
    judge2(cx, label, m0, t, t, m1, must_flag)
}

fn judge2(
    cx: &mut Ctx,
    label: &str,
    m0: &Model,
    t: &Target,
    t1: &Target,
    m1: &Model,
    must_flag: bool,
) {
    let (d0, _) = build(m0);
    let d1 = edited_doc(m1);
    let Some(o0) = final_out(&mut cx.engine, &d0) else {
        return skip(cx, "original does not evaluate");
    };
    let Some(o1) = final_out(&mut cx.engine, &d1) else {
        return skip(cx, "edited model does not evaluate");
    };
    let (rep0, b0) = (o0.brep().unwrap(), o0.body().unwrap());
    let (rep1, b1) = (o1.brep().unwrap(), o1.body().unwrap());
    let Some(w0) = want(m0, t) else {
        return skip(cx, "target not in original");
    };
    let src = faces_matching(rep0, b0, &w0);
    if src.len() != 1 {
        if std::env::var("KW_CORPUS_VERBOSE").is_ok() {
            eprintln!("{label}: {} source faces for {t:?}\n  {m0:?}", src.len());
            for f in rep0.body_faces(b0) {
                eprintln!(
                    "    face {:?}: plane {:?} cyl {:?} bbox {:?}",
                    f,
                    plane_of(rep0, f),
                    cyl_of(rep0, f),
                    bbox(rep0, f)
                );
            }
        }
        return skip(cx, "source face not unique");
    }
    let Some(reference) = o0.names.reference_to(rep0, src[0]) else {
        return skip(cx, "no name for source");
    };
    let truth: Option<FaceId> = match want(m1, t1) {
        None => None,
        Some(w1) => {
            let f = faces_matching(rep1, b1, &w1);
            match f.len() {
                0 => None,
                1 => Some(f[0]),
                _ => {
                    return skip(cx, "truth face not unique");
                }
            }
        }
    };
    let res = o1.names.resolve_face(rep1, &reference);
    let (mut c, mut miss, mut wrong) = (0, 0, 0);
    let mut note = String::new();
    match (&res, truth, must_flag) {
        (Resolved::Face(f, kind), Some(tf), false) => {
            if *f == tf {
                c = 1;
                match kind {
                    ResolveKind::Exact => cx.tally.correct_by_name += 1,
                    ResolveKind::Lineage => cx.tally.correct_by_lineage += 1,
                    ResolveKind::Fingerprint => cx.tally.correct_by_fingerprint += 1,
                }
            } else {
                wrong = 1;
                note = format!("resolved ({kind:?}) to the wrong face");
            }
        }
        (Resolved::Face(_, kind), None, _) => {
            wrong = 1;
            note = format!("resolved ({kind:?}) although the face is gone/ambiguous");
        }
        (Resolved::Face(_, kind), Some(_), true) => {
            wrong = 1;
            note = format!("resolved ({kind:?}) a genuinely ambiguous reference");
        }
        (Resolved::Ambiguous(_) | Resolved::Broken, None, _) => {
            c = 1;
            cx.tally.correct_flagged_gone += 1;
        }
        (Resolved::Ambiguous(_) | Resolved::Broken, Some(_), true) => {
            c = 1;
            cx.tally.correct_flagged_gone += 1;
        }
        (Resolved::Ambiguous(_) | Resolved::Broken, Some(_), false) => {
            miss = 1;
            note = format!("flagged ({res:?}) although a unique face exists");
        }
    }
    cx.tally.correct += c;
    cx.tally.flagged_missed += miss;
    cx.tally.wrong += wrong;
    if wrong > 0 {
        cx.tally.wrong_cases.push(format!(
            "[{}] {label}: {note}\n  target {t:?}\n  before {m0:?}\n  after {m1:?}",
            cx.family
        ));
    }
    if miss > 0 {
        cx.tally
            .missed_cases
            .push(format!("[{}] {label}: {note}\n  target {t:?}", cx.family));
    }
    match cx.by_family.iter_mut().find(|x| x.0 == cx.family) {
        Some(x) => {
            x.1 += c;
            x.2 += miss;
            x.3 += wrong;
        }
        None => cx.by_family.push((cx.family, c, miss, wrong)),
    }
}

// --------------------------------------------------------------------------------- generators

fn random_model(r: &mut Rng, want_boss: bool, want_hole: bool, max_count: u32) -> Model {
    let w = r.f(40.0, 56.0);
    let h = r.f(24.0, 36.0);
    let d = r.f(8.0, 16.0);
    let boss = want_boss.then(|| {
        let count = r.i(1, max_count);
        Boss {
            x: r.f(3.0, 6.0),
            y: r.f(2.5, h / 2.0 - 7.0),
            bw: r.f(2.0, 5.0),
            bh: r.f(2.0, 5.0),
            bd: r.f(2.0, 6.0),
            count,
            dx: r.f(6.5, 8.5),
            pat: count > 1,
        }
    });
    let hole = want_hole.then(|| {
        let count = r.i(1, max_count);
        HoleM {
            x: r.f(5.0, 8.0),
            y: r.f(h / 2.0 + 5.0, h - 5.0),
            r: r.f(1.0, 2.5),
            count,
            dx: r.f(6.5, 8.5),
            pat: count > 1,
        }
    });
    let pocket = r.chance(0.5).then(|| Pocket {
        x: w - r.f(10.0, 12.0),
        y: r.f(3.0, 8.0),
        pw: r.f(4.0, 8.0),
        ph: r.f(5.0, 12.0),
        pd: r.f(2.0, d * 0.6),
    });
    Model {
        w,
        h,
        d,
        boss,
        hole,
        pocket,
        slot: None,
        slot_suppressed: false,
        swap: false,
        extra: None,
        burn: 0,
    }
}

fn rand_model(r: &mut Rng, pb: f64, ph: f64, max_count: u32) -> Model {
    let b = r.chance(pb);
    let h = r.chance(ph);
    random_model(r, b, h, max_count)
}

fn base_targets(m: &Model, r: &mut Rng) -> Vec<Target> {
    let mut v = vec![
        Target::Base(Side::XMin),
        Target::Base(Side::XMax),
        Target::Base(Side::YMin),
        Target::Base(Side::YMax),
        Target::Base(Side::ZMin),
        Target::BaseTop,
    ];
    if let Some(b) = &m.boss {
        let k = r.i(0, b.count - 1);
        for s in [Side::XMin, Side::XMax, Side::YMin, Side::YMax, Side::ZMax] {
            v.push(Target::Boss(s, k));
        }
    }
    if let Some(h) = &m.hole {
        v.push(Target::HoleWall(r.i(0, h.count - 1)));
    }
    if m.pocket.is_some() {
        v.push(Target::PocketFloor);
        v.push(Target::PocketWall(Side::XMin));
        v.push(Target::PocketWall(Side::YMax));
    }
    v
}

fn pick<'a>(r: &mut Rng, v: &'a [Target]) -> &'a Target {
    &v[(r.next() % v.len() as u64) as usize]
}

fn jitter(r: &mut Rng, m: &Model, amount: f64) -> Model {
    let mut n = m.clone();
    let s = |r: &mut Rng, v: f64| v * r.f(1.0 - amount, 1.0 + amount);
    n.w = s(r, n.w).max(m.w * 0.9).min(m.w * 1.1);
    n.h = s(r, n.h).max(m.h * 0.9).min(m.h * 1.1);
    n.d = s(r, n.d);
    if let Some(b) = &mut n.boss {
        b.bd = s(r, b.bd);
        b.bw = s(r, b.bw).clamp(1.5, 6.0);
    }
    if let Some(h) = &mut n.hole {
        h.r = s(r, h.r).clamp(0.8, 3.0);
    }
    if let Some(p) = &mut n.pocket {
        p.pd = (s(r, p.pd)).min(n.d * 0.7);
    }
    n
}

fn run_corpus(per_family: usize, seed: u64) -> Ctx {
    let mut cx = Ctx {
        engine: Engine::with_threads(4),
        tally: Tally::default(),
        family: "",
        by_family: Vec::new(),
    };
    let mut r = Rng(seed | 1);

    // 1. dimension changes
    cx.family = "dimension change";
    for i in 0..per_family {
        let m0 = rand_model(&mut r, 0.7, 0.7, 3);
        let m1 = jitter(&mut r, &m0, 0.15);
        let ts = base_targets(&m0, &mut r);
        let t = pick(&mut r, &ts).clone();
        judge(&mut cx, &format!("dim#{i}"), &m0, &t, &m1, false);
    }
    // 2. reorder (boss and hole swapped), sometimes with a dimension change
    cx.family = "reorder";
    for i in 0..per_family {
        let m0 = random_model(&mut r, true, true, 3);
        let mut m1 = if r.chance(0.5) {
            jitter(&mut r, &m0, 0.1)
        } else {
            m0.clone()
        };
        m1.swap = true;
        let ts = base_targets(&m0, &mut r);
        let t = pick(&mut r, &ts).clone();
        judge(&mut cx, &format!("reorder#{i}"), &m0, &t, &m1, false);
    }
    // 3. insert a feature upstream
    cx.family = "insert upstream";
    for i in 0..per_family {
        let m0 = rand_model(&mut r, 0.7, 0.7, 3);
        let mut m1 = m0.clone();
        // an extra boss in the free corner (x > pocket, y > hole zone is not used by patterns)
        m1.extra = Some(Boss {
            x: m0.w - r.f(4.0, 7.0),
            y: m0.h - r.f(5.0, 8.0),
            bw: r.f(1.5, 3.0),
            bh: r.f(1.5, 3.0),
            bd: r.f(1.0, 4.0),
            count: 1,
            dx: 0.0,
            pat: false,
        });
        if let Some(p) = &mut m1.pocket {
            // keep the pocket clear of the extra boss
            p.y = p.y.min(m0.h * 0.45 - p.ph).max(1.5);
        }
        let ts = base_targets(&m0, &mut r);
        let t = pick(&mut r, &ts).clone();
        judge(&mut cx, &format!("insert#{i}"), &m0, &t, &m1, false);
    }
    // 4. pattern count changes
    cx.family = "pattern count";
    for i in 0..per_family {
        let boss_pat = r.chance(0.5);
        let mut m0 = random_model(&mut r, boss_pat, !boss_pat, 5);
        m0.pocket = None;
        let c0 = r.i(2, 5);
        let mut c1 = r.i(1, 5);
        if c1 == c0 {
            c1 = if c0 > 1 { c0 - 1 } else { c0 + 1 };
        }
        if let Some(b) = &mut m0.boss {
            b.count = c0;
            b.pat = true;
        }
        if let Some(h) = &mut m0.hole {
            h.count = c0;
            h.pat = true;
        }
        let mut m1 = m0.clone();
        if let Some(b) = &mut m1.boss {
            b.count = c1;
        }
        if let Some(h) = &mut m1.hole {
            h.count = c1;
        }
        let k = r.i(0, c0 - 1);
        let t = if boss_pat {
            let s =
                [Side::XMin, Side::XMax, Side::YMin, Side::YMax, Side::ZMax][r.i(0, 4) as usize];
            Target::Boss(s, k)
        } else {
            Target::HoleWall(k)
        };
        judge(
            &mut cx,
            &format!("pattern#{i} {c0}->{c1} k={k}"),
            &m0,
            &t,
            &m1,
            false,
        );
    }
    // 5. face splits and merges
    cx.family = "split/merge";
    for i in 0..per_family {
        let mut m0 = rand_model(&mut r, 0.0, 0.5, 1);
        m0.pocket = None;
        if let Some(h) = &mut m0.hole {
            h.x = r.f(m0.w * 0.55, m0.w * 0.7);
        }
        let sw = r.f(2.0, 4.0);
        let sx = r.f(m0.w * 0.3, m0.w * 0.4);
        let slot = Slot {
            x: sx,
            sw,
            sd: r.f(2.0, m0.d * 0.7),
        };
        match i % 4 {
            0 | 1 => {
                // split: asymmetric slot, reference to the top face before the slot exists
                let mut m1 = m0.clone();
                // the policy: a piece keeps the identity if it has at least 1.5x the area of the other;
                // with pieces closer in size the reference is ambiguous and must be flagged
                let (left, right) = (slot.x, m0.w - slot.x - slot.sw);
                let ratio = left.max(right) / left.min(right);
                m1.slot = Some(slot);
                judge2(
                    &mut cx,
                    &format!("split#{i}"),
                    &m0,
                    &Target::BaseTop,
                    &Target::BaseTopLarger,
                    &m1,
                    ratio < 1.5,
                );
            }
            2 => {
                // merge: the slot is removed again; the reference was to the larger fragment
                let mut m0s = m0.clone();
                m0s.slot = Some(slot);
                let mut m1 = m0s.clone();
                m1.slot_suppressed = true;
                judge(
                    &mut cx,
                    &format!("merge#{i}"),
                    &m0s,
                    &Target::BaseTopLarger,
                    &m1,
                    false,
                );
            }
            _ => {
                // a symmetric slot: the top face has two equal fragments, which must be flagged
                m0.hole = None;
                let mut m1 = m0.clone();
                m1.slot = Some(Slot {
                    x: (m0.w - sw) / 2.0,
                    sw,
                    sd: slot.sd,
                });
                judge(
                    &mut cx,
                    &format!("split-sym#{i}"),
                    &m0,
                    &Target::BaseTop,
                    &m1,
                    true,
                );
            }
        }
    }
    // 6. the same model rebuilt with fresh feature ids (every name changes), alone or with an edit
    cx.family = "fresh ids";
    for i in 0..per_family {
        let m0 = rand_model(&mut r, 0.7, 0.7, 3);
        let mut m1 = if r.chance(0.5) {
            jitter(&mut r, &m0, 0.05)
        } else {
            m0.clone()
        };
        m1.burn = r.i(20, 60);
        if r.chance(0.3) {
            m1.swap = true;
        }
        let ts = base_targets(&m0, &mut r);
        let t = pick(&mut r, &ts).clone();
        judge(&mut cx, &format!("fresh#{i}"), &m0, &t, &m1, false);
    }
    cx
}

#[test]
fn naming_corpus() {
    let per_family: usize = std::env::var("KW_CORPUS_PER_FAMILY")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(100);
    let seed: u64 = std::env::var("KW_CORPUS_SEED")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0x5EED_1234_ABCD);
    let cx = run_corpus(per_family, seed);
    let t = &cx.tally;
    eprintln!(
        "naming corpus: {} scenarios judged ({} skipped: {:?})\n  correct {} ({:.2} %): by name {}, by lineage {}, by fingerprint {}, correctly flagged {}\n  flagged although resolvable {}\n  WRONG {}",
        t.total(),
        t.skipped,
        t.skip_reasons,
        t.correct,
        100.0 * t.correct as f64 / t.total().max(1) as f64,
        t.correct_by_name,
        t.correct_by_lineage,
        t.correct_by_fingerprint,
        t.correct_flagged_gone,
        t.flagged_missed,
        t.wrong
    );
    for (f, c, m, w) in &cx.by_family {
        eprintln!("  {f:18} correct {c:4}  flagged {m:3}  wrong {w:3}");
    }
    for c in t.wrong_cases.iter().take(10) {
        eprintln!("WRONG {c}");
    }
    for c in t.missed_cases.iter().take(10) {
        eprintln!("MISSED {c}");
    }
    assert_eq!(t.wrong, 0, "silent wrong resolutions");
    assert!(t.total() >= 500, "only {} scenarios", t.total());
    assert!(
        t.correct as f64 >= 0.97 * t.total() as f64,
        "auto-resolved correctly: {} of {}",
        t.correct,
        t.total()
    );
}
