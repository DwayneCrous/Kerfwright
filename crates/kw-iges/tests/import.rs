#![allow(unused_imports, dead_code)]
use kw_brep::BRep;
use kw_iges::*;
use proptest::prelude::*;

/// Minimal IGES writer for test fixtures (fixed 80-column records).
#[derive(Default)]
struct Iges {
    d: Vec<(u32, i32, usize, usize, usize)>, // type, form, transform, p_start, p_lines
    p: Vec<String>,
    unit_flag: i32,
}

impl Iges {
    fn new(unit_flag: i32) -> Self {
        Iges {
            unit_flag,
            ..Default::default()
        }
    }

    fn fmt(x: f64) -> String {
        format!("{x:.12e}").replace('e', "E")
    }

    /// Adds an entity; returns its DE sequence number (odd).
    fn add(&mut self, ty: u32, form: i32, transform: usize, params: &[String]) -> usize {
        let mut fields = vec![ty.to_string()];
        fields.extend(params.iter().cloned());
        // chunk into ≤64-column lines at field boundaries
        let mut lines: Vec<String> = Vec::new();
        let mut cur = String::new();
        let n = fields.len();
        for (i, f) in fields.iter().enumerate() {
            let tok = format!("{f}{}", if i + 1 == n { ';' } else { ',' });
            if cur.len() + tok.len() > 64 {
                lines.push(std::mem::take(&mut cur));
            }
            cur.push_str(&tok);
        }
        lines.push(cur);
        let p_start = self.p.len() + 1;
        let de = 2 * self.d.len() + 1;
        for l in &lines {
            self.p
                .push(format!("{l:<64}{de:>8}P{:>7}", self.p.len() + 1));
        }
        self.d.push((ty, form, transform, p_start, lines.len()));
        de
    }

    fn pt(x: f64, y: f64, z: f64) -> [String; 3] {
        [Self::fmt(x), Self::fmt(y), Self::fmt(z)]
    }

    fn line(&mut self, a: [f64; 3], b: [f64; 3]) -> usize {
        let mut p = Vec::new();
        p.extend(Self::pt(a[0], a[1], a[2]));
        p.extend(Self::pt(b[0], b[1], b[2]));
        self.add(110, 0, 0, &p)
    }

    /// Bilinear planar patch through four corners (a, b along u; a, c along v).
    fn plane_patch(&mut self, a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> usize {
        let mut p: Vec<String> = ["1", "1", "1", "1", "0", "0", "1", "0", "0"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        for k in [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] {
            p.push(Self::fmt(k));
        }
        for q in [a, b, c, d] {
            p.extend(Self::pt(q[0], q[1], q[2]));
        }
        for k in [0.0, 1.0, 0.0, 1.0] {
            p.push(Self::fmt(k));
        }
        self.add(128, 0, 0, &p)
    }

    fn finish(&self) -> String {
        let mut out = String::new();
        let rec = |s: &str, sec: char, n: usize| format!("{s:<72}{sec}{n:>7}\n");
        out.push_str(&rec("Kerfwright test fixture", 'S', 1));
        let g = format!("1H,,1H;,4Hfile,4Hfile,6Hkwtest,6Hkwtest,32,38,6,308,15,4Hfile,1.0,{},2HMM,1,0.01,15H20260101.000000,1E-06,100.0,5Hme,5Hme,11,0,;", self.unit_flag);
        let mut gl: Vec<String> = Vec::new();
        let mut cur = String::new();
        for tok in g.split_inclusive(',') {
            if cur.len() + tok.len() > 72 {
                gl.push(std::mem::take(&mut cur));
            }
            cur.push_str(tok);
        }
        gl.push(cur);
        for (i, l) in gl.iter().enumerate() {
            out.push_str(&rec(l, 'G', i + 1));
        }
        for (i, (ty, form, tr, ps, pl)) in self.d.iter().enumerate() {
            let l1 = format!(
                "{ty:>8}{ps:>8}{:>8}{:>8}{:>8}{:>8}{tr:>8}{:>8}{:>8}",
                0, 0, 0, 0, 0, "00000000"
            );
            let l2 = format!(
                "{ty:>8}{:>8}{:>8}{pl:>8}{form:>8}{:>8}{:>8}{:>8}{:>8}",
                0, 0, 0, 0, "", ""
            );
            out.push_str(&rec(&l1, 'D', 2 * i + 1));
            out.push_str(&rec(&l2, 'D', 2 * i + 2));
        }
        for (i, l) in self.p.iter().enumerate() {
            let _ = i;
            out.push_str(&format!("{l}\n"));
        }
        out.push_str(&rec(
            &format!(
                "S{:>7}G{:>7}D{:>7}P{:>7}",
                1,
                gl.len(),
                2 * self.d.len(),
                self.p.len()
            ),
            'T',
            1,
        ));
        out
    }
}

/// Cube [0,w]x[0,d]x[0,h] as six bilinear patches with deliberately mixed parametrisations.
fn cube_patches(i: &mut Iges, w: f64, d: f64, h: f64) -> Vec<usize> {
    let p = |x: f64, y: f64, z: f64| [x * w, y * d, z * h];
    vec![
        i.plane_patch(p(0., 0., 0.), p(1., 0., 0.), p(0., 1., 0.), p(1., 1., 0.)), // z = 0
        i.plane_patch(p(0., 0., 1.), p(0., 1., 1.), p(1., 0., 1.), p(1., 1., 1.)), // z = h (swapped u/v)
        i.plane_patch(p(0., 0., 0.), p(1., 0., 0.), p(0., 0., 1.), p(1., 0., 1.)), // y = 0
        i.plane_patch(p(0., 1., 0.), p(0., 1., 1.), p(1., 1., 0.), p(1., 1., 1.)), // y = d
        i.plane_patch(p(0., 0., 0.), p(0., 1., 0.), p(0., 0., 1.), p(0., 1., 1.)), // x = 0
        i.plane_patch(p(1., 0., 0.), p(1., 0., 1.), p(1., 1., 0.), p(1., 1., 1.)), // x = w
    ]
}

fn trimmed_natural(i: &mut Iges, surf: usize) -> usize {
    i.add(
        144,
        0,
        0,
        &[surf.to_string(), "0".into(), "0".into(), "0".into()],
    )
}

#[test]
fn cube_of_trimmed_surfaces_with_natural_boundaries() {
    for (flag, scale) in [(2, 1.0), (1, 25.4)] {
        let mut ig = Iges::new(flag);
        let surfs = cube_patches(&mut ig, 2.0, 3.0, 4.0);
        for s in surfs {
            trimmed_natural(&mut ig, s);
        }
        let text = ig.finish();
        let mut rep = BRep::new();
        let r = import(&mut rep, &text).unwrap_or_else(|e| panic!("import failed: {e}"));
        assert_eq!(r.bodies.len(), 1, "{:?}", r.rejected_shells);
        assert_eq!(r.unit_mm, scale);
        let v = rep.volume(r.bodies[0]).unwrap();
        let want = 24.0 * scale.powi(3);
        assert!(
            (v - want).abs() < 1e-9 * want,
            "flag {flag}: volume {v} vs {want}"
        );
        assert!(rep.validate().is_ok(), "{:?}", rep.validate());
    }
}

#[test]
fn trimmed_surfaces_with_explicit_boundaries_and_a_hole() {
    // top face carries an explicit outer boundary (composite curve of four lines) and a square hole
    let mut ig = Iges::new(2);
    let (w, d, h) = (4.0, 4.0, 2.0);
    let surfs = cube_patches(&mut ig, w, d, h);
    let top = surfs[1];
    for (k, s) in surfs.iter().enumerate() {
        if k == 1 {
            continue;
        }
        trimmed_natural(&mut ig, *s);
    }
    // outer boundary of the top face: loop at z = h
    let l0 = ig.line([0., 0., h], [w, 0., h]);
    let l1 = ig.line([w, 0., h], [w, d, h]);
    let l2 = ig.line([w, d, h], [0., d, h]);
    let l3 = ig.line([0., d, h], [0., 0., h]);
    let comp = ig.add(
        102,
        0,
        0,
        &[
            "4".into(),
            l0.to_string(),
            l1.to_string(),
            l2.to_string(),
            l3.to_string(),
        ],
    );
    let outer = ig.add(
        142,
        0,
        0,
        &[
            "1".into(),
            top.to_string(),
            "0".into(),
            comp.to_string(),
            "1".into(),
        ],
    );
    // the hole: square in the middle; it needs side walls to make a valid solid, so build a 144 for each
    let (a, b) = (1.0, 3.0);
    let h0 = ig.line([a, a, h], [b, a, h]);
    let h1 = ig.line([b, a, h], [b, b, h]);
    let h2 = ig.line([b, b, h], [a, b, h]);
    let h3 = ig.line([a, b, h], [a, a, h]);
    let hc = ig.add(
        102,
        0,
        0,
        &[
            "4".into(),
            h0.to_string(),
            h1.to_string(),
            h2.to_string(),
            h3.to_string(),
        ],
    );
    let inner = ig.add(
        142,
        0,
        0,
        &[
            "1".into(),
            top.to_string(),
            "0".into(),
            hc.to_string(),
            "1".into(),
        ],
    );
    ig.add(
        144,
        0,
        0,
        &[
            top.to_string(),
            "1".into(),
            "1".into(),
            outer.to_string(),
            inner.to_string(),
        ],
    );
    // a blind hole of depth 1: four wall patches + a floor, bounded naturally... the walls must be trimmed to the hole
    // loop, which natural boundaries do; use patches with exactly the hole's extent
    let z0 = h - 1.0;
    let wall = |ig: &mut Iges, p0: [f64; 3], p1: [f64; 3], p2: [f64; 3], p3: [f64; 3]| {
        let s = ig.plane_patch(p0, p1, p2, p3);
        trimmed_natural(ig, s)
    };
    wall(&mut ig, [a, a, z0], [b, a, z0], [a, a, h], [b, a, h]);
    wall(&mut ig, [b, a, z0], [b, b, z0], [b, a, h], [b, b, h]);
    wall(&mut ig, [b, b, z0], [a, b, z0], [b, b, h], [a, b, h]);
    wall(&mut ig, [a, b, z0], [a, a, z0], [a, b, h], [a, a, h]);
    wall(&mut ig, [a, a, z0], [b, a, z0], [a, b, z0], [b, b, z0]);
    let mut rep = BRep::new();
    let r = import(&mut rep, &ig.finish()).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(
        r.bodies.len(),
        1,
        "{:?} {:?}",
        r.rejected_shells,
        r.faces_skipped
    );
    let v = rep.volume(r.bodies[0]).unwrap();
    let want = 4.0 * 4.0 * 2.0 - 2.0 * 2.0 * 1.0;
    assert!((v - want).abs() < 1e-9, "volume {v} vs {want}");
    assert!(rep.validate().is_ok(), "{:?}", rep.validate());
}

#[test]
fn open_shells_and_empty_files_are_reported() {
    let mut ig = Iges::new(2);
    let surfs = cube_patches(&mut ig, 1.0, 1.0, 1.0);
    for s in &surfs[..5] {
        trimmed_natural(&mut ig, *s);
    }
    let mut rep = BRep::new();
    match import(&mut rep, &ig.finish()) {
        Err(IgesError::NoSolids(why)) => {
            assert!(why.iter().any(|w| w.contains("not closed")), "{why:?}")
        }
        other => panic!("{other:?}"),
    }
    let ig = Iges::new(2);
    assert!(matches!(
        import(&mut rep, &ig.finish()),
        Err(IgesError::NothingToImport)
    ));
}

#[test]
fn manifold_solid_brep_entities() {
    // 186 cube from vertex/edge/loop/face/shell entities
    let mut ig = Iges::new(2);
    let s = 2.0;
    let verts: Vec<[f64; 3]> = (0..8)
        .map(|i| {
            [
                (i & 1) as f64 * s,
                ((i >> 1) & 1) as f64 * s,
                ((i >> 2) & 1) as f64 * s,
            ]
        })
        .collect();
    let mut vl = vec![verts.len().to_string()];
    for v in &verts {
        vl.extend(Iges::pt(v[0], v[1], v[2]));
    }
    let vlist = ig.add(502, 1, 0, &vl);
    let mut edge_pairs = Vec::new();
    for a in 0..8usize {
        for b in a + 1..8 {
            if (a ^ b).count_ones() == 1 {
                edge_pairs.push((a, b));
            }
        }
    }
    let mut el = vec![edge_pairs.len().to_string()];
    for (a, b) in &edge_pairs {
        let c = ig.line(verts[*a], verts[*b]);
        el.extend([
            c.to_string(),
            vlist.to_string(),
            (a + 1).to_string(),
            vlist.to_string(),
            (b + 1).to_string(),
        ]);
    }
    let elist = ig.add(504, 1, 0, &el);
    let edge_idx = |a: usize, b: usize| {
        edge_pairs
            .iter()
            .position(|e| *e == (a.min(b), a.max(b)))
            .unwrap()
            + 1
    };
    // faces: axis, side
    let mut face_des = Vec::new();
    for axis in 0..3usize {
        for side in 0..2usize {
            let (a1, a2) = ((axis + 1) % 3, (axis + 2) % 3);
            let idx = |u: usize, v: usize| (side << axis) | (u << a1) | (v << a2);
            let corners = [idx(0, 0), idx(1, 0), idx(1, 1), idx(0, 1)];
            let mut lp = vec!["4".to_string()];
            for k in 0..4 {
                let (a, b) = (corners[k], corners[(k + 1) % 4]);
                lp.extend([
                    "0".into(),
                    elist.to_string(),
                    edge_idx(a, b).to_string(),
                    "1".into(),
                    "0".into(),
                ]);
            }
            let loop_de = ig.add(508, 1, 0, &lp);
            let p = |u: f64, v: f64| {
                let mut c = [0.0; 3];
                c[axis] = side as f64 * s;
                c[a1] = u * s;
                c[a2] = v * s;
                c
            };
            let surf = ig.plane_patch(p(0., 0.), p(1., 0.), p(0., 1.), p(1., 1.));
            face_des.push(ig.add(
                510,
                1,
                0,
                &[
                    surf.to_string(),
                    "1".into(),
                    "1".into(),
                    loop_de.to_string(),
                ],
            ));
        }
    }
    let mut sh = vec!["6".to_string()];
    for f in &face_des {
        sh.extend([f.to_string(), "1".into()]);
    }
    let shell = ig.add(514, 1, 0, &sh);
    ig.add(186, 0, 0, &[shell.to_string(), "1".into(), "0".into()]);
    let mut rep = BRep::new();
    let r = import(&mut rep, &ig.finish()).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(
        r.bodies.len(),
        1,
        "{:?} {:?}",
        r.rejected_shells,
        r.faces_skipped
    );
    assert!((rep.volume(r.bodies[0]).unwrap() - 8.0).abs() < 1e-9);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(std::env::var("PROPTEST_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(256)))]

    #[test]
    fn arbitrary_text_never_panics(s in "(?s).{0,400}") {
        let mut rep = BRep::new();
        let _ = import(&mut rep, &s);
    }

    #[test]
    fn mutated_files_never_panic(
        edits in prop::collection::vec((any::<prop::sample::Index>(), any::<u8>()), 1..24),
        cut in any::<prop::sample::Index>(),
    ) {
        let mut ig = Iges::new(2);
        let surfs = cube_patches(&mut ig, 2.0, 3.0, 4.0);
        for s in surfs { trimmed_natural(&mut ig, s); }
        let mut bytes = ig.finish().into_bytes();
        for (i, v) in &edits {
            let k = i.index(bytes.len());
            bytes[k] = *v;
        }
        let keep = cut.index(bytes.len() + 1);
        bytes.truncate(keep);
        let text = String::from_utf8_lossy(&bytes).into_owned();
        let mut rep = BRep::new();
        let _ = import(&mut rep, &text);
    }
}
