//! Gerber viewer A: polygons and boolean polarity.

use crate::{fail, FabError, P2};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;
use std::collections::HashMap;

#[derive(Debug, Clone)]
enum Ap {
    Circle(f64),
    Rect(f64, f64),
    Obround(f64, f64),
}

#[derive(Debug, Clone)]
pub struct Primitive {
    pub dark: bool,
    pub contour: Vec<P2>,
}

#[derive(Debug, Clone, Default)]
pub struct Image {
    /// Result of applying the polarity in order: shapes made of contours (outer first, then holes).
    pub shapes: Vec<Vec<Vec<P2>>>,
    pub primitives: Vec<Primitive>,
}

const ARC_STEPS: usize = 64;

fn circle_poly(c: P2, r: f64) -> Vec<P2> {
    let n = ARC_STEPS * 2;
    (0..n)
        .map(|i| {
            let a = std::f64::consts::TAU * i as f64 / n as f64;
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        })
        .collect()
}

fn stadium(a: P2, b: P2, r: f64) -> Vec<P2> {
    if (a[0] - b[0]).abs() < 1e-12 && (a[1] - b[1]).abs() < 1e-12 {
        return circle_poly(a, r);
    }
    let ang = (b[1] - a[1]).atan2(b[0] - a[0]);
    let mut pts = Vec::new();
    for i in 0..=ARC_STEPS {
        let t =
            ang - std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * i as f64 / ARC_STEPS as f64;
        pts.push([b[0] + r * t.cos(), b[1] + r * t.sin()]);
    }
    for i in 0..=ARC_STEPS {
        let t =
            ang + std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * i as f64 / ARC_STEPS as f64;
        pts.push([a[0] + r * t.cos(), a[1] + r * t.sin()]);
    }
    pts
}

fn rect(c: P2, w: f64, h: f64) -> Vec<P2> {
    vec![
        [c[0] - w / 2.0, c[1] - h / 2.0],
        [c[0] + w / 2.0, c[1] - h / 2.0],
        [c[0] + w / 2.0, c[1] + h / 2.0],
        [c[0] - w / 2.0, c[1] + h / 2.0],
    ]
}

fn ccw(mut p: Vec<P2>) -> Vec<P2> {
    let n = p.len();
    let a: f64 = (0..n)
        .map(|i| p[i][0] * p[(i + 1) % n][1] - p[(i + 1) % n][0] * p[i][1])
        .sum();
    if a < 0.0 {
        p.reverse();
    }
    p
}

pub fn contour_area(p: &[P2]) -> f64 {
    let n = p.len();
    0.5 * (0..n)
        .map(|i| p[i][0] * p[(i + 1) % n][1] - p[(i + 1) % n][0] * p[i][1])
        .sum::<f64>()
}

/// Split the file into statements: extended `%…%` blocks and `*`-terminated words. Returns (line, text,
/// extended).
fn statements(text: &str) -> Result<Vec<(usize, String, bool)>, FabError> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut ext = false;
    let mut line = 1usize;
    let mut start_line = 1usize;
    for ch in text.chars() {
        match ch {
            '\n' => {
                line += 1;
            }
            '\r' => {}
            '%' => {
                if ext {
                    ext = false;
                    if !cur.trim().is_empty() {
                        return fail(start_line, "an extended command is not terminated by '*'");
                    }
                    cur.clear();
                } else {
                    ext = true;
                    start_line = line;
                    cur.clear();
                }
            }
            '*' => {
                if cur.trim().is_empty() {
                    return fail(line, "empty statement");
                }
                out.push((
                    if ext { start_line } else { line },
                    cur.trim().to_string(),
                    ext,
                ));
                cur.clear();
            }
            c => {
                if cur.is_empty() {
                    start_line = line;
                }
                cur.push(c)
            }
        }
    }
    if ext {
        return fail(start_line, "an extended command is not closed by '%'");
    }
    if !cur.trim().is_empty() {
        return fail(line, "the last statement is not terminated by '*'");
    }
    Ok(out)
}

pub fn parse(text: &str) -> Result<Image, FabError> {
    let stmts = statements(text)?;
    let mut fmt: Option<(usize, usize)> = None;
    let mut mm: Option<bool> = None;
    let mut aps: HashMap<u32, Ap> = HashMap::new();
    let mut cur_ap: Option<u32> = None;
    let mut dark = true;
    let mut region: Option<Vec<P2>> = None;
    let mut pos: P2 = [0.0, 0.0];
    let mut ended = false;
    let mut img = Image::default();
    let mut last_line = 1;
    for (ln, s, ext) in stmts {
        last_line = ln;
        if ended {
            return fail(ln, "content after M02");
        }
        if ext {
            if let Some(rest) = s.strip_prefix("FSLA") {
                // X<i><d>Y<i><d>
                let b = rest.as_bytes();
                if b.len() < 5
                    || b[0] != b'X'
                    || b[3] != b'Y'
                    || !b[1].is_ascii_digit()
                    || !b[2].is_ascii_digit()
                {
                    return fail(ln, "malformed format statement");
                }
                fmt = Some(((b[1] - b'0') as usize, (b[2] - b'0') as usize));
            } else if let Some(u) = s.strip_prefix("MO") {
                mm = Some(match u {
                    "MM" => true,
                    "IN" => false,
                    _ => return fail(ln, "unit is MM or IN"),
                });
            } else if let Some(rest) = s.strip_prefix("ADD") {
                let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
                let code: u32 = digits.parse().or_else(|_| fail(ln, "aperture number"))?;
                if code < 10 {
                    return fail(ln, "aperture numbers start at 10");
                }
                let def = &rest[digits.len()..];
                let (kind, params) = def
                    .split_once(',')
                    .ok_or(())
                    .or_else(|_| fail(ln, "aperture definition"))?;
                let nums: Result<Vec<f64>, _> =
                    params.split('X').map(|p| p.parse::<f64>()).collect();
                let nums = nums.or_else(|_| fail(ln, "aperture parameters"))?;
                let unit = if mm == Some(false) { 25.4 } else { 1.0 };
                let ap = match (kind, nums.len()) {
                    ("C", 1) => Ap::Circle(nums[0] * unit),
                    ("R", 2) => Ap::Rect(nums[0] * unit, nums[1] * unit),
                    ("O", 2) => Ap::Obround(nums[0] * unit, nums[1] * unit),
                    _ => {
                        return fail(
                            ln,
                            format!(
                                "unsupported aperture '{kind}' with {} parameters",
                                nums.len()
                            ),
                        )
                    }
                };
                aps.insert(code, ap);
            } else if s == "LPD" {
                dark = true;
            } else if s == "LPC" {
                dark = false;
            } else if s.starts_with("TF")
                || s.starts_with("TA")
                || s.starts_with("TD")
                || s.starts_with("TO")
            {
                // attributes carry no geometry
            } else {
                return fail(ln, format!("unknown extended command '{s}'"));
            }
            continue;
        }
        // word statements
        let mut rest = s.as_str();
        // G codes and coordinates and D codes
        let mut x = None;
        let mut y = None;
        let mut d: Option<u32> = None;
        while !rest.is_empty() {
            let c = rest.chars().next().unwrap();
            let tail = &rest[1..];
            let numlen = tail
                .find(|ch: char| !(ch.is_ascii_digit() || ch == '-' || ch == '+' || ch == '.'))
                .unwrap_or(tail.len());
            let num = &tail[..numlen];
            match c {
                'G' => {
                    let n: u32 = num.parse().or_else(|_| fail(ln, "G code"))?;
                    match n {
                        1 => {}
                        36 => {
                            if region.is_some() {
                                return fail(ln, "G36 inside a region");
                            }
                            region = Some(Vec::new());
                        }
                        37 => {
                            let r = region
                                .take()
                                .ok_or(())
                                .or_else(|_| fail(ln, "G37 without G36"))?;
                            if r.len() < 3 {
                                return fail(ln, "a region needs at least 3 points");
                            }
                            img.primitives.push(Primitive {
                                dark,
                                contour: ccw(r),
                            });
                        }
                        4 => {
                            rest = "";
                            continue;
                        }
                        2 | 3 => {
                            return fail(
                                ln,
                                "circular interpolation is not supported by this viewer",
                            )
                        }
                        75 | 74 => {}
                        _ => return fail(ln, format!("unknown G code G{n}")),
                    }
                }
                'X' | 'Y' => {
                    let (i, dd) = fmt
                        .ok_or(())
                        .or_else(|_| fail(ln, "coordinates before the format statement"))?;
                    if num.is_empty()
                        || !num
                            .trim_start_matches(['-', '+'])
                            .chars()
                            .all(|ch| ch.is_ascii_digit())
                    {
                        return fail(ln, format!("bad coordinate '{num}'"));
                    }
                    let v: i64 = num.parse().or_else(|_| fail(ln, "coordinate"))?;
                    let _ = i;
                    let unit = if mm == Some(false) { 25.4 } else { 1.0 };
                    let val = v as f64 / 10f64.powi(dd as i32) * unit;
                    if val.abs() > 1e5 {
                        return fail(ln, "coordinate out of range");
                    }
                    if c == 'X' {
                        x = Some(val)
                    } else {
                        y = Some(val)
                    }
                }
                'D' => {
                    d = Some(num.parse().or_else(|_| fail(ln, "D code"))?);
                }
                'M' => {
                    if num == "2" || num == "02" {
                        ended = true;
                    } else {
                        return fail(ln, format!("unknown M code M{num}"));
                    }
                }
                _ => return fail(ln, format!("unknown word '{c}'")),
            }
            rest = &tail[numlen..];
        }
        if mm.is_none() && (x.is_some() || y.is_some() || d.is_some()) {
            return fail(ln, "geometry before the unit statement");
        }
        let target = [x.unwrap_or(pos[0]), y.unwrap_or(pos[1])];
        match d {
            Some(n) if n >= 10 => {
                if !aps.contains_key(&n) {
                    return fail(ln, format!("aperture D{n} is not defined"));
                }
                cur_ap = Some(n);
            }
            Some(1) => {
                if let Some(r) = region.as_mut() {
                    r.push(target);
                } else {
                    let a = cur_ap
                        .ok_or(())
                        .or_else(|_| fail(ln, "draw before an aperture was selected"))?;
                    match &aps[&a] {
                        Ap::Circle(dia) => img.primitives.push(Primitive {
                            dark,
                            contour: ccw(stadium(pos, target, dia / 2.0)),
                        }),
                        _ => return fail(ln, "only circular apertures can draw"),
                    }
                }
                pos = target;
            }
            Some(2) => {
                if let Some(r) = region.as_mut() {
                    if !r.is_empty() {
                        return fail(ln, "a region is a single contour");
                    }
                    r.push(target);
                }
                pos = target;
            }
            Some(3) => {
                let a = cur_ap
                    .ok_or(())
                    .or_else(|_| fail(ln, "flash before an aperture was selected"))?;
                if region.is_some() {
                    return fail(ln, "a flash inside a region");
                }
                let contour = match &aps[&a] {
                    Ap::Circle(dia) => circle_poly(target, dia / 2.0),
                    Ap::Rect(w, h) => rect(target, *w, *h),
                    Ap::Obround(w, h) => {
                        if w >= h {
                            let k = (w - h) / 2.0;
                            stadium(
                                [target[0] - k, target[1]],
                                [target[0] + k, target[1]],
                                h / 2.0,
                            )
                        } else {
                            let k = (h - w) / 2.0;
                            stadium(
                                [target[0], target[1] - k],
                                [target[0], target[1] + k],
                                w / 2.0,
                            )
                        }
                    }
                };
                img.primitives.push(Primitive {
                    dark,
                    contour: ccw(contour),
                });
                pos = target;
            }
            Some(n) => return fail(ln, format!("D{n} is not a valid operation")),
            None => {
                if x.is_some() || y.is_some() {
                    // modal D01 (draw) continues
                    return fail(ln, "coordinates without an operation code");
                }
            }
        }
    }
    if fmt.is_none() {
        return fail(last_line, "the format statement (FS) is missing");
    }
    if mm.is_none() {
        return fail(last_line, "the unit statement (MO) is missing");
    }
    if region.is_some() {
        return fail(last_line, "a region (G36) is never closed");
    }
    if !ended {
        return fail(last_line, "the file does not end with M02");
    }
    // evaluate polarity in order
    let mut acc: Vec<Vec<Vec<P2>>> = Vec::new();
    for p in &img.primitives {
        let sub: Vec<Vec<P2>> = vec![p.contour.clone()];
        acc = if acc.is_empty() {
            if p.dark {
                sub.overlay(
                    &Vec::<Vec<P2>>::new(),
                    OverlayRule::Union,
                    FillRule::NonZero,
                )
            } else {
                Vec::new()
            }
        } else {
            let a: Vec<Vec<Vec<P2>>> = acc;
            let rule = if p.dark {
                OverlayRule::Union
            } else {
                OverlayRule::Difference
            };
            a.overlay(&sub, rule, FillRule::NonZero)
        };
    }
    img.shapes = acc;
    Ok(img)
}

impl Image {
    pub fn area(&self) -> f64 {
        self.shapes
            .iter()
            .map(|s| {
                s.iter()
                    .enumerate()
                    .map(|(i, c)| {
                        if i == 0 {
                            contour_area(c).abs()
                        } else {
                            -contour_area(c).abs()
                        }
                    })
                    .sum::<f64>()
            })
            .sum()
    }

    pub fn bounds(&self) -> Option<[f64; 4]> {
        let mut b: Option<[f64; 4]> = None;
        for s in &self.shapes {
            for c in s {
                for p in c {
                    b = Some(match b {
                        None => [p[0], p[1], p[0], p[1]],
                        Some(o) => [
                            o[0].min(p[0]),
                            o[1].min(p[1]),
                            o[2].max(p[0]),
                            o[3].max(p[1]),
                        ],
                    });
                }
            }
        }
        b
    }

    /// Area of the intersection with polygons `other` (outer contours, holes as later contours).
    pub fn intersection_area(&self, other: &[Vec<Vec<P2>>]) -> f64 {
        let a: Vec<Vec<Vec<P2>>> = self.shapes.clone();
        let o: Vec<Vec<Vec<P2>>> = other.to_vec();
        let r = a.overlay(&o, OverlayRule::Intersect, FillRule::NonZero);
        shapes_area(&r)
    }
}

pub fn shapes_area(shapes: &[Vec<Vec<P2>>]) -> f64 {
    shapes
        .iter()
        .map(|s| {
            s.iter()
                .enumerate()
                .map(|(i, c)| {
                    if i == 0 {
                        contour_area(c).abs()
                    } else {
                        -contour_area(c).abs()
                    }
                })
                .sum::<f64>()
        })
        .sum()
}

/// Union of contours (each filled), as shapes.
pub fn union_of(contours: &[Vec<P2>]) -> Vec<Vec<Vec<P2>>> {
    let c: Vec<Vec<P2>> = contours.iter().map(|p| ccw(p.clone())).collect();
    c.overlay(
        &Vec::<Vec<P2>>::new(),
        OverlayRule::Union,
        FillRule::NonZero,
    )
}

pub fn stadium_contour(a: P2, b: P2, r: f64) -> Vec<P2> {
    ccw(stadium(a, b, r))
}
