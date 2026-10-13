//! Gerber viewer B: a single-pass reader that paints a bitmap as it goes. It shares no code with viewer A.

use crate::{fail, FabError, P2};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct Canvas {
    pub x0: f64,
    pub y0: f64,
    pub px: f64,
    pub w: usize,
    pub h: usize,
    pub data: Vec<u8>,
}

impl Canvas {
    pub fn new(x0: f64, y0: f64, x1: f64, y1: f64, px: f64) -> Canvas {
        let w = ((x1 - x0) / px).ceil() as usize;
        let h = ((y1 - y0) / px).ceil() as usize;
        Canvas {
            x0,
            y0,
            px,
            w,
            h,
            data: vec![0; w * h],
        }
    }

    fn range(&self, lo: f64, hi: f64, origin: f64, n: usize) -> (usize, usize) {
        let a = (((lo - origin) / self.px - 0.5).floor().max(0.0)) as usize;
        let b = ((((hi - origin) / self.px + 0.5).ceil()).max(0.0) as usize).min(n);
        (a.min(n), b)
    }

    /// Paint with anti-aliasing: `sd` is the signed distance from a pixel centre to the shape's outline
    /// (negative inside). Coverage is `clamp(0.5 − sd/px)`, which keeps areas accurate for edges of any
    /// orientation (plain centre sampling aliases lines that run along the pixel lattice diagonals).
    fn paint(&mut self, bx: [f64; 4], dark: bool, sd: &dyn Fn(f64, f64) -> f64) {
        let pad = self.px;
        let (ix0, ix1) = self.range(bx[0] - pad, bx[2] + pad, self.x0, self.w);
        let (iy0, iy1) = self.range(bx[1] - pad, bx[3] + pad, self.y0, self.h);
        for iy in iy0..iy1 {
            let y = self.y0 + (iy as f64 + 0.5) * self.px;
            for ix in ix0..ix1 {
                let x = self.x0 + (ix as f64 + 0.5) * self.px;
                let cov = (0.5 - sd(x, y) / self.px).clamp(0.0, 1.0);
                if cov <= 0.0 {
                    continue;
                }
                let cell = &mut self.data[iy * self.w + ix];
                let c = (cov * 255.0).round() as u8;
                if dark {
                    *cell = (*cell).max(c);
                } else {
                    *cell = (*cell).min(255 - c);
                }
            }
        }
    }

    pub fn paint_cap(&mut self, a: P2, b: P2, r: f64, value: u8) {
        let bx = [
            a[0].min(b[0]) - r,
            a[1].min(b[1]) - r,
            a[0].max(b[0]) + r,
            a[1].max(b[1]) + r,
        ];
        self.paint(bx, value != 0, &|x, y| {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let l2 = dx * dx + dy * dy;
            let t = if l2 < 1e-300 {
                0.0
            } else {
                (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0)
            };
            let (qx, qy) = (a[0] + dx * t, a[1] + dy * t);
            (x - qx).hypot(y - qy) - r
        });
    }

    pub fn paint_poly(&mut self, poly: &[P2], value: u8) {
        let mut bx = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
        for p in poly {
            bx = [
                bx[0].min(p[0]),
                bx[1].min(p[1]),
                bx[2].max(p[0]),
                bx[3].max(p[1]),
            ];
        }
        let n = poly.len();
        self.paint(bx, value != 0, &|x, y| {
            let mut inside = false;
            let mut dmin = f64::MAX;
            let mut j = n - 1;
            for i in 0..n {
                let (a, b) = (poly[i], poly[j]);
                if (a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
                {
                    inside = !inside;
                }
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let l2 = dx * dx + dy * dy;
                let t = if l2 < 1e-300 {
                    0.0
                } else {
                    (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0)
                };
                dmin = dmin.min((x - (a[0] + dx * t)).hypot(y - (a[1] + dy * t)));
                j = i;
            }
            if inside {
                -dmin
            } else {
                dmin
            }
        });
    }

    pub fn paint_rect(&mut self, c: P2, w: f64, h: f64, value: u8) {
        self.paint(
            [
                c[0] - w / 2.0,
                c[1] - h / 2.0,
                c[0] + w / 2.0,
                c[1] + h / 2.0,
            ],
            value != 0,
            &|x, y| {
                let (qx, qy) = ((x - c[0]).abs() - w / 2.0, (y - c[1]).abs() - h / 2.0);
                qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0)
            },
        );
    }

    pub fn lit(&self) -> usize {
        self.data.iter().filter(|v| **v >= 128).count()
    }

    /// Covered area (sum of the pixel coverages).
    pub fn area(&self) -> f64 {
        self.data.iter().map(|v| *v as f64 / 255.0).sum::<f64>() * self.px * self.px
    }

    /// Intersection over union of the coverages of two canvases of the same size.
    pub fn iou(&self, o: &Canvas) -> f64 {
        assert_eq!((self.w, self.h), (o.w, o.h));
        let (mut i, mut u) = (0.0f64, 0.0f64);
        for (a, b) in self.data.iter().zip(&o.data) {
            let (a, b) = (*a as f64, *b as f64);
            i += a.min(b);
            u += a.max(b);
        }
        if u == 0.0 {
            1.0
        } else {
            i / u
        }
    }
}

enum Shape {
    C(f64),
    R(f64, f64),
    O(f64, f64),
}

/// Read `text` and paint it on a fresh canvas covering `[x0,x1]×[y0,y1]`.
pub fn render(text: &str, x0: f64, y0: f64, x1: f64, y1: f64, px: f64) -> Result<Canvas, FabError> {
    let mut cv = Canvas::new(x0, y0, x1, y1, px);
    let mut line = 1usize;
    let bytes: Vec<char> = text.chars().collect();
    let mut i = 0usize;
    let mut scale = 0.0f64; // coordinate divisor: 10^decimals, 0 until FS seen
    let mut unit_mm: Option<bool> = None;
    let mut apertures: HashMap<u32, Shape> = HashMap::new();
    let mut cur: Option<u32> = None;
    let mut value = 1u8;
    let mut region: Option<Vec<P2>> = None;
    let mut pos: P2 = [0.0; 2];
    let mut done = false;
    while i < bytes.len() {
        let c = bytes[i];
        if c == '\n' {
            line += 1;
            i += 1;
            continue;
        }
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        if done {
            return fail(line, "content after the end of the file");
        }
        if c == '%' {
            // extended command(s) up to the closing %
            let mut j = i + 1;
            let mut body = String::new();
            while j < bytes.len() && bytes[j] != '%' {
                if bytes[j] == '\n' {
                    line += 1;
                }
                body.push(bytes[j]);
                j += 1;
            }
            if j >= bytes.len() {
                return fail(line, "unterminated extended command");
            }
            if !body.trim_end().ends_with('*') {
                return fail(line, "an extended command is not terminated by '*'");
            }
            for cmd in body.split('*') {
                let cmd = cmd.trim();
                if cmd.is_empty() {
                    continue;
                }
                if let Some(r) = cmd.strip_prefix("FSLAX") {
                    // two digits integer/decimal for X then Y
                    let digs: Vec<char> = r.chars().collect();
                    if digs.len() != 5
                        || digs[2] != 'Y'
                        || !digs[0].is_ascii_digit()
                        || !digs[1].is_ascii_digit()
                    {
                        return fail(line, "malformed FS");
                    }
                    scale = 10f64.powi((digs[1] as u8 - b'0') as i32);
                } else if cmd == "MOMM" {
                    unit_mm = Some(true);
                } else if cmd == "MOIN" {
                    unit_mm = Some(false);
                } else if let Some(r) = cmd.strip_prefix("ADD") {
                    let k = r.find(|ch: char| !ch.is_ascii_digit()).unwrap_or(r.len());
                    let code: u32 = r[..k].parse().or_else(|_| fail(line, "aperture number"))?;
                    if code < 10 {
                        return fail(line, "aperture numbers start at 10");
                    }
                    let params: Vec<&str> = r[k + 2..].split('X').collect();
                    let vals: Result<Vec<f64>, _> =
                        params.iter().map(|p| p.parse::<f64>()).collect();
                    let v = vals.or_else(|_| fail(line, "aperture parameters"))?;
                    let u = if unit_mm == Some(false) { 25.4 } else { 1.0 };
                    let kind = &r[k..k + 1];
                    let sh = match (kind, v.len()) {
                        ("C", 1) => Shape::C(v[0] * u),
                        ("R", 2) => Shape::R(v[0] * u, v[1] * u),
                        ("O", 2) => Shape::O(v[0] * u, v[1] * u),
                        _ => return fail(line, "unsupported aperture"),
                    };
                    apertures.insert(code, sh);
                } else if cmd == "LPD" {
                    value = 1;
                } else if cmd == "LPC" {
                    value = 0;
                } else if cmd.starts_with("TF")
                    || cmd.starts_with("TA")
                    || cmd.starts_with("TD")
                    || cmd.starts_with("TO")
                {
                } else {
                    return fail(line, format!("unknown extended command '{cmd}'"));
                }
            }
            i = j + 1;
            continue;
        }
        // ordinary statement up to '*'
        let mut j = i;
        let mut stmt = String::new();
        while j < bytes.len() && bytes[j] != '*' {
            if bytes[j] == '\n' {
                line += 1;
            }
            stmt.push(bytes[j]);
            j += 1;
        }
        if j >= bytes.len() {
            return fail(line, "statement without '*'");
        }
        i = j + 1;
        let stmt = stmt.trim();
        if stmt.starts_with("G04") {
            continue;
        }
        // tokens: letters followed by signed digits
        let ch: Vec<char> = stmt.chars().collect();
        let (mut k, mut x, mut y, mut d) = (0usize, None::<f64>, None::<f64>, None::<u32>);
        while k < ch.len() {
            let l = ch[k];
            k += 1;
            let s = k;
            while k < ch.len() && (ch[k].is_ascii_digit() || ch[k] == '-' || ch[k] == '+') {
                k += 1;
            }
            let numtxt: String = ch[s..k].iter().collect();
            if numtxt.is_empty() {
                return fail(line, format!("'{l}' has no number"));
            }
            match l {
                'G' => match numtxt.parse::<u32>().unwrap_or(999) {
                    1 | 74 | 75 => {}
                    36 => {
                        if region.is_some() {
                            return fail(line, "nested region");
                        }
                        region = Some(vec![]);
                    }
                    37 => {
                        let r = region
                            .take()
                            .ok_or(())
                            .or_else(|_| fail(line, "G37 without G36"))?;
                        if r.len() < 3 {
                            return fail(line, "degenerate region");
                        }
                        cv.paint_poly(&r, value);
                    }
                    n => return fail(line, format!("unsupported G{n}")),
                },
                'X' | 'Y' => {
                    if scale == 0.0 {
                        return fail(line, "coordinates before FS");
                    }
                    let v: i64 = numtxt.parse().or_else(|_| fail(line, "bad coordinate"))?;
                    let val = v as f64 / scale * if unit_mm == Some(false) { 25.4 } else { 1.0 };
                    if l == 'X' {
                        x = Some(val)
                    } else {
                        y = Some(val)
                    }
                }
                'D' => d = Some(numtxt.parse().or_else(|_| fail(line, "bad D code"))?),
                'M' => {
                    if numtxt.parse::<u32>().ok() == Some(2) {
                        done = true;
                    } else {
                        return fail(line, "unknown M code");
                    }
                }
                other => return fail(line, format!("unknown word '{other}'")),
            }
        }
        if unit_mm.is_none() && (x.is_some() || y.is_some() || d.is_some()) {
            return fail(line, "geometry before MO");
        }
        let target = [x.unwrap_or(pos[0]), y.unwrap_or(pos[1])];
        match d {
            Some(n) if n >= 10 => {
                if !apertures.contains_key(&n) {
                    return fail(line, format!("undefined aperture D{n}"));
                }
                cur = Some(n);
            }
            Some(2) => {
                if let Some(r) = region.as_mut() {
                    if !r.is_empty() {
                        return fail(line, "region with several contours");
                    }
                    r.push(target);
                }
                pos = target;
            }
            Some(1) => {
                if let Some(r) = region.as_mut() {
                    r.push(target);
                } else {
                    match cur.and_then(|c| apertures.get(&c)) {
                        Some(Shape::C(dia)) => cv.paint_cap(pos, target, dia / 2.0, value),
                        Some(_) => return fail(line, "only circles draw"),
                        None => return fail(line, "draw without aperture"),
                    }
                }
                pos = target;
            }
            Some(3) => {
                match cur.and_then(|c| apertures.get(&c)) {
                    Some(Shape::C(dia)) => cv.paint_cap(target, target, dia / 2.0, value),
                    Some(Shape::R(w, h)) => cv.paint_rect(target, *w, *h, value),
                    Some(Shape::O(w, h)) => {
                        if w >= h {
                            let k = (w - h) / 2.0;
                            cv.paint_cap(
                                [target[0] - k, target[1]],
                                [target[0] + k, target[1]],
                                h / 2.0,
                                value,
                            )
                        } else {
                            let k = (h - w) / 2.0;
                            cv.paint_cap(
                                [target[0], target[1] - k],
                                [target[0], target[1] + k],
                                w / 2.0,
                                value,
                            )
                        }
                    }
                    None => return fail(line, "flash without aperture"),
                }
                pos = target;
            }
            Some(n) => return fail(line, format!("D{n} is not an operation")),
            None => {
                if x.is_some() || y.is_some() {
                    return fail(line, "coordinates without D code");
                }
            }
        }
    }
    if scale == 0.0 {
        return fail(line, "no FS statement");
    }
    if unit_mm.is_none() {
        return fail(line, "no MO statement");
    }
    if region.is_some() {
        return fail(line, "region never closed");
    }
    if !done {
        return fail(line, "no M02");
    }
    Ok(cv)
}
