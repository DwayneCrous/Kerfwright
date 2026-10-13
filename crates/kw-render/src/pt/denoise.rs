//! Edge-avoiding à-trous denoiser (after Dammertz et al. 2010, with the variance-guided luminance weight of
//! SVGF): the radiance is divided by the first-hit albedo, filtered in five growing 5x5 passes whose weights
//! stop at differences of normal, depth and luminance, and multiplied by the albedo again.

pub struct Guides {
    pub albedo: Vec<[f64; 3]>,
    pub normal: Vec<[f64; 3]>,
    pub depth: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct DenoiseParams {
    pub iterations: usize,
    pub sigma_n: f64,
    pub sigma_z: f64,
    pub sigma_l: f64,
}

impl Default for DenoiseParams {
    fn default() -> Self {
        DenoiseParams {
            iterations: 5,
            sigma_n: 64.0,
            sigma_z: 1.0,
            sigma_l: 2.0,
        }
    }
}

fn lum(c: [f64; 3]) -> f64 {
    0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]
}

pub fn denoise(img: &[[f64; 3]], g: &Guides, size: (u32, u32), p: &DenoiseParams) -> Vec<[f64; 3]> {
    let (w, h) = (size.0 as i64, size.1 as i64);
    let n = (w * h) as usize;
    let at = |x: i64, y: i64| (y * w + x) as usize;
    // demodulate
    let mut cur: Vec<[f64; 3]> = (0..n)
        .map(|i| {
            let a = g.albedo[i];
            [
                img[i][0] / a[0].max(0.02),
                img[i][1] / a[1].max(0.02),
                img[i][2] / a[2].max(0.02),
            ]
        })
        .collect();
    // spatial variance estimate of the luminance, 7x7
    let mut var: Vec<f64> = (0..n)
        .map(|i| {
            let (x, y) = (i as i64 % w, i as i64 / w);
            let (mut s, mut s2, mut c) = (0.0, 0.0, 0.0);
            for dy in -3..=3 {
                for dx in -3..=3 {
                    let (xx, yy) = (x + dx, y + dy);
                    if xx < 0 || yy < 0 || xx >= w || yy >= h {
                        continue;
                    }
                    let j = at(xx, yy);
                    if (g.depth[j] - g.depth[i]).abs() > 0.1 * g.depth[i].abs().min(1e3) {
                        continue;
                    }
                    let l = lum(cur[j]);
                    s += l;
                    s2 += l * l;
                    c += 1.0;
                }
            }
            if c < 2.0 {
                0.0
            } else {
                ((s2 / c) - (s / c).powi(2)).max(0.0)
            }
        })
        .collect();
    let kern = [1.0 / 16.0, 1.0 / 4.0, 3.0 / 8.0, 1.0 / 4.0, 1.0 / 16.0];
    for it in 0..p.iterations {
        let step = 1i64 << it;
        let mut next = vec![[0.0; 3]; n];
        let mut nvar = vec![0.0; n];
        // 3x3 blurred variance as the luminance scale
        let svar: Vec<f64> = (0..n)
            .map(|i| {
                let (x, y) = (i as i64 % w, i as i64 / w);
                let (mut s, mut c) = (0.0, 0.0);
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        let (xx, yy) = (x + dx, y + dy);
                        if xx >= 0 && yy >= 0 && xx < w && yy < h {
                            s += var[at(xx, yy)];
                            c += 1.0;
                        }
                    }
                }
                s / c
            })
            .collect();
        for y in 0..h {
            for x in 0..w {
                let i = at(x, y);
                let (lp, np, zp) = (lum(cur[i]), g.normal[i], g.depth[i]);
                let mut acc = [0.0; 3];
                let mut accv = 0.0;
                let mut wsum = 0.0;
                for (ky, kyw) in kern.iter().enumerate() {
                    for (kx, kxw) in kern.iter().enumerate() {
                        let (xx, yy) = (x + (kx as i64 - 2) * step, y + (ky as i64 - 2) * step);
                        if xx < 0 || yy < 0 || xx >= w || yy >= h {
                            continue;
                        }
                        let j = at(xx, yy);
                        let dn = (np[0] * g.normal[j][0]
                            + np[1] * g.normal[j][1]
                            + np[2] * g.normal[j][2])
                            .max(0.0);
                        let wn = dn.powf(p.sigma_n);
                        let dz = (zp - g.depth[j]).abs()
                            / (p.sigma_z * (step as f64) * 0.02 * zp.abs().clamp(0.5, 1e3) + 1e-6);
                        let wz = (-dz).exp();
                        let wl =
                            (-(lp - lum(cur[j])).abs() / (p.sigma_l * svar[i].sqrt() + 1e-4)).exp();
                        let wt = kxw * kyw * wn * wz * wl;
                        for k in 0..3 {
                            acc[k] += wt * cur[j][k];
                        }
                        accv += wt * wt * var[j];
                        wsum += wt;
                    }
                }
                if wsum > 1e-12 {
                    next[i] = [acc[0] / wsum, acc[1] / wsum, acc[2] / wsum];
                    nvar[i] = accv / (wsum * wsum);
                } else {
                    next[i] = cur[i];
                    nvar[i] = var[i];
                }
            }
        }
        cur = next;
        var = nvar;
    }
    (0..n)
        .map(|i| {
            let a = g.albedo[i];
            [
                cur[i][0] * a[0].max(0.02),
                cur[i][1] * a[1].max(0.02),
                cur[i][2] * a[2].max(0.02),
            ]
        })
        .collect()
}
