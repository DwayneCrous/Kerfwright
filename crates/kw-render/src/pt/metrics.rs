//! Image metrics: relative MSE on linear radiance, PSNR on the tonemapped image.

pub type Img = Vec<[f64; 3]>;

/// `sum (x - r)^2 / sum r^2` over all pixels and channels.
pub fn relative_mse(x: &[[f64; 3]], r: &[[f64; 3]]) -> f64 {
    let (mut num, mut den) = (0.0, 0.0);
    for (a, b) in x.iter().zip(r) {
        for k in 0..3 {
            num += (a[k] - b[k]).powi(2);
            den += b[k] * b[k];
        }
    }
    num / den.max(1e-300)
}

/// ACES filmic curve followed by the sRGB transfer function; the result is in [0, 1].
pub fn tonemap(l: f64) -> f64 {
    let a = ((l * (2.51 * l + 0.03)) / (l * (2.43 * l + 0.59) + 0.14)).clamp(0.0, 1.0);
    if a <= 0.0031308 {
        12.92 * a
    } else {
        1.055 * a.powf(1.0 / 2.4) - 0.055
    }
}

/// PSNR in dB between two linear images after tonemapping (peak 1).
pub fn psnr_tonemapped(x: &[[f64; 3]], r: &[[f64; 3]]) -> f64 {
    let mut se = 0.0;
    let mut n = 0.0;
    for (a, b) in x.iter().zip(r) {
        for k in 0..3 {
            se += (tonemap(a[k]) - tonemap(b[k])).powi(2);
            n += 1.0;
        }
    }
    let mse = se / n;
    if mse <= 0.0 {
        f64::INFINITY
    } else {
        -10.0 * mse.log10()
    }
}
