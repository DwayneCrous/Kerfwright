//! Low-level knot insertion/removal on homogeneous control points
//! `(w·x, w·y, w·z, w)`, shared by curves and (row/column-wise) surfaces.

use crate::knots::KnotVector;
use nalgebra::Vector4;

pub type Hom = Vector4<f64>;

/// Insert knot `u` once (Piegl & Tiller Eq. 5.15). Returns `None` if the knot
/// is outside the open domain or already has multiplicity `p`.
pub fn insert_knot_once(kv: &KnotVector, pw: &[Hom], u: f64) -> Option<(KnotVector, Vec<Hom>)> {
    let p = kv.degree();
    let (a, b) = kv.domain();
    if !(u > a && u < b) {
        return None;
    }
    let s = kv.multiplicity(u);
    if s >= p {
        return None;
    }
    let k = kv.find_span(u);
    let knots = kv.as_slice();
    let n = pw.len() - 1;
    let mut new_knots = Vec::with_capacity(knots.len() + 1);
    new_knots.extend_from_slice(&knots[..=k]);
    new_knots.push(u);
    new_knots.extend_from_slice(&knots[k + 1..]);

    let mut q = Vec::with_capacity(pw.len() + 1);
    for i in 0..=n + 1 {
        if i <= k - p {
            q.push(pw[i]);
        } else if i > k - s {
            q.push(pw[i - 1]);
        } else {
            let alpha = (u - knots[i]) / (knots[i + p] - knots[i]);
            q.push(pw[i] * alpha + pw[i - 1] * (1.0 - alpha));
        }
    }
    Some((KnotVector::new(p, new_knots).ok()?, q))
}

/// Try to remove knot `u` up to `num` times (Piegl & Tiller A5.8). `tol` is a
/// distance bound in homogeneous space. Returns the new knot vector, control
/// points, and the number of removals performed.
pub fn remove_knot(
    kv: &KnotVector,
    pw: &[Hom],
    u: f64,
    num: usize,
    tol: f64,
) -> (KnotVector, Vec<Hom>, usize) {
    let p = kv.degree() as isize;
    let knots_in = kv.as_slice();
    let s = kv.multiplicity(u) as isize;
    let (a, b) = kv.domain();
    if s == 0 || !(u > a && u < b) {
        return (kv.clone(), pw.to_vec(), 0);
    }
    // r = index of the last knot equal to u
    let r = (0..knots_in.len())
        .rev()
        .find(|&i| knots_in[i] == u)
        .unwrap() as isize;
    let mut knots = knots_in.to_vec();
    let mut pw = pw.to_vec();
    let n = pw.len() as isize - 1;
    let m = n + p + 1;
    let ord = p + 1;
    let fout = (2 * r - s - p) / 2;
    let mut last = r - s;
    let mut first = r - p;
    let mut temp = vec![Hom::zeros(); (2 * p + 1) as usize];
    let mut t: isize = 0;
    while (t as usize) < num {
        let off = first - 1;
        if off < 0 || (last + 1) as usize >= pw.len() {
            break;
        }
        temp[0] = pw[off as usize];
        temp[(last + 1 - off) as usize] = pw[(last + 1) as usize];
        let (mut i, mut j) = (first, last);
        let (mut ii, mut jj) = (1isize, last - off);
        while j - i > t {
            let alfi =
                (u - knots[i as usize]) / (knots[(i + ord + t) as usize] - knots[i as usize]);
            let alfj = (u - knots[(j - t) as usize])
                / (knots[(j + ord) as usize] - knots[(j - t) as usize]);
            temp[ii as usize] = (pw[i as usize] - temp[(ii - 1) as usize] * (1.0 - alfi)) / alfi;
            temp[jj as usize] = (pw[j as usize] - temp[(jj + 1) as usize] * alfj) / (1.0 - alfj);
            i += 1;
            ii += 1;
            j -= 1;
            jj -= 1;
        }
        let removable = if j - i < t {
            (temp[(ii - 1) as usize] - temp[(jj + 1) as usize]).norm() <= tol
        } else {
            let alfi =
                (u - knots[i as usize]) / (knots[(i + ord + t) as usize] - knots[i as usize]);
            let blended =
                temp[(ii + t + 1) as usize] * alfi + temp[(ii - 1) as usize] * (1.0 - alfi);
            (pw[i as usize] - blended).norm() <= tol
        };
        if !removable {
            break;
        }
        let (mut i, mut j) = (first, last);
        while j - i > t {
            pw[i as usize] = temp[(i - off) as usize];
            pw[j as usize] = temp[(j - off) as usize];
            i += 1;
            j -= 1;
        }
        first -= 1;
        last += 1;
        t += 1;
    }
    if t == 0 {
        return (kv.clone(), pw, 0);
    }
    for k in (r + 1)..=m {
        knots[(k - t) as usize] = knots[k as usize];
    }
    let mut j = fout;
    let mut i = j;
    for k in 1..t {
        if k % 2 == 1 {
            i += 1;
        } else {
            j -= 1;
        }
    }
    for k in (i + 1)..=n {
        pw[j as usize] = pw[k as usize];
        j += 1;
    }
    knots.truncate((m + 1 - t) as usize);
    pw.truncate((n + 1 - t) as usize);
    (
        KnotVector::new(kv.degree(), knots).expect("valid knots after removal"),
        pw,
        t as usize,
    )
}

/// Binomial coefficient as f64.
pub fn binom(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let mut r = 1.0;
    for i in 0..k {
        r = r * (n - i) as f64 / (i + 1) as f64;
    }
    r
}
