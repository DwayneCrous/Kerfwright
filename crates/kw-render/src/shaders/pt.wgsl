// Progressive path tracer: the light-transport model of crates/kw-render/src/pt/mod.rs.

struct Node { bmin: vec3<f32>, left: u32, bmax: vec3<f32>, right: u32 };
struct Mat { albedo: vec3<f32>, rough: f32, emission: vec3<f32>, metallic: f32, specular: f32, p0: f32, p1: f32, p2: f32 };
struct Params {
  origin: vec4<f32>,
  fwd: vec4<f32>,
  right: vec4<f32>,
  up: vec4<f32>,
  env: vec4<f32>,
  width: u32, height: u32, frame: u32, seed: u32,
  n_lights: u32, max_depth: u32, spp: u32, p0: u32,
  total_area: f32, q0: f32, q1: f32, q2: f32,
};

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> nodes: array<Node>;
@group(0) @binding(2) var<storage, read> tri_v0: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> tri_e1: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read> tri_e2: array<vec4<f32>>;
@group(0) @binding(5) var<storage, read> mats: array<Mat>;
@group(0) @binding(6) var<storage, read> lights: array<vec4<f32>>;
@group(0) @binding(7) var<storage, read_write> accum: array<vec4<f32>>;

const PI: f32 = 3.14159265358979;
const LEAF: u32 = 0x80000000u;
const EPS: f32 = 2e-4;

// ---- random numbers (PCG hash)
fn pcg(v: u32) -> u32 {
  let s = v * 747796405u + 2891336453u;
  let w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
  return (w >> 22u) ^ w;
}
var<private> rng: u32;
fn rnd() -> f32 {
  rng = pcg(rng);
  return f32(rng >> 8u) * (1.0 / 16777216.0);
}

// ---- BRDF (mirror of cpu_ref.rs)
fn fresnel(m: Mat, vh: f32) -> vec3<f32> {
  let f0 = mix(vec3<f32>(0.04), m.albedo, m.metallic);
  let k = pow(clamp(1.0 - vh, 0.0, 1.0), 5.0);
  return f0 + (vec3<f32>(1.0) - f0) * k;
}
fn ggx_d(a2: f32, nh: f32) -> f32 {
  let t = nh * nh * (a2 - 1.0) + 1.0;
  return a2 / (PI * t * t);
}
fn lambda(a2: f32, c: f32) -> f32 {
  let t2 = (1.0 - c * c) / (c * c);
  return (-1.0 + sqrt(1.0 + a2 * t2)) * 0.5;
}
fn brdf(m: Mat, n: vec3<f32>, wo: vec3<f32>, wi: vec3<f32>) -> vec3<f32> {
  let co = dot(n, wo);
  let ci = dot(n, wi);
  if (co <= 0.0 || ci <= 0.0) { return vec3<f32>(0.0); }
  let a = max(m.rough * m.rough, 1e-3);
  let a2 = a * a;
  let h = normalize(wo + wi);
  let nh = max(dot(n, h), 0.0);
  let vh = max(dot(wo, h), 0.0);
  let f = fresnel(m, vh);
  let d = ggx_d(a2, nh);
  let g = 1.0 / (1.0 + lambda(a2, co) + lambda(a2, ci));
  let spec = m.specular * d * g / (4.0 * ci * co);
  return (1.0 - m.metallic) * (vec3<f32>(1.0) - m.specular * f) * m.albedo / PI + spec * f;
}
fn p_spec(m: Mat) -> f32 {
  if (m.specular <= 0.0) { return 0.0; }
  if (m.metallic >= 1.0) { return 1.0; }
  return 0.5;
}
fn bsdf_pdf(m: Mat, n: vec3<f32>, wo: vec3<f32>, wi: vec3<f32>) -> f32 {
  let co = dot(n, wo);
  let ci = dot(n, wi);
  if (co <= 0.0 || ci <= 0.0) { return 0.0; }
  let ps = p_spec(m);
  let a = max(m.rough * m.rough, 1e-3);
  let h = normalize(wo + wi);
  let nh = max(dot(n, h), 0.0);
  let vh = max(dot(wo, h), 1e-9);
  return (1.0 - ps) * ci / PI + ps * ggx_d(a * a, nh) * nh / (4.0 * vh);
}
fn onb(n: vec3<f32>) -> mat3x3<f32> {
  var h = vec3<f32>(1.0, 0.0, 0.0);
  if (abs(n.x) >= 0.9) { h = vec3<f32>(0.0, 1.0, 0.0); }
  let t = normalize(cross(h, n));
  let b = cross(n, t);
  return mat3x3<f32>(t, b, n);
}
// returns wi (zero vector when the sample is rejected)
fn bsdf_sample(m: Mat, n: vec3<f32>, wo: vec3<f32>, u: vec3<f32>) -> vec3<f32> {
  let fr = onb(n);
  let ps = p_spec(m);
  var wi: vec3<f32>;
  if (u.z >= ps) {
    let r = sqrt(u.x);
    let ph = 2.0 * PI * u.y;
    wi = fr * vec3<f32>(r * cos(ph), r * sin(ph), sqrt(max(1.0 - u.x, 0.0)));
  } else {
    let a = max(m.rough * m.rough, 1e-3);
    let a2 = a * a;
    let ct = sqrt((1.0 - u.x) / (1.0 + (a2 - 1.0) * u.x));
    let st = sqrt(max(1.0 - ct * ct, 0.0));
    let ph = 2.0 * PI * u.y;
    let h = fr * vec3<f32>(st * cos(ph), st * sin(ph), ct);
    let vh = dot(wo, h);
    if (vh <= 0.0) { return vec3<f32>(0.0); }
    wi = 2.0 * vh * h - wo;
  }
  if (dot(n, wi) <= 0.0) { return vec3<f32>(0.0); }
  return wi;
}

// ---- geometry
struct Hit { t: f32, tri: u32, ok: bool };

fn tri_hit(i: u32, o: vec3<f32>, d: vec3<f32>, tmax: f32) -> f32 {
  let v0 = tri_v0[i].xyz;
  let e1 = tri_e1[i].xyz;
  let e2 = tri_e2[i].xyz;
  let p = cross(d, e2);
  let det = dot(e1, p);
  if (abs(det) < 1e-12) { return -1.0; }
  let inv = 1.0 / det;
  let s = o - v0;
  let u = dot(s, p) * inv;
  if (u < 0.0 || u > 1.0) { return -1.0; }
  let q = cross(s, e1);
  let v = dot(d, q) * inv;
  if (v < 0.0 || u + v > 1.0) { return -1.0; }
  let t = dot(e2, q) * inv;
  if (t > 1e-5 && t < tmax) { return t; }
  return -1.0;
}

fn intersect(o: vec3<f32>, d: vec3<f32>, tmax_in: f32) -> Hit {
  var h: Hit;
  h.ok = false;
  h.t = tmax_in;
  h.tri = 0u;
  let inv = vec3<f32>(1.0) / d;
  var stack: array<u32, 64>;
  var sp = 1u;
  stack[0] = 0u;
  loop {
    if (sp == 0u) { break; }
    sp = sp - 1u;
    let c = stack[sp];
    if ((c & LEAF) != 0u) {
      let ti = c & ~LEAF;
      let t = tri_hit(ti, o, d, h.t);
      if (t > 0.0) { h.t = t; h.tri = ti; h.ok = true; }
      continue;
    }
    let nd = nodes[c];
    let a = (nd.bmin - o) * inv;
    let b = (nd.bmax - o) * inv;
    let lo = min(a, b);
    let hi = max(a, b);
    let t0 = max(max(lo.x, lo.y), max(lo.z, 0.0));
    let t1 = min(min(hi.x, hi.y), min(hi.z, h.t));
    if (t0 <= t1) {
      stack[sp] = nd.left;
      stack[sp + 1u] = nd.right;
      sp = sp + 2u;
    }
  }
  return h;
}

fn tri_normal(i: u32) -> vec3<f32> {
  return normalize(cross(tri_e1[i].xyz, tri_e2[i].xyz));
}

fn radiance(o_in: vec3<f32>, d_in: vec3<f32>) -> vec3<f32> {
  var o = o_in;
  var d = d_in;
  var l = vec3<f32>(0.0);
  var thr = vec3<f32>(1.0);
  var prev_pdf = 0.0;
  for (var depth = 0u; depth <= P.max_depth; depth = depth + 1u) {
    let h = intersect(o, d, 1e30);
    if (!h.ok) { l = l + thr * P.env.xyz; break; }
    let p = o + d * h.t;
    let ng = tri_normal(h.tri);
    let facing = dot(ng, d) < 0.0;
    var n = ng;
    if (!facing) { n = -ng; }
    let m = mats[bitcast<u32>(tri_v0[h.tri].w)];
    let wo = -d;
    if (any(m.emission > vec3<f32>(0.0))) {
      if (facing) {
        var w = 1.0;
        if (depth > 0u && P.n_lights > 0u) {
          let pdf_l = h.t * h.t / (abs(dot(ng, wo)) * P.total_area);
          w = prev_pdf * prev_pdf / (prev_pdf * prev_pdf + pdf_l * pdf_l);
        }
        l = l + thr * m.emission * w;
      }
      break;
    }
    let po = p + n * EPS;
    if (P.n_lights > 0u) {
      let u = rnd() * P.total_area;
      var lo = 0u;
      var hi = P.n_lights - 1u;
      while (lo < hi) {
        let mid = (lo + hi) / 2u;
        if (lights[mid].y < u) { lo = mid + 1u; } else { hi = mid; }
      }
      let li = bitcast<u32>(lights[lo].x);
      var r1 = rnd();
      var r2 = rnd();
      if (r1 + r2 > 1.0) { r1 = 1.0 - r1; r2 = 1.0 - r2; }
      let q = tri_v0[li].xyz + tri_e1[li].xyz * r1 + tri_e2[li].xyz * r2;
      let to = q - po;
      let dist = length(to);
      let wi = to / dist;
      let nl = tri_normal(li);
      let cl = -dot(nl, wi);
      let ci = dot(n, wi);
      if (cl > 0.0 && ci > 0.0) {
        let sh = intersect(po, wi, dist - EPS);
        if (!sh.ok) {
          let pdf_l = dist * dist / (cl * P.total_area);
          let pdf_b = bsdf_pdf(m, n, wo, wi);
          let f = brdf(m, n, wo, wi);
          let le = mats[bitcast<u32>(tri_v0[li].w)].emission;
          let w = pdf_l * pdf_l / (pdf_l * pdf_l + pdf_b * pdf_b);
          l = l + thr * f * ci * le * (w / pdf_l);
        }
      }
    }
    let u3 = vec3<f32>(rnd(), rnd(), rnd());
    let wi = bsdf_sample(m, n, wo, u3);
    if (dot(wi, wi) == 0.0) { break; }
    let pb = bsdf_pdf(m, n, wo, wi);
    if (pb <= 0.0) { break; }
    let f = brdf(m, n, wo, wi);
    thr = thr * f * (dot(n, wi) / pb);
    prev_pdf = pb;
    if (depth >= 2u) {
      let q = clamp(max(thr.x, max(thr.y, thr.z)), 0.05, 0.95);
      if (rnd() > q) { break; }
      thr = thr / q;
    }
    o = po;
    d = wi;
  }
  return l;
}

@compute @workgroup_size(8, 8, 1)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  if (gid.x >= P.width || gid.y >= P.height) { return; }
  let pix = gid.y * P.width + gid.x;
  var sum = vec3<f32>(0.0);
  for (var s = 0u; s < P.spp; s = s + 1u) {
    rng = pcg(pix ^ pcg(P.frame * 8191u + s * 131u + P.seed));
    let px = f32(gid.x) + rnd();
    let py = f32(gid.y) + rnd();
    let x = 2.0 * px / f32(P.width) - 1.0;
    let y = 1.0 - 2.0 * py / f32(P.height);
    let d = normalize(P.fwd.xyz + P.right.xyz * x + P.up.xyz * y);
    let c = radiance(P.origin.xyz, d);
    if (all(c == c) && all(abs(c) < vec3<f32>(1e12))) { sum = sum + c; }
  }
  accum[pix] = accum[pix] + vec4<f32>(sum, 0.0);
}

// ---- BRDF test harness: n, wo, wi, albedo+rough, metallic+specular -> f.rgb, pdf
struct TestIn { n: vec4<f32>, wo: vec4<f32>, wi: vec4<f32>, a: vec4<f32>, b: vec4<f32> };
@group(1) @binding(0) var<storage, read> test_in: array<TestIn>;
@group(1) @binding(1) var<storage, read_write> test_out: array<vec4<f32>>;

@compute @workgroup_size(64, 1, 1)
fn brdf_test(@builtin(global_invocation_id) gid: vec3<u32>) {
  if (gid.x >= arrayLength(&test_in)) { return; }
  let t = test_in[gid.x];
  var m: Mat;
  m.albedo = t.a.xyz;
  m.rough = t.a.w;
  m.emission = vec3<f32>(0.0);
  m.metallic = t.b.x;
  m.specular = t.b.y;
  let f = brdf(m, t.n.xyz, t.wo.xyz, t.wi.xyz);
  test_out[gid.x] = vec4<f32>(f, bsdf_pdf(m, t.n.xyz, t.wo.xyz, t.wi.xyz));
}
