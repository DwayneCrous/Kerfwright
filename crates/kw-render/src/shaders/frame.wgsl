// Realtime frame: opaque PBR + view-space normal/depth, weighted blended OIT, GTAO, composite, edges.

struct Globals {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    cam_pos: vec4<f32>,
    light_dir: vec4<f32>,
    viewport: vec4<f32>, // width, height, line width (px), edge depth bias
    params: vec4<f32>,   // metallic, roughness, exposure, ambient
    proj: vec4<f32>,     // near, far, tan_x, tan_y
    extra: vec4<f32>,    // ao radius (world), ao strength, oit alpha scale, ao enabled
};
@group(0) @binding(0) var<uniform> g: Globals;

struct VsIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) id: u32,
    @location(3) color: vec4<f32>,
};
struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

@vertex
fn vs_main(v: VsIn) -> VsOut {
    var o: VsOut;
    o.clip = g.view_proj * vec4<f32>(v.pos, 1.0);
    o.world = v.pos;
    o.normal = v.normal;
    o.color = v.color;
    return o;
}

const PI: f32 = 3.14159265;

fn d_ggx(nh: f32, a: f32) -> f32 {
    let a2 = a * a;
    let d = nh * nh * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}
fn g_schlick(nx: f32, k: f32) -> f32 { return nx / (nx * (1.0 - k) + k); }
fn fresnel(vh: f32, f0: vec3<f32>) -> vec3<f32> { return f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - vh, 5.0); }

fn light(n: vec3<f32>, v: vec3<f32>, l: vec3<f32>, radiance: vec3<f32>, albedo: vec3<f32>, metallic: f32, rough: f32) -> vec3<f32> {
    let h = normalize(v + l);
    let nl = max(dot(n, l), 0.0);
    let nv = max(dot(n, v), 1e-4);
    let nh = max(dot(n, h), 0.0);
    let vh = max(dot(v, h), 0.0);
    let a = max(rough * rough, 0.02);
    let k = (rough + 1.0) * (rough + 1.0) / 8.0;
    let f0 = mix(vec3<f32>(0.04), albedo, metallic);
    let f = fresnel(vh, f0);
    let spec = d_ggx(nh, a) * g_schlick(nv, k) * g_schlick(nl, k) * f / (4.0 * nv * max(nl, 1e-4));
    let kd = (vec3<f32>(1.0) - f) * (1.0 - metallic);
    return (kd * albedo / PI + spec) * radiance * nl;
}

fn shade(i: VsOut, nrm: vec3<f32>) -> vec3<f32> {
    let lin = pow(i.color.rgb, vec3<f32>(2.2));
    let n = normalize(nrm);
    let v = normalize(g.cam_pos.xyz - i.world);
    let metallic = g.params.x;
    let rough = g.params.y;
    let key = normalize(g.light_dir.xyz);
    let fill = normalize(vec3<f32>(-key.x, -key.y, 0.3));
    var c = light(n, v, key, vec3<f32>(3.0), lin, metallic, rough);
    c = c + light(n, v, fill, vec3<f32>(0.8, 0.85, 1.0), lin, metallic, rough);
    let hemi = mix(vec3<f32>(0.10, 0.09, 0.08), vec3<f32>(0.35, 0.42, 0.55), n.z * 0.5 + 0.5);
    c = c + hemi * lin * g.params.w * (1.0 - metallic * 0.7);
    return c * g.params.z;
}

fn view_depth(world: vec3<f32>) -> f32 {
    return -(g.view * vec4<f32>(world, 1.0)).z;
}

// ---- opaque pass: linear HDR colour + (view normal, view depth) -----------------------------------

struct OpaqueOut {
    @location(0) color: vec4<f32>,
    @location(1) nd: vec4<f32>,
};

@fragment
fn fs_opaque(i: VsOut) -> OpaqueOut {
    var o: OpaqueOut;
    o.color = vec4<f32>(shade(i, i.normal), 1.0);
    let nv = normalize((g.view * vec4<f32>(normalize(i.normal), 0.0)).xyz);
    o.nd = vec4<f32>(nv, view_depth(i.world));
    return o;
}

// ---- weighted blended OIT (McGuire & Bavoil 2013) ---------------------------------------------------

struct OitOut {
    @location(0) accum: vec4<f32>,
    @location(1) reveal: f32,
};

@fragment
fn fs_oit(i: VsOut, @builtin(front_facing) front: bool) -> OitOut {
    var n = i.normal;
    if (!front) { n = -n; }
    let c = shade(i, n);
    let a = clamp(i.color.a * g.extra.z, 0.0, 0.999);
    let z = view_depth(i.world);
    let w = max(min(1.0, max(max(c.r, c.g), c.b) * a), a) * clamp(0.03 / (1e-5 + pow(z / 200.0, 4.0)), 1e-2, 3e3);
    var o: OitOut;
    o.accum = vec4<f32>(c * a, a) * w;
    o.reveal = a;
    return o;
}

// ---- fullscreen triangle ----------------------------------------------------------------------------

struct FsVs {
    @builtin(position) clip: vec4<f32>,
};

@vertex
fn vs_full(@builtin(vertex_index) vi: u32) -> FsVs {
    var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
    var o: FsVs;
    o.clip = vec4<f32>(p[vi], 0.0, 1.0);
    return o;
}

@group(1) @binding(0) var nd_tex: texture_2d<f32>;
@group(1) @binding(1) var hdr_tex: texture_2d<f32>;
@group(1) @binding(2) var accum_tex: texture_2d<f32>;
@group(1) @binding(3) var reveal_tex: texture_2d<f32>;
@group(1) @binding(4) var ao_tex: texture_2d<f32>;

fn view_pos(c: vec2<i32>, d: f32) -> vec3<f32> {
    let ndc = vec2<f32>((f32(c.x) + 0.5) / g.viewport.x * 2.0 - 1.0, 1.0 - (f32(c.y) + 0.5) / g.viewport.y * 2.0);
    return vec3<f32>(ndc.x * g.proj.z * d, ndc.y * g.proj.w * d, -d);
}

fn ign(p: vec2<f32>) -> f32 {
    return fract(52.9829189 * fract(dot(p, vec2<f32>(0.06711056, 0.00583715))));
}

const SLICES: i32 = 6;
const STEPS: i32 = 8;

// ---- GTAO (Jimenez et al. 2016), half resolution ----------------------------------------------------

@fragment
fn fs_gtao(@builtin(position) frag: vec4<f32>) -> @location(0) f32 {
    let full = vec2<i32>(i32(frag.x) * 2, i32(frag.y) * 2);
    let dims = vec2<i32>(i32(g.viewport.x), i32(g.viewport.y));
    if (full.x >= dims.x || full.y >= dims.y) { return 1.0; }
    let nd = textureLoad(nd_tex, full, 0);
    if (nd.w <= 0.0) { return 1.0; }
    let p = view_pos(full, nd.w);
    let n = normalize(nd.xyz);
    let v = normalize(-p);
    let radius = g.extra.x;
    // world radius -> pixels at this depth
    let px_per_unit = g.viewport.y / (2.0 * g.proj.w * nd.w);
    let r_px = clamp(radius * px_per_unit, 2.0, 200.0);
    var vis = 0.0;
    let noise = ign(frag.xy);
    for (var s = 0; s < SLICES; s = s + 1) {
        let phi = (f32(s) + noise) * PI / f32(SLICES);
        let dir = vec2<f32>(cos(phi), -sin(phi)); // screen y points down
        let dir3 = vec3<f32>(cos(phi), sin(phi), 0.0); // view-space direction
        var hc_pos = -1.0;
        var hc_neg = -1.0;
        for (var k = 1; k <= STEPS; k = k + 1) {
            let t = (f32(k) - 0.5 + 0.5 * noise) / f32(STEPS);
            let off = vec2<i32>(round(dir * (r_px * t * t + 1.0)));
            for (var side = 0; side < 2; side = side + 1) {
                let sgn = select(-1.0, 1.0, side == 0);
                let c = full + vec2<i32>(i32(sgn) * off.x, i32(sgn) * off.y);
                if (c.x < 0 || c.y < 0 || c.x >= dims.x || c.y >= dims.y) { continue; }
                let sd = textureLoad(nd_tex, c, 0).w;
                if (sd <= 0.0) { continue; }
                let sp = view_pos(c, sd);
                let delta = sp - p;
                let len = length(delta);
                if (len < 1e-5) { continue; }
                let hc = dot(delta / len, v);
                let fall = clamp((radius - len) / (0.4 * radius), 0.0, 1.0);
                let h = mix(-1.0, hc, fall);
                if (side == 0) { hc_pos = max(hc_pos, h); } else { hc_neg = max(hc_neg, h); }
            }
        }
        // slice plane basis
        let axis = normalize(cross(dir3, v));
        let ortho = dir3 - dot(dir3, v) * v;
        let proj_n = n - axis * dot(n, axis);
        let proj_len = length(proj_n);
        if (proj_len < 1e-4) { vis = vis + 1.0; continue; }
        let sign_n = select(-1.0, 1.0, dot(ortho, proj_n) >= 0.0);
        let cos_n = clamp(dot(proj_n, v) / proj_len, -1.0, 1.0);
        let nang = sign_n * acos(cos_n);
        var h1 = -acos(clamp(hc_neg, -1.0, 1.0));
        var h2 = acos(clamp(hc_pos, -1.0, 1.0));
        h1 = nang + max(h1 - nang, -PI * 0.5);
        h2 = nang + min(h2 - nang, PI * 0.5);
        let a1 = 0.25 * (-cos(2.0 * h1 - nang) + cos(nang) + 2.0 * h1 * sin(nang));
        let a2 = 0.25 * (-cos(2.0 * h2 - nang) + cos(nang) + 2.0 * h2 * sin(nang));
        // the unoccluded value of the slice integral, so that open surfaces come out as exactly 1
        let open_val = max(cos(nang) + nang * sin(nang), 1e-3);
        vis = vis + proj_len * min((a1 + a2) / open_val, 1.0) + (1.0 - proj_len);
    }
    return clamp(vis / f32(SLICES), 0.0, 1.0);
}

fn aces(x: vec3<f32>) -> vec3<f32> {
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), vec3<f32>(0.0), vec3<f32>(1.0));
}

// ---- composite: AO (bilateral upsample) + OIT resolve + tone mapping -------------------------------------

@fragment
fn fs_composite(@builtin(position) frag: vec4<f32>) -> @location(0) vec4<f32> {
    let c = vec2<i32>(i32(frag.x), i32(frag.y));
    var hdr = textureLoad(hdr_tex, c, 0).rgb;
    let nd = textureLoad(nd_tex, c, 0);
    if (nd.w > 0.0 && g.extra.w > 0.5) {
        let hc = vec2<i32>(c.x / 2, c.y / 2);
        var sum = 0.0;
        var wsum = 0.0;
        for (var dy = -1; dy <= 1; dy = dy + 1) {
            for (var dx = -1; dx <= 1; dx = dx + 1) {
                let q = hc + vec2<i32>(dx, dy);
                let fq = vec2<i32>(q.x * 2, q.y * 2);
                if (fq.x < 0 || fq.y < 0 || fq.x >= i32(g.viewport.x) || fq.y >= i32(g.viewport.y)) { continue; }
                let qd = textureLoad(nd_tex, fq, 0).w;
                let wgt = exp(-abs(qd - nd.w) / (0.02 * nd.w + 1e-4)) * select(1.0, 0.6, dx != 0 || dy != 0);
                sum = sum + wgt * textureLoad(ao_tex, q, 0).r;
                wsum = wsum + wgt;
            }
        }
        let ao = sum / max(wsum, 1e-5);
        // multi-bounce approximation keeps bright surfaces from going black
        let a = mix(1.0, ao, g.extra.y);
        hdr = hdr * a;
    }
    let acc = textureLoad(accum_tex, c, 0);
    let rev = textureLoad(reveal_tex, c, 0).r;
    if (acc.a > 1e-5) {
        let avg = acc.rgb / max(acc.a, 1e-5);
        hdr = avg * (1.0 - rev) + hdr * rev;
    }
    return vec4<f32>(aces(hdr), 1.0);
}

// ---- edges ------------------------------------------------------------------------------------------------

struct EdgeIn {
    @builtin(vertex_index) vi: u32,
    @location(0) p0: vec3<f32>,
    @location(1) p1: vec3<f32>,
    @location(2) id: u32,
};
struct EdgeOut {
    @builtin(position) clip: vec4<f32>,
};

@vertex
fn vs_edge(v: EdgeIn) -> EdgeOut {
    var ts = array<f32, 6>(0.0, 1.0, 1.0, 0.0, 1.0, 0.0);
    var sides = array<f32, 6>(-1.0, -1.0, 1.0, -1.0, 1.0, 1.0);
    let t = ts[v.vi];
    let side = sides[v.vi];
    var a = g.view_proj * vec4<f32>(v.p0, 1.0);
    var b = g.view_proj * vec4<f32>(v.p1, 1.0);
    var o: EdgeOut;
    let near_w = 0.01;
    if (a.w < near_w && b.w < near_w) {
        o.clip = vec4<f32>(2.0, 2.0, 2.0, 1.0);
        return o;
    }
    if (a.w < near_w) { a = mix(a, b, (near_w - a.w) / (b.w - a.w)); }
    if (b.w < near_w) { b = mix(b, a, (near_w - b.w) / (a.w - b.w)); }
    let half_vp = 0.5 * g.viewport.xy;
    let sa = a.xy / a.w * half_vp;
    let sb = b.xy / b.w * half_vp;
    var dir = sb - sa;
    let len = length(dir);
    if (len > 1e-6) { dir = dir / len; } else { dir = vec2<f32>(1.0, 0.0); }
    let nrm = vec2<f32>(-dir.y, dir.x);
    let half_w = 0.5 * g.viewport.z;
    var base = sa;
    var cp = a;
    var cap = -half_w;
    if (t > 0.5) { base = sb; cp = b; cap = half_w; }
    let sp = base + nrm * (side * half_w) + dir * cap;
    let ndc = sp / half_vp;
    o.clip = vec4<f32>(ndc * cp.w, cp.z - g.viewport.w * cp.w, cp.w);
    return o;
}

@fragment
fn fs_edge() -> @location(0) vec4<f32> {
    return vec4<f32>(0.01, 0.012, 0.016, 1.0);
}
