// PBR (Cook-Torrance GGX) mesh pass. Writes lit colour and the pick id.

struct Globals {
    view_proj: mat4x4<f32>,
    cam_pos: vec4<f32>,
    light_dir: vec4<f32>,
    viewport: vec4<f32>, // width, height, line width (px), edge depth bias
    params: vec4<f32>,   // metallic, roughness, exposure, ambient
    ids: vec4<u32>,      // hovered id, selected id, unused, unused
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
    @location(3) @interpolate(flat) id: u32,
};

@vertex
fn vs_main(v: VsIn) -> VsOut {
    var o: VsOut;
    o.clip = g.view_proj * vec4<f32>(v.pos, 1.0);
    o.world = v.pos;
    o.normal = v.normal;
    o.color = v.color;
    o.id = v.id;
    return o;
}

const PI: f32 = 3.14159265;

fn d_ggx(nh: f32, a: f32) -> f32 {
    let a2 = a * a;
    let d = nh * nh * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

fn g_schlick(nx: f32, k: f32) -> f32 {
    return nx / (nx * (1.0 - k) + k);
}

fn fresnel(vh: f32, f0: vec3<f32>) -> vec3<f32> {
    return f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - vh, 5.0);
}

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

fn aces(x: vec3<f32>) -> vec3<f32> {
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), vec3<f32>(0.0), vec3<f32>(1.0));
}

struct FsOut {
    @location(0) color: vec4<f32>,
    @location(1) id: u32,
};

@fragment
fn fs_main(i: VsOut) -> FsOut {
    var albedo = i.color.rgb;
    if (i.id == g.ids.x) { albedo = mix(albedo, vec3<f32>(1.0, 0.85, 0.2), 0.55); }
    if (i.id == g.ids.y) { albedo = mix(albedo, vec3<f32>(1.0, 0.45, 0.1), 0.7); }
    // sRGB-encoded vertex colours -> linear
    let lin = pow(albedo, vec3<f32>(2.2));
    let n = normalize(i.normal);
    let v = normalize(g.cam_pos.xyz - i.world);
    let metallic = g.params.x;
    let rough = g.params.y;

    let key = normalize(g.light_dir.xyz);
    let fill = normalize(vec3<f32>(-key.x, -key.y, 0.3));
    var c = light(n, v, key, vec3<f32>(3.0), lin, metallic, rough);
    c = c + light(n, v, fill, vec3<f32>(0.8, 0.85, 1.0), lin, metallic, rough);
    // hemisphere ambient (z-up world)
    let hemi = mix(vec3<f32>(0.10, 0.09, 0.08), vec3<f32>(0.35, 0.42, 0.55), n.z * 0.5 + 0.5);
    c = c + hemi * lin * g.params.w * (1.0 - metallic * 0.7);
    c = aces(c * g.params.z);

    var o: FsOut;
    o.color = vec4<f32>(c, 1.0); // surface/offscreen formats are *Srgb: hardware encodes
    o.id = i.id;
    return o;
}
