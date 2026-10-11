// B-rep edges as screen-space quads (constant pixel width), drawn from
// topological edge polylines, with a depth bias so they win against faces.

struct Globals {
    view_proj: mat4x4<f32>,
    cam_pos: vec4<f32>,
    light_dir: vec4<f32>,
    viewport: vec4<f32>,
    params: vec4<f32>,
    ids: vec4<u32>,
    sel: array<vec4<u32>, 16>,
};

@group(0) @binding(0) var<uniform> g: Globals;

struct VsIn {
    @builtin(vertex_index) vi: u32,
    @location(0) p0: vec3<f32>,
    @location(1) p1: vec3<f32>,
    @location(2) id: u32,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) @interpolate(flat) id: u32,
    // 0 plain, 1 hovered, 2 selected
    @location(1) @interpolate(flat) state: u32,
};

fn edge_state(id: u32) -> u32 {
    if (id == 0u || id == 0xFFFFFFFFu) { return 0u; }
    if (id == g.ids.y) { return 2u; }
    for (var k = 0u; k < g.ids.z; k = k + 1u) {
        if (g.sel[k / 4u][k % 4u] == id) { return 2u; }
    }
    if (id == g.ids.x) { return 1u; }
    return 0u;
}

@vertex
fn vs_main(v: VsIn) -> VsOut {
    var ts = array<f32, 6>(0.0, 1.0, 1.0, 0.0, 1.0, 0.0);
    var sides = array<f32, 6>(-1.0, -1.0, 1.0, -1.0, 1.0, 1.0);
    let t = ts[v.vi];
    let side = sides[v.vi];

    var a = g.view_proj * vec4<f32>(v.p0, 1.0);
    var b = g.view_proj * vec4<f32>(v.p1, 1.0);
    var o: VsOut;
    o.id = v.id;
    o.state = edge_state(v.id);
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
    var half_w = 0.5 * g.viewport.z;
    if (o.state != 0u) { half_w = half_w * 2.4; }

    var base = sa;
    var cp = a;
    var cap = -half_w;
    if (t > 0.5) { base = sb; cp = b; cap = half_w; }
    let sp = base + nrm * (side * half_w) + dir * cap;
    let ndc = sp / half_vp;
    o.clip = vec4<f32>(ndc * cp.w, cp.z - g.viewport.w * cp.w, cp.w);
    return o;
}

struct FsOut {
    @location(0) color: vec4<f32>,
    @location(1) id: u32,
};

@fragment
fn fs_main(i: VsOut) -> FsOut {
    var o: FsOut;
    o.color = vec4<f32>(0.01, 0.012, 0.016, 1.0);
    if (i.state == 1u) { o.color = vec4<f32>(1.0, 0.55, 0.0, 1.0); }
    if (i.state == 2u) { o.color = vec4<f32>(0.0, 0.45, 0.95, 1.0); }
    o.id = i.id;
    return o;
}
