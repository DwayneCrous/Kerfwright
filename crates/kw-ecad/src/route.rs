//! A multilayer grid router: A* on an octilinear grid with via and bend costs, obstacle maps inflated by
//! the clearance rules (with a margin that covers the chord error of diagonal steps), nets routed shortest
//! first as minimum spanning trees of their pads, and rip-up-and-retry for nets that fail.

use crate::board::{Board, Track, Via};
use crate::shape::{copper_items, distance, Item, Shape};
use crate::P2;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Debug, Clone)]
pub struct RouterParams {
    /// Grid pitch in mm.
    pub pitch: f64,
    /// Cost of a via in units of one grid step.
    pub via_cost: f64,
    pub bend_cost: f64,
    /// Rip-up-and-retry rounds for nets that did not route.
    pub ripup_rounds: usize,
    /// Extra distance added to every inflation (mm).
    pub margin: f64,
}

impl Default for RouterParams {
    fn default() -> Self {
        RouterParams {
            pitch: 0.1,
            via_cost: 12.0,
            bend_cost: 0.4,
            ripup_rounds: 8,
            margin: 0.012,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RouteReport {
    pub nets_total: usize,
    pub nets_routed: usize,
    pub failed: Vec<usize>,
    pub tracks: usize,
    pub vias: usize,
}

impl RouteReport {
    pub fn completion(&self) -> f64 {
        if self.nets_total == 0 {
            1.0
        } else {
            self.nets_routed as f64 / self.nets_total as f64
        }
    }
}

const FREE: u32 = 0;
const MULTI: u32 = u32::MAX;

struct Grid {
    nx: usize,
    ny: usize,
    layers: usize,
    pitch: f64,
    /// Obstacle owner for track centres, per layer.
    track: Vec<u32>,
    /// Obstacle owner for via centres, per layer.
    via: Vec<u32>,
    /// Cells holding copper of the net being routed (targets/sources).
    tree: Vec<bool>,
}

impl Grid {
    fn idx(&self, l: usize, x: usize, y: usize) -> usize {
        (l * self.ny + y) * self.nx + x
    }
    fn xy(&self, i: usize) -> (usize, usize, usize) {
        let x = i % self.nx;
        let y = (i / self.nx) % self.ny;
        (i / (self.nx * self.ny), x, y)
    }
    fn pos(&self, x: usize, y: usize) -> P2 {
        [x as f64 * self.pitch, y as f64 * self.pitch]
    }
    fn cell_of(&self, p: P2) -> (usize, usize) {
        (
            ((p[0] / self.pitch).round().max(0.0) as usize).min(self.nx - 1),
            ((p[1] / self.pitch).round().max(0.0) as usize).min(self.ny - 1),
        )
    }
    /// Forbid via centres closer than `radius` to `c` on every layer, for every net (hole-to-hole spacing).
    fn block_via_disc(&mut self, c: P2, radius: f64) {
        let x0 = (((c[0] - radius) / self.pitch).floor().max(0.0)) as usize;
        let y0 = (((c[1] - radius) / self.pitch).floor().max(0.0)) as usize;
        let x1 = ((((c[0] + radius) / self.pitch).ceil()) as usize).min(self.nx - 1);
        let y1 = ((((c[1] + radius) / self.pitch).ceil()) as usize).min(self.ny - 1);
        for y in y0..=y1 {
            for x in x0..=x1 {
                let p = self.pos(x, y);
                if (p[0] - c[0]).hypot(p[1] - c[1]) < radius {
                    for l in 0..self.layers {
                        let i = self.idx(l, x, y);
                        self.via[i] = MULTI;
                    }
                }
            }
        }
    }

    fn stamp(
        &mut self,
        shape: &Shape,
        layer: Option<usize>,
        net: Option<usize>,
        r_track: f64,
        r_via: f64,
    ) {
        let b = shape.bounds();
        let reach = r_track.max(r_via);
        let x0 = (((b[0] - reach) / self.pitch).floor().max(0.0)) as usize;
        let y0 = (((b[1] - reach) / self.pitch).floor().max(0.0)) as usize;
        let x1 = ((((b[2] + reach) / self.pitch).ceil()) as usize).min(self.nx - 1);
        let y1 = ((((b[3] + reach) / self.pitch).ceil()) as usize).min(self.ny - 1);
        let owner = net.map_or(MULTI - 1, |n| n as u32 + 1);
        let layers: Vec<usize> = match layer {
            Some(l) => vec![l],
            None => (0..self.layers).collect(),
        };
        for y in y0..=y1 {
            for x in x0..=x1 {
                let p = self.pos(x, y);
                let d = distance(&Shape::Cap { a: p, b: p, r: 0.0 }, shape);
                for &l in &layers {
                    let i = self.idx(l, x, y);
                    if d < r_track {
                        merge(&mut self.track[i], owner);
                    }
                    if d < r_via {
                        merge(&mut self.via[i], owner);
                    }
                }
            }
        }
    }
}

fn merge(slot: &mut u32, owner: u32) {
    *slot = match *slot {
        FREE => owner,
        o if o == owner => o,
        _ => MULTI,
    };
}

#[derive(Clone, Copy, PartialEq)]
struct Node {
    f: f64,
    i: usize,
}
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

pub struct Router {
    pub board: Board,
    pub params: RouterParams,
    grid: Grid,
}

const DIRS: [(i32, i32); 8] = [
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
];

impl Router {
    pub fn new(board: Board, params: RouterParams) -> Router {
        let nx = (board.width / params.pitch).floor() as usize + 1;
        let ny = (board.height / params.pitch).floor() as usize + 1;
        let n = nx * ny * board.layers;
        let grid = Grid {
            nx,
            ny,
            layers: board.layers,
            pitch: params.pitch,
            track: vec![FREE; n],
            via: vec![FREE; n],
            tree: vec![false; n],
        };
        let mut r = Router {
            board,
            params,
            grid,
        };
        r.rebuild_maps();
        r
    }

    /// Distances from the *surface* of foreign copper that a track centre and a via centre must keep.
    fn radii(&self) -> (f64, f64) {
        let rules = &self.board.rules;
        let m = self.params.margin;
        (
            rules.clearance + rules.track_width / 2.0 + m,
            rules.clearance + rules.via_dia / 2.0 + m,
        )
    }

    /// Recompute the obstacle maps from the board (pads, tracks, vias, board edge).
    fn rebuild_maps(&mut self) {
        let items = copper_items(&self.board);
        let (rt, rv) = self.radii();
        let g = &mut self.grid;
        g.track.fill(FREE);
        g.via.fill(FREE);
        let edge_t = self.board.rules.edge_clearance
            + self.board.rules.track_width / 2.0
            + self.params.margin;
        let edge_v =
            self.board.rules.edge_clearance + self.board.rules.via_dia / 2.0 + self.params.margin;
        // board edge: cells too close are blocked for everybody
        for l in 0..g.layers {
            for y in 0..g.ny {
                for x in 0..g.nx {
                    let p = g.pos(x, y);
                    let d = p[0]
                        .min(p[1])
                        .min(self.board.width - p[0])
                        .min(self.board.height - p[1]);
                    let i = g.idx(l, x, y);
                    if d < edge_t {
                        g.track[i] = MULTI;
                    }
                    if d < edge_v {
                        g.via[i] = MULTI;
                    }
                }
            }
        }
        for it in &items {
            g.stamp(&it.shape, it.layer, it.net, rt, rv);
        }
        // mounting holes
        for (c, d) in &self.board.holes {
            let s = Shape::Cap {
                a: *c,
                b: *c,
                r: d / 2.0,
            };
            g.stamp(&s, None, None, rt, rv);
        }
        // holes keep vias at a distance (hole-to-hole spacing), whatever the net
        let hole_gap = 0.25 + self.params.margin;
        for p in self.board.pads() {
            if let Some(d) = p.drill {
                g.block_via_disc(p.pos, d / 2.0 + hole_gap + self.board.rules.via_drill / 2.0);
            }
        }
        for v in &self.board.vias {
            g.block_via_disc(v.pos, v.drill + hole_gap);
        }
    }

    fn pad_cells(&self, net: usize) -> Vec<(P2, Option<usize>, usize)> {
        // position, layer (None = all), pad index into board.pads()
        self.board
            .pads()
            .iter()
            .enumerate()
            .filter(|(_, p)| p.net == Some(net))
            .map(|(i, p)| (p.pos, p.layer, i))
            .collect()
    }

    fn passable(&self, net: usize, i: usize, via: bool) -> bool {
        let o = if via {
            self.grid.via[i]
        } else {
            self.grid.track[i]
        };
        o == FREE || o == net as u32 + 1
    }

    /// Route every net; returns what could not be routed.
    pub fn route_all(&mut self) -> RouteReport {
        let nets: Vec<usize> = (0..self.board.nets.len())
            .filter(|n| self.pad_cells(*n).len() >= 2)
            .collect();
        let mut order = nets.clone();
        // shortest (half perimeter) first
        order.sort_by(|a, b| self.hpwl(*a).partial_cmp(&self.hpwl(*b)).unwrap());
        let mut failed: Vec<usize> = Vec::new();
        for n in &order {
            if self.board.tracks.iter().any(|t| t.net == *n) {
                continue;
            }
            if !self.route_net(*n) {
                failed.push(*n);
            }
        }
        // rip up the nets that stand in the way of the failed ones, route the failed ones first
        for _round in 0..self.params.ripup_rounds {
            if failed.is_empty() {
                break;
            }
            let todo = std::mem::take(&mut failed);
            for n in todo {
                let blockers = self.blockers(n);
                let saved: Vec<(usize, Vec<Track>, Vec<Via>)> = blockers
                    .iter()
                    .map(|b| {
                        (
                            *b,
                            self.board
                                .tracks
                                .iter()
                                .filter(|t| t.net == *b)
                                .cloned()
                                .collect(),
                            self.board
                                .vias
                                .iter()
                                .filter(|v| v.net == *b)
                                .cloned()
                                .collect(),
                        )
                    })
                    .collect();
                for b in &blockers {
                    self.rip(*b);
                }
                let ok = self.route_net(n);
                let mut all_back = true;
                for (b, _, _) in &saved {
                    if !self.route_net(*b) {
                        all_back = false;
                    }
                }
                if !ok || !all_back {
                    // undo: remove everything of these nets and restore the previous state
                    self.rip(n);
                    for (b, t, v) in &saved {
                        self.rip(*b);
                        self.board.tracks.extend(t.iter().cloned());
                        self.board.vias.extend(v.iter().cloned());
                    }
                    self.rebuild_maps();
                    failed.push(n);
                }
            }
        }
        let routed = nets.len() - failed.len();
        RouteReport {
            nets_total: nets.len(),
            nets_routed: routed,
            failed,
            tracks: self.board.tracks.len(),
            vias: self.board.vias.len(),
        }
    }

    fn hpwl(&self, net: usize) -> f64 {
        let ps = self.pad_cells(net);
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for (p, _, _) in &ps {
            x0 = x0.min(p[0]);
            x1 = x1.max(p[0]);
            y0 = y0.min(p[1]);
            y1 = y1.max(p[1]);
        }
        (x1 - x0) + (y1 - y0)
    }

    fn rip(&mut self, net: usize) {
        self.board.tracks.retain(|t| t.net != net);
        self.board.vias.retain(|v| v.net != net);
        self.rebuild_maps();
    }

    /// Nets whose copper lies on the straight corridor between the pads of `net` (candidates to rip).
    fn blockers(&self, net: usize) -> Vec<usize> {
        let pads = self.pad_cells(net);
        let mut found: Vec<usize> = Vec::new();
        let items: Vec<Item> = copper_items(&self.board);
        for w in pads.windows(2) {
            let seg = Shape::Cap {
                a: w[0].0,
                b: w[1].0,
                r: 1.5,
            };
            for it in &items {
                if let Some(n) = it.net {
                    if n != net
                        && !found.contains(&n)
                        && matches!(
                            it.kind,
                            crate::shape::ItemKind::Track | crate::shape::ItemKind::Via
                        )
                        && distance(&seg, &it.shape) == 0.0
                    {
                        found.push(n);
                    }
                }
            }
        }
        found.truncate(6);
        found
    }

    /// Connect all pads of `net` (nearest-first spanning tree). Returns false if some pad stays unconnected;
    /// the tracks already laid for the net are kept.
    pub fn route_net(&mut self, net: usize) -> bool {
        let pads = self.pad_cells(net);
        if pads.len() < 2 {
            return true;
        }
        // grow the tree from the first pad; always attach the unconnected pad nearest to the tree
        let mut connected: Vec<usize> = vec![0];
        let mut todo: Vec<usize> = (1..pads.len()).collect();
        self.grid.tree.fill(false);
        self.mark_pad(&pads[0]);
        // existing copper of the net (partial earlier attempt) joins the tree
        let existing: Vec<Track> = self
            .board
            .tracks
            .iter()
            .filter(|t| t.net == net)
            .cloned()
            .collect();
        for t in &existing {
            self.mark_segment(t);
        }
        let mut ok = true;
        while !todo.is_empty() {
            let (bi, _) = todo
                .iter()
                .enumerate()
                .map(|(k, p)| {
                    let d = connected
                        .iter()
                        .map(|c| {
                            (pads[*p].0[0] - pads[*c].0[0]).hypot(pads[*p].0[1] - pads[*c].0[1])
                        })
                        .fold(f64::MAX, f64::min);
                    (k, d)
                })
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap();
            let pi = todo.remove(bi);
            let target_hint = connected
                .iter()
                .map(|c| pads[*c].0)
                .min_by(|a, b| {
                    let da = (a[0] - pads[pi].0[0]).hypot(a[1] - pads[pi].0[1]);
                    let db = (b[0] - pads[pi].0[0]).hypot(b[1] - pads[pi].0[1]);
                    da.partial_cmp(&db).unwrap()
                })
                .unwrap();
            if self.connect(net, &pads[pi], target_hint) {
                connected.push(pi);
                self.mark_pad(&pads[pi]);
            } else {
                ok = false;
            }
        }
        ok
    }

    fn mark_pad(&mut self, p: &(P2, Option<usize>, usize)) {
        let (x, y) = self.grid.cell_of(p.0);
        match p.1 {
            Some(l) => {
                let i = self.grid.idx(l, x, y);
                self.grid.tree[i] = true;
            }
            None => {
                for l in 0..self.grid.layers {
                    let i = self.grid.idx(l, x, y);
                    self.grid.tree[i] = true;
                }
            }
        }
    }

    fn mark_segment(&mut self, t: &Track) {
        let (x0, y0) = self.grid.cell_of(t.a);
        let (x1, y1) = self.grid.cell_of(t.b);
        let n = (x1 as i64 - x0 as i64)
            .abs()
            .max((y1 as i64 - y0 as i64).abs());
        for k in 0..=n {
            let x = x0 as i64 + (x1 as i64 - x0 as i64) * k / n.max(1);
            let y = y0 as i64 + (y1 as i64 - y0 as i64) * k / n.max(1);
            let i = self.grid.idx(t.layer, x as usize, y as usize);
            self.grid.tree[i] = true;
        }
        for v in self.board.vias.iter().filter(|v| v.net == t.net) {
            let (x, y) = self.grid.cell_of(v.pos);
            for l in 0..self.grid.layers {
                let i = self.grid.idx(l, x, y);
                self.grid.tree[i] = true;
            }
        }
    }

    fn connect(&mut self, net: usize, from: &(P2, Option<usize>, usize), hint: P2) -> bool {
        let g = &self.grid;
        let (sx, sy) = g.cell_of(from.0);
        let starts: Vec<usize> = match from.1 {
            Some(l) => vec![g.idx(l, sx, sy)],
            None => (0..g.layers).map(|l| g.idx(l, sx, sy)).collect(),
        };
        let (hx, hy) = g.cell_of(hint);
        let nxy = g.nx * g.ny;
        let total = nxy * g.layers;
        let mut dist = vec![f64::INFINITY; total];
        let mut prev = vec![usize::MAX; total];
        let mut dir = vec![8u8; total];
        let mut heap = BinaryHeap::new();
        let h = |i: usize, gr: &Grid| -> f64 {
            let (_, x, y) = gr.xy(i);
            let (dx, dy) = ((x as f64 - hx as f64).abs(), (y as f64 - hy as f64).abs());
            dx.max(dy) + (std::f64::consts::SQRT_2 - 1.0) * dx.min(dy)
        };
        // a pad's own cell is always allowed as a start
        for &s in &starts {
            dist[s] = 0.0;
            heap.push(Node { f: h(s, g), i: s });
        }
        let mut goal = usize::MAX;
        let mut expansions = 0usize;
        while let Some(Node { i, .. }) = heap.pop() {
            let d0 = dist[i];
            if self.grid.tree[i] && !starts.contains(&i) {
                goal = i;
                break;
            }
            expansions += 1;
            if expansions > 4_000_000 {
                break;
            }
            let (l, x, y) = self.grid.xy(i);
            for (k, (dx, dy)) in DIRS.iter().enumerate() {
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if nx < 0 || ny < 0 || nx >= self.grid.nx as i32 || ny >= self.grid.ny as i32 {
                    continue;
                }
                let j = self.grid.idx(l, nx as usize, ny as usize);
                if !self.passable(net, j, false) && !self.grid.tree[j] {
                    continue;
                }
                // a diagonal step also needs both corner cells' neighbours free to avoid cutting a corner
                let step = if *dx != 0 && *dy != 0 {
                    std::f64::consts::SQRT_2
                } else {
                    1.0
                };
                let bend = if dir[i] < 8 && dir[i] as usize != k {
                    self.params.bend_cost
                } else {
                    0.0
                };
                let nd = d0 + step + bend;
                if nd < dist[j] {
                    dist[j] = nd;
                    prev[j] = i;
                    dir[j] = k as u8;
                    heap.push(Node {
                        f: nd + h(j, &self.grid),
                        i: j,
                    });
                }
            }
            // layer change
            // a via is drilled through every layer: its keep-out must be clear on all of them
            // and two vias of one path must keep hole-to-hole spacing: no layer change within six steps of the last
            let recent_via = {
                let mut c = i;
                let mut found = false;
                for _ in 0..6 {
                    if dir[c] == 9 {
                        found = true;
                        break;
                    }
                    if prev[c] == usize::MAX {
                        break;
                    }
                    c = prev[c];
                }
                found
            };
            if self.grid.layers > 1
                && !recent_via
                && (0..self.grid.layers).all(|m| self.passable(net, self.grid.idx(m, x, y), true))
            {
                for l2 in 0..self.grid.layers {
                    if l2 == l {
                        continue;
                    }
                    let j = self.grid.idx(l2, x, y);
                    let nd =
                        d0 + self.params.via_cost * (l2 as i64 - l as i64).unsigned_abs() as f64;
                    if nd < dist[j] {
                        dist[j] = nd;
                        prev[j] = i;
                        dir[j] = 9;
                        heap.push(Node {
                            f: nd + h(j, &self.grid),
                            i: j,
                        });
                    }
                }
            }
        }
        if goal == usize::MAX {
            return false;
        }
        // walk back
        let mut path = vec![goal];
        while prev[*path.last().unwrap()] != usize::MAX {
            path.push(prev[*path.last().unwrap()]);
        }
        path.reverse();
        self.lay_path(net, from, &path);
        true
    }

    fn lay_path(&mut self, net: usize, from: &(P2, Option<usize>, usize), path: &[usize]) {
        let w = self.board.rules.track_width;
        // split into per-layer runs, with vias where the layer changes
        let pts: Vec<(usize, usize, usize)> = path.iter().map(|i| self.grid.xy(*i)).collect();
        let mut run_start = 0;
        let mut new_tracks: Vec<Track> = Vec::new();
        let mut new_vias: Vec<Via> = Vec::new();
        let mut k = 1;
        // helper to emit a straight-run compressed polyline of path[a..=b] on its layer
        let emit = |a: usize, b: usize, new_tracks: &mut Vec<Track>, grid: &Grid| {
            if a >= b {
                return;
            }
            let layer = pts[a].0;
            let mut seg_start = a;
            for m in a + 1..=b {
                let end_of_run = m == b;
                let turns = !end_of_run && {
                    let d1 = (
                        pts[m].1 as i64 - pts[m - 1].1 as i64,
                        pts[m].2 as i64 - pts[m - 1].2 as i64,
                    );
                    let d2 = (
                        pts[m + 1].1 as i64 - pts[m].1 as i64,
                        pts[m + 1].2 as i64 - pts[m].2 as i64,
                    );
                    d1 != d2
                };
                if end_of_run || turns {
                    new_tracks.push(Track {
                        net,
                        layer,
                        width: w,
                        a: grid.pos(pts[seg_start].1, pts[seg_start].2),
                        b: grid.pos(pts[m].1, pts[m].2),
                    });
                    seg_start = m;
                }
            }
        };
        while k < pts.len() {
            if pts[k].0 != pts[k - 1].0 {
                emit(run_start, k - 1, &mut new_tracks, &self.grid);
                let p = self.grid.pos(pts[k].1, pts[k].2);
                if new_vias
                    .last()
                    .is_none_or(|v: &Via| (v.pos[0] - p[0]).hypot(v.pos[1] - p[1]) > 1e-9)
                {
                    new_vias.push(Via {
                        net,
                        pos: p,
                        dia: self.board.rules.via_dia,
                        drill: self.board.rules.via_drill,
                    });
                }
                run_start = k;
            }
            k += 1;
        }
        emit(run_start, pts.len() - 1, &mut new_tracks, &self.grid);
        // stubs from the pad centre to the first grid node
        let first = self.grid.pos(pts[0].1, pts[0].2);
        if (first[0] - from.0[0]).hypot(first[1] - from.0[1]) > 1e-9 {
            let l = pts[0].0;
            new_tracks.insert(
                0,
                Track {
                    net,
                    layer: l,
                    width: w,
                    a: from.0,
                    b: first,
                },
            );
        }
        for t in &new_tracks {
            self.mark_segment(t);
        }
        // the new copper becomes an obstacle for the nets that follow
        let (rt, rv) = self.radii();
        for t in &new_tracks {
            let s = Shape::Cap {
                a: t.a,
                b: t.b,
                r: t.width / 2.0,
            };
            self.grid.stamp(&s, Some(t.layer), Some(net), rt, rv);
        }
        let hole_gap = 0.25 + self.params.margin;
        for v in &new_vias {
            let s = Shape::Cap {
                a: v.pos,
                b: v.pos,
                r: v.dia / 2.0,
            };
            self.grid.stamp(&s, None, Some(net), rt, rv);
            self.grid.block_via_disc(v.pos, v.drill + hole_gap);
        }
        self.board.tracks.extend(new_tracks);
        self.board.vias.extend(new_vias);
    }
}
