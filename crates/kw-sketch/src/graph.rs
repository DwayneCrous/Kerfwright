//! Structural analysis of a compiled [`System`]: connected components and the
//! Dulmage–Mendelsohn decomposition (maximum matching + strongly connected
//! components) into a block-triangular solve order.

use crate::system::System;

pub const NONE: u32 = u32::MAX;

/// A connected set of equations and the unknown variables they touch.
#[derive(Clone, Debug, Default)]
pub struct Component {
    /// Global row ids.
    pub rows: Vec<u32>,
    /// Global ids of the unknown variables.
    pub cols: Vec<u32>,
}

/// Connected components of the row/unknown-variable graph.
pub fn components(sys: &System) -> Vec<Component> {
    let nr = sys.n_rows;
    let nv = sys.n_vars;
    // union-find over rows [0,nr) and variables [nr, nr+nv)
    let mut parent: Vec<u32> = (0..(nr + nv) as u32).collect();
    fn find(p: &mut [u32], mut x: u32) -> u32 {
        while p[x as usize] != x {
            p[x as usize] = p[p[x as usize] as usize];
            x = p[x as usize];
        }
        x
    }
    let mut pat = Vec::new();
    let mut touched = vec![false; nv];
    for r in 0..nr {
        sys.row_pattern(r, &mut pat);
        for &v in &pat {
            touched[v as usize] = true;
            let (a, b) = (
                find(&mut parent, r as u32),
                find(&mut parent, (nr as u32) + v),
            );
            if a != b {
                parent[a as usize] = b;
            }
        }
    }
    let mut index: Vec<u32> = vec![NONE; nr + nv];
    let mut comps: Vec<Component> = Vec::new();
    for r in 0..nr {
        let root = find(&mut parent, r as u32) as usize;
        if index[root] == NONE {
            index[root] = comps.len() as u32;
            comps.push(Component::default());
        }
        comps[index[root] as usize].rows.push(r as u32);
    }
    for (v, &touch) in touched.iter().enumerate() {
        if touch {
            let root = find(&mut parent, (nr + v) as u32) as usize;
            comps[index[root] as usize].cols.push(v as u32);
        }
    }
    comps
}

/// One diagonal block of the block-triangular form.
#[derive(Clone, Debug, Default)]
pub struct Block {
    /// Global rows solved by this block.
    pub rows: Vec<u32>,
    /// Global unknown variables it determines.
    pub cols: Vec<u32>,
    /// More rows than columns: least-squares block from the over-determined
    /// part of the decomposition.
    pub over: bool,
    /// Unique residual indices touched by the rows.
    pub res_list: Vec<u32>,
    /// For each entry of `res_list`: `(row index within this block, equation index)`.
    pub res_rows: Vec<Vec<(u16, u8)>>,
}

/// Build a block (with residual bookkeeping) from global rows and columns.
pub fn make_block(sys: &System, rows: Vec<u32>, cols: Vec<u32>, over: bool) -> Block {
    let mut res_list: Vec<u32> = Vec::new();
    let mut res_rows: Vec<Vec<(u16, u8)>> = Vec::new();
    for (bi, &gr) in rows.iter().enumerate() {
        let ri = sys.row_res[gr as usize];
        let pos = match res_list.iter().position(|&x| x == ri) {
            Some(p) => p,
            None => {
                res_list.push(ri);
                res_rows.push(Vec::new());
                res_list.len() - 1
            }
        };
        res_rows[pos].push((bi as u16, (gr as usize - sys.row_start[ri as usize]) as u8));
    }
    Block {
        rows,
        cols,
        over,
        res_list,
        res_rows,
    }
}

/// Block-triangular decomposition of one component.
#[derive(Clone, Debug, Default)]
pub struct Decomp {
    /// Blocks in dependency order (a block only depends on earlier blocks).
    pub blocks: Vec<Block>,
    /// For each block, the earlier blocks it reads.
    pub deps: Vec<Vec<u32>>,
    /// Unmatched unknowns: held at their current value (free parameters).
    pub free_cols: Vec<u32>,
    /// Dependency level of each block (parallel scheduling).
    pub level: Vec<u32>,
    /// Maximum matching size (rows matched).
    pub matched: usize,
}

struct Csr {
    ptr: Vec<u32>,
    adj: Vec<u32>,
}

fn csr_from(lists: &[Vec<u32>]) -> Csr {
    let mut ptr = Vec::with_capacity(lists.len() + 1);
    let mut adj = Vec::new();
    ptr.push(0);
    for l in lists {
        adj.extend_from_slice(l);
        ptr.push(adj.len() as u32);
    }
    Csr { ptr, adj }
}

/// Hopcroft–Karp phases: augment `(mr, mc)` to a maximum matching of `g`.
fn hk_augment(nrows: usize, g: &Csr, mr: &mut [u32], mc: &mut [u32]) {
    let mut dist = vec![0u32; nrows];
    let mut queue: Vec<u32> = Vec::with_capacity(nrows);
    let mut it = vec![0u32; nrows];
    let mut stack: Vec<u32> = Vec::new();
    loop {
        // BFS layers from free rows
        queue.clear();
        for r in 0..nrows {
            if mr[r] == NONE {
                dist[r] = 0;
                queue.push(r as u32);
            } else {
                dist[r] = NONE;
            }
        }
        let mut found = false;
        let mut head = 0;
        while head < queue.len() {
            let r = queue[head] as usize;
            head += 1;
            for k in g.ptr[r]..g.ptr[r + 1] {
                let c = g.adj[k as usize] as usize;
                let r2 = mc[c];
                if r2 == NONE {
                    found = true;
                } else if dist[r2 as usize] == NONE {
                    dist[r2 as usize] = dist[r] + 1;
                    queue.push(r2);
                }
            }
        }
        if !found {
            break;
        }
        // DFS augmenting along layers (iterative)
        it[..nrows].copy_from_slice(&g.ptr[..nrows]);
        for root in 0..nrows {
            if mr[root] != NONE {
                continue;
            }
            stack.clear();
            stack.push(root as u32);
            while let Some(&top) = stack.last() {
                let r = top as usize;
                if it[r] >= g.ptr[r + 1] {
                    dist[r] = NONE; // dead end
                    stack.pop();
                    continue;
                }
                let c = g.adj[it[r] as usize] as usize;
                it[r] += 1;
                let r2 = mc[c];
                if r2 == NONE {
                    let mut col = c as u32;
                    while let Some(rr) = stack.pop() {
                        let prev = mr[rr as usize];
                        mr[rr as usize] = col;
                        mc[col as usize] = rr;
                        col = prev;
                    }
                    break;
                } else if dist[r2 as usize] == dist[r] + 1 {
                    stack.push(r2);
                }
            }
        }
    }
}

/// Maximum matching of `full`, preferring the strong edges of `strong`
/// (candidates whose Jacobian entry is not negligible): a global greedy pass
/// takes the strongest edges first, then augmenting paths on `strong` and
/// finally on `full` restore maximum cardinality.
fn max_matching(
    nrows: usize,
    ncols: usize,
    strong: &Csr,
    strong_w: &[f64],
    full: &Csr,
) -> (Vec<u32>, Vec<u32>) {
    let mut mr = vec![NONE; nrows];
    let mut mc = vec![NONE; ncols];
    let mut edges: Vec<(f64, u32, u32)> = Vec::with_capacity(strong.adj.len());
    for r in 0..nrows {
        for k in strong.ptr[r]..strong.ptr[r + 1] {
            edges.push((strong_w[k as usize], r as u32, strong.adj[k as usize]));
        }
    }
    edges.sort_by(|a, b| {
        b.0.partial_cmp(&a.0)
            .unwrap()
            .then(a.1.cmp(&b.1))
            .then(a.2.cmp(&b.2))
    });
    for (_, r, c) in edges {
        if mr[r as usize] == NONE && mc[c as usize] == NONE {
            mr[r as usize] = c;
            mc[c as usize] = r;
        }
    }
    hk_augment(nrows, strong, &mut mr, &mut mc);
    hk_augment(nrows, full, &mut mr, &mut mc);
    (mr, mc)
}

/// Iterative Tarjan SCC. `adj` lists dependencies of each node; returns SCCs in
/// dependency-first order (every SCC appears after all SCCs it depends on).
fn tarjan(n: usize, adj: &Csr) -> Vec<Vec<u32>> {
    let mut index = vec![NONE; n];
    let mut low = vec![0u32; n];
    let mut on = vec![false; n];
    let mut st: Vec<u32> = Vec::new();
    let mut out: Vec<Vec<u32>> = Vec::new();
    let mut next = 0u32;
    // explicit DFS stack of (node, next edge position)
    let mut work: Vec<(u32, u32)> = Vec::new();
    for s in 0..n {
        if index[s] != NONE {
            continue;
        }
        work.push((s as u32, adj.ptr[s]));
        index[s] = next;
        low[s] = next;
        next += 1;
        st.push(s as u32);
        on[s] = true;
        while let Some(&(v, e)) = work.last() {
            let vu = v as usize;
            if e < adj.ptr[vu + 1] {
                work.last_mut().unwrap().1 += 1;
                let w = adj.adj[e as usize] as usize;
                if index[w] == NONE {
                    index[w] = next;
                    low[w] = next;
                    next += 1;
                    st.push(w as u32);
                    on[w] = true;
                    work.push((w as u32, adj.ptr[w]));
                } else if on[w] {
                    low[vu] = low[vu].min(index[w]);
                }
            } else {
                work.pop();
                if let Some(&(p, _)) = work.last() {
                    low[p as usize] = low[p as usize].min(low[vu]);
                }
                if low[vu] == index[vu] {
                    let mut comp = Vec::new();
                    loop {
                        let w = st.pop().unwrap();
                        on[w as usize] = false;
                        comp.push(w);
                        if w == v {
                            break;
                        }
                    }
                    out.push(comp);
                }
            }
        }
    }
    out
}

/// Decompose a component with `pinned` variables treated as constants.
///
/// Candidate variables of each equation are ordered by the magnitude of the
/// Jacobian entry at `x`, so the greedy start of the matching pairs every
/// equation with a variable it is actually sensitive to (a poorly chosen pair
/// gives huge Newton steps or a singular block).
pub fn decompose(sys: &System, comp: &Component, pinned: &[u32], x: &[f64]) -> Decomp {
    let mut pat = weighted_patterns(sys, comp, x);
    decompose_with(sys, comp, pinned, &mut |row, out| pat(row, out))
}

/// Like [`decompose`] but drops variables the equation does not depend on at
/// `x` (a coordinate of a point on an axis-aligned line, say). Used as a
/// fallback when a block fails to converge.
pub fn decompose_numeric(sys: &System, comp: &Component, pinned: &[u32], x: &[f64]) -> Decomp {
    let mut pat = weighted_patterns(sys, comp, x);
    decompose_with(sys, comp, pinned, &mut |row, out| {
        pat(row, out);
        let rowmax = out.iter().map(|(_, w)| w.abs()).fold(0.0f64, f64::max);
        out.retain(|(_, w)| w.abs() > 1e-9 * rowmax.max(1e-300));
    })
}

/// Per-row candidate variables sorted by decreasing `|∂row/∂var|` at `x`.
fn weighted_patterns<'a>(
    sys: &'a System,
    comp: &Component,
    x: &[f64],
) -> impl FnMut(usize, &mut Vec<(u32, f64)>) + 'a {
    use crate::residual::{Jac, MAXR, MAXV};
    let mut cache: std::collections::HashMap<u32, Vec<Vec<(u32, f64)>>> =
        std::collections::HashMap::new();
    let mut jl: Jac = [[0.0; MAXV]; MAXR];
    for &r in &comp.rows {
        let ri = sys.row_res[r as usize];
        cache.entry(ri).or_insert_with(|| {
            sys.jac_res(ri as usize, x, &mut jl);
            let res = &sys.res[ri as usize];
            let mut slots = Vec::new();
            (0..res.kind.arity())
                .map(|eq| {
                    res.kind.pattern(eq, res.vars.len(), &mut slots);
                    // accumulate duplicate slots (the same variable in several slots)
                    let mut acc: Vec<(u32, f64)> = Vec::new();
                    for &sl in &slots {
                        let g = res.vars[sl as usize];
                        if sys.fixed[g as usize] {
                            continue;
                        }
                        match acc.iter_mut().find(|(v, _)| *v == g) {
                            Some((_, w)) => *w += jl[eq][sl as usize],
                            None => acc.push((g, jl[eq][sl as usize])),
                        }
                    }
                    acc.sort_by(|a, b| {
                        b.1.abs()
                            .partial_cmp(&a.1.abs())
                            .unwrap()
                            .then(a.0.cmp(&b.0))
                    });
                    acc
                })
                .collect()
        });
    }
    move |row, out| {
        let ri = sys.row_res[row];
        out.clear();
        out.extend_from_slice(&cache[&ri][row - sys.row_start[ri as usize]]);
    }
}

fn decompose_with(
    sys: &System,
    comp: &Component,
    pinned: &[u32],
    pattern: &mut dyn FnMut(usize, &mut Vec<(u32, f64)>),
) -> Decomp {
    // local column index
    let mut col_local: Vec<u32> = Vec::new();
    let cols: Vec<u32> = comp
        .cols
        .iter()
        .copied()
        .filter(|c| !pinned.contains(c))
        .collect();
    // global → local via sorted binary search (cols are sorted ascending)
    let lookup = |g: u32| cols.binary_search(&g).ok().map(|i| i as u32);
    col_local.clear();
    let nrows = comp.rows.len();
    let ncols = cols.len();
    let mut lists: Vec<Vec<u32>> = Vec::with_capacity(nrows);
    let mut strong_lists: Vec<Vec<u32>> = Vec::with_capacity(nrows);
    let mut strong_w: Vec<f64> = Vec::new();
    let mut pat: Vec<(u32, f64)> = Vec::new();
    for &r in &comp.rows {
        pattern(r as usize, &mut pat);
        let rowmax = pat.iter().map(|(_, w)| w.abs()).fold(0.0f64, f64::max);
        let mut all = Vec::with_capacity(pat.len());
        let mut strong = Vec::new();
        for &(g, w) in &pat {
            if let Some(c) = lookup(g) {
                all.push(c);
                if w.abs() >= 0.02 * rowmax && w != 0.0 {
                    strong.push(c);
                    strong_w.push(w.abs() / rowmax);
                }
            }
        }
        lists.push(all);
        strong_lists.push(strong);
    }
    let g = csr_from(&lists);
    let gs = csr_from(&strong_lists);
    let (mr, mc) = max_matching(nrows, ncols, &gs, &strong_w, &g);
    let matched = mr.iter().filter(|&&c| c != NONE).count();

    // over-determined rows: reachable from unmatched rows by alternating paths
    let mut in_over = vec![false; nrows];
    let mut stack: Vec<u32> = Vec::new();
    for r in 0..nrows {
        if mr[r] == NONE {
            in_over[r] = true;
            stack.push(r as u32);
        }
    }
    while let Some(r) = stack.pop() {
        for k in g.ptr[r as usize]..g.ptr[r as usize + 1] {
            let c = g.adj[k as usize] as usize;
            let r2 = mc[c];
            if r2 != NONE && !in_over[r2 as usize] {
                in_over[r2 as usize] = true;
                stack.push(r2);
            }
        }
    }

    // nodes: every non-over row, plus one node for the over-determined block
    let over_rows: Vec<u32> = (0..nrows as u32).filter(|&r| in_over[r as usize]).collect();
    let has_over = !over_rows.is_empty();
    let mut node_of_row = vec![NONE; nrows];
    let mut nodes: Vec<Vec<u32>> = Vec::new(); // rows per node
    if has_over {
        for &r in &over_rows {
            node_of_row[r as usize] = 0;
        }
        nodes.push(over_rows.clone());
    }
    for r in 0..nrows as u32 {
        if !in_over[r as usize] {
            node_of_row[r as usize] = nodes.len() as u32;
            nodes.push(vec![r]);
        }
    }
    // dependency edges: row r depends on the row matched to each other column it touches
    let mut dep_lists: Vec<Vec<u32>> = vec![Vec::new(); nodes.len()];
    for r in 0..nrows {
        let n = node_of_row[r] as usize;
        if in_over[r] {
            continue; // over block depends on nothing outside itself
        }
        for k in g.ptr[r]..g.ptr[r + 1] {
            let c = g.adj[k as usize] as usize;
            if mr[r] == c as u32 {
                continue;
            }
            let owner = mc[c];
            if owner != NONE {
                let m = node_of_row[owner as usize];
                if m as usize != n {
                    dep_lists[n].push(m);
                }
            }
        }
        dep_lists[n].sort_unstable();
        dep_lists[n].dedup();
    }
    let dep_csr = csr_from(&dep_lists);
    let sccs = tarjan(nodes.len(), &dep_csr);

    let mut node_to_block = vec![NONE; nodes.len()];
    let mut blocks: Vec<Block> = Vec::with_capacity(sccs.len());
    for scc in &sccs {
        let b = blocks.len() as u32;
        let mut rows_local: Vec<u32> = Vec::new();
        for &n in scc {
            node_to_block[n as usize] = b;
            rows_local.extend_from_slice(&nodes[n as usize]);
        }
        rows_local.sort_unstable();
        let over = has_over && scc.contains(&0);
        let mut block_cols: Vec<u32> = Vec::new();
        if over {
            // all columns adjacent to over-determined rows
            for &r in &rows_local {
                for k in g.ptr[r as usize]..g.ptr[r as usize + 1] {
                    block_cols.push(cols[g.adj[k as usize] as usize]);
                }
            }
            block_cols.sort_unstable();
            block_cols.dedup();
        } else {
            for &r in &rows_local {
                let c = mr[r as usize];
                block_cols.push(cols[c as usize]);
            }
            block_cols.sort_unstable();
        }
        let rows: Vec<u32> = rows_local.iter().map(|&r| comp.rows[r as usize]).collect();
        blocks.push(make_block(sys, rows, block_cols, over));
    }
    // dependencies between blocks
    let mut deps: Vec<Vec<u32>> = vec![Vec::new(); blocks.len()];
    for (b, scc) in sccs.iter().enumerate() {
        for &n in scc {
            for &m in &dep_lists[n as usize] {
                let mb = node_to_block[m as usize];
                if mb as usize != b {
                    deps[b].push(mb);
                }
            }
        }
        deps[b].sort_unstable();
        deps[b].dedup();
    }
    let mut level = vec![0u32; blocks.len()];
    for b in 0..blocks.len() {
        level[b] = deps[b]
            .iter()
            .map(|&d| level[d as usize] + 1)
            .max()
            .unwrap_or(0);
    }
    let free_cols: Vec<u32> = (0..ncols)
        .filter(|&c| mc[c] == NONE)
        .map(|c| cols[c])
        .collect();
    Decomp {
        blocks,
        deps,
        free_cols,
        level,
        matched,
    }
}
