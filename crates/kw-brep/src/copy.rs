//! Copying bodies between (or within) `BRep`s, optionally transformed and/or inside-out.

use crate::ids::*;
use crate::store::{BRep, CoedgeSpec};
use kw_math::Transform3;
use kw_nurbs::fit::{transform_curve, transform_surface};
use std::collections::HashMap;

#[derive(Debug, Clone, Default)]
pub struct CopyOptions {
    pub transform: Option<Transform3>,
    /// Reverse the orientation (swap inside and outside).
    pub flip: bool,
}

#[derive(Debug, Clone, Default)]
pub struct CopyMap {
    pub vertices: HashMap<VertexId, VertexId>,
    pub edges: HashMap<EdgeId, EdgeId>,
    pub faces: HashMap<FaceId, FaceId>,
}

impl BRep {
    /// Copy `body` of `src` into `self`. Geometry is cloned (and transformed); pcurves are
    /// unchanged because parameter space is intrinsic to each surface.
    pub fn copy_body_from(
        &mut self,
        src: &BRep,
        body: BodyId,
        opts: &CopyOptions,
    ) -> (BodyId, CopyMap) {
        let mut map = CopyMap::default();
        let mut curve_map: HashMap<CurveId, CurveId> = HashMap::new();
        let mut surf_map: HashMap<SurfaceId, SurfaceId> = HashMap::new();
        let src_body = &src.bodies[body];
        let new_body = self.add_body(src_body.kind);
        for &lump in &src_body.lumps {
            let nl = self.add_lump(new_body);
            for &shell in &src.lumps[lump].shells {
                let ns = self.add_shell(nl);
                for &face in &src.shells[shell].faces {
                    let sf = &src.faces[face];
                    let surf = *surf_map.entry(sf.surface).or_insert_with(|| {
                        let g = &src.surfaces[sf.surface];
                        let g = match &opts.transform {
                            Some(t) => transform_surface(g, t),
                            None => g.clone(),
                        };
                        self.add_surface(g)
                    });
                    let sense = if opts.flip { sf.sense.flip() } else { sf.sense };
                    let nf = self.add_face(ns, surf, sense);
                    map.faces.insert(face, nf);
                    for &lp in &sf.loops {
                        let mut uses: Vec<CoedgeSpec> = Vec::new();
                        for co in src.loop_coedges(lp) {
                            let c = &src.coedges[co];
                            let edge = match map.edges.get(&c.edge) {
                                Some(e) => *e,
                                None => {
                                    let e = &src.edges[c.edge];
                                    let vmap = |v: VertexId, this: &mut BRep, map: &mut CopyMap| {
                                        *map.vertices.entry(v).or_insert_with(|| {
                                            let mut p = src.vertices[v].point;
                                            if let Some(t) = &opts.transform {
                                                p = t.apply_point(&p);
                                            }
                                            let nv = this.add_vertex(p);
                                            this.vertices[nv].tolerance = src.vertices[v].tolerance;
                                            nv
                                        })
                                    };
                                    let (vs, ve) = (
                                        vmap(e.start, self, &mut map),
                                        vmap(e.end, self, &mut map),
                                    );
                                    let curve = e.curve.map(|cid| {
                                        *curve_map.entry(cid).or_insert_with(|| {
                                            let g = &src.curves[cid];
                                            let g = match &opts.transform {
                                                Some(t) => transform_curve(g, t),
                                                None => g.clone(),
                                            };
                                            self.add_curve(g)
                                        })
                                    });
                                    let ne = self.add_edge(vs, ve, curve, e.param_range);
                                    self.edges[ne].tolerance = e.tolerance;
                                    map.edges.insert(c.edge, ne);
                                    ne
                                }
                            };
                            let pcurve = c.pcurve.map(|pid| {
                                // pcurves are per-use: clone so later edits cannot alias
                                self.add_curve(src.curves[pid].clone())
                            });
                            uses.push(CoedgeSpec {
                                edge,
                                sense: c.sense,
                                pcurve,
                            });
                        }
                        if opts.flip {
                            uses.reverse();
                            for u in &mut uses {
                                u.sense = u.sense.flip();
                            }
                        }
                        self.add_loop(nf, src.loops[lp].kind, &uses);
                    }
                }
            }
        }
        (new_body, map)
    }
}
