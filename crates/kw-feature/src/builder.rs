//! Convenience constructors for the common features, used by tests, scripts and applications. Every
//! helper adds ordinary features to the [`Document`]; nothing here is special to the engine.

use crate::doc::{Document, TimelineError};
use crate::id::FeatureId;
use crate::naming::Reference;
use crate::recipe::{SketchRecipe, VarBinding};
use crate::spec::*;
use kw_expr::Param;
use kw_sketch::Sketch;

fn rect_recipe(x0: Param, y0: Param, w: Param, h: Param) -> SketchRecipe {
    let mut sk = Sketch::new();
    let p: Vec<_> = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        .iter()
        .map(|c| sk.add_point(c.0, c.1))
        .collect();
    let l: Vec<_> = (0..4).map(|i| sk.add_line(p[i], p[(i + 1) % 4])).collect();
    sk.horizontal(l[0]).unwrap();
    sk.vertical(l[1]).unwrap();
    sk.horizontal(l[2]).unwrap();
    sk.vertical(l[3]).unwrap();
    let dw = sk.dim_distance(p[0], p[1]).unwrap();
    let dh = sk.dim_distance(p[1], p[2]).unwrap();
    let (vw, vh) = (sk.dimension_var(dw).unwrap(), sk.dimension_var(dh).unwrap());
    let v0 = sk.point_vars(p[0]).unwrap();
    let mut rec = SketchRecipe::from_sketch(&sk);
    rec.bindings = vec![
        VarBinding {
            var: v0.x.0 as u32,
            value: x0,
        },
        VarBinding {
            var: v0.y.0 as u32,
            value: y0,
        },
        VarBinding {
            var: vw.0 as u32,
            value: w,
        },
        VarBinding {
            var: vh.0 as u32,
            value: h,
        },
    ];
    rec
}

fn circle_recipe(cx: Param, cy: Param, d: Param) -> SketchRecipe {
    let mut sk = Sketch::new();
    let c = sk.add_point(0.0, 0.0);
    let circle = sk.add_circle(c, 0.5);
    let dd = sk.dim_diameter(circle).unwrap();
    let vd = sk.dimension_var(dd).unwrap();
    let vc = sk.point_vars(c).unwrap();
    let mut rec = SketchRecipe::from_sketch(&sk);
    rec.bindings = vec![
        VarBinding {
            var: vc.x.0 as u32,
            value: cx,
        },
        VarBinding {
            var: vc.y.0 as u32,
            value: cy,
        },
        VarBinding {
            var: vd.0 as u32,
            value: d,
        },
    ];
    rec
}

impl Document {
    pub fn add_parameter(&mut self, name: &str, value: Param) -> Result<FeatureId, TimelineError> {
        self.add(
            name,
            FeatureSpec::Parameter {
                name: name.to_string(),
                value,
            },
        )
    }

    /// A rectangle with its lower-left corner at (`x0`, `y0`) in the plane, `w` wide and `h` high.
    pub fn add_rect_sketch(
        &mut self,
        name: &str,
        plane: PlaneSpec,
        x0: Param,
        y0: Param,
        w: Param,
        h: Param,
    ) -> Result<FeatureId, TimelineError> {
        self.add(
            name,
            FeatureSpec::Sketch(SketchSpec {
                plane,
                recipe: rect_recipe(x0, y0, w, h),
            }),
        )
    }

    pub fn add_circle_sketch(
        &mut self,
        name: &str,
        plane: PlaneSpec,
        cx: Param,
        cy: Param,
        diameter: Param,
    ) -> Result<FeatureId, TimelineError> {
        self.add(
            name,
            FeatureSpec::Sketch(SketchSpec {
                plane,
                recipe: circle_recipe(cx, cy, diameter),
            }),
        )
    }

    pub fn add_extrude(
        &mut self,
        name: &str,
        sketch: FeatureId,
        depth: Param,
        op: BodyOp,
    ) -> Result<FeatureId, TimelineError> {
        self.add(
            name,
            FeatureSpec::Extrude(ExtrudeSpec {
                sketch,
                depth,
                symmetric: false,
                flip: false,
                op,
                base: None,
                extent: Default::default(),
            }),
        )
    }

    pub fn add_fillet(
        &mut self,
        name: &str,
        edges: Vec<[Reference; 2]>,
        radius: Param,
    ) -> Result<FeatureId, TimelineError> {
        self.add(
            name,
            FeatureSpec::Fillet(FilletSpec {
                edges: edges.into_iter().map(|faces| EdgeRef { faces }).collect(),
                radius,
                base: None,
            }),
        )
    }

    pub fn add_linear_pattern(
        &mut self,
        name: &str,
        source: FeatureId,
        spacing: [Param; 3],
        count: Param,
    ) -> Result<FeatureId, TimelineError> {
        self.add(
            name,
            FeatureSpec::LinearPattern(LinearPatternSpec {
                source,
                spacing,
                count,
                base: None,
            }),
        )
    }
}
