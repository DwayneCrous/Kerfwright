//! Dragging a component with the pointer: a free component follows the pointer in a plane
//! facing the camera; a component on a joint moves the joint's values so that the grabbed point stays as
//! near the pointer ray as the joint allows (within its limits).

use crate::numeric::solve_small;
use crate::{Assembly, AssemblyError, Placement};
use kw_expr::{Param, Quantity};
use kw_math::{Point3, Transform3, Vector3};

/// A ray in world coordinates (the direction need not be unit length).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray {
    pub origin: [f64; 3],
    pub dir: [f64; 3],
}

impl Ray {
    fn parts(&self) -> (Vector3<f64>, Vector3<f64>) {
        (
            Vector3::new(self.origin[0], self.origin[1], self.origin[2]),
            Vector3::new(self.dir[0], self.dir[1], self.dir[2]).normalize(),
        )
    }
}

#[derive(Debug, Clone)]
enum Kind {
    /// Translate in the plane through the grab point with this normal.
    Free {
        normal: Vector3<f64>,
        start_world: Vector3<f64>,
        anchor: usize,
        anchor_start: Transform3,
    },
    /// Drive the values of a joint.
    Joint { joint: usize, values: Vec<f64> },
}

/// A drag in progress.
#[derive(Debug, Clone)]
pub struct Drag {
    instance: usize,
    /// The grabbed point in the instance's own coordinates.
    grab_local: Vector3<f64>,
    kind: Kind,
}

impl Drag {
    /// Start dragging `instance` by the world point `grab_world`; `view_dir` is the camera's viewing
    /// direction (the plane a free component slides in is perpendicular to it).
    pub fn begin(
        asm: &Assembly,
        instance: usize,
        grab_world: [f64; 3],
        view_dir: [f64; 3],
    ) -> Result<Drag, AssemblyError> {
        let placed = asm.solve()?;
        let t = placed
            .get(instance)
            .ok_or(AssemblyError::UnknownInstance(instance))?;
        let g = Point3::new(grab_world[0], grab_world[1], grab_world[2]);
        let local = t.inverse().apply_point(&g).coords;
        let name = asm.instances[instance].name.clone();
        let kind = if asm.is_free(instance) {
            let anchor = asm.float_anchor(instance);
            Kind::Free {
                normal: Vector3::new(view_dir[0], view_dir[1], view_dir[2]).normalize(),
                start_world: g.coords,
                anchor,
                anchor_start: placed[anchor],
            }
        } else if let Some(j) = asm.drive_joint(instance) {
            let scope = asm.scope()?;
            let values = asm.joint_values(&asm.joints[j], &scope)?;
            Kind::Joint { joint: j, values }
        } else {
            return Err(AssemblyError::Invalid(format!(
                "'{name}' cannot be moved: it is grounded or held rigidly to a grounded component"
            )));
        };
        Ok(Drag {
            instance,
            grab_local: local,
            kind,
        })
    }

    /// The joint being driven, if the drag moves one.
    pub fn joint(&self) -> Option<usize> {
        match &self.kind {
            Kind::Joint { joint, .. } => Some(*joint),
            _ => None,
        }
    }

    /// Move the assembly so that the grabbed point follows the pointer ray.
    pub fn update(&mut self, asm: &mut Assembly, ray: &Ray) -> Result<(), AssemblyError> {
        let (o, d) = ray.parts();
        match &mut self.kind {
            Kind::Free {
                normal,
                start_world,
                anchor,
                anchor_start,
            } => {
                let denom = d.dot(normal);
                if denom.abs() < 1e-9 {
                    return Ok(());
                }
                let t = (*start_world - o).dot(normal) / denom;
                let hit = o + d * t;
                let delta = hit - *start_world;
                let moved = anchor_start.then(&Transform3::translation(delta));
                asm.instances[*anchor].at = Placement::from_transform(&moved);
                Ok(())
            }
            Kind::Joint { joint, values } => {
                let (j, i, grab) = (*joint, self.instance, self.grab_local);
                let kind = asm.joints[j].kind;
                let limits = asm.joints[j].limits.clone();
                let dims = kind.dims();
                let k = values.len();
                let clamp = |u: &mut [f64]| {
                    for (x, lim) in u.iter_mut().zip(&limits) {
                        if let Some((lo, hi)) = lim {
                            *x = x.clamp(*lo, *hi);
                        }
                    }
                };
                let set = |asm: &mut Assembly, u: &[f64]| {
                    asm.joints[j].values = u
                        .iter()
                        .zip(dims)
                        .map(|(x, dim)| {
                            if *dim == kw_expr::Dim::ANGLE {
                                Param::Lit(Quantity::angle(*x))
                            } else {
                                Param::length(*x)
                            }
                        })
                        .collect();
                };
                // where the grabbed point is for values `u`, and its offset from the ray
                let residual =
                    |asm: &mut Assembly, u: &[f64]| -> Result<Vector3<f64>, AssemblyError> {
                        set(asm, u);
                        let placed = asm.settle()?;
                        let w = placed[i].apply_point(&Point3::from(grab)).coords;
                        let rel = w - o;
                        Ok(rel - d * rel.dot(&d))
                    };
                let mut u = values.clone();
                clamp(&mut u);
                for _ in 0..30 {
                    let r0 = residual(asm, &u)?;
                    if r0.norm() < 1e-10 {
                        break;
                    }
                    // finite-difference Jacobian (3 x k)
                    let mut jac = vec![[0.0f64; 3]; k];
                    for c in 0..k {
                        let eps = if dims[c] == kw_expr::Dim::ANGLE {
                            1e-6
                        } else {
                            1e-5
                        };
                        // a step inside the limits (the other way when sitting on the upper one)
                        let mut up = u.clone();
                        up[c] += eps;
                        clamp(&mut up);
                        if (up[c] - u[c]).abs() < 0.5 * eps {
                            up[c] = u[c] - eps;
                            clamp(&mut up);
                        }
                        let h = up[c] - u[c];
                        if h.abs() < 1e-300 {
                            continue;
                        }
                        let r1 = residual(asm, &up)?;
                        let col = (r1 - r0) / h;
                        jac[c] = [col.x, col.y, col.z];
                    }
                    // damped normal equations (JᵀJ + λI) Δ = −Jᵀ r
                    let mut a = vec![vec![0.0f64; k]; k];
                    let mut b = vec![0.0f64; k];
                    for r in 0..k {
                        for c in 0..k {
                            a[r][c] = (0..3).map(|e| jac[r][e] * jac[c][e]).sum();
                        }
                        b[r] = -(0..3)
                            .map(|e| jac[r][e] * [r0.x, r0.y, r0.z][e])
                            .sum::<f64>();
                    }
                    let lambda = 1e-9
                        + 1e-6
                            * a.iter()
                                .enumerate()
                                .map(|(i, row)| row[i])
                                .fold(0.0, f64::max);
                    for (r, row) in a.iter_mut().enumerate() {
                        row[r] += lambda;
                    }
                    let Some(delta) = solve_small(a, b) else {
                        break;
                    };
                    // limit the step so a jump across the circle cannot happen
                    let mut step = 1.0f64;
                    for c in 0..k {
                        let cap = if dims[c] == kw_expr::Dim::ANGLE {
                            0.4
                        } else {
                            25.0
                        };
                        step = step.min(cap / delta[c].abs().max(1e-300));
                    }
                    let before = u.clone();
                    for c in 0..k {
                        u[c] += delta[c] * step;
                    }
                    clamp(&mut u);
                    let moved: f64 = u.iter().zip(&before).map(|(x, y)| (x - y).abs()).sum();
                    if moved < 1e-10 {
                        break;
                    }
                }
                set(asm, &u);
                *values = u;
                // the final values must be a valid pose (and the solved joints keep theirs)
                asm.settle().map(|_| ())
            }
        }
    }

    /// Restore the joint values (or placement) of the drag's start.
    pub fn cancel(&self, asm: &mut Assembly) {
        match &self.kind {
            Kind::Free {
                anchor,
                anchor_start,
                ..
            } => asm.instances[*anchor].at = Placement::from_transform(anchor_start),
            Kind::Joint { .. } => {}
        }
    }
}
