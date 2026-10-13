//! View frames and orthographic projection. The world is z-up. A view looks along `dir`;
//! `right` and `up` span the sheet, with `right = dir × up`, so the sheet is seen from the
//! side the viewer stands on.

use crate::model::Orientation;
use crate::sheet::ProjectionAngle;

pub type V3 = [f64; 3];

pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn unit(a: V3) -> V3 {
    let n = dot(a, a).sqrt();
    [a[0] / n, a[1] / n, a[2] / n]
}
fn neg(a: V3) -> V3 {
    [-a[0], -a[1], -a[2]]
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewFrame {
    pub right: V3,
    pub up: V3,
    /// Direction of sight, from the eye into the scene.
    pub dir: V3,
}

/// Which side of its parent a projected view sits on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Right,
    Left,
    Above,
    Below,
}

impl ViewFrame {
    pub fn new(dir: V3, up: V3) -> ViewFrame {
        let dir = unit(dir);
        // make `up` exactly perpendicular to `dir`
        let d = dot(up, dir);
        let up = unit([up[0] - d * dir[0], up[1] - d * dir[1], up[2] - d * dir[2]]);
        ViewFrame {
            right: cross(dir, up),
            up,
            dir,
        }
    }

    pub fn of(o: Orientation) -> ViewFrame {
        let z = [0.0, 0.0, 1.0];
        match o {
            Orientation::Front => ViewFrame::new([0.0, 1.0, 0.0], z),
            Orientation::Back => ViewFrame::new([0.0, -1.0, 0.0], z),
            Orientation::Top => ViewFrame::new([0.0, 0.0, -1.0], [0.0, 1.0, 0.0]),
            Orientation::Bottom => ViewFrame::new([0.0, 0.0, 1.0], [0.0, -1.0, 0.0]),
            Orientation::Right => ViewFrame::new([-1.0, 0.0, 0.0], z),
            Orientation::Left => ViewFrame::new([1.0, 0.0, 0.0], z),
            // seen from the front, right and above
            Orientation::Iso => ViewFrame::new([-1.0, 1.0, -1.0], z),
            Orientation::Custom { dir, up } => ViewFrame::new(dir, up),
        }
    }

    /// Sheet coordinates and depth (larger = farther from the eye) of a world point.
    pub fn project(&self, p: V3) -> ([f64; 2], f64) {
        ([dot(p, self.right), dot(p, self.up)], dot(p, self.dir))
    }

    /// The orientation name of this frame when it matches a standard one, else `Custom`.
    pub fn orientation(&self) -> Orientation {
        for o in [
            Orientation::Front,
            Orientation::Back,
            Orientation::Top,
            Orientation::Bottom,
            Orientation::Left,
            Orientation::Right,
            Orientation::Iso,
        ] {
            let f = ViewFrame::of(o);
            if dot(f.dir, self.dir) > 1.0 - 1e-9 && dot(f.up, self.up) > 1.0 - 1e-9 {
                return o;
            }
        }
        Orientation::Custom {
            dir: self.dir,
            up: self.up,
        }
    }

    /// The frame of a view placed on `side` of this one. In third-angle projection a view
    /// placed to the right shows the model as seen from the right; first-angle projection
    /// puts the opposite view there.
    pub fn neighbour(&self, side: Side, angle: ProjectionAngle) -> ViewFrame {
        let side = match angle {
            ProjectionAngle::Third => side,
            ProjectionAngle::First => match side {
                Side::Right => Side::Left,
                Side::Left => Side::Right,
                Side::Above => Side::Below,
                Side::Below => Side::Above,
            },
        };
        match side {
            Side::Right => ViewFrame::new(neg(self.right), self.up),
            Side::Left => ViewFrame::new(self.right, self.up),
            Side::Above => ViewFrame::new(neg(self.up), self.dir),
            Side::Below => ViewFrame::new(self.up, neg(self.dir)),
        }
    }
}

/// Sheet-space offset at which a projected view of size `child` goes next to a parent of size
/// `parent` (both `(width, height)` centred on their origins), leaving `gap` mm between them.
pub fn neighbour_offset(side: Side, parent: [f64; 2], child: [f64; 2], gap: f64) -> [f64; 2] {
    match side {
        Side::Right => [(parent[0] + child[0]) / 2.0 + gap, 0.0],
        Side::Left => [-(parent[0] + child[0]) / 2.0 - gap, 0.0],
        Side::Above => [0.0, (parent[1] + child[1]) / 2.0 + gap],
        Side::Below => [0.0, -(parent[1] + child[1]) / 2.0 - gap],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: V3, b: V3) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12)
    }

    #[test]
    fn standard_views_have_the_usual_axes() {
        let f = ViewFrame::of(Orientation::Front);
        assert!(close(f.right, [1.0, 0.0, 0.0]) && close(f.up, [0.0, 0.0, 1.0]));
        let t = ViewFrame::of(Orientation::Top);
        assert!(close(t.right, [1.0, 0.0, 0.0]) && close(t.up, [0.0, 1.0, 0.0]));
        let r = ViewFrame::of(Orientation::Right);
        assert!(close(r.right, [0.0, 1.0, 0.0]) && close(r.up, [0.0, 0.0, 1.0]));
        for o in [
            Orientation::Back,
            Orientation::Bottom,
            Orientation::Left,
            Orientation::Iso,
        ] {
            let v = ViewFrame::of(o);
            assert!(dot(v.right, v.up).abs() < 1e-12 && dot(v.right, v.dir).abs() < 1e-12);
        }
    }

    #[test]
    fn third_angle_neighbours_of_front_are_the_standard_views() {
        let f = ViewFrame::of(Orientation::Front);
        let third = ProjectionAngle::Third;
        assert_eq!(
            f.neighbour(Side::Right, third).orientation(),
            Orientation::Right
        );
        assert_eq!(
            f.neighbour(Side::Left, third).orientation(),
            Orientation::Left
        );
        assert_eq!(
            f.neighbour(Side::Above, third).orientation(),
            Orientation::Top
        );
        assert_eq!(
            f.neighbour(Side::Below, third).orientation(),
            Orientation::Bottom
        );
        // first angle swaps them
        let first = ProjectionAngle::First;
        assert_eq!(
            f.neighbour(Side::Right, first).orientation(),
            Orientation::Left
        );
        assert_eq!(
            f.neighbour(Side::Above, first).orientation(),
            Orientation::Bottom
        );
    }

    #[test]
    fn a_point_seen_from_the_front_projects_to_x_and_z() {
        let (p, depth) = ViewFrame::of(Orientation::Front).project([3.0, 5.0, 7.0]);
        assert_eq!(p, [3.0, 7.0]);
        assert_eq!(depth, 5.0);
    }
}
