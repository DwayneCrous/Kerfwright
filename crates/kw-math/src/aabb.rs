use nalgebra::{Point3, Vector3};

/// Axis-aligned bounding box. An empty box has `min > max`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: Point3<f64>,
    pub max: Point3<f64>,
}

impl Aabb {
    pub const EMPTY: Aabb = Aabb {
        min: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        max: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    };

    pub fn from_points<'a>(pts: impl IntoIterator<Item = &'a Point3<f64>>) -> Self {
        let mut b = Self::EMPTY;
        for p in pts {
            b.extend(p);
        }
        b
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    pub fn extend(&mut self, p: &Point3<f64>) {
        self.min = Point3::new(
            self.min.x.min(p.x),
            self.min.y.min(p.y),
            self.min.z.min(p.z),
        );
        self.max = Point3::new(
            self.max.x.max(p.x),
            self.max.y.max(p.y),
            self.max.z.max(p.z),
        );
    }

    pub fn union(&self, o: &Aabb) -> Aabb {
        let mut b = *self;
        if !o.is_empty() {
            b.extend(&o.min);
            b.extend(&o.max);
        }
        b
    }

    pub fn center(&self) -> Point3<f64> {
        nalgebra::center(&self.min, &self.max)
    }

    pub fn extent(&self) -> Vector3<f64> {
        self.max - self.min
    }

    pub fn diagonal(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            self.extent().norm()
        }
    }

    pub fn contains(&self, p: &Point3<f64>, slack: f64) -> bool {
        p.x >= self.min.x - slack
            && p.x <= self.max.x + slack
            && p.y >= self.min.y - slack
            && p.y <= self.max.y + slack
            && p.z >= self.min.z - slack
            && p.z <= self.max.z + slack
    }

    pub fn intersects(&self, o: &Aabb) -> bool {
        !(self.is_empty()
            || o.is_empty()
            || self.max.x < o.min.x
            || o.max.x < self.min.x
            || self.max.y < o.min.y
            || o.max.y < self.min.y
            || self.max.z < o.min.z
            || o.max.z < self.min.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extend_and_union() {
        let a = Aabb::from_points(&[Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 3.0)]);
        let b = Aabb::from_points(&[Point3::new(-1.0, 5.0, 0.0)]);
        let u = a.union(&b);
        assert_eq!(u.min, Point3::new(-1.0, 0.0, 0.0));
        assert_eq!(u.max, Point3::new(1.0, 5.0, 3.0));
        assert!(Aabb::EMPTY.is_empty());
        assert!(a.intersects(&u) && !a.intersects(&Aabb::EMPTY));
    }
}
