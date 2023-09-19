use crate::base::{Distortion, Point2};

struct PlumbBob {
    k1: f64,
    k2: f64,
    p1: f64,
    p2: f64,
    k3: f64,
}

impl PlumbBob {
    fn new(k1: f64, k2: f64, p1: f64, p2: f64, k3: f64) -> Self {
        Self { k1, k2, p1, p2, k3 }
    }
}

impl Distortion for PlumbBob {
    fn distort(&self, point: &Point2) -> Point2 {
        let x = point.x;
        let y = point.y;
        let r2 = x * x + y * y;
        let r4 = r2 * r2;
        let r6 = r4 * r2;
        let radial = 1.0 + self.k1 * r2 + self.k2 * r4 + self.k3 * r6;
        let tangential_x = 2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x * x);
        let tangential_y = self.p1 * (r2 + 2.0 * y * y) + 2.0 * self.p2 * x * y;
        Point2::new(
            radial * x + tangential_x,
            radial * y + tangential_y,
        )
    }
}


