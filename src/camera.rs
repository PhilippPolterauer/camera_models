use crate::base::{CameraMatrix, Distortion, Point2, Point3, Pose, Projection, Transform, Vector3};
#[derive(Debug, Clone, Copy)]
struct Pinhole {
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    skew: f64,
}

impl Default for Pinhole {
    fn default() -> Self {
        Self::new(1.0, 1.0, 0.0, 0.0, 0.0)
    }
}

impl Pinhole {
    fn new(fx: f64, fy: f64, cx: f64, cy: f64, skew: f64) -> Self {
        Self {
            fx,
            fy,
            cx,
            cy,
            skew,
        }
    }
    fn from_resolution_fov(resolution: (u32, u32), fov: (f64, f64)) -> Self {
        let (width, height) = resolution;
        let (fov_x, fov_y) = fov;
        let fx = width as f64 / (2.0 * (fov_x / 2.0).tan());
        let fy = height as f64 / (2.0 * (fov_y / 2.0).tan());
        let cx = width as f64 / 2.0;
        let cy = height as f64 / 2.0;
        Self::new(fx, fy, cx, cy, 0.0)
    }
}

impl Projection for Pinhole {
    fn project(&self, point: &Point3) -> Point2 {
        let Point2 { coords } = Point2::from_homogeneous(point.coords).unwrap();
        let (x, y) = (coords.x, coords.y);
        let x = self.fx * x + self.skew * y + self.cx;
        let y = self.fy * y + self.cy;
        Point2::new(x, y)
    }
}

impl Into<CameraMatrix> for Pinhole {
    fn into(self) -> CameraMatrix {
        let mut m = CameraMatrix::zeros();
        m[(0, 0)] = self.fx;
        m[(0, 1)] = self.skew;
        m[(0, 2)] = self.cx;
        m[(1, 1)] = self.fy;
        m[(1, 2)] = self.cy;
        m
    }
}

struct Camera<T: Projection, V: Distortion> {
    projection: T,
    distortion: V,
    worldpose: Pose,
}

impl<T: Projection, V: Distortion> Projection for Camera<T, V> {
    fn project(&self, world_point: &Point3) -> Point2 {
        let camera_trafo: Transform = self.worldpose.into();
        let point = camera_trafo.inverse() * world_point;
        let point = self.projection.project(&point);
        self.distortion.distort(&point)
    }
}

struct Fisheye {
    k1: f64,
    k2: f64,
    k3: f64,
    k4: f64,
}
// impl Projection for Fisheye {
//     fn project()
// }
