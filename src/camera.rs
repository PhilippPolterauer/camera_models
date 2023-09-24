use crate::base::{CameraMatrix, CameraRay, Pose};
use crate::distortion::CameraDistortion;
use crate::projection::CameraProjection;

pub struct CameraModel<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion,
{
    projection: T,
    distortion: V,
}
impl<T, V> CameraModel<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion,
{
    pub fn new(projection: T, distortion: V) -> Self {
        Self {
            projection,
            distortion,
        }
    }
    pub fn distortion(&self) -> &V {
        &self.distortion
    }
    pub fn projection(&self) -> &T {
        &self.projection
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Pinhole {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub skew: f64,
}

impl Default for Pinhole {
    fn default() -> Self {
        Self::new(1.0, 1.0, 0.0, 0.0, 0.0)
    }
}

impl Pinhole {
    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64, skew: f64) -> Self {
        Self {
            fx,
            fy,
            cx,
            cy,
            skew,
        }
    }
    pub fn from_resolution_fov(resolution: (u32, u32), fov: (f64, f64)) -> Self {
        let (width, height) = resolution;
        let (fov_x, fov_y) = fov;
        let fx = width as f64 / (2.0 * (fov_x / 2.0).tan());
        let fy = height as f64 / (2.0 * (fov_y / 2.0).tan());
        let cx = width as f64 / 2.0;
        let cy = height as f64 / 2.0;
        Self::new(fx, fy, cx, cy, 0.0)
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

pub struct Camera<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion,
{
    model: CameraModel<T, V>,
    worldpose: Pose,
}
impl<T, V> Camera<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion,
{
    pub fn new(model: CameraModel<T, V>, worldpose: Pose) -> Self {
        Self { model, worldpose }
    }
    pub fn pose(&self) -> &Pose {
        &self.worldpose
    }
    pub fn model(&self) -> &CameraModel<T, V> {
        &self.model
    }
}

