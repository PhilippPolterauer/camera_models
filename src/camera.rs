use std::fmt::Debug;

use crate::distortion::CameraDistortion;
use crate::projection::CameraProjection;
use cv::nalgebra::Matrix2x3;
use nalgebra::{Isometry3, Vector3};

#[derive(Clone)]
pub struct PixelIndex<T>(pub T, pub T);

impl PixelIndex<f64> {
    pub fn x(&self) -> &f64 {
        &self.0
    }

    pub fn y(&self) -> &f64 {
        &self.1
    }

    pub fn nearest(self) -> PixelIndex<u32> {
        PixelIndex(self.x().round() as u32, self.y().round() as u32)
    }
}

impl Into<PixelIndex<u32>> for PixelIndex<f64> {
    fn into(self) -> PixelIndex<u32> {
        let PixelIndex(u, v) = self;
        PixelIndex(u.round() as u32, v.round() as u32)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraRay<T: Copy> {
    x: T,
    y: T,
}

impl<T: num_traits::One + Clone + Copy> CameraRay<T> {
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
    pub fn x(&self) -> T {
        self.x
    }
    pub fn y(&self) -> T {
        self.y
    }
    pub fn z(&self) -> T {
        T::one()
    }
    pub fn xy(&self) -> (T, T) {
        (self.x(), self.y())
    }
    pub fn xyz(&self) -> (T, T, T) {
        (self.x(), self.y(), self.z())
    }
    pub fn as_vector(&self) -> Vector3<T> {
        Vector3::new(self.x, self.y, self.z())
    }
}

pub struct CameraModel<T, V> {
    projection: T,
    distortion: V,
}
impl<T, V> CameraModel<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion<f64>,
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
    pub fn project<U: Into<CameraRay<f64>>>(&self, ray: U) -> PixelIndex<f64> {
        let ray = ray.into();
        let distorted = self.distortion().distort(&ray);
        self.projection().project(&distorted)
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
        let fx = (width as f64) / (2.0 * (fov_x / 2.0).tan());
        let fy = height as f64 / (2.0 * (fov_y / 2.0).tan());
        let cx = width as f64 / 2.0;
        let cy = height as f64 / 2.0;
        Self::new(fx, fy, cx, cy, 0.0)
    }
    pub fn matrix(self) -> Matrix2x3<f64> {
        Matrix2x3::new(self.fx, self.skew, self.cx, 0.0, self.fy, self.cy)
    }
}

pub struct Camera<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion<f64>,
{
    model: CameraModel<T, V>,
    view: Isometry3<f64>,
}
impl<T, V> Camera<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion<f64>,
{
    pub fn new(model: CameraModel<T, V>, view: Isometry3<f64>) -> Self {
        Self { model, view }
    }
    pub fn pose(&self) -> &Isometry3<f64> {
        &self.view
    }
    pub fn model(&self) -> &CameraModel<T, V> {
        &self.model
    }
}
