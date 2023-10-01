use crate::distortion::CameraDistortion;
use crate::projection::CameraProjection;
use approx::{abs_diff_eq, relative_eq, AbsDiffEq, RelativeEq};
use nalgebra::{Isometry3, Vector3};
use std::fmt::Debug;

#[derive(Clone, PartialEq, Debug)]
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraRay<T: Copy + RelativeEq> {
    x: T,
    y: T,
}
impl<T: Copy + RelativeEq<T>> AbsDiffEq for CameraRay<T> {
    type Epsilon = T::Epsilon;
    fn default_epsilon() -> Self::Epsilon {
        T::default_epsilon()
    }
    fn abs_diff_eq(&self, other: &Self, _: Self::Epsilon) -> bool {
        abs_diff_eq!(self.x, other.x) && abs_diff_eq!(self.y, other.y)
    }
}

impl<T: RelativeEq + Copy + AbsDiffEq> RelativeEq for CameraRay<T> {
    fn default_max_relative() -> Self::Epsilon {
        T::default_max_relative()
    }
    fn relative_eq(&self, other: &Self, _: T::Epsilon, _: T::Epsilon) -> bool {
        return relative_eq!(self.x, other.x) && relative_eq!(self.y, other.y);
    }
}
impl<T: num_traits::One + Clone + Copy + RelativeEq> CameraRay<T> {
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

pub struct Fisheye {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub skew: f64,
}
