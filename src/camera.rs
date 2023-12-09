use crate::distortion::CameraDistortion;
use crate::projection::CameraProjection;

use nalgebra::{
    Isometry3, Vector3,
};
use std::fmt::Debug;

#[derive(Clone, PartialEq, Debug)]

// TODO think about a better name for this
// maybe:
// - PixelCoordinate
// - PixelLocation
// - PixelIndex
// - PixelPosition
// - PixelPoint
// - ImageLocation
// - ImageCoordinate
// - ImageIndex
// - ImagePosition
// - ImagePoint

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

#[derive(Debug, Clone, Copy)]
pub struct CameraRay {
    pub vector: Vector3<f64>,
}
impl PartialEq for CameraRay {
    fn eq(&self, other: &Self) -> bool {
        let angle = self.vector.angle(&other.vector);
        println!("angle: {}, eps: {}", angle, f64::EPSILON.sqrt());
        angle < f64::EPSILON.sqrt()
    }
}
impl Eq for CameraRay {}

impl CameraRay {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            vector: Vector3::new(x, y, z),
        }
    }
    pub fn xy(&self) -> (f64, f64) {
        (self.vector.x, self.vector.y)
    }
}

pub struct CameraModel<T, V> {
    projection: T,
    distortion: V,
}
impl<T, V> CameraModel<T, V>
where
    T: CameraProjection,
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
    pub fn project<U: Into<CameraRay>>(&self, ray: U) -> PixelIndex<f64> {
        let ray = ray.into();
        let distorted = self.distortion().distort(&ray);
        self.projection().project(&distorted)
    }
}

pub struct Camera<T, V>
where
    T: CameraProjection,
    V: CameraDistortion,
{
    model: CameraModel<T, V>,
    view: Isometry3<f64>,
}
impl<T, V> Camera<T, V>
where
    T: CameraProjection,
    V: CameraDistortion,
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
