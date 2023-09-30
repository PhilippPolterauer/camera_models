
// use crate::base::{CameraRay, Point, Point2, Transform, PixelIndex};
use crate::camera::{CameraRay, Pinhole, PixelIndex};

pub trait CameraProjection<T: Copy> {
    fn project(&self, rhs: &CameraRay<T>) -> PixelIndex<T>;
    fn unproject(&self, rhs: &PixelIndex<T>) -> CameraRay<T>;
}

impl CameraProjection<f64> for Pinhole {
    fn project(&self, ray: &CameraRay<f64>) -> PixelIndex<f64> {
        let Pinhole {
            fx,
            fy,
            cx,
            cy,
            skew,
        } = *self;
        let (x, y) = ray.xy();
        let u = fx * x + skew * y + cx;
        let v = fy * y + cy;
        PixelIndex(u, v)
    }
    fn unproject(&self, PixelIndex(u, v): &PixelIndex<f64>) -> CameraRay<f64> {
        let Pinhole {
            fx,
            fy,
            cx,
            cy,
            skew,
        } = self;

        let y = (v - cy) / fy;
        let x = (u - cx - skew * y) / fx;
        CameraRay::new(x, y)
    }
}

