use crate::base::{CameraRay, Point, Point2, Transform, PixelIndex};
use crate::camera::{Camera, CameraModel, Pinhole};
use crate::CameraDistortion;

pub trait Projection<T> {
    fn project(&self, rhs: &T) -> PixelIndex<f64>;
    // fn unproject(&self, rhs: &Point2) -> T;
}
pub trait CameraProjection<T> {
    fn project(&self, rhs: &CameraRay) -> PixelIndex<T>;
    fn unproject(&self, rhs: &PixelIndex<T>) -> CameraRay;
}

impl<T, V> Projection<CameraRay> for CameraModel<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion,
{
    fn project(&self, rhs: &CameraRay) -> PixelIndex<f64> {
        let ray = self.distortion().distort(&rhs);
        self.projection().project(&ray)
    }
}

impl CameraProjection<f64> for Pinhole {
    fn project(&self, ray: &CameraRay) -> PixelIndex<f64> {
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
    fn unproject(&self, PixelIndex(u,v): &PixelIndex<f64>) -> CameraRay {
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

impl<T, V> Projection<Point> for Camera<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion,
{
    fn project(&self, point: &Point) -> PixelIndex<f64> {
        let point = self.pose().transform_into(point);
        let ray = CameraRay::try_from(&point).unwrap();
        self.model().project(&ray)
    }
}


