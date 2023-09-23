use crate::base::{CameraRay, Point, Point2, Transform};
use crate::camera::{Camera, CameraModel, Pinhole};
use crate::CameraDistortion;

pub trait Projection<T> {
    fn project(&self, rhs: &T) -> Point2;
    // fn unproject(&self, rhs: &Point2) -> T;
}
pub trait CameraProjection {
    fn project(&self, rhs: &CameraRay) -> Point2;
    fn unproject(&self, rhs: &Point2) -> CameraRay;
}

impl<T, V> Projection<CameraRay> for CameraModel<T, V>
where
    T: CameraProjection,
    V: CameraDistortion,
{
    fn project(&self, rhs: &CameraRay) -> Point2 {
        let ray = self.distortion().distort(&rhs);
        self.projection().project(&ray)
    }
}

impl CameraProjection for Pinhole {
    fn project(&self, ray: &CameraRay) -> Point2 {
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
        Point2::new(u, v)
    }
    fn unproject(&self, rhs: &Point2) -> CameraRay {
        let Pinhole {
            fx,
            fy,
            cx,
            cy,
            skew,
        } = *self;
        let (u, v) = (rhs.x, rhs.y);
        let y = (v - cy) / fy;
        let x = (u - cx - skew * y) / fx;
        CameraRay::new(x, y)
    }
}

impl<T, V> Projection<Point> for Camera<T, V>
where
    T: CameraProjection,
    V: CameraDistortion,
{
    fn project(&self, point: &Point) -> Point2 {
        let point = self.pose().transform_into(point);
        let ray = CameraRay::try_from(&point).unwrap();
        self.model().project(&ray)
    }
}


