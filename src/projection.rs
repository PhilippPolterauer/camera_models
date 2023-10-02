// use crate::base::{CameraRay, Point, Point2, Transform, PixelIndex};
use crate::camera::{CameraRay, PixelIndex};
use approx::RelativeEq;
use nalgebra::{Matrix2x3, Rotation3, UnitVector3, Vector3};
#[derive(Debug, Clone, Copy)]
pub struct Pinhole {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub skew: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Fisheye {
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

pub trait CameraProjection<T: Copy + RelativeEq> {
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

impl CameraProjection<f64> for Fisheye {
    fn project(&self, ray: &CameraRay<f64>) -> PixelIndex<f64> {
        let Fisheye {
            fx,
            fy,
            cx,
            cy,
            skew,
        } = self;

        let (x, y, z) = ray.xyz();
        let ray_in = UnitVector3::new_normalize(Vector3::new(x, y, z));
        let ray_z = Vector3::<f64>::z_axis();
        
        let angle = Vector3::angle(&ray_z, &ray_in);
        let axis = ray_z.cross(&ray_in).try_normalize(f64::EPSILON);
        match axis {
            Some(axis) => {
                let axis = axis*angle;
                let theta_x = axis.y;
                let theta_y = -axis.x;
                let u = fx * theta_x + skew * theta_y + cx;
                let v = fy * theta_y + cy;
                PixelIndex(u, v)
            }
            None => {
                let u = cx;
                let v = cy;
                PixelIndex(*u, *v)
            }
        }

    }
    fn unproject(&self, PixelIndex(u, v): &PixelIndex<f64>) -> CameraRay<f64> {
        let Fisheye {
            fx,
            fy,
            cx,
            cy,
            skew,
        } = self;

        let theta_y = (v - cy) / fy;
        let theta_x = (u - cx - skew * theta_y) / fx;
        let ray = Vector3::z_axis();
        let rot = Rotation3::from_scaled_axis(Vector3::new(-theta_y, theta_x, 0.0));
        let ray = rot * ray;
        let x: f64 = ray.x;
        let y = ray.y;
        let z = ray.z;
        CameraRay::new(x / z, y / z)
    }
}

#[cfg(test)]
mod tests {
    use approx::{assert_relative_eq, AbsDiffEq};

    use super::*;
    const PROJECTION: Fisheye = Fisheye {
        fx: 1.0,
        fy: 1.0,
        cx: 0.0,
        cy: 0.0,
        skew: 0.0,
    };
    #[test]
    fn test_pixel2pixel() {
        for (u, v) in vec![(0.0, 0.0), (0.2, 1.0), (1.0, 0.0), (0.9, 1.0)] {
            let src = PixelIndex(u, v);
            let ray = PROJECTION.unproject(&src);
            let dst = PROJECTION.project(&ray);
            assert!(src.0.abs_diff_eq(&dst.0, 0.00001) && src.1.abs_diff_eq(&dst.1, 0.00001))
        }
    }
    #[test]
    fn test_ray2ray() {
        for (x, y, z) in vec![
            (0.0, 0.0, 1.0),
            (0.2, 0.0, 1.0),
            (0.0, 0.2, 1.0),
            (0.2, 0.2, 1.0),
            (0.4, 0., 1.0),
            (-0.4, 0., 1.0),
            (-0.4, 0.2, 1.0),
            (-0.3, -0.4, 1.0),
            (0., -0.4, 1.0),
            (0., 0.3, 1.0),
        ] {
            let src = CameraRay::new(x / z, y / z);
            let mid = PROJECTION.project(&src);
            let dst = PROJECTION.unproject(&mid);
            assert!(src.x().abs_diff_eq(&dst.x(), 0.00001) && src.y().abs_diff_eq(&dst.y(), 0.00001))
        }
    }
}
