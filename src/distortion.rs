use crate::camera::CameraRay;
use serde::Deserialize;

pub trait CameraDistortion<T>
where
    T: Copy,
{
    fn distort(&self, ray: &CameraRay<T>) -> CameraRay<T>;
}
impl<T: Copy> CameraDistortion<T> for Ideal {
    fn distort(&self, ray: &CameraRay<T>) -> CameraRay<T> {
        *ray
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct PlumbBob {
    k1: f64,
    k2: f64,
    p1: f64,
    p2: f64,
    k3: f64,
}
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct Fisheye {
    pub k1: f64,
    pub k2: f64,
    pub k3: f64,
    pub k4: f64,
    pub s: f64,
}

pub struct Ideal {}

impl PlumbBob {
    pub fn new(k1: f64, k2: f64, p1: f64, p2: f64, k3: f64) -> Self {
        Self { k1, k2, p1, p2, k3 }
    }
    pub fn params(x: f64, y: f64) -> (f64, f64, f64, f64, f64, f64) {
        let x2 = x * x;
        let y2 = y * y;
        let xy = x * y;
        let r2 = x2 + y2;
        let r4 = r2 * r2;
        let r6 = r4 * r2;
        (x2, y2, xy, r2, r4, r6)
    }
}
impl Fisheye {
    pub fn params(x: f64, y: f64) -> (f64, f64, f64, f64, f64, f64, f64, f64) {
        let r2 = x * x + y * y;
        let r = r2.sqrt();
        let theta = r.atan();
        let theta2 = theta * theta;
        let theta4 = theta2 * theta2;
        let theta6 = theta4 * theta2;
        let theta8 = theta6 * theta2;
        (r, theta, theta2, theta4, theta6, theta8, x, y)
    }
}

impl CameraDistortion<f64> for PlumbBob {
    fn distort(&self, ray: &CameraRay<f64>) -> CameraRay<f64> {
        let (x, y) = ray.xy();
        let (x2, y2, xy, r2, r4, r6) = PlumbBob::params(x, y);
        let radial = 1.0 + self.k1 * r2 + self.k2 * r4 + self.k3 * r6;
        let tangential_x = 2.0 * self.p1 * xy + self.p2 * (r2 + 2.0 * x2);
        let tangential_y = self.p1 * (r2 + 2.0 * y2) + 2.0 * self.p2 * xy;
        CameraRay::new(radial * x + tangential_x, radial * y + tangential_y)
    }
}

impl CameraDistortion<f64> for Fisheye {
    // fromhttps://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
    //   a=x/z and b=y/zr2=a2+b2θ=atan(r)
    fn distort(&self, ray: &CameraRay<f64>) -> CameraRay<f64> {
        let (x, y) = ray.xy();
        let (r, theta, theta2, theta4, theta6, theta8, x, y) = Self::params(x, y);
        let theta_d_r = theta
            * (1.0 + self.k1 * theta2 + self.k2 * theta4 + self.k3 * theta6 + self.k4 * theta8)
            / r;
        CameraRay::new(theta_d_r * x, theta_d_r * y)
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_plumb() {
        use super::*;
        let p = PlumbBob::new(0.0, 0.0, 0.0, 0.0, 0.0);
        let p2 = p.distort(&CameraRay::new(1.0, 1.0));
        assert_eq!(p2, CameraRay::new(1.0, 1.0));
    }
    #[test]
    fn test_roundtrip() {
        use super::*;
        let p = PlumbBob::new(0.0, 0.0, 0.0, 0.0, 0.0);
        let p2 = p.distort(&CameraRay::new(1.0, 1.0));
        let p3 = p.distort(&p2);
        assert_eq!(p3, CameraRay::new(1.0, 1.0));
    }
}
