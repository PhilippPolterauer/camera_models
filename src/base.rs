use nalgebra::{Matrix2x3, UnitQuaternion};
pub type Point3 = nalgebra::Point3<f64>;
pub type Rotation = UnitQuaternion<f64>;
pub type Translation = Point3;
pub type Point2 = nalgebra::Point2<f64>;
pub type Vector3 = nalgebra::Vector3<f64>;
pub type UVector3 = nalgebra::UnitVector3<f64>;
pub type CameraMatrix = Matrix2x3<f64>;

#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Pose {
    pub rotation: Rotation,
    pub translation: Point3,
}
impl Pose {
    pub fn new(rotation: Rotation, translation: Translation) -> Self {
        Self {
            rotation,
            translation,
        }
    }
    pub fn identity() -> Self {
        Self {
            rotation: Rotation::identity(),
            translation: Point3::origin(),
        }
    }
}
pub struct Transform {
    pub rotation: Rotation,
    pub translation: Vector3,
}

pub struct Plane {
    origin: Point3,
    normal: UVector3,
}
pub struct Ray {
    origin: Point3,
    direction: UVector3,
}
pub struct Line {
    origin: Point3,
    direction: UVector3,
}
pub trait Projection {
    fn project(&self, point: &Point3) -> Point2;
}

pub trait Distortion {
    fn distort(&self, point: &Point2) -> Point2;
}

pub trait Transformable {
    fn transform(&self, transform: &Transform) -> Self;
}

impl Transform {
    pub fn new(rotation: Rotation, translation: Vector3) -> Self {
        Self {
            rotation,
            translation,
        }
    }
    pub fn identity() -> Self {
        Self {
            rotation: Rotation::identity(),
            translation: Vector3::zeros(),
        }
    }
    pub fn inverse(&self) -> Self {
        Self {
            rotation: self.rotation.inverse(),
            translation: self.rotation.inverse() * -self.translation,
        }
    }
    pub fn apply<T: Transformable>(&self, obj: &T) -> T {
        obj.transform(self)
    }
}

fn intersect_ray_plane(ray: &Ray, plane: &Plane) -> Point3 {
    let d = plane.normal.dot(&ray.direction);
    let n = plane.normal.dot(&ray.origin.coords);
    let t = (n / d) * -1.0;
    ray.origin + ray.direction.into_inner() * t
}

trait Intersectable<T> {
    type Output;
    fn intersect(&self, other: &T) -> Self::Output;
    fn has_intersection(&self, _other: &T) -> bool {
        true
    }
}

impl Intersectable<Ray> for Plane {
    type Output = Point3;
    fn intersect(&self, ray: &Ray) -> Self::Output {
        intersect_ray_plane(ray, self)
    }
}
impl Intersectable<Plane> for Ray {
    type Output = Point3;
    fn intersect(&self, plane: &Plane) -> Self::Output {
        intersect_ray_plane(self, plane)
    }
}
impl Intersectable<Plane> for Plane {
    type Output = Ray;
    fn intersect(&self, plane: &Plane) -> Self::Output {
        // find closest point to origin that lines on both planes
        let n1 = self.normal.into_inner();
        let n2 = plane.normal.into_inner();
        let _linedirection = n2.cross(&n1);
        let dir: Vector3 = n2 - (n1 * n1.dot(&n2));

        let _line = Line {
            origin: plane.origin,
            direction: UVector3::new_normalize(dir),
        };

        Ray {
            origin: Point3::origin(),
            direction: UVector3::new_normalize(dir),
        }
    }
}

#[cfg(test)]
#[test]
fn test_inverse() {
    let t = Transform::new(
        Rotation::from_euler_angles(0.0, 0.0, 0.0),
        Vector3::new(1.0, 2.0, 3.0),
    );
    let t_inv = t.inverse();
    let t_inv_inv = t_inv.inverse();
    assert_eq!(t_inv_inv.rotation, t.rotation);
    assert_eq!(t_inv_inv.translation, t.translation);
}
#[test]
fn test_quaternion() {
    let q = Rotation::from_euler_angles(0.0, 0.0, 0.0);
    let q_inv = q.inverse();
    let q_inv_inv = q_inv.inverse();
    assert_eq!(q_inv_inv, q);
    assert_eq!(
        Rotation::identity() * Vector3::new(1.0, 2.0, 3.0),
        Vector3::new(1.0, 2.0, 3.0)
    )
}
