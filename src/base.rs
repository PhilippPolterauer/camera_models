use nalgebra::{Matrix2x3, UnitQuaternion};
pub type Point3 = nalgebra::Point3<f64>;
pub type Rotation = UnitQuaternion<f64>;
pub type Translation = Point3;
pub type Point2 = nalgebra::Point2<f64>;
pub type Vector3 = nalgebra::Vector3<f64>;
pub type CameraMatrix = Matrix2x3<f64>;

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
}
pub struct Transform {
    pub rotation: Rotation,
    pub translation: Vector3,
}

pub struct Plane {
    origin: Rotation,
    normal: Vector3,
}
pub struct Ray {
    origin: Point3,
    direction: Vector3,
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
