use crate::base::{Point3, Pose, Transform, Transformable};
use std::ops::Mul;
impl Into<Transform> for Pose {
    fn into(self) -> Transform {
        Transform::new(self.rotation, self.translation.coords)
    }
}
impl Transformable for Pose {
    fn transform(&self, transform: &Transform) -> Pose {
        Pose::new(
            transform.rotation * self.rotation,
            transform.rotation * self.translation + transform.translation,
        )
    }
}
impl Transformable for Transform {
    fn transform(&self, transform: &Transform) -> Transform {
        Transform {
            rotation: transform.rotation * self.rotation,
            translation: transform.rotation * self.translation + transform.translation,
        }
    }
}
impl Transformable for Point3 {
    fn transform(&self, transform: &Transform) -> Point3 {
        transform.rotation * self + transform.translation
    }
}

impl<T: Transformable> Mul<T> for Transform {
    type Output = T;
    fn mul(self, rhs: T) -> T {
        self.apply(&rhs)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::{Rotation, Transformable, Vector3};
    #[test]
    fn test_transforming() {
        let p1 = Pose::identity();
        let t1 = Transform::new(Rotation::identity(), Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(p1.transform(&t1), t1 * p1);
    }
}
