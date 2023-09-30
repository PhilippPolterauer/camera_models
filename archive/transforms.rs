use crate::base::{Point, Pose, Transform, Transformable};
use std::ops::Mul;

impl Transformable for Pose {
    fn transform(&self, transform: &Transform) -> Pose {
        Pose::new(
            transform.rotation * self.rotation,
            transform.rotation * self.origin + transform.translation,
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
impl Transformable for Point {
    fn transform(&self, transform: &Transform) -> Point {
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
    use crate::base::{Rotation, Transformable, Vector};
    #[test]
    fn test_transforming() {
        let p1 = Pose::identity();
        let t1 = Transform::new(Rotation::identity(), Vector::new(1.0, 0.0, 0.0));
        assert_eq!(p1.transform(&t1), t1 * p1);
    }
}
