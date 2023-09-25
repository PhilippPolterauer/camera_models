use nalgebra::{Matrix2x3, RealField, UnitQuaternion, Vector2};
use std::convert::From;
pub type Point = nalgebra::Point3<f64>;
pub type Rotation = UnitQuaternion<f64>;
pub type Point2 = nalgebra::Point2<f64>;
pub type Vector = nalgebra::Vector3<f64>;
pub type UVector = nalgebra::UnitVector3<f64>;
pub type CameraMatrix = Matrix2x3<f64>;
pub type ImagePoint<T> = nalgebra::Point2<T>;
pub struct PixelIndex<T>(pub T, pub T);

impl PixelIndex<f64> {
    pub fn x(&self) -> &f64 {
        &self.0
    }
    pub fn y(&self) -> &f64 {
        &self.1
    }

    pub fn nearest(self) -> PixelIndex<u32> {
        PixelIndex(*self.x() as u32, *self.y() as u32)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraRay {
    ray: Vector2<f64>,
}
impl CameraRay {
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            ray: Vector2::new(x, y),
        }
    }
    pub fn x(&self) -> f64 {
        self.ray.x
    }
    pub fn y(&self) -> f64 {
        self.ray.y
    }
    pub fn xy(&self) -> (f64, f64) {
        (self.x(), self.y())
    }
}

impl TryFrom<&Point> for CameraRay {
    type Error = &'static str;
    fn try_from(point: &Point) -> Result<Self, Self::Error> {
        if point.z <= 0.0 {
            Result::Err("Point must be in front of camera")
        } else {
            Result::Ok(Self {
                ray: Vector2::new(point.x / point.z, point.y / point.z),
            })
        }
    }
}
impl TryFrom<Point> for CameraRay {
    type Error = &'static str;
    fn try_from(point: Point) -> Result<Self, Self::Error> {
        Self::try_from(&point)
    }
}

impl<T: Into<f64>> From<PixelIndex<T>> for Point2 {
    fn from(PixelIndex(x, y): PixelIndex<T>) -> Self {
        Self::new(x.into() as f64, y.into() as f64)
    }
}
impl From<Point2> for PixelIndex<u32> {
    fn from(point: Point2) -> Self {
        Self(point.x as u32, point.y as u32)
    }
}

#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Pose {
    pub rotation: Rotation,
    pub origin: Point,
}
impl Pose {
    pub fn new(rotation: Rotation, origin: Point) -> Self {
        Self { rotation, origin }
    }

    pub fn identity() -> Self {
        Self {
            rotation: Rotation::identity(),
            origin: Point::origin(),
        }
    }

    pub fn transform_into(&self, point: &Point) -> Point {
        Transform::from(*self).apply(point)
    }
}
// conversions
impl From<Pose> for Transform {
    fn from(pose: Pose) -> Self {
        Self {
            rotation: pose.rotation,
            translation: pose.origin.coords,
        }
    }
}
pub struct Plane {
    normal: UVector,
    d: f64,
}
impl Plane {
    pub fn new(normal: UVector, d: f64) -> Self {
        Self { normal, d }
    }
    pub fn from_origin_normal(origin: Point, normal: UVector) -> Self {
        // the plane are all points where (P-O) dot n = 0
        // we can write the plane as ax + by + cz = d or
        // (x-x0)*n.x + (y-y0)*n.y + (z-z0)*n.z = 0
        // x*n.x + y*n.y + z*n.z = x0*n.x + y0*n.y + z0*n.z
        // a = n.x, b = n.y, c = n.z, d = O dot n
        let normal = UVector::new_normalize(*normal);
        let d = origin.coords.dot(&normal);
        Self { normal, d }
    }
    pub fn from_line_dir(line: &Line, dir: &UVector) -> Self {
        let normal = line.direction.cross(dir);
        Self::from_origin_normal(line.origin, UVector::new_normalize(normal))
    }
    pub fn from_line_point(line: &Line, point: &Point) -> Self {
        let dir = point - line.origin;
        let normal = line.direction.cross(&dir);
        Self::from_origin_normal(line.origin, UVector::new_normalize(normal))
    }
    pub fn normal(&self) -> UVector {
        self.normal
    }
    pub fn origin(&self) -> Point {
        Point::origin() + self.normal.normalize() * self.d
    }
}
#[derive(Clone, PartialEq, Debug)]
pub struct Ray {
    origin: Point,
    direction: UVector,
}
impl Ray {
    pub fn new(origin: Point, direction: UVector) -> Self {
        Self { origin, direction }
    }
    pub fn direction(&self) -> UVector {
        self.direction
    }
    pub fn origin(&self) -> Point {
        self.origin
    }
}
#[derive(Debug, Clone)]
pub struct Line {
    origin: Point,
    direction: UVector,
}
impl Line {
    pub fn new(origin: Point, direction: UVector) -> Self {
        Self { origin, direction }
    }
    pub fn direction(&self) -> UVector {
        self.direction
    }
    pub fn origin(&self) -> Point {
        self.origin
    }
}

pub struct LineSegment {
    start: Point,
    end: Point,
}
impl LineSegment {
    pub fn new(start: Point, end: Point) -> Self {
        Self { start, end }
    }
    pub fn start(&self) -> Point {
        self.start
    }
    pub fn end(&self) -> Point {
        self.end
    }
    pub fn direction(&self) -> UVector {
        UVector::new_normalize(self.end - self.start)
    }
}

pub struct Transform {
    pub rotation: Rotation,
    pub translation: Vector,
}

pub enum Object {
    Point(Point),
    Line(Line),
    Ray(Ray),
    Plane(Plane),
    LineSegment(LineSegment),
}
#[derive(PartialEq, Debug)]
pub enum Intersection {
    Point(Point),
    Line(Line),
    Ray(Ray),
}

impl PartialEq for Plane {
    fn eq(&self, other: &Self) -> bool {
        self.normal == other.normal && self.d == other.d
    }
}

impl PartialEq for Line {
    fn eq(&self, other: &Self) -> bool {
        // line1 equation is P(t1) = O1 + t1 * D1
        // line2 equation is P(t2) = O2 + t2 * D2
        // if line1 and line2 have an intersection then O1 + t1 * D1 == O2 + t2 * D2
        // O2 - O1 == t1 * D1 - t2 * D2
        // A * [t1, t2] = B, where A = [D1, -D2] and B = O2-O1
        let d1 = self.direction();
        let d2 = other.direction();
        let o1 = self.origin();
        let o2 = other.origin();

        d1 == d2 && (o2 - o1).cross(&d1).norm() == 0.0
    }

    fn ne(&self, other: &Self) -> bool {
        !self.eq(other)
    }
}

trait Container<T> {
    fn contains(&self, obj: &T) -> bool;
}

impl Container<Point> for Plane {
    fn contains(&self, point: &Point) -> bool {
        self.normal().dot(&point.coords) == self.d
    }
}
impl Container<Line> for Plane {
    fn contains(&self, line: &Line) -> bool {
        self.contains(&line.origin) && self.normal.dot(&line.direction) == 0.0
    }
}
impl Container<Ray> for Plane {
    fn contains(&self, ray: &Ray) -> bool {
        self.contains(&ray.origin) && self.normal.dot(&ray.direction) == 0.0
    }
}
impl Container<LineSegment> for Plane {
    fn contains(&self, line_segment: &LineSegment) -> bool {
        self.contains(&line_segment.start()) && self.contains(&line_segment.end())
    }
}

impl Container<Plane> for Plane {
    fn contains(&self, plane: &Plane) -> bool {
        self == plane
    }
}

pub trait Transformable {
    fn transform(&self, transform: &Transform) -> Self;
}

impl Transform {
    pub fn new(rotation: Rotation, translation: Vector) -> Self {
        Self {
            rotation,
            translation,
        }
    }
    pub fn identity() -> Self {
        Self {
            rotation: Rotation::identity(),
            translation: Vector::zeros(),
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
    pub fn unapply<T: Transformable>(&self, obj: &T) -> T {
        obj.transform(&self.inverse())
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_pose_new() {
        let rotation = Rotation::from_axis_angle(&Vector::z_axis(), 0.5);
        let translation = Point::new(1.0, 2.0, 3.0);
        let pose = Pose::new(rotation, translation);
        assert_eq!(pose.rotation, rotation);
        assert_eq!(pose.origin, translation);
    }

    #[test]
    fn test_pose_identity() {
        let pose = Pose::identity();
        assert_eq!(pose.rotation, Rotation::identity());
        assert_eq!(pose.origin, Point::origin());
    }

    #[test]
    fn test_plane_new() {
        let normal = UVector::new_normalize(Vector::new(1.0, 0.0, 0.0));
        let d = 1.0;
        let plane = Plane::new(normal, d);
        assert_eq!(plane.normal(), normal);
        assert_eq!(plane.origin(), Point::new(d, 0.0, 0.0));
    }

    #[test]
    fn test_plane_from_origin_normal() {
        let origin = Point::new(1.0, 2.0, 3.0);
        let normal = UVector::new_normalize(Vector::new(1.0, 1.0, 1.0));
        let plane = Plane::from_origin_normal(origin, normal);
        assert_eq!(plane.normal(), normal);
        assert_relative_eq!(plane.origin(), Point::new(2., 2.0, 2.0));
    }

    #[test]
    fn test_plane_from_line_dir() {
        let line = Line::new(
            Point::new(1.0, 2.0, 3.0),
            UVector::new_normalize(Vector::new(1.0, 0.0, 0.0)),
        );
        let dir = UVector::new_normalize(Vector::new(0.0, 1.0, 0.0));
        let plane = Plane::from_line_dir(&line, &dir);
        assert_eq!(
            plane.normal(),
            UVector::new_normalize(Vector::new(0.0, 0.0, 1.0))
        );
        assert_eq!(plane.origin(), Point::new(0., 0., 3.0));
    }

    #[test]
    fn test_plane_from_line_point() {
        let line = Line::new(
            Point::new(1.0, 2.0, 3.0),
            UVector::new_normalize(Vector::new(1.0, 0.0, 0.0)),
        );
        let point = Point::new(1.0, 3.0, 3.0);
        let plane = Plane::from_line_point(&line, &point);
        assert_eq!(
            plane.normal(),
            UVector::new_normalize(Vector::new(0.0, 0.0, 1.0))
        );
        assert_eq!(plane.origin(), Point::new(0.0, 0.0, 3.0));
    }

    #[test]
    fn test_ray_new() {
        let origin = Point::new(1.0, 2.0, 3.0);
        let direction = UVector::new_normalize(Vector::new(1.0, 0.0, 0.0));
        let ray = Ray::new(origin, direction);
        assert_eq!(ray.origin(), origin);
        assert_eq!(ray.direction(), direction);
    }

    #[test]
    fn test_line_new() {
        let origin = Point::new(1.0, 2.0, 3.0);
        let direction = UVector::new_normalize(Vector::new(1.0, 0.0, 0.0));
        let line = Line::new(origin, direction);
        assert_eq!(line.origin(), origin);
        assert_eq!(line.direction(), direction);
    }

    #[test]
    fn test_line_segment_new() {
        let start = Point::new(1.0, 2.0, 3.0);
        let end = Point::new(4.0, 5.0, 6.0);
        let line_segment = LineSegment::new(start, end);
        assert_eq!(line_segment.start(), start);
        assert_eq!(line_segment.end(), end);
        assert_relative_eq!(
            line_segment.direction(),
            UVector::new_normalize(Vector::new(1.0, 1.0, 1.0))
        );
    }
    #[test]
    fn test_line_equality() {
        let origin1 = Point::new(0.0, 0.0, 0.0);
        let direction1 = UVector::new_normalize(Vector::new(1.0, 1.0, 1.0));
        let line1 = Line::new(origin1, direction1);

        let origin2 = Point::new(0.0, 0.0, 0.0);
        let direction2 = UVector::new_normalize(Vector::new(1.0, 1.0, 1.0));
        let line2 = Line::new(origin2, direction2);

        assert_eq!(line1.eq(&line2), true);

        let origin3 = Point::new(1.0, 1.0, 1.0);
        let direction3 = UVector::new_normalize(Vector::new(1.0, 1.0, 1.0));
        let line3 = Line::new(origin3, direction3);

        assert_eq!(line1.eq(&line3), true);

        let origin4 = Point::new(1.0, 0., 0.);
        let direction4 = UVector::new_normalize(Vector::new(1.0, 0.0, 0.0));
        let line4 = Line::new(origin4, direction4);

        assert_eq!(line1.eq(&line4), false);
    }
}
