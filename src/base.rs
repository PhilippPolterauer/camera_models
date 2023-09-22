use nalgebra::{Matrix2x3, Matrix3x2, UnitQuaternion};
pub type Point = nalgebra::Point3<f64>;
pub type Rotation = UnitQuaternion<f64>;
pub type Translation = Point;
pub type Point2 = nalgebra::Point2<f64>;
pub type Vector = nalgebra::Vector3<f64>;
pub type UVector = nalgebra::UnitVector3<f64>;
pub type CameraMatrix = Matrix2x3<f64>;

#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Pose {
    pub rotation: Rotation,
    pub translation: Point,
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
            translation: Point::origin(),
        }
    }
}
pub struct Plane {
    normal: Vector,
    d: f64,
}
impl Plane {
    pub fn new(normal: Vector, d: f64) -> Self {
        Self { normal, d }
    }
    pub fn from_origin_normal(origin: Point, normal: UVector) -> Self {
        // P - O = n * d, where d is the distance from the origin to the plane
        // the plane are all points where (P-O) dot n = 0
        // so we can write the plane as ax + by + cz = d or
        // so we can write the plane as a'x + b'y + c'z = 1
        // (x-x0)*n.x + (y-y0)*n.y + (z-z0)*n.z = 0
        // x*n.x + y*n.y + z*n.z = x0*n.x + y0*n.y + z0*n.z
        // a = n.x, b = n.y, c = n.z, d = O dot n
        let d = origin.coords.dot(&normal);
        Self {
            normal: normal.into_inner(),
            d,
        }
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
    pub fn normal(&self) -> Vector {
        self.normal
    }
    pub fn anchor(&self) -> Point {
        Point::origin() + self.normal.normalize() * self.d
    }
}
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
enum Intersection {
    Point(Point),
    Line(Line),
}

impl PartialEq for Plane {
    fn eq(&self, other: &Self) -> bool {
        self.normal / self.d == other.normal / other.d
    }
}

impl PartialEq for Line {
    fn eq(&self, other: &Self) -> bool {
        // line1 equation is P(t1) = O1 + t1 * D1
        // line2 equation is P(t2) = O2 + t2 * D2
        // if line1 and line2 have an intersection then O1 + t1 * D1 == O2 + t2 * D2
        // O2 - O1 == t1 * D1 - t2 * D2
        // A * [t1, t2] = B, where A = [D1, -D2] and B = O2-O1

        let a_mat =
            Matrix3x2::from_columns(&[self.direction.into_inner(), -other.direction.into_inner()]);
        let b = other.origin - self.origin;
        let t = a_mat;
        true
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

pub trait Projection {
    fn project(&self, point: &Point) -> Point2;
}

pub trait Distortion {
    fn distort(&self, point: &Point2) -> Point2;
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
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(test)]
    mod tests {
        use approx::assert_relative_eq;

        use super::*;

        #[test]
        fn test_pose_new() {
            let rotation = Rotation::from_axis_angle(&Vector::z_axis(), 0.5);
            let translation = Translation::from(Vector::new(1.0, 2.0, 3.0));
            let pose = Pose::new(rotation, translation);
            assert_eq!(pose.rotation, rotation);
            assert_eq!(pose.translation, translation);
        }

        #[test]
        fn test_pose_identity() {
            let pose = Pose::identity();
            assert_eq!(pose.rotation, Rotation::identity());
            assert_eq!(pose.translation, Point::origin());
        }

        #[test]
        fn test_plane_new() {
            let normal = Vector::new(1.0, 0.0, 0.0);
            let d = 1.0;
            let plane = Plane::new(normal, d);
            assert_eq!(plane.normal(), normal);
            assert_eq!(plane.anchor(), Point::new(d, 0.0, 0.0));
        }

        #[test]
        fn test_plane_from_origin_normal() {
            let origin = Point::new(1.0, 2.0, 3.0);
            let normal = UVector::new_normalize(Vector::new(1.0, 1.0, 1.0));
            let plane = Plane::from_origin_normal(origin, normal);
            assert_eq!(plane.normal(), normal.into_inner());
            assert_relative_eq!(plane.anchor(), Point::new(2., 2.0, 2.0));
        }

        #[test]
        fn test_plane_from_line_dir() {
            let line = Line::new(
                Point::new(1.0, 2.0, 3.0),
                UVector::new_normalize(Vector::new(1.0, 0.0, 0.0)),
            );
            let dir = UVector::new_normalize(Vector::new(0.0, 1.0, 0.0));
            let plane = Plane::from_line_dir(&line, &dir);
            assert_eq!(plane.normal(), Vector::new(0.0, 0.0, 1.0));
            assert_eq!(plane.anchor(), Point::new(0., 0., 3.0));
        }

        #[test]
        fn test_plane_from_line_point() {
            let line = Line::new(
                Point::new(1.0, 2.0, 3.0),
                UVector::new_normalize(Vector::new(1.0, 0.0, 0.0)),
            );
            let point = Point::new(1.0, 3.0, 3.0);
            let plane = Plane::from_line_point(&line, &point);
            assert_eq!(plane.normal(), Vector::new(0.0, 0.0, 1.0));
            assert_eq!(plane.anchor(), Point::new(0.0, 0.0, 3.0));
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
    }
}
