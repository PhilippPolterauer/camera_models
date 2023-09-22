use crate::base::{Line, Plane, Point, Ray, UVector, Vector, Object};
use nalgebra::{Matrix3x2};

// fn intersect_line_line(p: &Line, l: &Line)-> Option<Point>{
//     // check if two lines 
//     // t*D + O = s*E + P
//     // [D, -E]*[t;s] = P - O 
//     // 

//     let u = p.direction().as_ref();
//     let v = l.direction().as_ref();

//     let a = Matrix3x2::from_columns(&[u,v]);
//     let b = l.origin() - p.origin();

//     let decomp = a.lu();
//     let x = decomp.solve(&b).unwrap();
// }

// fn intersect_point(p: Point, other: Object) -> Option<Point> {
//     match other {
//         Object::Point(p2) => {
//             if p == p2 {
//                 Some(p)
//             } else {
//                 None
//             }
//         },
//         Object::Ray(ray) => {
//             let v =Vector::new(1.,1.,1.);
//             v.di

//             if ray.origin == p {
//                 Some(p)
//             } else {
//                 None
//             }
//         },
//         Object::Line(line) => {
//             if line.origin == p {
//                 Some(p)
//             } else {
//                 None
//             }
//         },
//         Object::LineSegment(line_segment) => {
//             if line_segment.start == p || line_segment.end == p {
//                 Some(p)
//             } else {
//                 None
//             }
//         },
//         Object::Plane(plane) => {
//             if plane.contains(&p) {
//                 Some(p)
//             } else {
//                 None
//             }
//         }
//     }
// }

// fn intersect_line(line: Line, other: Object) -> Option<Intersection> {
//     match other {
//         Object::Point(p2) => intersect_point(p2, line),
//         Object::Line(line) => {
//             if line.contains(line) {
//                 Some(Intersection::Line(line))
//             } else {
//                 None
//             }
//         },
//         Object::Plane(plane) => {
//             let d = line.origin.dot(&plane.normal);

//             // a line is defined as P = O + tD, where O is the origin and D is the direction

//             // x = x0 + t * dx
//             // y = y0 + t * dy
//             // z = z0 + t * dz

//             // the equivalen equation system is given by the following:

//             // plane.contains(line)
//             if d == 0.0 {
//                 // line is parallel to plane
//                 if plane.contains(&line) {
//                     // line lies in plane
//                     Some(Intersection::Line(line))
//                 } else {
//                     // line lies outside plane
//                     None
//                 }
//             } else {
//                 // line intersects plane
//                 let t = (plane.d - line.origin.dot(&plane.normal)) / d;
//                 let p = line.origin + t * line.direction;
//                 Some(Intersection::Point(p))
//             }
//         },
//         Object::Ray(ray2) => {
//             todo!("intersect line and ray")
//         },
//         Object::LineSegment(line_segment) => {
//             if line.contains(line_segment) {
//                 Some(Intersection::LineSegment(line_segment))
//             } else {
//                 None
//             }
//         }
//     }
// }

// fn intersect_ray(ray: Ray, other: Object) {
//     match other {
//         Object::Point(p2) => intersect_point(p2, ray),
//         Object::Line(line) => intersect_line(line, ray),
//         Object::Ray(ray2) => {
//             // if ray lies inside plane given by ray and ray2 then compute the intersection point and return it
//             let plane = Plane::from_line_point(&ray, &ray2.origin);

//             // let p = ray2.origin + ray2.direction * t;
//             // if ray.contains(ray2) {
//             //     //
//             //     Some(Intersection::Point(p))
//             // } else {
//             //     None
//             // }

//             // if ray lies outside plane given by ray and ray2 then return None
//         }
//         Object::Plane(plane) => intersect_plane(plane, Object::Ray(ray)),
//         Object::LineSegment(line_segment) => if line_segment.contains(ray.origin) {},
//     }
// }
// fn intersect_linesegment(line_segment: LineSegment, other: Object) -> Option<Intersection> {
//     match other {
//         Object::Point(p2) => intersect_point(p2, Object::LineSegment(line_segment)),
//         Object::Line(line) => intersect_line(line, Object::LineSegment(line_segment)),
//         Object::Ray(ray) => intersect_ray(ray, Object::LineSegment(line_segment)),
//         Object::Plane(plane) => intersect_plane(plane, Object::LineSegment(line_segment)),
//         Object::LineSegment(line_segment2) => {
//             if line_segment.contains(line_segment2) {
//                 Some(Intersection::LineSegment(line_segment2))
//             } else {
//                 None
//             }
//         }
//     }
// }

// fn intersect_plane(plane: Plane, other: Object) -> Option<Intersection> {
//     match other {
//         Object::Point(p2) => intersect_point(p2, Object::Plane(plane)),
//         Object::Line(l) => intersect_line(l, Object::Plane(plane)),
//         Object::Ray(ray) => intersect_ray(ray, Object::Plane(plane)),
//         Object::Plane(plane2) => intersect_plane(plane2, Object::Plane(plane)),
//         Object::LineSegment(line_segment) => {
//             intersect_linesegment(line_segment, Object::Plane(plane))
//         }
//     }
// }

// fn intersect(obj1: Object, obj2: Object) -> Option<Intersection> {
//     match obj1 {
//         Object::Point(p) => intersect_point(p, obj2),
//         Object::Line(l) => intersect_line(l, obj2),
//         Object::Ray(ray) => intersect_ray(ray, obj2),
//         Object::Plane(plane) => intersect_plane(plane, obj2),
//         Object::LineSegment(line_segment) => intersect_linesegment(line_segment, obj2),
//     }
// }
