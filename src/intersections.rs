use crate::base::{Intersection, Line, Object, Plane, Ray};

pub fn intersect(a: Object, b: Object) -> Option<Intersection> {
    match (a, b) {
        (Object::Line(l), Object::Plane(p)) => intersect_line_plane(&l, &p),
        (Object::Plane(p), Object::Line(l)) => intersect_line_plane(&l, &p),
        (Object::Ray(r), Object::Plane(p)) => intersect_ray_plane(&r, &p),
        (Object::Plane(p), Object::Ray(r)) => intersect_ray_plane(&r, &p),
        _ => None,
    }
}

fn intersect_line_plane(l: &Line, p: &Plane) -> Option<Intersection> {
    let d = l.direction();
    // D*t + O1 = P(t)
    // (P(t) - O2) * N = 0
    // (D*t + O1 - O2) * N = 0
    // t = -(O1-O2)*N/(D*N)
    let dn = d.dot(&p.normal());
    let dori = (p.origin() - l.origin()).dot(&p.normal());
    if dn == 0.0 {
        // line is parallel to plane
        if dori == 0.0 {
            // line lies in plane
            Some(Intersection::Line(l.clone()))
        } else {
            // line lies outside plane
            None
        }
    } else {
        // line intersects plane
        let t = dori / dn;
        let p = l.origin() + l.direction().into_inner() * t;
        Some(Intersection::Point(p))
    }
}

fn intersect_ray_plane(ray: &Ray, plane: &Plane) -> Option<Intersection> {
    // D*t + OL = P(t)
    // (P(t) - OP) * N = 0
    // (D*t + OL - OP) * N = 0
    // t = -(OL-OP)*N/(D*N)
    let dir = ray.direction();
    let op = plane.origin();
    let ol = ray.origin();
    let pn = plane.normal();
    let dn = dir.dot(&pn);

    let don = (op - ol).dot(&pn);
    if don == 0.0 {
        if dn == 0.0 {
            // ray lies in plane
            Some(Intersection::Ray(ray.clone()))
        } else {
            Some(Intersection::Point(ol))
        }
    } else {
        if dn == 0.0 {
            // ray is parallel to plane
            None
        } else {
            let t = don / dn;
            if t > 0.0 {
                let p = ol + dir.into_inner() * t;
                Some(Intersection::Point(p))
            } else {
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::{Line, Plane, Point, Ray, UVector, Vector};

    #[test]
    fn test_intersect_line_plane() {
        let line = Line::new(
            Point::new(0.0, 0.0, 0.0),
            UVector::new_normalize(Vector::new(1.0, 0.0, 0.0)),
        );
        let plane = Plane::new(UVector::new_normalize(Vector::new(0.0, 1.0, 0.0)), 0.0);
        let intersection = intersect_line_plane(&line, &plane);
        assert!(matches!(intersection, Some(Intersection::Line(_line))));

        let line = Line::new(
            Point::new(0.0, 0.0, 0.0),
            UVector::new_normalize(Vector::new(0.0, 1.0, 0.0)),
        );
        let plane = Plane::new(UVector::new_normalize(Vector::new(0.0, 1.0, 0.0)), 0.0);
        let intersection = intersect_line_plane(&line, &plane);
        assert!(matches!(intersection, Some(Intersection::Point(_))));
    }

    #[test]
    fn test_intersect_ray_plane() {
        let ray = Ray::new(
            Point::new(0.0, 0.0, 0.0),
            UVector::new_normalize(Vector::new(1.0, 0.0, 0.0)),
        );
        let plane = Plane::new(UVector::new_normalize(Vector::new(1.0, 0.0, 0.0)), 0.0);
        let intersection = intersect_ray_plane(&ray, &plane);
        match intersection {
            Some(Intersection::Point(point)) => {
                assert_eq!(point, Point::new(0.0, 0.0, 0.0));
            }
            other => panic!(
                "Expected ray-plane intersection as a Point at origin, got other {:?}",
                other
            ),
        }
        // ray with
        let ray = Ray::new(
            Point::new(0.0, 0.0, 0.0),
            UVector::new_normalize(Vector::new(1.0, 1.0, 0.0)),
        );
        let plane = Plane::new(UVector::new_normalize(Vector::new(0.0, 0.0, 1.0)), 1.0);
        let intersection = intersect_ray_plane(&ray, &plane);
        assert!(matches!(intersection, None));

        // ray inside plane
        let ray = Ray::new(
            Point::new(0.0, 0.0, 0.0),
            UVector::new_normalize(Vector::new(1.0, 1.0, 0.0)),
        );
        let plane = Plane::new(UVector::new_normalize(Vector::new(0.0, 0.0, 1.0)), 0.0);
        let intersection = intersect_ray_plane(&ray, &plane);
        assert_eq!(intersection, Some(Intersection::Ray(ray)));
    }
}
