use crate::aabb::*;
use math::{Point3, Vec3};

#[derive(Debug, PartialEq)]
pub enum SphereAABBOverlap {
    SubsetOfAABB,   // sphere being subset of AABB.
    IntersectsAABB, // sphere intersecting AABB.
    ContainsAABB,   // sphere fully containing AABB.
    Disjoint,       // sphere is completely disjoint from AABB.
}

impl AABB {
    pub fn sphere_intersect(
        &self,
        origin: Point3,
        radius: f32,
        assume_intersects: bool,
    ) -> SphereAABBOverlap {
        let r2 = radius * radius;

        let size = self.size();

        let diagonal_radius_squared = size.norm_squared();
        let aabb_center = self.center();

        // basic test, ||C - O||^2 < r^2 + dr^2 implies that they (the AABB and sphere) could intersect, the opposite implies they definitely don't intersect.
        let displacement = aabb_center - origin;
        let diff = displacement.norm_squared();

        if diff > diagonal_radius_squared + r2 {
            // they definitely don't intersect
            return SphereAABBOverlap::Disjoint;
        } else if diff + diagonal_radius_squared < r2 {
            // sphere is larger and definitely encloses AABB
            return SphereAABBOverlap::ContainsAABB;
        }

        let mut any_overlap = false;
        let mut any_not_overlap = false;
        for point in self.get_points().iter() {
            if (*point - origin).norm_squared() < r2 {
                // sphere contains one of the corners of the box.
                any_overlap = true;
            } else {
                // sphere does not contain one of the corners of the box.
                any_not_overlap = true;
            }
        }
        let all_overlap = !any_not_overlap;
        if all_overlap {
            // this is probably redundant considering the above check using spheres and diagonals but whatever
            return SphereAABBOverlap::ContainsAABB;
        } else if any_overlap || assume_intersects {
            return SphereAABBOverlap::IntersectsAABB;
        } else {
            // as of this point in the algorithm's flow, none of the AABB's corners were contained within the sphere.
            // thus we need to check against each plane.

            // exists: P in plane such that ||P - O||^2 < r^2
            // P in plane can be parameterized as P0 + (S - S * N) * v
            // ||P0 + (S - S * N) * v - O||^2 < r^2
            // reduces to a system of equations with 2 variables
            // sps N = X_bar
            // then
            // (P0.x + S.x * u - O.x) ^ 2 + (P0.y + S.y * v - O.y) ^ 2 + C = r^2
            // where C = (P0.z + S.z * flip - O.z) ^2
            // effectively the same as subbing all r^2 for r^2 - C
            // redefine R = sqrt(r^2 - (S.z - O.z)^2)
            // AND
            // 0 <= u <= 1, 0 <= v <= 1
            // let Tx = O.x - P0.x, Sx = S.x
            // ditto
            // (Sx*u - Tx)^2 + (Sy*v - Ty)^2 < r^2
            // (Sx*u - Tx)^2 < r^2 - (Sy*v - Ty)^2
            // Sx*u - Tx < sqrt(r^2 - (Sy*v - Ty)^2)
            // Sx*u < Tx + sqrt(r^2 - (Sy*v - Ty)^2)
            // u = (Tx + sqrt(r^2 - (Sy*v - Ty)^2)) / Sx
            // but u and v are between 0 and 1
            // u = (Tx + sqrt(r^2 - (Sy*v - Ty)^2)) / Sx, 0 <= v <= 1
            // u only exists when (Sy*v-Ty)^2 < r^2
            // (Sy*v-Ty)^2 < r^2
            // -(Sy*v-Ty) < r
            // (Ty - r) / Sy < v < (Ty + r) / Sy
            // implying that the only valid interval for v is [(Ty - r) / Sy, (Ty + r) / Sy] intersected with [0, 1]

            let mut any_intersections = false;

            'outer: for flip in [0, 1].iter() {
                for n_axis in [0, 1, 2].iter() {
                    let axis = match n_axis {
                        0 => Vec3::X,
                        1 => Vec3::Y,
                        _ => Vec3::Z,
                    };
                    let scaled_normal = Vec3::from_raw(axis.0 * size.0);
                    let plane_origin = self.min + (*flip as f32) * scaled_normal;
                    let (s_u, s_v) = match n_axis {
                        0 => (size.y(), size.z()),
                        1 => (size.x(), size.z()),
                        _ => (size.x(), size.y()),
                    };
                    let (t_u, t_v) = match n_axis {
                        0 => (origin.y() - plane_origin.y(), origin.z() - plane_origin.z()),
                        1 => (origin.x() - plane_origin.x(), origin.z() - plane_origin.z()),
                        _ => (origin.x() - plane_origin.x(), origin.y() - plane_origin.y()),
                    };

                    let c = ((origin.0 + size.0 * (*flip as f32) - plane_origin.0) * axis.0).sum();
                    if c * c > r2 {
                        // plane too far.
                        // no intersection possible on this plane in this case.
                        continue;
                    }
                    let new_r = (r2 - c * c).sqrt();

                    let (min_v, max_v) = (
                        ((t_v - new_r) / s_v).max(0.0),
                        ((t_v + new_r) / s_v).min(1.0),
                    );
                    // let center_v = t_v / s_v;
                    // let (min_v_dist_from_center, max_v_dist_from_center) =
                    //     ((min_v - center_v).abs(), (max_v - center_v).abs());

                    let (min_u, max_u) = (
                        ((t_u - new_r) / s_u).max(0.0),
                        ((t_u + new_r) / s_u).min(1.0),
                    );
                    // let center_u = t_u / s_u;
                    // let (min_u_dist_from_center, max_u_dist_from_center) =
                    //     ((min_u - center_u).abs(), (max_u - center_u).abs());
                    if max_v - min_v == 0.0 || max_u - min_u == 0.0 {
                        // interval is empty, no in-bounds intersection on this plane.
                        continue;
                    } else {
                        // found intersection

                        // maybe compute how far in the sphere penetrates, to determine which quadrants are "dirty"
                        // let (u, v) = if min_u_dist_from_center < max_u_dist_from_center {
                        //     if min_v_dist_from_center < max_v_dist_from_center {
                        //         (min_u, min_v)
                        //     } else {
                        //         (min_u, max_v)
                        //     }
                        // } else {
                        //     if min_v_dist_from_center < max_v_dist_from_center {
                        //         (max_u, min_v)
                        //     } else {
                        //         (max_u, max_v)
                        //     }
                        // };
                        any_intersections = true;
                        break 'outer;
                    }
                }
            }

            if any_intersections {
                SphereAABBOverlap::IntersectsAABB
            } else {
                if self.contains(origin) {
                    SphereAABBOverlap::SubsetOfAABB
                } else {
                    SphereAABBOverlap::Disjoint
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_intersection() {
        let aabb = AABB::new(Point3::new(3.0, 2.0, 1.0), Point3::new(5.0, 7.0, 3.0));

        let sphere_center = Point3::new(2.0, 1.0, 2.0);
        let sphere_radius = 3.0;

        let res = aabb.sphere_intersect(sphere_center, sphere_radius, false);
        println!("{:?}", res);
        assert_eq!(res, SphereAABBOverlap::IntersectsAABB);
    }
    #[test]
    fn test_intersection1() {
        let aabb = AABB::new(Point3::new(3.0, 2.0, 1.0), Point3::new(5.0, 7.0, 3.0));

        let sphere_center = Point3::new(2.0, 0.0, 0.0);
        let sphere_radius = 9.0;

        let res = aabb.sphere_intersect(sphere_center, sphere_radius, false);

        println!("{:?}", res);
        assert_eq!(res, SphereAABBOverlap::ContainsAABB);
    }
    #[test]
    fn test_intersection2() {
        let aabb = AABB::new(Point3::new(3.0, 2.0, 1.0), Point3::new(5.0, 7.0, 3.0));

        let sphere_center = Point3::new(4.0, 4.0, 2.0);
        let sphere_radius = 0.8;

        let res = aabb.sphere_intersect(sphere_center, sphere_radius, false);
        println!("{:?}", res);
        assert_eq!(res, SphereAABBOverlap::SubsetOfAABB);
    }
    #[test]
    fn test_intersection3() {
        let aabb = AABB::new(Point3::new(3.0, 2.0, 1.0), Point3::new(5.0, 7.0, 3.0));

        let sphere_center = Point3::new(2.0, 4.0, 1.0);
        let sphere_radius = 1.3;

        let res = aabb.sphere_intersect(sphere_center, sphere_radius, false);
        println!("{:?}", res);
        assert_eq!(res, SphereAABBOverlap::IntersectsAABB);
    }
    #[test]
    fn test_intersection4() {
        let aabb = AABB::new(Point3::new(3.0, 2.0, 1.0), Point3::new(5.0, 7.0, 3.0));

        let sphere_center = Point3::new(10.0, 0.0, 0.0);
        let sphere_radius = 5.0;

        let res = aabb.sphere_intersect(sphere_center, sphere_radius, false);
        println!("{:?}", res);
        assert_eq!(res, SphereAABBOverlap::Disjoint);
    }
}
