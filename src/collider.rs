use crate::Contact;
use crate::rigid_body::*;
use macroquad::prelude::*;

#[derive(Clone)]
pub enum Collider {
    Circle { offset: Vec2, radius: f32 },
    AABB { min: Vec2, max: Vec2 },
}

/// returns the point on the aabb surface that is nearest to the given point
fn point_aabb_nearest_point(point: Vec2, aabb: &Collider, body: &RigidBody2D) -> Vec2 {
    if let Collider::AABB { min, max } = aabb {
        let world_min = body.position + *min;
        let world_max = body.position + *max;
        let mut nearest_point = vec2(0.0, 0.0);

        if point.x <= world_max.x && point.x >= world_min.x {
            nearest_point.x = point.x
        } else if point.x < world_min.x {
            nearest_point.x = world_min.x;
        } else if point.x > world_max.x {
            nearest_point.x = world_max.x;
        } else if point.x - world_min.x > world_max.x - point.x {
            nearest_point.x = world_max.x;
        } else {
            nearest_point.x = world_min.x
        }

        if point.y <= world_max.y && point.y >= world_min.y {
            nearest_point.y = point.y
        } else if point.y < world_min.y {
            nearest_point.y = world_min.y;
        } else if point.y > world_max.y {
            nearest_point.y = world_max.y;
        } else if point.y - world_min.y > world_max.y - point.y {
            nearest_point.y = world_max.y;
        } else {
            nearest_point.y = world_min.y
        }

        nearest_point
    } else {
        panic!();
    }
}

fn is_close_to_zero(vector: Vec2) -> bool {
    approx::abs_diff_eq!(vector.x, 0.0) && approx::abs_diff_eq!(vector.y, 0.0)
}
fn test_aabb_circle(
    aabb: &Collider,
    circle: &Collider,
    aabb_body: &RigidBody2D,
    circle_body: &RigidBody2D,
    aabb_index: usize,
    circle_index: usize,
) -> Option<Contact> {
    let con = test_circle_aabb(
        circle,
        aabb,
        circle_body,
        aabb_body,
        aabb_index,
        circle_index,
    );
    let Some(mut contact) = con else {
        return None;
    };
    contact.normal *= -1.0;
    Some(contact)
}

fn test_circle_aabb(
    circle: &Collider,
    aabb: &Collider,
    circle_body: &RigidBody2D,
    aabb_body: &RigidBody2D,
    circle_index: usize,
    aabb_index: usize,
) -> Option<Contact> {
    match (circle, aabb) {
        (Collider::Circle { radius, .. }, Collider::AABB { min, max }) => {
            // Get the circle's world position.
            let circle_world_pos = circle.world_circle(circle_body.position).unwrap();
            // Compute the squared distance from the circle's center to the AABB.
            let nearest_point_to_center =
                point_aabb_nearest_point(circle_world_pos, aabb, aabb_body);

            let dist = Vec2::distance(nearest_point_to_center, circle_world_pos);
            let collision_vector = nearest_point_to_center - circle_world_pos;

            let world_min = *min + aabb_body.position;
            let world_max = *max + aabb_body.position;

            let mut normal = collision_vector;
            if is_close_to_zero(normal) {
                let distance_left = (circle_world_pos.x - world_min.x).abs();
                let distance_right = (circle_world_pos.x - world_max.x).abs();
                let distance_bottom = (circle_world_pos.y - world_min.y).abs();
                let distance_top = (circle_world_pos.y - world_max.y).abs();

                let min_distance = f32::min(
                    distance_left,
                    f32::min(distance_right, f32::min(distance_bottom, distance_top)),
                );

                if min_distance == distance_left {
                    normal = -Vec2::X;
                }
                if min_distance == distance_right {
                    normal = Vec2::X;
                }
                if min_distance == distance_bottom {
                    normal = -Vec2::Y;
                }
                if min_distance == distance_top {
                    normal = Vec2::Y;
                }
            }

            normal = normal.normalize();

            // if a collision has occured, compute how it actually happened
            if dist < *radius {
                Some(Contact {
                    point: nearest_point_to_center,
                    pen_depth: *radius - dist,
                    normal,
                    body_a_index: circle_index,
                    body_b_index: aabb_index,
                })
            } else {
                None
            }
        }
        _ => None,
    }
}

impl Collider {
    // transform the position from local collider coordinates to world coodinates (relative to some body)
    pub fn world_aabb(&self, body_pos: Vec2) -> Option<(Vec2, Vec2)> {
        match self {
            Collider::AABB { min, max } => Some((body_pos + *min, body_pos + *max)),
            _ => None,
        }
    }

    pub fn world_circle(&self, owner_pos: Vec2) -> Option<Vec2> {
        match self {
            Collider::Circle { offset, .. } => Some(owner_pos + *offset),
            _ => None,
        }
    }

    pub fn collides_with(
        &self, // collider_a
        body_a: &RigidBody2D,
        body_b: &RigidBody2D,
        collider_b: &Collider,
        body_a_index: usize,
        body_b_index: usize,
    ) -> Option<Contact> {
        match (self, collider_b) {
            // two circles collide if the squared distance between them is smaller than the sum of their squared radii
            (
                Collider::Circle {
                    radius: radius_a, ..
                },
                Collider::Circle {
                    radius: radius_b, ..
                },
            ) => {
                let pos_a = self.world_circle(body_a.position).unwrap();
                let pos_b = collider_b.world_circle(body_b.position).unwrap();
                let position_difference = pos_b - pos_a;

                // this can be used to calculate the distance
                let dist = pos_a.distance(pos_b);

                if dist < radius_a + radius_b {
                    let normal = position_difference / dist;

                    // compute the middle_point between the surfaces of circles
                    let surface_a = pos_a + normal * *radius_a;
                    let surface_b = pos_b - normal * *radius_b;
                    let point = (surface_a + surface_b) * 0.5;

                    Some(Contact {
                        point,
                        normal,
                        pen_depth: radius_a + radius_b - dist,
                        body_a_index,
                        body_b_index,
                    })
                } else {
                    None
                }
            }

            // a circle collides with an aabb if the
            // distance from the center of the circle
            // to the closest point on the aabb is
            // smaller than the radius of the circle
            (Collider::AABB { .. }, Collider::Circle { .. }) => {
                test_aabb_circle(self, collider_b, body_a, body_b, body_a_index, body_b_index)
            }

            (Collider::Circle { .. }, Collider::AABB { .. }) => {
                test_circle_aabb(self, collider_b, body_a, body_b, body_a_index, body_b_index)
            }

            (Collider::AABB { .. }, Collider::AABB { .. }) => {
                let min_max_a = self.world_aabb(body_a.position).unwrap();
                let min_max_b = collider_b.world_aabb(body_b.position).unwrap();

                let min_a = min_max_a.0;
                let max_a = min_max_a.1;

                let min_b = min_max_b.0;
                let max_b = min_max_b.1;

                let is_colliding = max_a.x >= min_b.x
                    && max_b.x >= min_a.x
                    && max_a.y >= min_b.y
                    && max_b.y >= min_a.y;

                let overlap_min = vec2(min_a.x.max(min_b.x), min_a.y.max(min_b.y));
                let overlap_max = vec2(max_a.x.min(max_b.x), max_a.y.min(max_b.y));

                let contact_point = (overlap_min + overlap_max) * 0.5;

                if is_colliding {
                    let x_overlap = f32::min(max_a.x, max_b.x) - f32::max(min_a.x, min_b.x);

                    // let x_overlap =
                    //     f32::min(max_a.x.max(max_b.x), max_b.x) - f32::max(min_a.x, min_b.x);
                    let y_overlap = f32::min(max_a.y, max_b.y) - f32::max(min_a.y, min_b.y);

                    let depth = f32::min(x_overlap, y_overlap);
                    // if the penetration depth is negative, then there is no penetration, so there is no collision
                    if depth < 0.0 {
                        return None;
                    }

                    let mut normal: Vec2;
                    if x_overlap > y_overlap {
                        normal = Vec2::Y;
                        let top_penetration = max_a.y - min_b.y;
                        let bottom_penetration = max_b.y - min_a.y;

                        // the object needs to be pushed upwards
                        // because the upper penetration is smaller
                        if bottom_penetration < top_penetration {
                            normal *= -1.0;
                        }
                    } else {
                        normal = Vec2::X;
                        let left_penetration = max_a.x - min_b.x;
                        let right_penetration = max_b.x - min_a.x;

                        // the object needs to be pushed to the left
                        // because the left penetration is smaller
                        if left_penetration < right_penetration {
                            normal *= -1.0;
                        }
                    }

                    return Some(Contact {
                        pen_depth: depth,
                        point: contact_point,
                        normal,
                        body_a_index,
                        body_b_index,
                    });
                }
                None
            }
        }
    }
}
