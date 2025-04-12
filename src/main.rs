mod camera;
mod collider;
mod object;
mod rigid_body;

use approx; // For the macro assert_relative_eq!
use core::panic;

use camera::Camera;
use collider::*;
use macroquad::prelude::*;
use macroquad::ui::root_ui;
use object::*;
use rigid_body::*;

// returns the point on the aabb surface that is nearest to the given point
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

// https://www.r-5.org/files/books/computers/algo-list/realtime-3d/Christer_Ericson-Real-Time_Collision_Detection-EN.pdf
fn sq_dist_point_aabb(point: Vec2, aabb: &Collider, body: &RigidBody2D) -> f32 {
    if let Collider::AABB { min, max } = aabb {
        let world_min = body.position + *min;
        let world_max = body.position + *max;
        let mut sq_dist: f32 = 0.0;

        let v = point.x;
        if v < world_min.x {
            sq_dist += (world_min.x - v) * (world_min.x - v);
        }
        if v > world_max.x {
            sq_dist += (v - world_max.x) * (v - world_max.x);
        }

        let v = point.y;
        if v < world_min.y {
            sq_dist += (world_min.y - v) * (world_min.y - v);
        }
        if v > world_max.y {
            sq_dist += (v - world_max.y) * (v - world_max.y);
        }

        sq_dist
    } else {
        panic!("sq_dist_aabb called on non-AABB collider");
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

fn draw_zoom_ui(zoom: Vec2) {
    root_ui().label(None, &format!("Zoom: {:.2} x {:.2}", zoom.x, zoom.y));
}

fn draw_spawn_ui() {
    root_ui().label(None, &format!("Spawn Menu: "));
}

fn handle_camera_movement(camera: &mut Camera) {
    if is_key_down(KeyCode::Z) {
        camera.zoom_in();
    }
    if is_key_down(KeyCode::X) {
        camera.zoom_out();
    }
    if is_key_down(KeyCode::A) {
        camera.pos += -Vec2::X
    }

    if is_key_down(KeyCode::D) {
        camera.pos += Vec2::X
    }

    if is_key_down(KeyCode::W) {
        camera.pos += Vec2::Y
    }
    if is_key_down(KeyCode::S) {
        camera.pos += -Vec2::Y
    }
}

fn gravity_acceleration() -> Vec2 {
    vec2(0.0, -9.81)
}

#[derive(Debug)]
struct Contact {
    point: Vec2,  // point of contact
    normal: Vec2, // from body_a's point of view

    pen_depth: f32, // how deep body_a is inside of body_b

    body_a_index: usize,
    body_b_index: usize,
}

fn resolve_interpenetration(objects: &mut [Object], contact: &Contact, dt: f32) {
    let (l, r) = objects.split_at_mut(contact.body_b_index);
    let body_a = l[contact.body_a_index].body.as_mut().unwrap();
    let body_b = r[0].body.as_mut().unwrap();

    // relative velocity along the normal
    // TODO: add angular velocity to the calculation
    let v_n = (body_b.vel - body_a.vel).dot(contact.normal);

    // slop is there to reduce jittering
    let slop = 0.01; // allow for 1 cm of slop

    // this makes it so that the bodies don't drastically move apart but are rather gently moved
    // apart each frame
    let bias_factor = 0.2;
    let bias_vel = (bias_factor / dt) * f32::max(0.0, contact.pen_depth - slop);

    // TODO: add inertia tensor
    // NOTE:
    // this is quasi the effective mass
    let k_n = body_a.inverse_mass + body_b.inverse_mass;

    // magnitude of the impulse
    // if the relative velocity is greater than zero, the bodies are already
    // moving apart
    let restitution = body_a.restitution * body_b.restitution;
    let p_n = f32::max(((1.0 + restitution) * (-v_n + bias_vel)) / k_n, 0.0);
    let p = p_n * contact.normal;

    if !body_a.is_static {
        body_a.apply_impulse(-p);
    }
    if !body_b.is_static {
        body_b.apply_impulse(p);
    }
}

fn check_collision(objects: &[Object]) -> Vec<Contact> {
    let mut contacts = vec![];
    for i in 0..objects.len() {
        // this makes it so you can access two disjunct parts of the array at once
        let (left, right) = objects.split_at(i + 1);
        let a = &left[i];
        for (j, b) in right.iter().enumerate() {
            let b_index = i + 1 + j;
            let (Some(collider_a), Some(body_a)) = (&a.collider, &a.body) else {
                continue;
            };
            let (Some(collider_b), Some(body_b)) = (&b.collider, &b.body) else {
                continue;
            };

            if let Some(contact) = collider_a.collides_with(body_a, body_b, collider_b, i, b_index)
            {
                contacts.push(contact);
            }
        }
    }
    contacts
}

// TODO: delete later
fn apply_gravity(objects: &mut [Object]) {
    for object in objects.iter_mut() {
        let (Some(_), Some(body)) = (&object.collider, &mut object.body) else {
            continue;
        };

        body.apply_force(gravity_acceleration() / body.inverse_mass);
    }
}

#[macroquad::main("Physixx")]
async fn main() {
    // circle
    let col1 = Collider::Circle {
        offset: vec2(0.0, 0.0),
        radius: 3.0,
    };
    let mut rg1 = RigidBody2DBuilder::new()
        .with_shape(col1.clone())
        .with_position(vec2(10.0, 10.0))
        .with_restitution(1.0)
        .with_inverse_mass(1.0)
        .build();
    let obj1 = ObjectBuilder::new()
        .with_body(rg1)
        .with_collider(col1)
        .with_color(YELLOW)
        .with_name("circle".to_string())
        .build();

    let col2 = Collider::AABB {
        min: vec2(0.0, -10.0),
        max: vec2(200.0, 0.0),
    };
    let mut rg2 = RigidBody2DBuilder::new()
        .make_static()
        .with_position(Vec2::ZERO)
        .with_shape(col2.clone())
        .with_restitution(0.3)
        .build();
    let obj2 = ObjectBuilder::new()
        .with_body(rg2)
        .with_collider(col2)
        .with_color(PINK)
        .with_name("floor".to_string())
        .build();

    // Rectangle 3
    let col3 = Collider::AABB {
        min: vec2(0.0, -10.0),
        max: vec2(20.0, 0.0),
    };
    let mut rg3 = RigidBody2DBuilder::new()
        .with_shape(col3.clone())
        .with_position(vec2(-30.0, 10.0))
        .with_inverse_mass(1.0 / 300.0)
        .build();

    let obj3 = ObjectBuilder::new()
        .with_body(rg3)
        .with_collider(col3)
        .with_color(GREEN)
        .with_name("some_rect".to_string())
        .build();

    let mut objects = [obj1, obj2, obj3];
    let mut camera = Camera::default();

    loop {
        // handle camera input and movement
        handle_camera_movement(&mut camera);
        draw_zoom_ui(camera.zoom);

        clear_background(WHITE);
        let dt = 1. / 60.;

        // apply_gravity
        apply_gravity(&mut objects);
        let iterations = 15; // the accuracy increases with the number of iterations
        for _ in 0..iterations {
            let contacts = check_collision(&mut objects);
            for contact in contacts {
                let screen_point = camera.world_to_screen(contact.point);
                draw_circle_lines(screen_point.x, screen_point.y, 1.0, 1.0, BLACK);
                let screen_point = camera.world_to_screen(contact.point);
                let normal = vec2(contact.normal.x, -contact.normal.y); // flip Y
                let normal_end = screen_point + normal * 10.0; // scale for visibility

                draw_circle_lines(screen_point.x, screen_point.y, 2.0, 1.0, BLACK);

                draw_line(
                    screen_point.x,
                    screen_point.y,
                    normal_end.x,
                    normal_end.y,
                    1.0,
                    RED,
                );
                resolve_interpenetration(&mut objects, &contact, dt);
            }
        }
        for object in objects.as_mut() {
            object.body.as_mut().unwrap().update(dt);
            object.draw(&camera);
        }

        next_frame().await;
    }
}
