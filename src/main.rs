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

    let relative_vel = (body_b.vel - body_a.vel);
    // according to documentation, .perp() rotates the vector clockwise by 90 degrees
    let tangent = contact.normal.perp();

    // tangent velocity
    let v_t = relative_vel.dot(tangent);

    // relative velocity along the normal
    // TODO: add angular velocity to the calculation
    let v_n = relative_vel.dot(contact.normal);

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

    // this is the effective mass for the friction calculation
    // here we dot multiply with tangent vector instead of the normal vector
    let k_t = body_a.inverse_mass + body_b.inverse_mass;

    // magnitude of the impulse
    // if the relative velocity is greater than zero, the bodies are already
    // moving apart
    let restitution = body_a.restitution * body_b.restitution;
    let p_n = f32::max(((1.0 + restitution) * (-v_n + bias_vel)) / k_n, 0.0);

    // friction impulse
    let actual_mu = body_a.mu * body_b.mu;
    let p_t = f32::clamp(-v_t / k_t, -actual_mu * p_n, actual_mu * p_n);

    let p_friction = p_t * tangent;
    let p = p_n * contact.normal;

    if !body_a.is_static {
        body_a.apply_impulse(-p_friction);
        body_a.apply_impulse(-p);
    }
    if !body_b.is_static {
        body_b.apply_impulse(p_friction);
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
    let col0 = Collider::Circle {
        offset: vec2(0.0, 0.0),
        radius: 3.0,
    };
    let mut rg0 = RigidBody2DBuilder::new()
        .with_shape(col0.clone())
        .with_position(vec2(200.0, 10.0))
        .with_restitution(1.0)
        .with_inverse_mass(0.00000000001)
        .with_vel(vec2(-45.0, 0.0))
        .build();
    let obj0 = ObjectBuilder::new()
        .with_body(rg0)
        .with_collider(col0)
        .with_color(YELLOW)
        .with_name("circle".to_string())
        .build();

    // circle
    let col1 = Collider::Circle {
        offset: vec2(0.0, 0.0),
        radius: 0.5,
    };
    let mut rg1 = RigidBody2DBuilder::new()
        .with_shape(col1.clone())
        .with_position(vec2(10.0, 10.0))
        .with_restitution(1.0)
        .with_inverse_mass(1.0)
        .with_vel(vec2(-10.0, 0.0))
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
        .with_position(vec2(-50.0, 0.0))
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
        .with_inverse_mass(1.0 / 300000000000.0)
        .build();

    let obj3 = ObjectBuilder::new()
        .with_body(rg3)
        .with_collider(col3)
        .with_color(GREEN)
        .with_name("some_rect".to_string())
        .build();

    let mut objects = [obj0, obj1, obj2, obj3];
    let mut camera = Camera::default();

    loop {
        // handle camera input and movement
        handle_camera_movement(&mut camera);
        draw_zoom_ui(camera.zoom);

        clear_background(WHITE);
        let dt = get_frame_time();

        // apply_gravity
        apply_gravity(&mut objects);
        let iterations = 10; // the accuracy increases with the number of iterations
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
