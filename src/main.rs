use approx; // For the macro assert_relative_eq!
use core::panic;
use macroquad::{prelude::*, ui::root_ui};

struct Camera {
    screen_dims: Vec2,
    pos: Vec2,
    zoom: Vec2,
    // the factor to multiply with/divide by when performing a zoom/unzoom operation
    zoom_factor: f32,
}

struct ObjectBuilder {
    body: Option<RigidBody2D>,
    collider: Option<Collider>,
    color: Option<Color>,
    name: Option<String>,
}

impl ObjectBuilder {
    fn new() -> Self {
        Self {
            body: None,
            collider: None,
            color: None,
            name: None,
        }
    }

    fn with_body(mut self, body: RigidBody2D) -> Self {
        self.body = Some(body);
        self
    }

    fn with_collider(mut self, collider: Collider) -> Self {
        self.collider = Some(collider);
        self
    }

    fn with_color(mut self, color: Color) -> Self {
        self.color = Some(color);
        self
    }

    fn with_name(mut self, name: String) -> Self {
        self.name = Some(name);
        self
    }

    fn build(self) -> Object {
        let color = self.color.expect("Expected the user to pass a color");
        let name = self.name.unwrap_or_else(|| "some_object".to_string());
        Object {
            body: self.body,
            collider: self.collider,
            color: color,
            name: name,
        }
    }
}

impl Camera {
    fn zoom_in(&mut self) {
        self.zoom.x *= self.zoom_factor;
        self.zoom.y *= self.zoom_factor;
    }

    fn zoom_out(&mut self) {
        self.zoom.x /= self.zoom_factor;
        self.zoom.y /= self.zoom_factor;
    }

    fn screen_middle(&self) -> Vec2 {
        Vec2 {
            x: self.screen_dims.x / 2.0,
            y: self.screen_dims.y / 2.0,
        }
    }

    fn world_to_screen(&self, world_pos: Vec2) -> Vec2 {
        /*
        get the relative position of
        the object in respect to the
        position the camera is looking at
        and apply the zoom so that the new coordinate is correct.

        Formula: screen_pos = (obj_pos - cam_pos) * zoom + screen_center
        */
        let relative_position: Vec2 = (world_pos - self.pos) * self.zoom;

        self.screen_middle() + relative_position
    }

    /// Converts the position on the screen to the position in the world using the camera parameters
    fn screen_to_world(&self, screen_pos: Vec2) -> Vec2 {
        /*
        get the relative position of the object in respect to the screen center
        unapply the zoom of the camera.

        Formula: world_pos = (screen_pos - screen_center) / zoom + cam_pos
        */
        let relative_position: Vec2 = (screen_pos - self.screen_middle()) / self.zoom;

        self.pos + relative_position
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            pos: Vec2::ZERO,
            zoom: vec2(24.0, -24.0),
            zoom_factor: 1.1,
            screen_dims: vec2(screen_width(), screen_height()),
        }
    }
}

struct RigidBody2D {
    position: Vec2,
    acc: Vec2,
    vel: Vec2,
    // accumulated force
    accum_force: Vec2,
    inverse_mass: f32,
    is_static: bool,
}

impl RigidBody2D {
    fn apply_force(&mut self, force: Vec2) {
        self.accum_force += force;
    }

    fn apply_impulse(&mut self, impulse: Vec2) {
        self.vel += impulse * self.inverse_mass;
    }

    fn update(&mut self, dt: f32) {
        if self.inverse_mass == 0.0 || self.is_static {
            return;
        }
        let new_pos = self.position + self.vel * dt + self.acc * (dt * dt * 0.5);
        let new_acc = self.accum_force * self.inverse_mass;
        let new_vel = self.vel + (self.acc + new_acc) * (dt * 0.5);

        self.position = new_pos;
        self.vel = new_vel;
        self.acc = new_acc;

        // reset the accumulated forces after update
        self.accum_force = Vec2::ZERO;
    }
}

enum Collider {
    Circle { offset: Vec2, radius: f32 },
    AABB { min: Vec2, max: Vec2 },
}

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
    // Ensure the first collider is an AABB and the second is a Circle.
    if let (Collider::AABB { min, max }, Collider::Circle { radius, .. }) = (aabb, circle) {
        // Compute the world-space boundaries of the AABB.
        let world_min = aabb_body.position + *min;
        let world_max = aabb_body.position + *max;
        // Obtain the circle's world position.
        let circle_world_pos = circle.world_circle(circle_body.position).unwrap();

        // Check if the circle's center lies inside the AABB.
        let is_inside = circle_world_pos.x >= world_min.x
            && circle_world_pos.x <= world_max.x
            && circle_world_pos.y >= world_min.y
            && circle_world_pos.y <= world_max.y;

        // Depending on whether the center is inside, choose how to compute the collision.
        let (normal, penetration, contact_point) = if is_inside {
            // Compute distances from the circle's center to each AABB edge.
            let dist_left = circle_world_pos.x - world_min.x;
            let dist_right = world_max.x - circle_world_pos.x;
            let dist_bottom = circle_world_pos.y - world_min.y;
            let dist_top = world_max.y - circle_world_pos.y;

            // Pick the smallest distance to determine the nearest edge.
            let min_distance = dist_left.min(dist_right).min(dist_bottom.min(dist_top));
            // From the AABB perspective, choose the normal such that it points outward.
            // (Notice the signs are reversed compared to the circle-first case.)
            let normal = if min_distance == dist_left {
                Vec2::X // circle is closest to left edge; push rightward
            } else if min_distance == dist_right {
                -Vec2::X // closest to right; push leftward
            } else if min_distance == dist_bottom {
                Vec2::Y // closest to bottom; push upward
            } else {
                -Vec2::Y // closest to top; push downward
            };
            // Penetration is how far the circle's radius exceeds the minimal distance.
            let penetration = *radius - min_distance;
            // Place the contact point at the circle's perimeter along the collision normal.
            let contact_point = circle_world_pos - normal * (*radius);
            (normal, penetration, contact_point)
        } else {
            // If the circle center is outside the AABB, use clamping.
            let nearest_point = point_aabb_nearest_point(circle_world_pos, aabb, aabb_body);
            let collision_vector = circle_world_pos - nearest_point;
            let dist = collision_vector.length();
            let normal = if dist == 0.0 {
                // Fallback in case of an unlikely zero-length vector.
                Vec2::X
            } else {
                collision_vector / dist
            };
            let penetration = *radius - dist;
            (normal, penetration, nearest_point)
        };

        // If there is penetration, a collision contact is detected.
        if penetration > 0.0 {
            return Some(Contact {
                point: contact_point,
                pen_depth: penetration,
                normal: normal.normalize(),
                body_a_index: aabb_index,
                body_b_index: circle_index,
            });
        }
    }
    None
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
    fn world_aabb(&self, body_pos: Vec2) -> Option<(Vec2, Vec2)> {
        match self {
            Collider::AABB { min, max } => Some((body_pos + *min, body_pos + *max)),
            _ => None,
        }
    }

    fn world_circle(&self, owner_pos: Vec2) -> Option<Vec2> {
        match self {
            Collider::Circle { offset, .. } => Some(owner_pos + *offset),
            _ => None,
        }
    }

    fn collides_with(
        &self, // collider_a
        body_a: &RigidBody2D,
        body_b: &RigidBody2D,
        collider_b: &Collider,
        body_a_index: usize,
        body_b_index: usize,
    ) -> Option<Contact> {
        // HACK: This is not actually needed, because you can obviously have collisions between
        // non-static objects
        // if !(my_body.is_static || other_body.is_static) {
        //     return None;
        // }

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

struct Object {
    body: Option<RigidBody2D>,
    collider: Option<Collider>,
    color: Color,
    name: String,
}

impl Object {
    fn draw(&self, camera: &Camera) {
        let Some(body) = &self.body else {
            return;
        };
        let Some(collider) = &self.collider else {
            return;
        };

        match collider {
            Collider::Circle { offset, radius } => {
                let world_pos = body.position + *offset;
                let screen_pos = camera.world_to_screen(world_pos);
                let screen_radius = *radius * camera.zoom.x; // assume uniform zoom
                draw_circle_lines(screen_pos.x, screen_pos.y, screen_radius, 2.0, self.color);
            }

            Collider::AABB { min, max } => {
                let world_min = body.position + *min;
                let world_max = body.position + *max;

                let top_left = vec2(world_min.x, world_max.y); // because Y+ is up
                let size = world_max - world_min;

                let screen_top_left = camera.world_to_screen(top_left);
                let screen_size = size * camera.zoom;

                draw_rectangle_lines(
                    screen_top_left.x,
                    screen_top_left.y,
                    screen_size.x,
                    -screen_size.y, // flip Y for screen space
                    2.0,
                    self.color,
                );
            }
        }
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
        camera.pos += -Vec2::X * 2.0;
    }

    if is_key_down(KeyCode::D) {
        camera.pos += Vec2::X * 2.0;
    }

    if is_key_down(KeyCode::W) {
        camera.pos += Vec2::Y * 2.0;
    }
    if is_key_down(KeyCode::S) {
        camera.pos += -Vec2::Y * 2.0;
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
    let slop = 0.02; // allow for 1 cm of slop

    let bias_factor = 0.2;
    let bias_vel = (bias_factor / dt) * f32::max(0.0, contact.pen_depth - slop);

    // TODO: add inertia tensor
    //
    // NOTE:
    // this is quasi the effective mass
    let k_n = body_a.inverse_mass + body_b.inverse_mass;

    // magnitude of the impulse
    // if the relative velocity is greater than zero, the bodies are already
    // moving apart
    let restitution = 0.5;
    let P_n = f32::max(((1.0 + restitution) * (-v_n + bias_vel)) / k_n, 0.0);
    let P = P_n * contact.normal;

    if !body_a.is_static {
        body_a.apply_impulse(-P);
    }
    if !body_b.is_static {
        body_b.apply_impulse(P);
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
    let mut rg1: RigidBody2D = RigidBody2D {
        position: vec2(0.0, 0.0),
        acc: vec2(0.0, 0.0),
        vel: vec2(0.0, 0.0),
        accum_force: vec2(0.0, 0.0),
        inverse_mass: 1.0 / 5.0,
        is_static: false,
    };
    let col1: Collider = Collider::Circle {
        offset: vec2(0.0, 0.0),
        radius: 3.0,
    };
    let obj1: Object = ObjectBuilder::new()
        .with_body(rg1)
        .with_collider(col1)
        .with_color(YELLOW)
        .with_name("circle".to_string())
        .build();

    // Rectangle 2
    let mut rg2: RigidBody2D = RigidBody2D {
        position: vec2(-100.0, -10.0),
        vel: vec2(0.0, 0.0),
        acc: vec2(0.0, 0.0),
        accum_force: vec2(0.0, 0.0),
        inverse_mass: 1.0 / 1000.0,
        is_static: true,
    };
    let col2: Collider = Collider::AABB {
        min: vec2(0.0, -10.0),
        max: vec2(200.0, 0.0),
    };
    let obj2: Object = ObjectBuilder::new()
        .with_body(rg2)
        .with_collider(col2)
        .with_color(PINK)
        .with_name("floor".to_string())
        .build();

    // Rectangle 3
    let mut rg3: RigidBody2D = RigidBody2D {
        position: vec2(-30.0, 20.0),
        vel: vec2(1.0, -2.0),
        acc: vec2(0.0, 0.0),
        accum_force: vec2(0.0, 0.0),
        inverse_mass: 1.0 / 100.0,
        is_static: false,
    };
    let col3: Collider = Collider::AABB {
        min: vec2(0.0, -10.0),
        max: vec2(20.0, 0.0),
    };
    let obj3: Object = ObjectBuilder::new()
        .with_body(rg3)
        .with_collider(col3)
        .with_color(GREEN)
        .with_name("some_rect".to_string())
        .build();

    let mut objects = [obj2, obj1, obj3];
    let mut camera = Camera::default();

    loop {
        // handle camera input and movement
        handle_camera_movement(&mut camera);
        draw_zoom_ui(camera.zoom);

        clear_background(WHITE);
        let dt = 1. / 60.;

        // apply_gravity
        apply_gravity(&mut objects);
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
        for object in objects.as_mut() {
            object.body.as_mut().unwrap().update(dt);
            object.draw(&camera);
        }

        next_frame().await;
    }
}
