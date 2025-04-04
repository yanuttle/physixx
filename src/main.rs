
use core::panic;
use std::vec;
use approx; // For the macro assert_relative_eq!
use macroquad::{prelude::*, ui::root_ui};

struct Camera {
    screen_dims: Vec2,
    pos: Vec2,
    zoom: Vec2,
    // the factor to multiply with/divide by when performing a zoom/unzoom operation
    zoom_factor: f32
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
            y: self.screen_dims.y / 2.0}
    }

    fn world_to_screen(&self, world_pos: Vec2) -> Vec2{
        /*
        get the relative position of
        the object in respect to the
        position the camera is looking at
        and apply the zoom so that the new coordinate is correct.

        Formula: screen_pos = (obj_pos - cam_pos) * zoom + screen_center
        */
        let relative_position: Vec2 = (world_pos - self.pos) * self.zoom ;

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
            screen_dims: vec2(screen_width(), screen_height())
        }
    }
}

struct RigidBody2D {
    position: Vec2,
    vel: Vec2,
    // accumulated force
    accum_force: Vec2,
    mass: f32,
    is_static: bool
}


impl RigidBody2D {
    fn apply_force(&mut self, force: Vec2) {
        self.accum_force += force;
    }

    fn update(&mut self, dt: f32) {


        if self.mass == 0.0 || self.is_static { return; }

        let acc = self.accum_force / self.mass;
        self.vel += acc * dt;
        self.position += self.vel * dt;

        // reset the accumulated forces after update
        self.accum_force = Vec2::ZERO;

    }
}



enum Collider {
    Circle {
        offset: Vec2,
        radius: f32
    },
    AABB {
        min: Vec2, 
        max: Vec2
    }
}

// returns the point on the aabb surface that is nearest to the given point
fn point_aabb_nearest_point(point: Vec2, aabb: &Collider, body: &RigidBody2D) -> Vec2 {
    if let Collider::AABB { min, max } = aabb {
        let world_min = body.position + *min;
        let world_max = body.position + *max;
        let mut nearest_point = vec2(0.0, 0.0);
        
        if point.x <= world_max.x && point.x >= world_min.x { nearest_point.x = point.x }
        else if point.x < world_min.x { nearest_point.x = world_min.x;}
        else if point.x > world_max.x { nearest_point.x = world_max.x;} 
        else if point.x - world_min.x > world_max.x - point.x {nearest_point.x = world_max.x;}
        else {nearest_point.x = world_min.x}

        if point.y <= world_max.y && point.y >= world_min.y { nearest_point.y = point.y }
        else if point.y < world_min.y { nearest_point.y = world_min.y;}
        else if point.y > world_max.y { nearest_point.y = world_max.y;} 
        else if point.y - world_min.y > world_max.y - point.y {nearest_point.y = world_max.y;}
        else {nearest_point.y = world_min.y}

        nearest_point
    }
    else {
        panic!();
    }
}

// https://www.r-5.org/files/books/computers/algo-list/realtime-3d/Christer_Ericson-Real-Time_Collision_Detection-EN.pdf
fn sq_dist_point_aabb(point: Vec2, aabb: &Collider, body: &RigidBody2D) -> f32 {
    if let Collider::AABB{min, max} = aabb {
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
    approx::abs_diff_eq!(vector.x, 0.0)
    &&
    approx::abs_diff_eq!(vector.y, 0.0)
}

fn test_circle_aabb(circle: &Collider, aabb: &Collider, circle_body: &RigidBody2D, aabb_body: &RigidBody2D) -> Option<Collision>{
    match (circle, aabb) {
        (
            Collider::Circle { radius, ..},
            Collider::AABB { min, max }
        ) => {
            // Get the circle's world position.
            let circle_world_pos = circle.world_circle(circle_body.position).unwrap();
            // Compute the squared distance from the circle's center to the AABB.
            let nearest_point_to_center = point_aabb_nearest_point(circle_world_pos, aabb, aabb_body);


            let dist = Vec2::distance(nearest_point_to_center, circle_world_pos);
            let collision_vector = nearest_point_to_center - circle_world_pos;

            let world_min = *min + aabb_body.position;
            let world_max = *max + aabb_body.position;



            let mut normal = collision_vector;
            if is_close_to_zero(normal) {
                let distance_left = (circle_world_pos.x - world_min.x).abs();
                let distance_right = (circle_world_pos.x - world_max.x).abs();
                let distance_bottom= (circle_world_pos.y - world_min.x).abs();
                let distance_top= (circle_world_pos.y - world_max.x).abs();

                let min_distance = f32::min(distance_left,
                                 f32::min(distance_right,
                                 f32::min(distance_bottom,distance_top)));

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
                Some(Collision {
                    depth: *radius - dist,
                    normal 
                })

            } else {
                None
            }

        },
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
            Collider::Circle { offset,..} => { Some(owner_pos + *offset)},
            _ => None
        }
    }

    fn collides_with(&self, my_body: &RigidBody2D, other_body: &RigidBody2D,  other: &Collider) -> Option<Collision>{
        match (self, other) {
            // two circles collide if the squared distance between them is smaller than the sum of their squared radii 
            (Collider::Circle { radius: radius_a , ..},
            Collider::Circle { radius: radius_b , ..}) => 
            {
                let pos_a = self.world_circle(my_body.position).unwrap();
                let pos_b = other.world_circle(other_body.position).unwrap();
                let position_difference = pos_b - pos_a;

                // this can be used to calculate the distance 
                let dist = pos_a.distance(pos_b);

                if dist < radius_a + radius_b {
                    let normal = position_difference / dist;
                    Some(Collision { normal, depth: radius_a + radius_b - dist})
                }
                else {
                    None
                }
            },
            
            // a circle collides with an aabb if the
            // distance from the center of the circle
            // to the closest point on the aabb is 
            // smaller than the radius of the circle
            (Collider::AABB { .. },
            Collider::Circle { .. }) =>
            {
                test_circle_aabb(other, self, other_body, my_body)
            },

            (Collider::Circle { .. },
            Collider::AABB { .. }) =>
            {
                test_circle_aabb(self, other, my_body, other_body)
            },

            (Collider::AABB { .. },
            Collider::AABB { .. }) => {
                let min_max_a = self.world_aabb(my_body.position).unwrap();
                let min_max_b =  other.world_aabb(other_body.position).unwrap();

                let min_a = min_max_a.0;
                let max_a = min_max_a.1;

                let min_b = min_max_b.0;
                let max_b = min_max_b.1;

                let is_colliding = max_a.x >= min_b.x && max_b.x >= min_a.x &&
                max_a.y >= min_b.y && max_b.y >= min_a.y;


                if is_colliding {
                    let x_overlap = f32::min(max_a.x.max(max_b.x), max_b.x) - f32::max(min_a.x, min_b.x);
                    let y_overlap = f32::min(max_a.y, max_b.y) - f32::max(min_a.y, min_b.y);

                    let depth = f32::min(x_overlap, y_overlap);
                    // if the penetration depth is negative, then there is no penetration, so there is no collision
                    if depth < 0.0 { return None; }
                    
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
                    }
                    else {
                        normal = Vec2::X;
                        let left_penetration = max_a.x - min_b.x;
                        let right_penetration = max_b.x - min_a.x;

                        // the object needs to be pushed to the left
                        // because the left penetration is smaller
                        if left_penetration < right_penetration {
                            normal *= -1.0;
                        }
                    }

                    return Some(Collision {
                        depth,
                        normal
                    })
                }
                None

                
            }

        }

    }
}


struct Object {
    body: Option<RigidBody2D>,
    collider: Option<Collider>,
    color: Color
}

impl Object {

    fn draw(&self, camera: &Camera) {
        let Some(body) = &self.body else { return; };
        let Some(collider) = &self.collider else { return; };

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
                    self.color
                );
            }
        }
    }

}

struct ObjectBuilder {
    body: Option<RigidBody2D>,
    collider: Option<Collider>,
    color: Option<Color>
}

impl ObjectBuilder {
    fn new() -> Self {
        Self {
            body: None,
            collider: None,
            color: None   
        }
    }

    fn with_body(mut self, body: RigidBody2D) -> Self{
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

    fn build(self) -> Object {
        if let Some(c) = self.color {
            Object {
                body: self.body,
                collider: self.collider,
                color: c
            }

        }
         else {panic!();}
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
        camera.pos += Vec2::X * 2.0 ;
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
struct Collision {
    normal: Vec2,
    depth: f32 
}

fn resolve_collision(body: &mut RigidBody2D, collision: Collision) {

    if !body.is_static {
        body.position -= 2.0*collision.normal * collision.depth;
    }




}

fn check_collisions(objects: &mut [Object]) {
    for i in 0..objects.len() {
        let (left, right) = objects.split_at_mut(i + 1);
        let a = &mut left[i];
        for b in right {

            let (Some(collider_a), Some(body_a)) = (&a.collider, a.body.as_mut()) else { continue };
            let (Some(collider_b), Some(body_b)) = (&b.collider, b.body.as_mut()) else { continue };

            if let Some(collision) = collider_a.collides_with(body_a, body_b, collider_b) {
                println!("Collision: (normal, depth) -- {} {}", collision.normal, collision.depth);
                resolve_collision(body_a, collision);
            }
        }
    }
}

// TODO: delete later
fn apply_gravity(objects: &mut [Object]) {
    for object in objects.iter_mut()  {
        let (Some(_), Some(body)) = (&object.collider, &mut object.body) else {continue };

        body.apply_force(body.mass * gravity_acceleration()); 

    }
}

#[macroquad::main("Physixx")]
async fn main() {

    // circle
    let mut rg1: RigidBody2D = RigidBody2D { position: vec2(0.0, 0.0), vel: vec2(0.0, 0.0), accum_force: vec2(0.0, 0.0), mass: 5.0, is_static: false };
    let col1: Collider = Collider::Circle { offset: vec2(0.0, 0.0), radius: 3.0 };
    let obj1: Object = ObjectBuilder::new()
                                .with_body(rg1)
                                .with_collider(col1)
                                .with_color(YELLOW)
                                .build();


    // Rectangle 2
    let mut rg2: RigidBody2D = RigidBody2D { position: vec2(-100.0, -10.0), vel: vec2(-2.0, 10.0), accum_force: vec2(0.0, 0.0), mass: 10.0, is_static: true};
    let col2: Collider = Collider::AABB { min: vec2(0.0, -10.0), max: vec2(200.0, 0.0) };
    let obj2: Object = ObjectBuilder::new()
                            .with_body(rg2)
                            .with_collider(col2)
                            .with_color(PINK)
                            .build();

    // Rectangle 3
    let mut rg3: RigidBody2D = RigidBody2D { position: vec2(-30.0, 20.0), vel: vec2(1.0, -2.0), accum_force: vec2(0.0, 0.0), mass: 3.0, is_static: false };
    let col3: Collider = Collider::AABB { min: vec2(0.0, -10.0), max: vec2(20.0, 0.0) };
    let obj3: Object = ObjectBuilder::new()
                            .with_body(rg3)
                            .with_collider(col3)
                            .with_color(GREEN)
                            .build();

    let mut objects = [obj1, obj2, obj3];
    // TODO: uncomment
    // let mut camera = Camera::default();
    let mut camera = Camera::default();

    loop {
        // handle camera input and movement 
        handle_camera_movement(&mut camera);
        draw_zoom_ui(camera.zoom);

        clear_background(WHITE);
        let dt = 1./60.;

        // do physics
        check_collisions(&mut objects);

        // apply_gravity
        apply_gravity(&mut objects);

        
        

        
        for object in objects.as_mut() {
            object.body.as_mut().unwrap().update(dt);
            object.draw(&camera);
        }


        next_frame().await;
        // println!("{:?}", masses);
    }
}
