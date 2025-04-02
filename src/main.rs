
use macroquad::{color, prelude::*, ui::{hash, root_ui, widgets::{self, Group}}};

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
    vel: Vec2
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

fn test_circle_aabb(circle: &Collider, aabb: &Collider, circle_body: &RigidBody2D, aabb_body: &RigidBody2D) -> bool {
    match (circle, aabb) {
        (
            Collider::Circle { radius, offset: circle_offset },
            Collider::AABB {min, max }
        ) => {
            // Get the circle's world position.
            let circle_world_pos = circle.world_circle(circle_body.position).unwrap();
            // Compute the squared distance from the circle's center to the AABB.
            let sq_dist = sq_dist_point_aabb(circle_world_pos, aabb, aabb_body);
            // Compare squared distances to avoid computing a square root.
            sq_dist < radius * radius
        },
        _ => false,
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

    fn collides_with(&self, my_body: &RigidBody2D, other_body: &RigidBody2D,  other: &Collider) -> bool {
        match (self, other) {
            // two circles collide if the squared distance between them is smaller than the sum of their squared radii 
            (Collider::Circle { radius: radius_a , ..},
            Collider::Circle { radius: radius_b , ..}) => 
            {
                let pos_a = self.world_circle(my_body.position).unwrap();
                let pos_b = other.world_circle(other_body.position).unwrap();

                let dist_sq = pos_a.distance_squared(pos_b);

                dist_sq < (radius_a * radius_a) + (radius_b * radius_b)
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

                max_a.x >= min_b.x && max_b.x >= min_a.x &&
                max_a.y >= min_b.y && max_b.y >= min_a.y 
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
                let screen_radius = *radius * camera.zoom.x.abs(); // assume uniform zoom
                draw_circle(screen_pos.x, screen_pos.y, screen_radius, self.color);
            }

            Collider::AABB { min, max } => {
                let world_min = body.position + *min;
                let world_max = body.position + *max;

                let top_left = vec2(world_min.x, world_max.y); // because Y+ is up
                let size = world_max - world_min;

                let screen_top_left = camera.world_to_screen(top_left);
                let screen_size = size * camera.zoom;

                draw_rectangle(
                    screen_top_left.x,
                    screen_top_left.y,
                    screen_size.x,
                    -screen_size.y, // flip Y for screen space
                    self.color
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
        camera.pos += Vec2::X * 2.0 ;
    }

    if is_key_down(KeyCode::W) {
        camera.pos += Vec2::Y * 2.0;
    }
    if is_key_down(KeyCode::S) {
        camera.pos += -Vec2::Y * 2.0;
    }

}


fn gravity_acceleration() -> f32 {
    9.81
}

fn check_collisions(objects: &[Object]) {
    for i in 0..objects.len() {
        for j in (i + 1)..objects.len() {
            let a = &objects[i];
            let b = &objects[j];

            let (Some(collider_a), Some(body_a)) = (&a.collider, &a.body) else { continue };
            let (Some(collider_b), Some(body_b)) = (&b.collider, &b.body) else { continue };

            if collider_a.collides_with(body_a, body_b, collider_b) {
                println!("Collision detected between object {} and {}", i, j);
                // You can handle response here later (like bouncing or destroying)
            }
        }
    }
}


#[macroquad::main("Physixx")]
async fn main() {

    // circle
    let rg1: RigidBody2D = RigidBody2D { position: vec2(0.0, 0.0), vel: vec2(0.0, 0.0) };
    let col1: Collider = Collider::Circle { offset: vec2(0.0, 0.0), radius: 3.0 };
    let obj1: Object = Object { body: Some(rg1), collider: Some(col1), color: BLACK };

    // Rectangle 
    let rg2: RigidBody2D = RigidBody2D { position: vec2(0.0, 0.0), vel: vec2(0.0, 0.0) };
    let col2: Collider = Collider::AABB { min: vec2(0.0, -10.0), max: vec2(20.0, 0.0) };
    let obj2: Object = Object { body: Some(rg2), collider: Some(col2), color: YELLOW};

    let mut objects = [obj1, obj2];
    let mut camera = Camera::default();

    print!("{:?}", camera.world_to_screen(vec2(0.0, 0.0)));

    loop {
        let dt = 1./60.;
        // handle camera input and movement 
        handle_camera_movement(&mut camera);
        draw_zoom_ui(camera.zoom);


        check_collisions(&objects);

        clear_background(WHITE);
        
        for object in &objects {
            object.draw(&camera);
        }


        next_frame().await;
        // println!("{:?}", masses);
    }
}
