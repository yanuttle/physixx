use macroquad::{color, prelude::*, ui::root_ui};


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
        let rel_pos: Vec2 = (world_pos - self.pos) * self.zoom ;

        self.screen_middle() + rel_pos
    }

    /// Converts the position on the screen to the position in the world using the camera parameters
    fn screen_to_world(&self, screen_pos: Vec2) -> Vec2 {
        /*
        get the relative position of the object in respect to the screen center
        unapply the zoom of the camera.

        Formula: world_pos = (screen_pos - screen_center) / zoom + cam_pos
        */
        let rel_pos: Vec2 = (screen_pos - self.screen_middle()) / self.zoom;

        self.pos + rel_pos
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            pos: Vec2::ZERO,
            zoom: vec2(24.0, -24.0),
            zoom_factor: 1.1,
            screen_dims: vec2(screen_height(), screen_width())
        }
    }
}
trait Body {
    fn apply_force(&mut self, force: Vec2);
    // Point masses don't have a torque so we don't want to have to implement applying torque there.
    fn apply_torque(&mut self, torque: f32);
    fn update(&mut self, dt: f32);
    fn position(&self) -> Vec2;
} 

/// Represents a point mass, an idealization of some object 
struct PointMass {
    pos: Vec2,
    vel: Vec2,
    /// represents the accumulated forces that are supposed to be applied in the next update
    force: Vec2,
    mass: f32
}

enum Shape2D {
    Circle {radius: f32},
    Rectangle {width: f32, height: f32}
}

struct Collider {
    body: Option<RigidBody2D>,
    offset: Vec2,
    shape: Shape2D,
    
    // TODO: remove later, this does not belong here.
    color: Color
}

struct RigidBody2D {
    pos: Vec2,
    angle: f32,

    // linear velocity of the body
    lin_vel: Vec2,
    // angular velocity of the body
    ang_vel: f32, 
    mass: f32,
    inertia: f32,
    is_fixed: bool,
    has_gravity: bool,
    force: Vec2,
    torque: Vec2,
}

trait Drawable {
    /// draw something on screen relative to some camera
    fn draw(&self, camera: &Camera);
}



// for debugging now.
impl Drawable for Collider {
    fn draw(&self, camera: &Camera) {

        // if there is rigid body attached, use its position, otherwise, use offset
        let pos = match self.body.as_ref() {
            Some(body) => body.pos + self.offset,
            None => self.offset
        };

        let screen_pos = camera.world_to_screen(pos);

        match self.shape {
            Shape2D::Circle { radius } => {
                // when zoom has different x and y components, it can distort what the object looks like, so the circle can be stretched out or squashed
                // it means it is scaled in the x (width) and y (direction) by the zoom
                let scaled_circle = radius * camera.zoom;
                draw_ellipse(screen_pos.x, screen_pos.y, scaled_circle.x, scaled_circle.y, 0.0, self.color);
            }

            Shape2D::Rectangle { width, height } => {
                let scaled_rectangle: Vec2 = Vec2 {x: width, y: height} * camera.zoom;
                draw_rectangle(screen_pos.x, screen_pos.y, scaled_rectangle.x, scaled_rectangle.y, self.color);
            }
        }

    }
}

fn draw_zoom_ui(zoom: Vec2) {
    root_ui().label(None, &format!("Zoom: {:.2} x {:.2}", zoom.x, zoom.y));
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

/// Implementations of specific body functions for the PointMass struct
impl Body for PointMass {
    
    fn apply_force(&mut self, force: Vec2) {
        self.force += force;   
    }

    fn apply_torque(&mut self, _torque: f32) {}

    /// Advance the simulation by one step.
    fn update(&mut self, dt: f32) {
        assert_ne!(self.mass, 0.0, "THE MASS OF AN OBJECT SHOULD NEVER BE ZERO!");
        // it may be null sometimes tho, like if I want it to be stationary 

        let acc = self.force / self.mass;

        // multiply both with dt for correct units
        self.vel += acc * dt;
        self.pos += self.vel * dt;

        // zero out the forces for the next step
        self.force = Vec2::ZERO;
        assert_eq!(self.force, Vec2::ZERO);
    }

    fn position(&self) -> Vec2 {
        return self.pos;
    }
}

#[macroquad::main("Physixx")]
async fn main() {

    let mut camera = Camera::default();

    // create a collider that will stay fixed in space and collide with things (like a floor)
    let floor = Collider {
        body: None,
        offset: Vec2::ZERO,
        shape: Shape2D::Rectangle { width: 400.0, height: 0.5 },
        color: BLACK 
    };

    let rect_rb = RigidBody2D {
        ang_vel: 0.0,
        pos: Vec2 {x: 0.0, y: 5.0},
        angle: 0.0,
        lin_vel: Vec2::X,
        mass: 5.0, // kg
        inertia: 0.0,
        is_fixed: false,
        has_gravity: true,
        force: Vec2::ZERO,
        torque: Vec2::ZERO 
    };

    let rect_collider = Collider {
        body: Some(rect_rb),
        offset: Vec2 { x: 0.0, y: 0.0 },
        shape: Shape2D::Rectangle { width: 40.0, height: 40.0 },
        color: RED
    };


    // for now let y be positive, later, when drawing we will change it
    // a_... means that whatever follows is an acceleration 
    let a_gravity = vec2(0.0, -9.81);

    loop {
        let dt = 1./100.;
        // handle camera input and movement 
        handle_camera_movement(&mut camera);



        draw_zoom_ui(camera.zoom);
        

        clear_background(WHITE);
        rect_collider.draw(&camera);
        floor.draw(&camera);

        next_frame().await;
    }
}
