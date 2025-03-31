use macroquad::prelude::*;


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
        let middle = Vec2 {
            x: self.screen_dims.x / 2.0,
            y: self.screen_dims.y / 2.0
        };

        return middle;
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

        let screen_pos= self.screen_middle() + rel_pos;

        return screen_pos;
    }

    /// Converts the position on the screen to the position in the world using the camera parameters
    fn screen_to_world(&self, screen_pos: Vec2) -> Vec2 {
        /*
        get the relative position of the object in respect to the screen center
        unapply the zoom of the camera.

        Formula: world_pos = (screen_pos - screen_center) / zoom + cam_pos
        */
        let rel_pos: Vec2 = (screen_pos - self.screen_middle()) / self.zoom;

        let world_pos= self.pos + rel_pos;

        return world_pos;
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
    fn update(&mut self, dt: f32);
    fn position(&mut self) -> Vec2;
} 

/// Represents a point mass, an idealization of some object 
struct PointMass {
    pos: Vec2,
    vel: Vec2,
    /// represents the accumulated forces that are supposed to be applied in the next update
    force: Vec2,
    mass: f32
}

trait Drawable {
    fn draw(&self, camera: &Camera);
}


struct Circle{
    // pointer that uniquely owns a type that implements the Body trait
    body: PointMass,
    color: Color,
    radius: f32
}


impl Drawable for Circle {
    fn draw(&self, camera: &Camera) {
        let screen_pos = camera.world_to_screen(self.body.pos);
        draw_circle(screen_pos.x, screen_pos.y, self.radius, self.color);
    }

}

impl Circle {
    fn apply_force(&mut self, force: Vec2) {
        self.body.apply_force(force);
    }

    fn update(&mut self, dt: f32) {
        self.body.update(dt);
    }
}

/// Implementations of specific body functions for the PointMass struct
impl Body for PointMass {
    
    /// 
    fn apply_force(&mut self, force: Vec2) {
        self.force += force;   
    }


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

    fn position(&mut self) -> Vec2 {
        return self.pos;
    }
}

#[macroquad::main("Physixx")]
async fn main() {

    let camera = Camera::default();

    let mut circle = Circle {
        body: PointMass {
            pos: Vec2 {x: 0.0, y: 0.0},
            vel: vec2(3.0, 0.0),
            force: Vec2::ZERO,
            mass: 100.0,
        },
        color: RED,
        radius: 3.0,
    };

    // for now let y be positive, later, when drawing we will change it
    // a_... means that whatever follows is an acceleration 
    let a_gravity = vec2(0.0, -1.0);

    loop {
        let dt = get_frame_time();

        // apply the force of gravity to the object
        circle.apply_force(a_gravity * circle.body.mass);
        circle.update(dt);

        clear_background(WHITE);
        println!("world: {}", circle.body.pos);
        println!("screen: {}", camera.world_to_screen(circle.body.pos));

        circle.draw(&camera);

        next_frame().await;
    }
}
