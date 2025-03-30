use macroquad::prelude::*;
use nalgebra::Vector2;

const PIXELS_PER_METER: f32 = 24.0;

fn world_to_screen(coord: Vec2) -> Vec2{
    let new_coord = Vec2 {
        x: PIXELS_PER_METER * coord.x,
        y: -PIXELS_PER_METER * coord.y
    };

    return new_coord;
}

fn screen_to_world(coord: Vec2) -> Vec2 {
    let new_coord = Vec2 {
        x: coord.x / PIXELS_PER_METER,
        y: -coord.y / PIXELS_PER_METER 

    };

    return new_coord;
}

/// Functions every body must implement in order for the simulation to work correctly
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
    fn draw(&self);
}


struct Circle{
    // pointer that uniquely owns a type that implements the Body trait
    body: PointMass,
    color: Color,
    radius: f32
}


impl Drawable for Circle {
    fn draw(&self) {
        let screen_pos = world_to_screen(self.body.pos);
        draw_circle(screen_pos.x, screen_pos.y, self.radius, self.color);
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

    let screen_middle = Vec2 {
        x: screen_width() / 2.0,
        y: screen_height() / 2.0
    };

    let mut circle = Circle {
        body: PointMass {
            pos: screen_to_world(screen_middle),
            vel: vec2(3.0, 0.0),
            force: Vec2::ZERO,
            mass: 100.0,
        },
        color: RED,
        radius: 3.0,
    };

    // for now let y be positive, later, when drawing we will change it
    let gravity = vec2(0.0, -3.0);

    loop {
        let dt = get_frame_time();

        circle.body.apply_force(gravity * circle.body.mass);
        circle.body.update(dt);

        clear_background(WHITE);
        println!("world: {}", circle.body.pos);
        println!("screen: {}", world_to_screen(circle.body.pos));

        circle.draw();

        next_frame().await;
    }
}
