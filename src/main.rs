
use std::vec;

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
trait Body {
    fn apply_force(&mut self, force: Vec2);
    // Point masses don't have a torque so we don't want to have to implement applying torque there.
    // fn apply_torque(&mut self, torque: f32);
    fn update(&mut self, dt: f32);
    fn position(&self) -> Vec2;
} 

/// min-max bounding box for AABB collision-detection
struct AABB {
    // min-max top-left and bottom right corner of the box
    dimensions: Vec2
}

/// A static is a part of the scene, it interacts with objects by not allowing them to pass through. can be used for floors or other things.
struct Static {
    // position of the top left corner in world coordinates
    position: Vec2,
    // width, height
    size: Vec2,
    aabb: AABB
}

/// Represents a point mass, an idealization of some object 
#[derive(Debug)]
struct PointMass {
    pos: Vec2,
    vel: Vec2,
    /// represents the accumulated forces that are supposed to be applied in the next update
    force: Vec2,
    mass: f32,

    // for drawing and collisions
    radius: f32,

    color: Color 
}

trait Drawable {
    /// draw something on screen relative to some camera
    fn draw(&self, camera: &Camera);
}

impl Drawable for PointMass {
    /// This function draws a point mass as a cicle of radius PointMass.radius and center PointMass.center
    fn draw(&self, camera: &Camera) {
        let screen_position: Vec2 = camera.world_to_screen(self.pos);
        let screen_radius = self.radius * camera.zoom;
        draw_ellipse(screen_position.x, screen_position.y, screen_radius.x, screen_radius.y, 90.0, self.color);
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

/// Implementations of specific body functions for the PointMass struct
impl Body for PointMass {
    
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

    fn position(&self) -> Vec2 {
        return self.pos;
    }
}

#[macroquad::main("Physixx")]
async fn main() {

    // all of the point masses
    let mut masses: Vec<PointMass> = vec![];
    
    // define a floor
    let mut statics: Vec<Static> = vec![
        Static {position: vec2(-50.0, -10.0), size: vec2(1.0, 100.0), aabb: AABB { dimensions:vec2(1.0,100.0) }}
    ];

    let mut camera = Camera::default();

    print!("{:?}", camera.world_to_screen(vec2(0.0, 0.0)));

    loop {
        let dt = 1./60.;
        // handle camera input and movement 
        handle_camera_movement(&mut camera);


        widgets::Window::new(hash!(), vec2(10., screen_height() - 110.), vec2(320., 100.))
            .label("mass_spawner")
            .titlebar(true)
            .ui(&mut *root_ui(), |ui| {
                    Group::new(hash!(), Vec2::new(300., 80.)).ui(ui, |ui| {
                        if ui.button(Vec2::new(260., 55.), "spawn ball") {
                            masses.push(PointMass { pos: vec2(0.0, 0.0), vel: vec2(2.0, 0.0), force: vec2(0.0,0.0), mass: 1.0, radius: 1.0, color: BLACK });
                        }
                    }) ;
            });


        draw_zoom_ui(camera.zoom);
        draw_spawn_ui();


        clear_background(WHITE);

        for mass in &mut masses {
            mass.draw(&camera);
            mass.update(dt);
        }


        next_frame().await;

        // println!("{:?}", masses);
    }
}
