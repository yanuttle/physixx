use ::macroquad::prelude::*;

pub struct Camera {
    pub screen_dims: Vec2,
    pub pos: Vec2,
    pub zoom: Vec2,
    // the factor to multiply with/divide by when performing a zoom/unzoom operation
    pub zoom_factor: f32,
}

impl Camera {
    pub fn zoom_in(&mut self) {
        self.zoom.x *= self.zoom_factor;
        self.zoom.y *= self.zoom_factor;
    }

    pub fn zoom_out(&mut self) {
        self.zoom.x /= self.zoom_factor;
        self.zoom.y /= self.zoom_factor;
    }

    pub fn screen_middle(&self) -> Vec2 {
        Vec2 {
            x: self.screen_dims.x / 2.0,
            y: self.screen_dims.y / 2.0,
        }
    }

    pub fn world_to_screen(&self, world_pos: Vec2) -> Vec2 {
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
    pub fn screen_to_world(&self, screen_pos: Vec2) -> Vec2 {
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
