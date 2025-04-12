use crate::Camera;
use crate::collider::*;
use crate::rigid_body::*;
use macroquad::prelude::*;

pub struct Object {
    pub body: Option<RigidBody2D>,
    pub collider: Option<Collider>,
    pub color: Color,
    pub name: String,
}

impl Object {
    pub fn draw(&self, camera: &Camera) {
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

pub struct ObjectBuilder {
    pub body: Option<RigidBody2D>,
    pub collider: Option<Collider>,
    pub color: Option<Color>,
    pub name: Option<String>,
}

impl ObjectBuilder {
    pub fn new() -> Self {
        Self {
            body: None,
            collider: None,
            color: None,
            name: None,
        }
    }

    pub fn with_body(mut self, body: RigidBody2D) -> Self {
        self.body = Some(body);
        self
    }

    pub fn with_collider(mut self, collider: Collider) -> Self {
        self.collider = Some(collider);
        self
    }

    pub fn with_color(mut self, color: Color) -> Self {
        self.color = Some(color);
        self
    }

    pub fn with_name(mut self, name: String) -> Self {
        self.name = Some(name);
        self
    }

    pub fn build(self) -> Object {
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
