use crate::Collider;
use macroquad::prelude::*;

pub struct RigidBody2DBuilder {
    position: Vec2,
    angle: f32,
    angular_vel: f32,
    vel: Vec2,
    // accumulated force
    accum_force: Vec2,
    accum_torque: f32,
    inverse_mass: f32,
    inverse_inertia: f32,
    is_static: bool,
    shape: Option<Collider>,
    restitution: f32,
    mu: f32,
}

impl RigidBody2DBuilder {
    pub fn new() -> Self {
        Self {
            position: Vec2::ZERO,
            angle: 0.0,
            vel: Vec2::ZERO,
            angular_vel: 0.0,
            accum_force: Vec2::ZERO,
            accum_torque: 0.0,
            inverse_mass: 1.0,
            inverse_inertia: 1.0,
            is_static: false,
            shape: None,
            restitution: 0.5,
            mu: 0.3,
        }
    }

    pub fn with_position(mut self, position: Vec2) -> Self {
        self.position = position;
        self
    }

    pub fn with_angle(mut self, angle: f32) -> Self {
        self.angle = angle;
        self
    }

    pub fn with_shape(mut self, shape: Collider) -> Self {
        self.shape = Some(shape);
        self
    }

    pub fn with_inverse_mass(mut self, inv_mass: f32) -> Self {
        self.inverse_mass = inv_mass;
        self
    }

    pub fn with_angular_vel(mut self, ang_vel: f32) -> Self {
        self.angular_vel = ang_vel;
        self
    }

    pub fn with_vel(mut self, vel: Vec2) -> Self {
        self.vel = vel;
        self
    }

    pub fn make_static(mut self) -> Self {
        self.is_static = true;
        self
    }

    pub fn with_restitution(mut self, restitution: f32) -> Self {
        self.restitution = restitution;
        self
    }

    pub fn with_mu(mut self, mu: f32) -> Self {
        self.mu = mu;
        self
    }

    pub fn build(self) -> RigidBody2D {
        // calculate the inverse inertia of the body if a shape was provided
        let mut rb = RigidBody2D {
            position: self.position,
            angle: self.angle,
            angular_vel: self.angular_vel,
            vel: self.vel,
            accum_force: self.accum_force,
            accum_torque: self.accum_torque,
            inverse_mass: self.inverse_mass,
            inverse_inertia: self.inverse_inertia,
            is_static: self.is_static,
            restitution: self.restitution,
            mu: self.mu,
        };

        if rb.is_static {
            rb.inverse_mass = 0.0;
            rb.inverse_inertia = 0.0;
            return rb;
        }

        if let Some(shape) = self.shape {
            match shape {
                Collider::AABB { min, max } => {
                    let h = (max.y - min.y).abs();
                    let w = (max.x - min.x).abs();
                    let m = 1.0 / self.inverse_mass;
                    rb.inverse_inertia = (1.0 / 12.0) * m * (w * w + h * h);
                }
                Collider::Circle { radius, .. } => {
                    let m = 1.0 / self.inverse_mass;
                    rb.inverse_inertia = 0.5 * m * radius * radius;
                }
            }
        }

        rb
    }
}

pub struct RigidBody2D {
    pub position: Vec2,
    pub angle: f32,
    // acc: Vec2,
    pub angular_vel: f32,
    pub vel: Vec2,
    // accumulated force
    pub accum_force: Vec2,
    pub accum_torque: f32,
    pub inverse_mass: f32,
    pub inverse_inertia: f32,
    pub is_static: bool,
    pub restitution: f32,
    pub mu: f32, // coefficient of friction for this object
                 // this is not accurate but i will do it just like with restitution
}

impl RigidBody2D {
    pub fn apply_force(&mut self, force: Vec2) {
        self.accum_force += force;
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) {
        self.vel += impulse * self.inverse_mass;
    }

    /// update using verlet integration
    pub fn update(&mut self, dt: f32) {
        if self.inverse_mass == 0.0 || self.is_static {
            return;
        }
        // NOTE: this is euler
        let new_vel = self.vel + dt * self.inverse_mass * self.accum_force;
        let new_pos = self.position + new_vel * dt;

        let new_ang_vel = self.angular_vel + dt * self.inverse_inertia * self.accum_torque;
        let new_angle = self.angle + new_ang_vel * dt;

        // NOTE: this is verlet
        // let new_pos = self.position + self.vel * dt + self.acc * (dt * dt * 0.5);
        // let new_acc = self.accum_force * self.inverse_mass;
        // let new_vel = self.vel + (self.acc + new_acc) * (dt * 0.5);

        self.position = new_pos;
        self.angle = new_angle;
        self.angular_vel = new_ang_vel;
        self.vel = new_vel;

        // reset the accumulated forces and torques after update
        self.accum_force = Vec2::ZERO;
        self.accum_torque = 0.0;
    }
}
