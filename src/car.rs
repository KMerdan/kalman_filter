// use piston_window::*;
// use rand::prelude::*;
// use rand_distr::{Distribution, Normal};
// use nalgebra::{Matrix4, Vector4, Vector2, Vector3, Matrix2};

use crate::state::CarState;
use crate::state::Rectangular;

#[derive(Debug)]
pub struct KinematicBicycleModel {
    pub dt: f64,
    pub wheelbase: f64,
    pub max_steer: f64,
    pub state: CarState,
}

impl KinematicBicycleModel {
    pub fn _new(wheelbase: f64, max_steer: f64, delta_time: f64) -> Self {
        Self {
            dt: delta_time,
            wheelbase,
            max_steer,
            state: CarState {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
                velocity: 0.0,
            },
        }
    }

    pub fn _update(&self, x: f64, y: f64, yaw: f64, velocity: f64, acceleration: f64, steering_angle: f64) -> CarState{
        let steering_angle = steering_angle.max(-self.max_steer).min(self.max_steer);
        let x = x + velocity * yaw.cos() * self.dt;
        let y = y + velocity * yaw.sin() * self.dt;
        let yaw = yaw + velocity / self.wheelbase * steering_angle * self.dt;
        let velocity = velocity + acceleration * self.dt;
        CarState {
            x,
            y,
            yaw,
            velocity,
        }
    }
}

// a car struture that based on the KinematicBicycleModel
// it has a state (x, y, yaw, velocity)
// it has a KinematicBicycleModel
// it has a function that takes an acceleration and a steering angle and updates the state of the car
#[derive(Debug)]
pub struct Car {
    pub state: CarState,
    pub rectangular: Rectangular,
    pub width: f64,
    pub length: f64,
    pub acceleration: f64,
    pub steering_angle: f64,
    pub model: KinematicBicycleModel,
}

impl Car {
    pub fn new(x: f64, y: f64, yaw: f64, width: f64, length: f64, velocity: f64, wheelbase: f64, max_steer: f64, delta_time: f64) -> Self {
        Self {
            state: CarState {
                x,
                y,
                yaw,
                velocity,
            },
            width,
            length,
            rectangular: Rectangular {
                x1 : x - (width / 2.0),
                y1 : y - (length / 2.0),
                x2 : x + (width / 2.0),
                y2 : y + (length / 2.0),
                x3 : x - (width / 2.0),
                y3 : y + (length / 2.0),
                x4 : x + (width / 2.0),
                y4 : y - (length / 2.0),
            },
            acceleration: 0.0,
            steering_angle: 0.0,
            model: KinematicBicycleModel::_new(wheelbase, max_steer, delta_time),
        }
    }

    pub fn update(&mut self, acceleration: f64, steering_angle: f64) {
        self.state = self.model._update(self.state.x, self.state.y, self.state.yaw, self.state.velocity, acceleration, steering_angle);
        
        let x1 = self.state.x - (self.width / 2.0);
        let y1 = self.state.y - (self.length / 2.0);
        let x3 = self.state.x + (self.width / 2.0);
        let y3 = self.state.y + (self.length / 2.0);
        let x2 = self.state.x - (self.width / 2.0);
        let y2 = self.state.y + (self.length / 2.0);
        let x4 = self.state.x + (self.width / 2.0);
        let y4 = self.state.y - (self.length / 2.0);

        //rotate the four corners of the car based on the yaw
        self.rectangular.x1 = x1 * self.state.yaw.cos() - y1 * self.state.yaw.sin();
        self.rectangular.y1 = x1 * self.state.yaw.sin() + y1 * self.state.yaw.cos();
        self.rectangular.x2 = x2 * self.state.yaw.cos() - y2 * self.state.yaw.sin();
        self.rectangular.y2 = x2 * self.state.yaw.sin() + y2 * self.state.yaw.cos();
        self.rectangular.x3 = x3 * self.state.yaw.cos() - y3 * self.state.yaw.sin();
        self.rectangular.y3 = x3 * self.state.yaw.sin() + y3 * self.state.yaw.cos();
        self.rectangular.x4 = x4 * self.state.yaw.cos() - y4 * self.state.yaw.sin();
        self.rectangular.y4 = x4 * self.state.yaw.sin() + y4 * self.state.yaw.cos();

        self.acceleration = acceleration;
        self.steering_angle = steering_angle;

    }

}

