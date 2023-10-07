// use piston_window::*;
// use rand::prelude::*;
// use rand_distr::{Distribution, Normal};
// use nalgebra::{Matrix4, Vector4, Vector2, Vector3, Matrix2};
use nalgebra::Matrix4;

use crate::state::{CarState, get_time_stamp, CarColor};
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
                dt: delta_time,
                time_stamp: get_time_stamp(),
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
                velocity: 0.0,
                width: None,
                length: None,
            },
        }
    }

    pub fn _update(
        &self,
        x: f64,
        y: f64,
        yaw: f64,
        velocity: f64,
        acceleration: f64,
        steering_angle: f64,
    ) -> CarState {
        let steering_angle = steering_angle.max(-self.max_steer).min(self.max_steer);
        let x = x + velocity * yaw.cos() * self.dt;
        let y = y + velocity * yaw.sin() * self.dt;
        let yaw = yaw + velocity / self.wheelbase * steering_angle * self.dt;
        let velocity = velocity + acceleration * self.dt;
        CarState {
            dt: self.dt,
            time_stamp: get_time_stamp(),
            x,
            y,
            yaw,
            velocity,
            width: None,
            length: None,
        }
    }

    pub fn _jacobian(
        &self,
        x: f64,
        y: f64,
        yaw: f64,
        velocity: f64,
        steering_angle: f64,
    ) -> Matrix4<f64> {
        let mut jacobian = Matrix4::identity();
        jacobian[(0, 2)] = -velocity * yaw.sin() * self.dt;
        jacobian[(0, 3)] = yaw.cos() * self.dt;
        jacobian[(1, 2)] = velocity * yaw.cos() * self.dt;
        jacobian[(1, 3)] = yaw.sin() * self.dt;
        jacobian[(2, 3)] = self.dt / self.wheelbase * steering_angle;
        jacobian
    }
}

// a car struture that based on the KinematicBicycleModel
// it has a state (x, y, yaw, velocity)
// it has a KinematicBicycleModel
// it has a function that takes an acceleration and a steering angle and updates the state of the car
#[derive(Debug)]
pub struct Car {
    pub state: CarState,
    // pub rectangular: Rectangular,
    pub width: f64,
    pub length: f64,
    pub acceleration: f64,
    pub steering_angle: f64,
    pub model: KinematicBicycleModel,
    pub color: Option<CarColor>,
}

impl Car {
    pub fn new(
        x: f64,
        y: f64,
        yaw: f64,
        width: f64,  //TODO remove width and length from here
        length: f64, //TODO remove width and length from here
        velocity: f64,
        wheelbase: f64,
        max_steer: f64,
        delta_time: f64,
        color: Option<CarColor>,
    ) -> Self {
        Self {
            state: CarState {
                dt: delta_time,
                time_stamp: get_time_stamp(),
                x,
                y,
                yaw,
                velocity,
                width: Some(width),
                length: Some(length),
            },
            width,  //TODO remove width and length from here
            length, //TODO remove width and length from here
            color,
            acceleration: 0.0,
            steering_angle: 0.0,
            model: KinematicBicycleModel::_new(wheelbase, max_steer, delta_time),
        } 
            
    }

    pub fn step(&mut self, acceleration: f64, steering_angle: f64) -> Rectangular {
        self.state = self.model._update(
            self.state.x,
            self.state.y,
            self.state.yaw,
            self.state.velocity,
            acceleration,
            steering_angle,
        );
        self.acceleration = acceleration;
        self.steering_angle = steering_angle;
        self.state.to_rectangular( self.color)
    }
}
