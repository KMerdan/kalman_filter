use nalgebra::Matrix4;


use crate::state::CarState;
use crate::car::{Car, KinematicBicycleModel};
use crate::sensor_measurement::SensorState;

#[warn(dead_code)]
pub struct KalmanFilter {
    pub state: CarState,
    pub covariance: Matrix4<f64>,
    model: KinematicBicycleModel,
    pub history: Vec<(f64,f64,f64,f64,f64, f64)>,
}

// estimate the state of the car based on the sensor measurement
