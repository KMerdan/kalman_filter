use nalgebra::Matrix4;

use crate::car::{Car, KinematicBicycleModel};
use crate::state::{CarState, Rectangular};

pub struct KalmanFilter {
    pub rectangular: Rectangular,
    pub state: CarState,
    pub covariance: Matrix4<f64>,
    model: KinematicBicycleModel,
    pub history: Vec<(f64, f64, f64, f64, f64, f64)>,
}

// estimate the state of the car based on the sensor measurement
impl KalmanFilter {}
