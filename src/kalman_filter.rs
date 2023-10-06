use nalgebra::Matrix4;


use crate::state::{CarState, Rectangular};
use crate::car::{Car, KinematicBicycleModel};
use crate::sensor_measurement::SensorState;

#[warn(dead_code)]
pub struct KalmanFilter {
    pub rectangular: Rectangular,
    pub state: CarState,
    pub covariance: Matrix4<f64>,
    model: KinematicBicycleModel,
    pub history: Vec<(f64,f64,f64,f64,f64, f64)>,
}

// estimate the state of the car based on the sensor measurement
impl KalmanFilter {
    pub fn new(car: &Car, sensor: &SensorState) -> Self {
        let mut covariance = Matrix4::identity();
        covariance.fill_diagonal(0.1);
        Self {
            rectangular: car.rectangular,
            state: car.state,
            covariance,
            model: KinematicBicycleModel::_new(car.model.wheelbase, car.model.max_steer, car.model.dt),
            history: Vec::new(),
        }
    }

    pub fn update(&mut self, sensor: &SensorState) {
        // predict
        let mut jacobian = self.model._jacobian(
            self.state.x,
            self.state.y,
            self.state.yaw,
            self.state.velocity,
            sensor.steering_angle,
        );
        let mut state = self.model._update(
            self.state.x,
            self.state.y,
            self.state.yaw,
            self.state.velocity,
            sensor.acceleration,
            sensor.steering_angle,
        );
        let mut covariance = jacobian * self.covariance * jacobian.transpose();
        covariance[(0, 0)] += 0.1;
        covariance[(1, 1)] += 0.1;
        covariance[(2, 2)] += 0.1;
        covariance[(3, 3)] += 0.1;

        // update
        let mut kalman_gain = covariance * covariance.transpose();
        kalman_gain[(0, 0)] += 0.1;
        kalman_gain[(1, 1)] += 0.1;
        kalman_gain[(2, 2)] += 0.1;
        kalman_gain[(3, 3)] += 0.1;
        kalman_gain = covariance * kalman_gain.try_inverse().unwrap();

        let mut residual = sensor.state.to_matrixv4() - state.to_matrixv4();
        residual[(0, 0)] += 0.1;
        residual[(1, 0)] += 0.1;
        residual[(2, 0)] += 0.1;
        residual[(3, 0)] += 0.1;

        state = CarState::from_matrixv4(state.to_matrixv4() + kalman_gain * residual);
        covariance = (Matrix4::identity() - kalman_gain) * covariance;

        self.state = state;
        self.covariance = covariance;
        self.rectangular = self.state.to_rectangular(sensor.state.width.unwrap_or(40.0), sensor.state.length.unwrap_or(20.0));
        self.history.push((self.state.x, self.state.y, self.state.yaw, self.state.velocity, sensor.acceleration, sensor.steering_angle));
    }
}