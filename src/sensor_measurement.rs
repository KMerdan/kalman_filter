use rand::rngs::ThreadRng;
use rand::thread_rng;
use rand_distr::{Distribution, Normal};
use std::fmt;

use crate::sensors::{Encoder, GPS, IMU};
use crate::state::{CarColor, CarState};

use crate::state::Rectangular;

pub struct SensorSet {
    pub gps: GPS::GpsXYZ,
    pub imu: IMU::IMUDevice,
    pub encoder: Encoder::WheelEncoder,
    pub wheel_encoder: Vec<()>,
    pub measured_state: CarState,
}

impl SensorSet {
    pub fn new(actual_car: &CarState) -> Self {
        Self {
            gps: GPS::GpsXYZ::new(None),
            imu: IMU::IMUDevice::new(),
            encoder: Encoder::WheelEncoder::new(),
            wheel_encoder: Vec::new(),
            measured_state: actual_car.clone(),
        }
    }

    pub fn from_carstate(&mut self, car: &CarState) {
        self.gps.from_carstate(car);
        self.imu.from_carstate(car);
        self.encoder.from_carstate(car);
        // self.wheel_encoder.push(self.encoder.from_carstate(car).clone());
    }

    pub fn get_observed_state(&mut self, car: &CarState) -> Rectangular{
        self.from_carstate(car);
        self.measured_state.time_stamp = self.gps.gps_values.last().unwrap().time_stamp;
        // self.measured_state.x = self.gps.get_local_xyz(None).x;
        // self.measured_state.y = self.gps.get_local_xyz(None).y;
        self.measured_state.x = self.imu.previous_x;
        self.measured_state.y = self.imu.previous_y;
        self.measured_state.yaw = self.imu.previous_yaw;
        self.measured_state.velocity = self.imu.previous_velocity;
        self.measured_state.to_rectangular(Some(CarColor::Red))
    }
}

impl fmt::Display for SensorSet {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SensorSet {{ Position: {}/{}, yaw: {}, velocity: {} }}",
            self.measured_state.x, self.measured_state.y, self.measured_state.yaw, self.measured_state.velocity
        )
    }
}
