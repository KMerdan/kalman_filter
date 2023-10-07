use rand::rngs::ThreadRng;
use rand::thread_rng;
use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::fmt;

use crate::state::CarState;
use crate::state::get_time_stamp;

#[derive(Debug, Copy, Clone)]
pub struct GPSPoint {
    pub time_stamp: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub speed: f64,
}

impl GPSPoint {
    pub fn new() -> Self {
        Self {
            time_stamp: get_time_stamp(),
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            speed: 0.0,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct XYZValues {
    pub time_stamp: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl XYZValues {
    pub fn new() -> Self {
        Self {
            time_stamp: get_time_stamp(),
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

impl std::ops::Sub for XYZValues {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            time_stamp: self.time_stamp,
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}


// Gps device structure
#[derive(Debug, Clone)]
pub struct GpsXYZ {
    pub initial: bool,
    pub gps_values: Vec<GPSPoint>,
    pub xyz_values: Vec<XYZValues>,
    pub covariances: Vec<f64>,
    earth_radius: f64,
    rng: ThreadRng,
    normal: Normal<f64>,
}

impl GpsXYZ {
    pub fn new(earth_radius: Option<f64>) -> Self {
        Self {
            initial: true,
            gps_values: Vec::new(),
            xyz_values: Vec::new(),
            covariances: Vec::new(),
            earth_radius: earth_radius.unwrap_or(6371000.0),
            rng: thread_rng(),
            normal: Normal::new(0.0, 0.1).unwrap(),
        }
    }

    pub fn from_carstate(&mut self, car: &CarState) {
        let noise_ratio = 0.1;
        let gps_noise = self.normal.sample(&mut self.rng) * noise_ratio;
        let gps_speed_noise = self.normal.sample(&mut self.rng) * noise_ratio * 100.0;

        let latitude = car.x.atan2(car.y) + gps_noise;
        let longitude = self.earth_radius.atan2(car.y) + gps_noise;
        let altitude = self.earth_radius + gps_noise;
        let current_time = car.time_stamp;
        self.gps_values.push(GPSPoint {
            time_stamp: current_time,
            latitude,
            longitude,
            altitude,
            speed: car.velocity + gps_speed_noise,
        });
        let x = car.x + gps_noise;
        let y = car.y + gps_noise;
        let z = car.yaw + gps_noise;
        self.xyz_values.push(XYZValues {
            x,
            y,
            z,
            time_stamp: current_time,
        });
    }

    pub fn get_local_xyz(&mut self, idx: Option<usize>) -> XYZValues {
        let index = idx.unwrap_or(self.gps_values.len() - 1);
        self.xyz_values[index]
    }
}

impl fmt::Display for GpsXYZ {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "GPS @{}: \n\tlatitude: {}\n\tlongitude: {}\n\taltitude: {}\n\tspeed: {}\n",
            self.gps_values.last().unwrap().time_stamp,
            self.gps_values.last().unwrap().latitude,
            self.gps_values.last().unwrap().longitude,
            self.gps_values.last().unwrap().altitude,
            self.gps_values.last().unwrap().speed
        )
    }
}
