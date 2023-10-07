use rand::rngs::ThreadRng;
use rand::thread_rng;
use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::fmt;

use crate::state::{get_time_stamp, CarState};

//9-axis IMU device structure
#[derive(Debug, Copy, Clone)]
pub struct IMU9Axis {
    pub time_stamp: f64,
    pub acce_x: f64,
    pub acce_y: f64,
    acc_z: f64,
    gyro_x: f64,
    gyro_y: f64,
    pub gyro_z: f64,
    mag_x: f64,
    mag_y: f64,
    mag_z: f64,
}

pub struct IMUDevice {
    initial:bool,
    pub imu_recorder: Vec<IMU9Axis>,
    rng: ThreadRng,
    normal: Normal<f64>,
    pub previous_yaw: f64,
    pub previous_velocity: f64,
    pub previous_x: f64,
    pub previous_y: f64,
}

//implement a method where it takes ground velocity and yaw, and reverse calculate the IMU data in high frequency.
impl IMU9Axis {
    pub fn new() -> Self {
        Self {
            time_stamp: get_time_stamp(),
            acce_x: 0.0,
            acce_y: 0.0,
            acc_z: 0.0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            mag_x: 0.0,
            mag_y: 0.0,
            mag_z: 0.0,
        }
    }
}

impl IMUDevice {
    pub fn new() -> Self {
        Self {
            initial: true,
            imu_recorder: vec![IMU9Axis::new()],
            rng: thread_rng(),
            normal: Normal::new(0.0, 0.15).unwrap(),
            previous_yaw: 0.0,
            previous_velocity: 0.0,
            previous_x: 0.0,
            previous_y: 0.0,
        }
    }

    pub fn from_carstate(&mut self, car: &CarState) {
        let gyro_noise_ratio = 0.01;
        let acce_noise_ratio = 0.01;
        let gyro_noise = self.normal.sample(&mut self.rng) * gyro_noise_ratio;
        let acce_noise = self.normal.sample(&mut self.rng) * acce_noise_ratio;
        if self.initial {
            self.previous_yaw = car.yaw;
            self.previous_velocity = car.velocity;
            self.previous_x = car.x;
            self.previous_y = car.y;
            self.initial = false;
        }
        let mut imu_data = IMU9Axis::new();
        imu_data.time_stamp = get_time_stamp();
        // let dt = imu_data.time_stamp - self.imu_recorder.last().unwrap().time_stamp;
        let dt = car.dt;
        imu_data.acce_x = ((car.velocity - self.previous_velocity )* car.yaw.cos() + acce_noise)/dt;
        imu_data.acce_y = ((car.velocity - self.previous_velocity )* car.yaw.sin() + acce_noise)/dt;
        imu_data.gyro_z = ((car.yaw + gyro_noise)- self.previous_yaw)/dt;
        println!("car_yaw.cos: {}, car_yaw.sin: {}", car.yaw.cos(), car.yaw.sin());
        self.imu_recorder.push(imu_data.clone());
        self.get_imu_velocity_yaw(None, Some(dt));

    }

    pub fn get_imu_data(&mut self, idx: Option<usize>) -> IMU9Axis {
        self.imu_recorder[idx.unwrap_or(self.imu_recorder.len() - 1)]
    }

    pub fn get_imu_velocity_yaw(&mut self, idx: Option<usize>, dt:Option<f64>) {
        let _acce_x = self.imu_recorder[idx.unwrap_or(self.imu_recorder.len() - 1)].acce_x;
        let _acce_y = self.imu_recorder[idx.unwrap_or(self.imu_recorder.len() - 1)].acce_y;
    
        let _dt = self.imu_recorder[idx.unwrap_or(self.imu_recorder.len() - 1)].time_stamp
        - self.imu_recorder[idx.unwrap_or(self.imu_recorder.len() - 2)].time_stamp;
        

        self.previous_velocity = self.previous_velocity + (_acce_x.powi(2) + _acce_y.powi(2)).sqrt() * dt.unwrap_or(_dt);
        self.previous_x = self.previous_x + self.previous_velocity * dt.unwrap_or(_dt) * self.previous_yaw.cos();
        self.previous_y = self.previous_y + self.previous_velocity * dt.unwrap_or(_dt) * self.previous_yaw.sin();
        println!("previous_yaw.cos: {}, previous_yaw.sin: {}", self.previous_yaw.cos(), self.previous_yaw.sin());
        self.previous_yaw = self.previous_yaw + self.imu_recorder[idx.unwrap_or(self.imu_recorder.len() - 1)].gyro_z * dt.unwrap_or(_dt);
    }
}

impl fmt::Display for IMUDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let imu_data = self.imu_recorder.last().unwrap();
        write!(
            f,
            "IMU data @{}: \n\tacce_x: {}\n\tacce_y: {}\n\tacce_z: {}\n\tgyro_x: {}\n\tgyro_y: {}\n\tgyro_z: {}\n\tmag_x: {}\n\tmag_y: {}\n\tmag_z: {}\n",
            imu_data.time_stamp,
            imu_data.acce_x,
            imu_data.acce_y,
            imu_data.acc_z,
            imu_data.gyro_x,
            imu_data.gyro_y,
            imu_data.gyro_z,
            imu_data.mag_x,
            imu_data.mag_y,
            imu_data.mag_z,
        )
    }
}
