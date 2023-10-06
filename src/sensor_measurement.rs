use rand::rngs::ThreadRng;
use rand::thread_rng;
use rand_distr::{Distribution, Normal};

use crate::car::{Car, KinematicBicycleModel};
use crate::state::CarState;

pub struct SensorState {
    pub state: CarState,
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
    pub x3: f64,
    pub y3: f64,
    pub x4: f64,
    pub y4: f64,
    pub steering_angle: f64,
    pub acceleration: f64,
    rng: ThreadRng,
    normal: Normal<f64>,
    pub history: Vec<(f64, f64, f64, f64, f64, f64)>,
    observation_model: KinematicBicycleModel,
}

impl SensorState {
    pub fn new(car: &Car) -> Self {
        Self {
            state: car.state,
            x1: car.x1,
            y1: car.y1,
            x2: car.x2,
            y2: car.y2,
            x3: car.x3,
            y3: car.y3,
            x4: car.x4,
            y4: car.y4,
            steering_angle: car.steering_angle,
            acceleration: car.acceleration,
            rng: thread_rng(),
            normal: Normal::new(0.0, 0.1).unwrap(),
            history: Vec::new(),
            observation_model: KinematicBicycleModel::_new(
                car.model.wheelbase,
                car.model.max_steer,
                car.model.dt,
            ),
        }
    }

    // assume there are a wheel odometry and a steering angle sensor to this car
    // the wheel odometry gives the velocity of the car with some noise
    // the steering angle sensor gives the steering angle of the car also with some noise
    pub fn get_state_from_sensor(&mut self, car: &Car) {
        let accle_noise = self.normal.sample(&mut self.rng);
        let steering_noise = self.normal.sample(&mut self.rng);

        self.acceleration = car.acceleration + accle_noise;
        self.steering_angle = car.steering_angle + steering_noise;

        self.state = self.observation_model._update(
            self.state.x,
            self.state.y,
            self.state.yaw,
            self.state.velocity,
            self.acceleration,
            self.steering_angle,
        );
        self.history.push((
            self.state.x,
            self.state.y,
            self.state.yaw,
            self.state.velocity,
            car.acceleration,
            car.steering_angle,
        ));
    }

    pub fn get_rect(&mut self, car: &Car) {
        // self.get_state_from_sensor(car);
        let x1 = self.state.x - (car.width / 2.0);
        let y1 = self.state.y - (car.length / 2.0);
        let x3 = self.state.x + (car.width / 2.0);
        let y3 = self.state.y + (car.length / 2.0);
        let x2 = self.state.x - (car.width / 2.0);
        let y2 = self.state.y + (car.length / 2.0);
        let x4 = self.state.x + (car.width / 2.0);
        let y4 = self.state.y - (car.length / 2.0);

        //rotate the four corners of the car based on the yaw
        self.x1 = x1 * self.state.yaw.cos() - y1 * self.state.yaw.sin();
        self.y1 = x1 * self.state.yaw.sin() + y1 * self.state.yaw.cos();
        self.x2 = x2 * self.state.yaw.cos() - y2 * self.state.yaw.sin();
        self.y2 = x2 * self.state.yaw.sin() + y2 * self.state.yaw.cos();
        self.x3 = x3 * self.state.yaw.cos() - y3 * self.state.yaw.sin();
        self.y3 = x3 * self.state.yaw.sin() + y3 * self.state.yaw.cos();
        self.x4 = x4 * self.state.yaw.cos() - y4 * self.state.yaw.sin();
        self.y4 = x4 * self.state.yaw.sin() + y4 * self.state.yaw.cos();
    }
}
