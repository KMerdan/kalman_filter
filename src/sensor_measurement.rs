use rand::rngs::ThreadRng;
use rand::thread_rng;
use rand_distr::{Distribution, Normal};

use crate::car::{Car, KinematicBicycleModel};
use crate::state::CarState;

use crate::state::Rectangular;


pub struct SensorState {
    pub state: CarState,
    pub rectangular: Rectangular,
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
            rectangular: car.rectangular,
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
        let noise_ratio = 0.05;
        let accle_noise = self.normal.sample(&mut self.rng)*noise_ratio;
        let steering_noise = self.normal.sample(&mut self.rng)*noise_ratio;

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

        self.rectangular = self.state.to_rectangular(car.width, car.length);
    }
}

