use imageproc::noise;
use rand::rngs::ThreadRng;
use rand::thread_rng;
use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::fmt;

use crate::state::CarState;

//wheel encoder device structure
#[derive(Debug, Clone)]
pub struct Encoder {
    pub time_stamp: f64,            // Time stamp of the encoder reading
    pub name: String, // A descriptive name for the encoder (e.g., "Left Wheel Encoder")
    pub encoder_type: String, // Type of the encoder (e.g., "Quadrature", "Incremental", "Absolute")
    pub resolution: Option<u32>, // Encoder resolution (counts per revolution)
    pub reverse_direction: bool, // Indicates if the encoder counts need to be reversed
    pub count: i32,   // Current count or position of the encoder
    pub last_count: i32, // Previous count (for tracking changes)
    pub velocity: f64, // Angular velocity (radians per second or other units)
    pub noise_std_dev: Option<f64>, // Standard deviation of the noise (Gaussian noise)
}

impl Encoder {
    pub fn new(
        name: String,
        encoder_type: String,
        resolution: u32,
        reverse_direction: bool,
        noise_std_dev: Option<f64>,
    ) -> Self {
        Self {
            time_stamp: 0.0,
            name,
            encoder_type,
            resolution: Some(resolution),
            reverse_direction,
            count: 0,
            last_count: 0,
            velocity: 0.0,
            noise_std_dev: Some(0.0),
        }
    }
}

pub struct WheelEncoder {
    pub encoder_recorder: Vec<Encoder>,
    rng: ThreadRng,
    normal: Normal<f64>,
}

impl WheelEncoder {
    pub fn new() -> Self {
        Self {
            encoder_recorder: Vec::new(),
            rng: thread_rng(),
            normal: Normal::new(0.0, 0.1).unwrap(),
        }
    }

    pub fn from_carstate(&mut self, car: &CarState) {
        let mut encoder = Encoder::new(
            "Left Wheel Encoder".to_string(),
            "Quadrature".to_string(),
            10000,
            true,
            Some(0.0),
        );
        encoder.time_stamp = car.time_stamp;
        // Calculate the ground truth change in count
        //TODO: this is car speed, it should be the wheel angular speed, but i dont care for now
        let delta_count_gt = car.velocity * car.dt / encoder.resolution.unwrap() as f64;

        // Introduce noise (Gaussian noise with standard deviation noise_std_dev)
        let mut rng = rand::thread_rng();
        let noise = rng.gen::<f64>() * encoder.noise_std_dev.unwrap();

        // Apply noise to the change in count
        let delta_count_with_noise = delta_count_gt + noise;

        // Update the encoder's count based on the change (possibly reverse direction)
        if encoder.reverse_direction {
            encoder.count -= delta_count_with_noise as i32;
        } else {
            encoder.count += delta_count_with_noise as i32;
        }
        self.encoder_recorder.push(encoder.clone());
    }

    pub fn get_count(&mut self) -> i32 {
        self.encoder_recorder.last_mut().unwrap().count
    }
}

impl fmt::Display for WheelEncoder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let encoder = self.encoder_recorder.last().unwrap();
        write!(
            f,
            "Encoder: {{ time: {}, name: {}, encoder_type: {}, resolution: {:?}, reverse_direction: {}, count: {}, last_count: {}, velocity: {}, noise_std_dev: {:?} }}\n",
            encoder.time_stamp, encoder.name, encoder.encoder_type, encoder.resolution, encoder.reverse_direction, encoder.count, encoder.last_count, encoder.velocity, encoder.noise_std_dev
        )
    }
}
