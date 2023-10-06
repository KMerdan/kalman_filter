
use rand::Rng;
use rand::rngs::ThreadRng;
use rand::thread_rng;
use rand_distr::{Distribution, Normal};

//9-axis IMU device structure
#[derive(Debug, Clone)]
pub struct IMU {
    pub acce_x: f64,
    pub acce_y: f64,
    pub gyro_z: f64,
    rng: ThreadRng,
    normal: Normal<f64>,
    previous_gyro_z: f64,

}

//implement a method where it takes ground velocity and yaw, and reverse calculate the IMU data in high frequency.
impl IMU {
    pub fn new() -> Self {
        Self {
            acce_x: 0.0,
            acce_y: 0.0,
            gyro_z: 0.0,
            rng: thread_rng(),
            normal: Normal::new(0.0, 0.1).unwrap(),
            previous_gyro_z: 0.0,

        }
    }
    pub fn get_imu(&mut self, velocity: f64, yaw: f64) {
        let acce_noise = self.normal.sample(&mut self.rng);
        let gyro_noise = self.normal.sample(&mut self.rng);
        self.acce_x = velocity * yaw.cos() + acce_noise * 0.1;
        self.acce_y = velocity * yaw.sin() + acce_noise * 0.1;
        self.gyro_z = self.previous_gyro_z - (yaw + gyro_noise * 0.1);
        self.previous_gyro_z = self.gyro_z;

    }
}



#[derive(Debug, Copy, Clone)]
pub struct GPSPoint {
    pub time_stamp: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub speed: f64,
    pub course: f64,
}

#[derive(Debug, Copy, Clone)]
pub struct XYZValues {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

// Gps device structure
#[derive(Debug, Clone)]
pub struct GPS {
    pub gps_values: Vec<GPSPoint>,
    pub xyz_values: Vec<XYZValues>,
    earth_radius : f64,
}
//implement gps value to cartisian coordinate
impl GPS {

    pub fn new(earth_radius:Option<f64>) -> Self {
        Self {
            gps_values: Vec::new(),
            xyz_values: Vec::new(),
            earth_radius: earth_radius.unwrap_or(6371000.0),
        }
    }

    pub fn record(&mut self, time_stamp: f64, latitude: f64, longitude: f64, altitude: f64, speed: f64, course: f64) {
        self.gps_values.push(GPSPoint {
            time_stamp,
            latitude : latitude.to_radians(),
            longitude : longitude.to_radians(),
            altitude : altitude.to_radians(),
            speed,// only will calculate the speed when converting to cartesian
            course,
        });
    }

    // Method to convert GPS coordinates to Cartesian coordinates (ECEF)
    pub fn to_cartesian(&mut self, idx: Option<usize>){
        let _idx = idx.unwrap_or(self.gps_values.len()-1);
        let x = self.earth_radius * self.gps_values[_idx].latitude.cos() * self.gps_values[_idx].longitude.cos();
        let y = self.earth_radius * self.gps_values[_idx].latitude.cos() * self.gps_values[_idx].longitude.sin();
        let z = self.earth_radius * self.gps_values[_idx].latitude.sin();
        self.xyz_values.push(XYZValues {
            x,
            y,
            z,
        });
        self.gps_values[_idx].speed = self.calculate_speed(Some(_idx));
    }

    pub fn get_xyz(&mut self, idx: usize) -> XYZValues {
        println!("xyz_values: {:?}", self.xyz_values);
        self.xyz_values[idx]
    }

    pub fn calculate_speed(&mut self, idx:Option<usize>) -> f64 {
        let idx = idx.unwrap_or(self.xyz_values.len()-1);
        let x = self.xyz_values[idx].x - self.xyz_values[idx-1].x;
        let y = self.xyz_values[idx].y - self.xyz_values[idx-1].y;
        let z = self.xyz_values[idx].z - self.xyz_values[idx-1].z;
        let speed = (x.powi(2) + y.powi(2) + z.powi(2)).sqrt();
        speed
    }

    //Method to calculate the convariance of the gps by using the speed and history of the gps
    pub fn get_covariance(&mut self, idx: Option<usize>) -> f64 {
        let idx = idx.unwrap_or(self.xyz_values.len()-1);
        let mut speed_history = Vec::new();
        for i in 0..idx {
            speed_history.push(self.gps_values[i].speed);
        }
        let mean = speed_history.iter().sum::<f64>() / speed_history.len() as f64;
        let mut variance = 0.0;
        for i in 0..idx {
            variance += (speed_history[i] - mean).powi(2);
        }
        variance = variance / speed_history.len() as f64;
        variance
    }

}

//wheel encoder device structure
#[derive(Debug, Clone)]
pub struct WheelEncoder {
    pub name: String,           // A descriptive name for the encoder (e.g., "Left Wheel Encoder")
    pub encoder_type: String,   // Type of the encoder (e.g., "Quadrature", "Incremental", "Absolute")
    pub resolution: u32,       // Encoder resolution (counts per revolution)
    pub reverse_direction: bool, // Indicates if the encoder counts need to be reversed
    pub count: i32,            // Current count or position of the encoder
    pub last_count: i32,       // Previous count (for tracking changes)
    pub velocity: f64,         // Angular velocity (radians per second or other units)
    pub noise_std_dev: f64,    // Standard deviation of the noise (Gaussian noise)
}

impl WheelEncoder {
    pub fn new(name: String, encoder_type: String, resolution: u32, reverse_direction: bool) -> Self {
        Self {
            name,
            encoder_type,
            resolution,
            reverse_direction,
            count: 0,
            last_count: 0,
            velocity: 0.0,
            noise_std_dev: 0.0,
        }
    }

    pub fn get_count(&mut self, velocity: f64, delta_time: f64) {
        // Calculate the ground truth change in count
        let delta_count_gt = velocity * delta_time / self.resolution as f64;

        // Introduce noise (Gaussian noise with standard deviation noise_std_dev)
        let mut rng = rand::thread_rng();
        let noise = rng.gen::<f64>() * self.noise_std_dev;

        // Apply noise to the change in count
        let delta_count_with_noise = delta_count_gt + noise;

        // Update the encoder's count based on the change (possibly reverse direction)
        if self.reverse_direction {
            self.count -= delta_count_with_noise as i32;
        } else {
            self.count += delta_count_with_noise as i32;
        }
    }
}
