use nalgebra::Matrix4;
use piston_window::color;
use std::fmt;
use std::time::{Duration, SystemTime};

use image::{ImageBuffer, Rgba, RgbaImage};
use imageproc::drawing::draw_line_segment_mut;

pub fn get_time_stamp() -> f64 {
    let now = SystemTime::now();
    let since_epoch = now.duration_since(SystemTime::UNIX_EPOCH).unwrap();
    let seconds = since_epoch.as_secs();
    let nanos = since_epoch.subsec_nanos();
    Duration::new(seconds, nanos).as_secs_f64()
}

#[derive(Debug, Copy, Clone)]
pub enum CarColor {
    Red,
    Green,
    Blue,
}

impl CarColor {
    pub fn to_rgba(&self) -> Rgba<u8> {
        match self {
            CarColor::Red => Rgba([255, 0, 0, 255]),
            CarColor::Green => Rgba([0, 255, 0, 255]),
            CarColor::Blue => Rgba([0, 0, 255, 255]),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Rectangular {
    pub color: CarColor,
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
    pub x3: f64,
    pub y3: f64,
    pub x4: f64,
    pub y4: f64,
}

impl Rectangular {
    pub fn new(color: Option<CarColor>) -> Self {
        Self {
            color: color.unwrap_or(CarColor::Blue),
            x1: 0.0,
            y1: 0.0,
            x2: 0.0,
            y2: 0.0,
            x3: 0.0,
            y3: 0.0,
            x4: 0.0,
            y4: 0.0,
        }
    }
    pub fn draw_rect(&self, image_buffer: &mut RgbaImage) {
        draw_line_segment_mut(
            image_buffer,
            (self.x1 as f32, self.y1 as f32),
            (self.x2 as f32, self.y2 as f32),
            self.color.to_rgba(),
        );
        draw_line_segment_mut(
            image_buffer,
            (self.x2 as f32, self.y2 as f32),
            (self.x3 as f32, self.y3 as f32),
            self.color.to_rgba(),
        );
        draw_line_segment_mut(
            image_buffer,
            (self.x3 as f32, self.y3 as f32),
            (self.x4 as f32, self.y4 as f32),
            self.color.to_rgba(),
        );
        draw_line_segment_mut(
            image_buffer,
            (self.x4 as f32, self.y4 as f32),
            (self.x1 as f32, self.y1 as f32),
            self.color.to_rgba(),
        );
    }
}

#[derive(Debug, Copy, Clone)]
pub struct CarState {
    pub dt: f64,
    pub time_stamp: f64,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub velocity: f64,
    pub width: Option<f64>,
    pub length: Option<f64>,
}

impl CarState {
    pub fn new() -> Self {
        Self {
            dt: 0.1,
            time_stamp: get_time_stamp(),
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            velocity: 0.0,
            width: Some(40.0),
            length: Some(20.0),
        }
    }
}

//implement the addition and subtraction for car_state
impl std::ops::Add for CarState {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            dt: self.dt,
            time_stamp: self.time_stamp,
            x: self.x + other.x,
            y: self.y + other.y,
            yaw: self.yaw + other.yaw,
            velocity: self.velocity + other.velocity,
            width: self.width,
            length: self.length,
        }
    }
}

impl std::ops::Sub for CarState {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            dt: self.dt,
            time_stamp: self.time_stamp,
            x: self.x - other.x,
            y: self.y - other.y,
            yaw: self.yaw - other.yaw,
            velocity: self.velocity - other.velocity,
            width: self.width,
            length: self.length,
        }
    }
}

//implement the multiplication for car_state
impl std::ops::Mul<f64> for CarState {
    type Output = Self;
    fn mul(self, other: f64) -> Self {
        Self {
            dt: self.dt,
            time_stamp: self.time_stamp,
            x: self.x * other,
            y: self.y * other,
            yaw: self.yaw * other,
            velocity: self.velocity * other,
            width: self.width,
            length: self.length,
        }
    }
}

impl CarState {
    pub fn to_rectangular(&mut self, color: Option<CarColor>) -> Rectangular {
        let width = self.width.unwrap_or(40.0);
        let length = self.length.unwrap_or(20.0);
        let mut rect = Rectangular::new(color);
        let x1 = self.x - (width / 2.0);
        let y1 = self.y - (length / 2.0);
        let x3 = self.x + (width / 2.0);
        let y3 = self.y + (length / 2.0);
        let x2 = self.x - (width / 2.0);
        let y2 = self.y + (length / 2.0);
        let x4 = self.x + (width / 2.0);
        let y4 = self.y - (length / 2.0);

        // Rotate the four corners of the car based on the yaw.
        let sin = self.yaw.sin();
        let cos = self.yaw.cos();

        let rotate = |x: f64, y: f64| (x * cos - y * sin, x * sin + y * cos);

        (rect.x1, rect.y1) = rotate(x1, y1);
        (rect.x2, rect.y2) = rotate(x2, y2);
        (rect.x3, rect.y3) = rotate(x3, y3);
        (rect.x4, rect.y4) = rotate(x4, y4);
        rect
    }
    pub fn to_matrixv4(&self) -> Matrix4<f64> {
        let mut matrix = Matrix4::identity();
        matrix[(0, 0)] = self.x;
        matrix[(1, 0)] = self.y;
        matrix[(2, 0)] = self.yaw;
        matrix[(3, 0)] = self.velocity;
        matrix
    }

    pub fn from_matrixv4(matrix: Matrix4<f64>) -> Self {
        Self {
            dt: 0.1,
            time_stamp: get_time_stamp(),
            x: matrix[(0, 0)],
            y: matrix[(1, 0)],
            yaw: matrix[(2, 0)],
            velocity: matrix[(3, 0)],
            width: None,
            length: None,
        }
    }

    pub fn update_time_stamp(&mut self) {
        self.time_stamp = get_time_stamp();
    }
}

impl fmt::Display for CarState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "CarState {{ time: {}, x: {}, y: {}, yaw: {}, velocity: {}, width: {:?}, length: {:?} }}\n",
            self.time_stamp, self.x, self.y, self.yaw, self.velocity, self.width, self.length
        )
    }
}

//test the car state
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_car_state() {
        let mut car_state = CarState::new();
        car_state.x = 1.0;
        car_state.y = 2.0;
        car_state.yaw = 3.0;
        car_state.velocity = 4.0;
        car_state.width = Some(5.0);
        car_state.length = Some(6.0);
        car_state.update_time_stamp();
        println!("{}", car_state);
    }
    #[test]
    fn test_car_state_to_rectangular() {
        let mut car_state = CarState::new();
        car_state.x = 1.0;
        car_state.y = 2.0;
        car_state.yaw = 3.0;
        car_state.velocity = 4.0;
        car_state.width = Some(5.0);
        car_state.length = Some(6.0);
        car_state.update_time_stamp();
        let rectangular = car_state.to_rectangular(Some(CarColor::Red));
        println!("{:?}", rectangular);
    }
}
