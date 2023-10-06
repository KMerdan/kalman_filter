#[derive(Debug, Copy, Clone)]
pub struct CarState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub velocity: f64,
}

//implement the addition and subtraction for car_state
impl std::ops::Add for CarState {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            yaw: self.yaw + other.yaw,
            velocity: self.velocity + other.velocity,
        }
    }
}

impl std::ops::Sub for CarState {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            yaw: self.yaw - other.yaw,
            velocity: self.velocity - other.velocity,
        }
    }
}

//implement the multiplication for car_state
impl std::ops::Mul<f64> for CarState {
    type Output = Self;
    fn mul(self, other: f64) -> Self {
        Self {
            x: self.x * other,
            y: self.y * other,
            yaw: self.yaw * other,
            velocity: self.velocity * other,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Rectangular {
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
    pub x3: f64,
    pub y3: f64,
    pub x4: f64,
    pub y4: f64,
}
