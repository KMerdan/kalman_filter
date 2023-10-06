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

impl Rectangular {
    pub fn new() -> Self {
        Self {
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
}
#[derive(Debug, Copy, Clone)]
pub struct CarState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub velocity: f64,
}

impl CarState {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            velocity: 0.0,
        }
    }
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

impl CarState {
    pub fn to_rectangular(&mut self, width: f64, length: f64)-> Rectangular{
        let mut rect = Rectangular::new();
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
}
