extern crate image;
extern crate imageproc;
extern crate nalgebra;
extern crate piston_window;

mod car;
mod kalman_filter;
mod sensor_measurement;
mod sensors;
mod state;

use car::Car;
use sensor_measurement::SensorSet;
use state::{CarColor, CarState, Rectangular};

use image::{ImageBuffer, Rgba, RgbaImage};

use piston_window::*;

fn main() {
    let mut i = 0;
    let mut car = Car::new(0.0, 240.0, 0.0, 40.0, 20.0, 0.0, 2.0, 0.5, 0.1, None);
    let mut sensor_measurement = SensorSet::new(&car.state);

    let screen_width = 640 * 2;
    let screen_height = 480;

    let mut window: PistonWindow = WindowSettings::new("Car", [screen_width, screen_height])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut image_buffer: RgbaImage = ImageBuffer::new(screen_width, screen_height);

    while let Some(event) = window.next() {
        let gt_viz_rect = car.step(0.1, 0.001);
        let sensor_viz_rect = sensor_measurement.get_observed_state(&car.state);
        println!("CarActual {{ Position: {}/{}, yaw: {}, velocity: {} }}", car.state.x, car.state.y, car.state.yaw, car.state.velocity);

        // Clear the image buffer and draw on it
        for pixel in image_buffer.pixels_mut() {
            *pixel = Rgba([1, 1, 1, 0]); // Set the background to transparent
        }

        gt_viz_rect.draw_rect(&mut image_buffer);
        sensor_viz_rect.draw_rect(&mut image_buffer);
        println!("{}",sensor_measurement);

        // Create a texture from the ImageBuffer
        let texture = Texture::from_image(
            &mut window.create_texture_context(),
            &image_buffer,
            &TextureSettings::new(),
        )
        .unwrap();

        // Display the texture on the window
        window.draw_2d(&event, |context, graphics, _| {
            clear([1.0; 4], graphics);
            image(&texture, context.transform, graphics);
        });

        i += 1;
        if i > 2000 {
            break;
        }
    }
}
