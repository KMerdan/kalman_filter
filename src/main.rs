extern crate piston_window;
extern crate image;
extern crate imageproc;
extern crate nalgebra;

mod car;
mod state;
mod kalman_filter;
mod sensor_measurement;

use car::Car;
use sensor_measurement::SensorState;

use piston_window::*;
use image::{Rgba, RgbaImage, ImageBuffer};
use imageproc::drawing::draw_line_segment_mut; 


fn main() {
    let mut i = 0;
    let mut car = Car::new(0.0, 0.0, 0.0, 20.0, 40.0, 0.0, 2.0, 0.5, 0.1);
    let mut sensor = SensorState::new(&car);
    
    let mut window: PistonWindow = WindowSettings::new("Car", [640, 480])
        .exit_on_esc(true)
        .build()
        .unwrap();
    let mut image_buffer: RgbaImage = ImageBuffer::new(640, 480);

    let ground_truth_colcor = Rgba([0, 255, 0, 255]);
    let measurement_color = Rgba([255, 0, 0, 255]);

    while let Some(event) = window.next() {
        car.update(0.1, 0.01);
        sensor.get_state_from_sensor(&car);
        sensor.get_rect(&car);
   
                
        // Clear the image buffer and draw on it
        for pixel in image_buffer.pixels_mut() {
            *pixel = Rgba([1, 1, 1, 0]); // Set the background to transparent
        }
        
        // draw ground truth car
        draw_line_segment_mut(&mut image_buffer, (car.x1 as f32, car.y1 as f32), (car.x2 as f32, car.y2 as f32), ground_truth_colcor);
        draw_line_segment_mut(&mut image_buffer, (car.x2 as f32, car.y2 as f32), (car.x3 as f32, car.y3 as f32), ground_truth_colcor);
        draw_line_segment_mut(&mut image_buffer, (car.x3 as f32, car.y3 as f32), (car.x4 as f32, car.y4 as f32), ground_truth_colcor);
        draw_line_segment_mut(&mut image_buffer, (car.x4 as f32, car.y4 as f32), (car.x1 as f32, car.y1 as f32), ground_truth_colcor);  

        // draw measurement car
        draw_line_segment_mut(&mut image_buffer, (sensor.x1 as f32, sensor.y1 as f32), (sensor.x2 as f32, sensor.y2 as f32), measurement_color);
        draw_line_segment_mut(&mut image_buffer, (sensor.x2 as f32, sensor.y2 as f32), (sensor.x3 as f32, sensor.y3 as f32), measurement_color);
        draw_line_segment_mut(&mut image_buffer, (sensor.x3 as f32, sensor.y3 as f32), (sensor.x4 as f32, sensor.y4 as f32), measurement_color);
        draw_line_segment_mut(&mut image_buffer, (sensor.x4 as f32, sensor.y4 as f32), (sensor.x1 as f32, sensor.y1 as f32), measurement_color);


        // Create a texture from the ImageBuffer
        let texture = Texture::from_image(&mut window.create_texture_context(), &image_buffer, &TextureSettings::new()).unwrap();

        // Display the texture on the window
        window.draw_2d(&event, |context, graphics, _| {
            clear([1.0; 4], graphics);
            image(&texture, context.transform, graphics);
        });


        i += 1;
        if i > 1000 {
            break;
        }
    }
}
