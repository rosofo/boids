use gl::{glutin, Surface};
use glium as gl;
use nalgebra_glm as na;

mod draw;
mod geometry;
mod utilities;
use utilities::*;

fn main() {
    let (display, mut events_loop) =
        new_window("Hello World", glutin::dpi::LogicalSize::new(1024.0, 768.0));

    let mut closed = false;
    while !closed {
        events_loop.poll_events(|e| {
            if window_closed(&e) {
                closed = true
            }

            match &e {
                glutin::Event::DeviceEvent { event, .. } => match event {
                    _ => (),
                },
                _ => (),
            };
        });

        let pvs = [(na::vec3(0.0, 0.0, 1.0), na::rotation2d(1.0) * na::vec3(0.0, 1.0, 1.0))];
        let mut boids: Vec<draw::Vertex> = vec![];
        for (pos, vel) in &pvs {
            let model = geometry::position_velocity_to_model(pos, vel, &geometry::boid(0.05));
            for v in &model {
                boids.push(draw::vec3_to_vertex(&v));
            }
        }

        let vertex_buffer = gl::vertex::VertexBuffer::new(&display, &boids).unwrap();
        let program = draw::simple_program(&display).unwrap();

        let mut frame = display.draw();
        frame.clear_color(0.0, 0.0, 0.0, 1.0);
        frame
            .draw(
                &vertex_buffer,
                &gl::index::NoIndices(gl::index::PrimitiveType::TriangleStrip),
                &program,
                &gl::uniform! {},
                &Default::default(),
            )
            .unwrap();
        frame.finish().unwrap();
    }
}
