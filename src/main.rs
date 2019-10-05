use glium as gl;
use gl::{glutin, Surface};
use nalgebra_glm as na;

mod draw;
mod utilities;
use utilities::*;

fn main() {
    let (display, mut events_loop) =
        new_window("Hello World",
                   glutin::dpi::LogicalSize::new(1024.0, 768.0));

    let mut closed = false;
    while !closed {
        events_loop.poll_events(|e| if window_closed(e) { closed = true });

        let m = na::mat2x2(0.2, 0.0, 0.0, 0.2);
        let shape: Vec<draw::Vertex> = [
            na::vec2(-0.1, -0.3),
            na::vec2(0.0, 0.0),
            na::vec2(0.0, -0.2),
            na::vec2(0.1, -0.3),
        ].iter().map(|v| draw::Vertex { position: (m * v).into() }).collect();

        let vertex_buffer = gl::vertex::VertexBuffer::new(&display, &shape).unwrap();
        let indices: gl::IndexBuffer<u16>
            = gl::index::IndexBuffer::new(
                &display,
                gl::index::PrimitiveType::TrianglesList,
                &[0, 1, 2, 3, 2, 1]
            ).unwrap();
        let program = draw::simple_program(&display).unwrap();

        let mut frame = display.draw();
        frame.draw(&vertex_buffer, &indices, &program, &gl::uniform! {},
                   &Default::default()).unwrap();
        frame.finish().unwrap();
    }
}