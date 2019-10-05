use glium as gl;
use gl::{glutin, Surface};

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

        let mut frame = display.draw();

        let shape: Vec<draw::Vertex> = vec![];

        let vertex_buffer = glium::vertex::VertexBuffer::new(&display, &shape).unwrap();
        let indices = gl::index::NoIndices(gl::index::PrimitiveType::TrianglesList);

        let program = draw::simple_program(&display).unwrap();

        frame.draw(&vertex_buffer, &indices, &program, &gl::uniforms::EmptyUniforms,
                   &Default::default()).unwrap();
        frame.finish().unwrap();
    }
}