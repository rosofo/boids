use glium as gl;
use gl::{glutin};

pub fn new_window(title: &str, size: glutin::dpi::LogicalSize)
    -> (gl::Display, glutin::EventsLoop) {

    let wb = gl::glutin::WindowBuilder::new()
        .with_dimensions(size)
        .with_title(title);

    let cb = glutin::ContextBuilder::new();

    let events_loop = glutin::EventsLoop::new();
    let display = gl::Display::new(wb, cb, &events_loop).unwrap();

    return (display, events_loop);
}

pub fn window_closed(event: glutin::Event) -> bool {
    match event {
        glutin::Event::WindowEvent { event: glutin::WindowEvent::CloseRequested, .. }
            => true,
        _   => false,
    }
}