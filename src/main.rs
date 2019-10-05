use glium as gl;
use gl::{glutin, Surface};
use std::time::Instant;

mod draw;
mod utilities;
use utilities::*;

fn main() {
    let (display, mut events_loop) =
        new_window("Hello World",
                   glutin::dpi::LogicalSize::new(1024.0, 768.0));

    let start = Instant::now();
    let mut closed = false;
    while !closed {
        events_loop.poll_events(|e| if window_closed(e) { closed = true });

        let mut frame = display.draw();

        if start.elapsed().as_millis() / 1000 % 2 == 0 {
            frame.clear_color(0.2, 0.3, 0.3, 1.0)
        } else {
            frame.clear_color(0.0, 1.0, 0.0, 1.0)
        }

        frame.finish().unwrap();
    }
}