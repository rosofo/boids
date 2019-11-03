use gl::glutin;
use glium as gl;
use nalgebra_glm as na;
use std::f32::consts::PI;

pub fn new_window(
    title: &str,
    size: glutin::dpi::LogicalSize,
) -> (gl::Display, glutin::EventsLoop) {
    let wb = gl::glutin::WindowBuilder::new()
        .with_dimensions(size)
        .with_title(title);

    let cb = glutin::ContextBuilder::new();

    let events_loop = glutin::EventsLoop::new();
    let display = gl::Display::new(wb, cb, &events_loop).unwrap();

    (display, events_loop)
}

pub fn window_closed(event: &glutin::Event) -> bool {
    match event {
        glutin::Event::WindowEvent {
            event: glutin::WindowEvent::CloseRequested,
            ..
        } => true,
        _ => false,
    }
}

pub fn mean(vectors: &[na::Vec2]) -> na::Vec2 {
    let sum: na::Vec2 = vectors.iter().sum();
    let length: f32 = vectors.len() as f32;
    if length > 0.0 {
        sum / length
    } else {
        sum
    }
}

pub fn clockwise_angle_from_j_hat(vec: &na::Vec2) -> f32 {
    if vec == &na::zero() {
        return 0.0;
    }
    let vec = vec.normalize();

    let angle = vec.angle(&na::vec2(0.0, 1.0));
    if vec.x < 0.0 {
        (2.0 * PI) - angle
    } else {
        angle
    }
}

pub fn executable_directory() -> Option<std::path::PathBuf> {
    let executable_path = std::env::current_exe().unwrap();
    executable_path.parent().map(std::path::Path::to_path_buf)
}

mod tests {
    use crate::test_utils::vec2;
    use crate::utilities::*;
    use assert_approx_eq::*;
    use nalgebra_glm as na;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn test_clockwise_angle_from_j_hat_bounded(
            vec in vec2(na::vec2(-1.0, -1.0), na::vec2(1.0, 1.0))
        ) {
            let angle = clockwise_angle_from_j_hat(&vec);
            assert!(angle <= 2.0 * PI);
            assert!(angle >= 0.0);
        }

        #[test]
        fn test_clockwise_angle_from_j_hat_greater_than_pi_when_negative_x(
            vec in vec2(na::vec2(-1.0, -1.0), na::vec2(-0.01, 1.0))
        ) {
            let angle = clockwise_angle_from_j_hat(&vec);
            assert!(angle > PI);
        }
    }

    #[test]
    fn test_quarter_pis() {
        let i_hat = na::vec2(1.0, 0.0);
        let j_hat = na::vec2(0.0, 1.0);
        let inv_i_hat = na::vec2(-1.0, 0.0);
        let inv_j_hat = na::vec2(0.0, -1.0);

        assert_approx_eq!(clockwise_angle_from_j_hat(&i_hat), PI / 2.0);
        assert_approx_eq!(clockwise_angle_from_j_hat(&j_hat), 0.0);
        assert_approx_eq!(clockwise_angle_from_j_hat(&inv_i_hat), PI + (PI / 2.0));
        assert_approx_eq!(clockwise_angle_from_j_hat(&inv_j_hat), PI);
    }
}
