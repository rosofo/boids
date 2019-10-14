use gl::{glutin, Surface};
use glium as gl;
use nalgebra_glm as na;
use std::time::Instant;

mod draw;
mod geometry;
mod physics;
mod utilities;
use utilities::*;

fn main() {
    let (display, mut events_loop) =
        new_window("Boids", glutin::dpi::LogicalSize::new(1024.0, 768.0));

    let mut entities = [
        physics::entity(na::vec2(0.0, 0.0), 1.0),
        physics::entity(na::vec2(0.1, 0.3), 2.0),
        physics::entity(na::vec2(0.2, 0.45), 5.0)
    ];
    let mut closed = false;
    let mut delta = Instant::now();
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


        if delta.elapsed().as_secs_f64() >= 0.016 {
            for entity in &mut entities {
                physics::add_force(entity, &na::vec2(-0.01, 0.0));
                physics::step_physics(entity, 0.016);
            }

            delta = Instant::now();
        }

        let pos_vels = entities.iter().map(|e| (e.pos, e.vel));
        let mut boids: Vec<draw::Vertex> = vec![];
        for (pos, vel) in pos_vels {
            let model = geometry::position_velocity_to_model(&pos, &vel, &geometry::boid(0.05));
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
                &gl::index::NoIndices(gl::index::PrimitiveType::TrianglesList),
                &program,
                &gl::uniform! {},
                &Default::default(),
            )
            .unwrap();
        frame.finish().unwrap();
    }
}
