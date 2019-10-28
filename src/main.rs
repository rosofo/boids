use gl::{glutin};
use glium as gl;
use nalgebra_glm as na;
use std::time::Instant;

mod boids;
mod config;
mod draw;
mod generation;
mod geometry;
mod physics;
mod test_utils;
mod utilities;
use utilities::*;

fn main() {
    let mut session_config = config::read_config("boids.toml");
    let (config_receiver, _hotwatch) = config::watch_config("boids.toml");

    let influence_radius = boids::goals::InfluenceRadius(session_config.influence_radius);
    let goal_functions: Vec<Box<dyn Fn(&physics::Entity, &boids::World) -> boids::goals::Goal>> = vec![
        Box::new(|_, world| boids::goals::center_of_mass(world, influence_radius)),
    ];

    let (display, mut events_loop) =
        new_window("Boids", glutin::dpi::LogicalSize::new(1024.0, 768.0));
    let program = draw::simple_program(&display).unwrap();

    let mut drawer = draw::drawer(display, program);

    let mut world =
        generation::random_world(session_config.population, na::vec2(-1.0, -1.0), na::vec2(1.0, 1.0));
    let mut delta = Instant::now();

    let mut closed = false;
    while !closed {
        events_loop.poll_events(|e| {
            if window_closed(&e) {
                closed = true
            }
        });

        session_config = config_receiver.try_recv().unwrap_or_else(|_| session_config);

        if delta.elapsed().as_secs_f64() >= 0.016 {
            world = boids::step_world(&world, &session_config, &goal_functions[..], 0.016);
            delta = Instant::now();
        }

        let pos_rots = world.0.iter().map(|e| (e.pos, e.rot));
        for (pos, rot) in pos_rots {
            let model = geometry::position_rotation_to_model(&pos, &rot, &geometry::boid(0.02));
            drawer.add_model(model);
        }

        let mut frame = drawer.display.draw();
        drawer.draw(&mut frame);
        frame.finish().unwrap();
    }
}
