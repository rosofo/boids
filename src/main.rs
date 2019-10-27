use gl::{glutin, Surface};
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

fn step_world<F>(
    world: &boids::World,
    config: &config::Config,
    goal_functions: &[F],
    delta_time: f32,
) -> boids::World where
    F: Fn(&physics::Entity, &boids::World) -> boids::goals::Goal,
{
    let mut new_world = world.clone().map_with_rest_of_world(|boid, world| {
        let resultant_goal =
            boids::goals::resultant_goal(boid, world, config.influence_radius, goal_functions);
        let updated =
            boids::strategies::v1(resultant_goal, boid, config.max_ang_vel, config.max_force);
        updated
    });

    new_world.0.iter_mut().for_each(|entity| {
        physics::step_entity_physics(entity, config.drag_coefficient, delta_time)
    });

    new_world
}

fn main() {
    let config = config::Config {
        influence_radius: boids::goals::InfluenceRadius(1.0),
        drag_coefficient: 1.0,
        population: 10,
        max_force: 0.1,
        max_ang_vel: 0.5,
    };

    let goal_functions: Vec<Box<dyn Fn(&physics::Entity, &boids::World) -> boids::goals::Goal>> = vec![
        Box::new(|_, world| boids::goals::center_of_mass(world, config.influence_radius)),
    ];

    let (display, mut events_loop) =
        new_window("Boids", glutin::dpi::LogicalSize::new(1024.0, 768.0));

    let mut world =
        generation::random_world(config.population, na::vec2(-1.0, -1.0), na::vec2(1.0, 1.0));
    let mut delta = Instant::now();

    let mut closed = false;
    while !closed {
        events_loop.poll_events(|e| {
            if window_closed(&e) {
                closed = true
            }
        });

        if delta.elapsed().as_secs_f64() >= 0.016 {
            world = step_world(&world, &config, &goal_functions[..], 0.016);
            delta = Instant::now();
        }

        let pos_rots = world.0.iter().map(|e| (e.pos, e.rot));
        let mut boids: Vec<draw::Vertex> = vec![];
        for (pos, rot) in pos_rots {
            let model = geometry::position_rotation_to_model(&pos, &rot, &geometry::boid(0.02));
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
