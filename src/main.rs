use gl::glutin;
use gl::Surface;
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
use physics::Entity;
use utilities::*;

fn main() {
    let executable_directory = utilities::executable_directory().unwrap();
    let config_file_path = executable_directory.join(std::path::Path::new("boids.toml"));
    let config_file_path_str = config_file_path
        .to_str()
        .expect("Tried to convert config file path to str, but it contains non UTF8 characters");

    let mut session_config = config::read(config_file_path_str);
    let (config_receiver, _hotwatch) = config::watch(config_file_path_str.to_owned());

    let influence_radius = boids::goals::InfluenceRadius(session_config.influence_radius);

    let goal_functions: Vec<Box<dyn Fn(&physics::Entity, &[&Entity]) -> boids::goals::Goal>> = vec![
        //Box::new(|boid, _| boids::goals::static_goal(boid, na::vec2(0.75, 0.0))),
        Box::new(|_, world| boids::goals::center_of_mass(world, influence_radius)),
        Box::new(|_, world| boids::goals::keep_distance(world, influence_radius, 0.025)),
        Box::new(|_, world| {
            boids::goals::Goal(boids::goals::same_direction(world, influence_radius).0 * 1.5)
        }),
        Box::new(|boid, _| {
            boids::goals::Goal(boids::goals::bound(boid, na::vec2(1.0, 1.0), 0.3).0 * 20.0)
        }),
    ];

    fn config_model_funcs(
        config: &config::Config,
    ) -> Vec<Box<dyn Fn(&physics::Entity) -> geometry::Model>> {
        let drag_coefficient = config.drag_coefficient;
        let max_force = config.max_force;

        vec![
            Box::new(|entity| {
                geometry::boid().map(|v| {
                    let matrix = geometry::position_rotation_to_matrix(&entity.pos, &entity.rot);
                    matrix * na::scaling2d(&na::vec2(0.005, 0.005)) * v
                })
            }),
            /*Box::new(move |entity| {
                geometry::arrow_vector(
                    &(4.0 * physics::drag_force(0.25, drag_coefficient, entity.vel)),
                )
                .map(|v| na::translation2d(&entity.pos) * na::scaling2d(&na::vec2(0.1, 0.1)) * v)
            }),
            Box::new(|entity| {
                geometry::arrow_vector(&entity.vel).map(|v| {
                    na::translation2d(&entity.pos) * na::scaling2d(&na::vec2(0.1, 0.1)) * v
                })
            }),
            Box::new(move |entity| {
                geometry::arrow_vector(
                    &((entity.resultant_force
                        - physics::drag_force(0.25, drag_coefficient, entity.vel))
                        * max_force
                        * 4.0),
                )
                .map(|v| na::translation2d(&entity.pos) * na::scaling2d(&na::vec2(0.1, 0.1)) * v)
            }),*/
        ]
    }

    let mut model_funcs = config_model_funcs(&session_config);

    let (display, mut events_loop) =
        new_window("Boids", glutin::dpi::LogicalSize::new(1024.0, 768.0));
    let program = draw::simple_program(&display).unwrap();

    let mut drawer = draw::drawer(display, program);

    let mut world = generation::random_world(
        session_config.population,
        na::vec2(-1.0, -1.0),
        na::vec2(1.0, 1.0),
    );
    let mut delta = Instant::now();

    let mut closed = false;
    while !closed {
        events_loop.poll_events(|e| {
            if window_closed(&e) {
                closed = true
            }
        });

        if let Ok(new_config) = config_receiver.try_recv() {
            println!("config file read");
            session_config = new_config;
            model_funcs = config_model_funcs(&session_config);
        }

        if delta.elapsed().as_secs_f64() >= 0.016 {
            if session_config.behaviour_active {
                world = boids::step_world(&world, &session_config, &goal_functions[..]);
            }

            physics::step_world(&mut world, 0.016, &session_config);
            delta = Instant::now();
        }

        let models = geometry::world_to_models(&world, &model_funcs);

        for model in models {
            drawer.add_model(model);
        }

        let mut frame = drawer.display.draw();
        frame.clear_color(0.0, 0.0, 0.0, 1.0);
        drawer.draw(&mut frame);
        frame.finish().unwrap();
    }
}
