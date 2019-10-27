use crate::boids::World;
use crate::physics::Entity;
use nalgebra_glm as na;
use rand::prelude::*;
use std::f32::consts::PI;

pub fn random_world(population: u32, min_pos: na::Vec2, max_pos: na::Vec2) -> World {
    let mut rng = thread_rng();
    let mut boids = Vec::new();

    for _ in 0..population {
        let pos = na::vec2(
            rng.gen_range(min_pos.x, max_pos.x),
            rng.gen_range(min_pos.y, max_pos.y),
        );
        let rot = na::rotate_vec2(&na::vec2(1.0, 0.0), rng.gen_range(0.0, 2.0 * PI));

        boids.push(Entity {
            pos: pos,
            rot: rot,
            ..Default::default()
        });
    }

    World(boids)
}
