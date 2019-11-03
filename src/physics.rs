use crate::boids;
use crate::config;
use assert_approx_eq::*;
use nalgebra_glm as na;
use std::f32::consts::PI;

#[derive(Debug, Clone, Copy)]
pub struct Entity {
    pub pos: na::Vec2,
    pub vel: na::Vec2,
    pub angular_vel: f32,
    pub rot: na::Vec2,
    pub mass: f32,
    pub area: f32,
    pub resultant_force: na::Vec2,
}

impl Default for Entity {
    fn default() -> Entity {
        Entity {
            pos: na::zero(),
            vel: na::zero(),
            angular_vel: 0.0,
            rot: na::vec2(1.0, 0.0),
            mass: 1.0,
            area: 0.25,
            resultant_force: na::zero(),
        }
    }
}

pub fn add_force(entity: &mut Entity, force: na::Vec2) {
    entity.resultant_force += force;
}

pub fn collinear_force(vector: na::Vec2, magnitude: f32) -> na::Vec2 {
    if vector == na::zero() {
        vector
    } else {
        magnitude * na::normalize(&vector)
    }
}

pub fn step_world(
    boids::World(world): &mut boids::World,
    delta_time: f32,
    config: &config::Config,
) {
    for entity in world {
        step_entity(entity, config.drag_coefficient, delta_time);
    }
}

pub fn step_entity(entity: &mut Entity, drag_coefficient: f32, delta_time: f32) {
    let mut dot_c = na::normalize_dot(&entity.vel, &entity.rot);
    if dot_c < 0.0 {
        dot_c = 0.0;
    }

    let drag = drag_force(
        entity.area + (4.0 * (1.0 - dot_c)),
        drag_coefficient,
        entity.vel,
    );
    add_force(entity, drag);

    let acceleration = entity.resultant_force / entity.mass;
    entity.resultant_force = na::zero();

    entity.vel = integrate(entity.vel, acceleration, delta_time);
    entity.pos = integrate(entity.pos, entity.vel, delta_time);
    if entity.pos.x > 1.0 {
        entity.pos.x = 1.0
    }
    if entity.pos.x < -1.0 {
        entity.pos.x = -1.0
    }
    if entity.pos.y > 1.0 {
        entity.pos.y = 1.0
    }
    if entity.pos.y < -1.0 {
        entity.pos.y = -1.0
    }

    entity.rot = na::rotate_vec2(&entity.rot, entity.angular_vel * delta_time);
}

pub fn integrate(value: na::Vec2, rate_of_change: na::Vec2, delta_time: f32) -> na::Vec2 {
    value + (rate_of_change * delta_time)
}

pub fn drag_force(area: f32, c: f32, vel: na::Vec2) -> na::Vec2 {
    let vel_magnitude_sq = na::magnitude2(&vel);
    let drag_magnitude = 0.5 * area * c * -vel_magnitude_sq;

    collinear_force(vel, drag_magnitude)
}

mod tests {
    use crate::physics::*;
    use crate::test_utils::*;
    use assert_approx_eq::*;
    use nalgebra_glm as na;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn test_drag_force(
            c in 0.0..2.0f32,
            vel in vec2(na::vec2(-2.0, -2.0), na::vec2(2.0, 2.0))
        ) {
            let drag = drag_force(1.0, c, vel);
            let delta_time = 0.016;
            let mass = 1.0;

            let new_vel = vel + ((drag / mass) * delta_time);

            assert_approx_eq!(na::angle(&vel, &new_vel), 0.0, 1e-3f32);
        }
    }
}
