use nalgebra_glm as na;

#[derive(Debug)]
pub struct Entity {
    pub pos: na::Vec2,
    pub vel: na::Vec2,
    pub angular_vel: f32,
    pub rot: na::Vec2,
    pub mass: f32,
    pub forces: Vec<na::Vec2>,
}

impl Default for Entity {
    fn default() -> Entity {
        Entity {
            pos: na::zero(),
            vel: na::zero(),
            angular_vel: 0.0,
            rot: na::vec2(1.0, 0.0),
            mass: 1.0,
            forces: vec![],
        }
    }
}

impl Clone for Entity {
    fn clone(&self) -> Entity {
        Entity {
            pos: self.pos.clone(),
            vel: self.vel.clone(),
            angular_vel: self.angular_vel,
            rot: self.rot.clone(),
            mass: self.mass,
            forces: self.forces.clone(),
        }
    }
}

pub fn add_force(entity: &mut Entity, force: &na::Vec2) {
    entity.forces.push(force.clone());
}

pub fn step_entity_physics(entity: &mut Entity, drag_coefficient: f32, delta_time: f32) {
    let drag = drag_force(1.0, drag_coefficient, &entity.vel);
    add_force(entity, &drag);

    let resultant: na::Vec2 = entity.forces.iter().fold(na::zero(), |acc, e| acc + e);
    let acceleration = resultant / entity.mass;

    entity.vel = entity.vel + (acceleration * delta_time);
    entity.pos = entity.pos + (entity.vel * delta_time);
    entity.rot = na::rotate_vec2(&entity.rot, entity.angular_vel * delta_time);
}

pub fn drag_force(area: f32, c: f32, vel: &na::Vec2) -> na::Vec2 {
    let force_magnitude = 0.5 * area * c * na::dot(vel, vel);

    if force_magnitude == 0.0 { na::zero() }
    else { -force_magnitude * na::normalize(vel) }
}

pub fn forward_force(entity: &Entity, magnitude: f32) -> na::Vec2 {
    if magnitude < 0.0 { na::zero() }
    else { magnitude * na::normalize(&entity.rot) }
}

mod tests {
    use proptest::prelude::*;
    use crate::test_utils::*;
    use crate::physics::*;
    use nalgebra_glm as na;
    use assert_approx_eq::*;

    proptest! {
        #[test]
        fn test_drag_force(
            c in 0.0..2.0f32,
            vel in vec2(na::vec2(-2.0, -2.0), na::vec2(2.0, 2.0))
        ) {
            let drag = drag_force(1.0, c, &vel);
            let delta_time = 0.016;
            let mass = 1.0;

            let new_vel = vel + ((drag / mass) * delta_time);

            assert_approx_eq!(na::angle(&vel, &new_vel), 0.0, 1e-3f32);
        }
    }
}