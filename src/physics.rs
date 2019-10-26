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

pub fn step_physics(entity: &mut Entity, delta_time: f32) {
    let resultant: na::Vec2 = entity.forces.iter().fold(na::zero(), |acc, e| acc + e);
    let acceleration = resultant / entity.mass;

    entity.vel = entity.vel + (acceleration * delta_time);
    entity.pos = entity.pos + (entity.vel * delta_time);
    entity.rot = na::rotate_vec2(&entity.rot, entity.angular_vel * delta_time);
}

pub fn drag_force(area: f32, c: f32, vel: &na::Vec2) -> na::Vec2 {
    let force_magnitude = 0.5 * area * c * na::dot(vel, vel);

    -force_magnitude * na::normalize(vel)
}

pub fn forward_force(entity: &Entity, magnitude: f32) -> na::Vec2 {
    magnitude * na::normalize(&entity.rot)
}
