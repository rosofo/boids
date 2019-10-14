use nalgebra_glm as na;

pub struct Entity {
    pub pos: na::Vec2,
    pub vel: na::Vec2,
    pub mass: f32,
    pub forces: Vec<na::Vec2>,
}

pub fn entity(pos: na::Vec2, mass: f32) -> Entity {
    Entity {
        pos: pos,
        vel: na::zero(),
        mass: mass,
        forces: vec![]
    }
}

pub fn add_force(entity: &mut Entity, force: &na::Vec2) {
    entity.forces.push(force.clone());
}

pub fn step_physics(entity: &mut Entity, delta_time: f32) {
    let resultant: na::Vec2 = entity.forces.iter().fold(na::zero(), |acc, e| acc + e);
    let acceleration = resultant / entity.mass;

    entity.vel = entity.vel + (acceleration * delta_time);
    entity.pos = entity.pos + (entity.vel   * delta_time);
}
