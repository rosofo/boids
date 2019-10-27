use crate::boids::goals::InfluenceRadius;

pub struct Config {
    pub influence_radius: InfluenceRadius,
    pub drag_coefficient: f32,
    pub population: u32,
    pub max_force: f32,
    pub max_ang_vel: f32,
}
