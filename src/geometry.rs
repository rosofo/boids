use nalgebra_glm as na;

pub type Model = Vec<na::Vec3>;

pub fn boid(scale: f32) -> Model {
    let m = na::scaling2d(&na::vec2(scale, scale));
    vec![
        na::vec3(0.0, 1.0, 1.0),
        na::vec3(-1.0, -1.0, 1.0),
        na::vec3(0.0, 0.0, 1.0),
        na::vec3(0.0, 1.0, 1.0),
        na::vec3(1.0, -1.0, 1.0),
        na::vec3(0.0, 0.0, 1.0),
    ]
    .iter()
    .map(|v| m * v)
    .collect()
}

pub fn position_velocity_to_matrix(pos: &na::Vec3, vel: &na::Vec3) -> na::Mat3 {
    let angle = na::angle(&na::vec2(0.0, 1.0), &vel.xy());
    na::translation2d(&pos.xy()) * na::rotation2d(angle)
}

pub fn position_velocity_to_model(pos: &na::Vec3, vel: &na::Vec3, model: &[na::Vec3]) -> Model {
    model
        .iter()
        .map(|v| position_velocity_to_matrix(pos, vel) * v)
        .collect()
}


#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use nalgebra_glm::normalize as normalize;

    proptest! {
        #[test]
        fn na_angle(x in -1.0..1.0, y in -1.0..1.0) {
            if x != 0.0 && y != 0.0 {
                assert_eq!(na::angle(&normalize(&na::vec2(x, 0.0)), &normalize(&na::vec2(0.0, y))), 1.5707963267948966);
            }
        }
    }
}