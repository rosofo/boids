use crate::boids::World;
use crate::physics::Entity;
use glium::index::PrimitiveType;
use nalgebra_glm as na;

pub struct Model(pub Vec<na::Vec3>, pub PrimitiveType);

impl Model {
    pub fn map<F: Fn(&na::Vec3) -> na::Vec3>(&self, f: F) -> Model {
        Model(self.0.iter().map(f).collect(), self.1)
    }
}

pub fn world_to_models<F: Fn(&Entity) -> Model>(
    World(entities): &World,
    funcs: &[F],
) -> Vec<Model> {
    entities
        .iter()
        .flat_map(|entity| funcs.iter().map(move |f| f(entity)))
        .collect()
}

pub fn boid() -> Model {
    Model(
        vec![
            na::vec3(0.0, 1.0, 1.0),
            na::vec3(-1.0, -1.0, 1.0),
            na::vec3(0.0, 0.0, 1.0),
            na::vec3(0.0, 1.0, 1.0),
            na::vec3(1.0, -1.0, 1.0),
            na::vec3(0.0, 0.0, 1.0),
        ],
        PrimitiveType::TrianglesList,
    )
}

pub fn arrow_vector(vector: &na::Vec2) -> Model {
    let magnitude = na::magnitude(vector);
    let scaled = magnitude * 0.2;
    let mut angle = na::angle(&na::vec2(0.0, 1.0), vector);
    if vector.x > 0.0 {
        angle = -angle;
    }

    Model(
        vec![
            na::vec3(0.0, 0.0, 1.0),
            na::vec3(0.0, magnitude, 1.0),
            na::vec3(0.0, magnitude, 1.0),
            na::vec3(-scaled, magnitude - scaled, 1.0),
            na::vec3(0.0, magnitude, 1.0),
            na::vec3(scaled, magnitude - scaled, 1.0),
        ]
        .iter()
        .map(|v| na::rotation2d(angle) * v)
        .collect(),
        PrimitiveType::LinesList,
    )
}

pub fn position_rotation_to_matrix(pos: &na::Vec2, rot: &na::Vec2) -> na::Mat3 {
    let mut rotation = na::normalize(&rot.xy());
    if rot.xy() == na::zero() {
        rotation = na::vec2(0.0, 1.0)
    }

    let mut angle = na::angle(&na::vec2(0.0, 1.0), &rotation);
    if rotation.x > 0.0 {
        angle = -angle
    }
    na::translation2d(&pos.xy()) * na::rotation2d(angle)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra_glm::normalize;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn na_angle(x in -1.0..1.0, y in -1.0..1.0) {
            if x != 0.0 && y != 0.0 {
                assert_eq!(na::angle(&normalize(&na::vec2(x, 0.0)), &normalize(&na::vec2(0.0, y))), 1.5707963267948966);
            }
        }
    }
}
