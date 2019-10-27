use glium::index::PrimitiveType;
use nalgebra_glm as na;

pub struct Model(pub Vec<na::Vec3>, pub PrimitiveType);

pub fn boid(scale: f32) -> Model {
    let m = na::scaling2d(&na::vec2(scale, scale));
    Model(
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
        .collect(),
        PrimitiveType::TrianglesList,
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

pub fn position_rotation_to_model(
    pos: &na::Vec2,
    vel: &na::Vec2,
    Model(verts, kind): &Model,
) -> Model {
    Model(
        verts
            .iter()
            .map(|v| position_rotation_to_matrix(pos, vel) * v)
            .collect(),
        *kind,
    )
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
