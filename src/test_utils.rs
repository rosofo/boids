use nalgebra_glm as na;
use proptest::prelude::*;
use crate::physics::Entity;

prop_compose! {
    pub fn vec2(min: na::Vec2, max: na::Vec2)
            (x in min.x..max.x, y in min.y..max.y) -> na::Vec2 {
                na::vec2(x, y)
    }
}

prop_compose! {
    pub fn entity_at_pos(min: na::Vec2, max: na::Vec2)(vector in vec2(min, max)) -> Entity {
        Entity {
            pos: vector,
            .. Default::default()
        }
    }
}
