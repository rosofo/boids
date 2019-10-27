use nalgebra_glm as na;
use std::collections::VecDeque;

use crate::physics::*;

pub struct World(pub Vec<Entity>);

impl World {
    pub fn map_with_rest_of_world(&self, f: impl Fn(&Entity, &World) -> Entity) -> World {
        if self.0.is_empty() {
            return World(Vec::new());
        }

        let mut result = vec![];
        let mut deque = VecDeque::from(self.0.clone());
        for _ in 0..deque.len() {
            let current = deque.pop_front().unwrap();

            result.push(f(&current, &World(Vec::from(deque.clone()))));

            deque.push_back(current);
        }

        World(result)
    }
}

pub fn origin_at_boid(boid: &Entity, World(boids): &World) -> World {
    World(
        boids
            .iter()
            .map(|b| {
                let mut cloned = b.clone();
                cloned.pos = cloned.pos - boid.pos;
                cloned
            })
            .collect(),
    )
}

pub mod goals {
    use crate::boids::*;
    use crate::utilities::*;
    use nalgebra_glm as na;

    /// 0 <= magnitude <= 1
    pub struct Goal(pub na::Vec2);

    /// 0 <= magnitude <= 1
    pub struct ResultantGoal(pub na::Vec2);

    #[derive(Clone, Copy)]
    /// Must never be zero
    pub struct InfluenceRadius(pub f32);

    pub fn resultant_goal<F>(
        boid: &Entity,
        other_boids: &World,
        radius: InfluenceRadius,
        goal_functions: &[F],
    ) -> ResultantGoal
    where
        F: Fn(&Entity, &World) -> Goal,
    {
        let influential_boids = region_of_influence(boid, other_boids, radius);

        let goals: Vec<na::Vec2> = goal_functions
            .iter()
            .map(|f| f(boid, &influential_boids).0)
            .collect();
        ResultantGoal(mean(&goals))
    }

    pub fn region_of_influence(
        boid: &Entity,
        world: &World,
        InfluenceRadius(radius): InfluenceRadius,
    ) -> World {
        let boid_perspective_world = origin_at_boid(boid, world);
        World(
            boid_perspective_world
                .0
                .into_iter()
                .filter(|b| na::magnitude(&b.pos) <= radius)
                .collect(),
        )
    }

    pub fn center_of_mass(World(boids): &World, InfluenceRadius(radius): InfluenceRadius) -> Goal {
        let positions: Vec<na::Vec2> = boids.iter().map(|boid| boid.pos).collect();
        Goal(mean(&positions) / radius)
    }

    mod tests {
        use crate::boids::goals::*;
        use crate::test_utils::*;
        use nalgebra_glm as na;
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn test_region_of_influence(boids in proptest::collection::vec(entity_at_pos(na::vec2(-3.0, -3.0), na::vec2(3.0, 3.0)), 10..20)) {
                let origin_boid: Entity = Entity { pos: na::zero(), .. Default::default() };
                let world = World(boids);
                let World(influential_boids) = region_of_influence(&origin_boid, &world, InfluenceRadius(2.0));

                for boid in influential_boids {
                    assert!(na::magnitude(&boid.pos) <= 2.0)
                }
            }
        }

        proptest! {
            #[test]
            fn test_center_of_mass(boids in proptest::collection::vec(entity_at_pos(na::vec2(-2.0, -2.0), na::vec2(2.0, 2.0)), 0..10)) {
                let origin_boid: Entity = Entity { pos: na::zero(), .. Default::default() };
                let world = World(boids);
                let influential_boids = region_of_influence(&origin_boid, &world, InfluenceRadius(2.0));
                let Goal(goal) = center_of_mass(&influential_boids, InfluenceRadius(2.0));

                assert!(na::magnitude(&goal) <= 1.0);
            }
        }
    }
}

/// Strategies translate goals into boidss
pub mod strategies {
    use crate::boids::goals::ResultantGoal;
    use crate::physics;
    use nalgebra_glm as na;
    use std::f32::consts::PI;

    pub fn v1(
        ResultantGoal(g): ResultantGoal,
        boid: &physics::Entity,
        max_ang_vel: f32,
        max_force: f32,
    ) -> physics::Entity {
        let angle_between = na::angle(&g, &boid.rot);

        let angle_coefficient = if angle_between <= PI {
            (PI - angle_between) / PI
        } else {
            (angle_between % PI) / PI
        };
        let force =
            physics::forward_force(&boid, angle_coefficient * na::magnitude(&g) * max_force);

        let angle_coefficient = if angle_between <= PI {
            angle_between / PI
        } else {
            (PI - (angle_between % PI)) / PI
        };
        let ang_vel = angle_coefficient * max_ang_vel;

        let mut updated = boid.clone();
        physics::add_force(&mut updated, &force);
        updated.angular_vel = ang_vel;
        updated
    }

    mod tests {
        use crate::boids::strategies::*;
        use crate::physics::Entity;
        use crate::test_utils::*;
        use assert_approx_eq::*;
        use nalgebra_glm as na;
        use proptest::prelude::*;
        use std::f32::consts::PI;

        // I wrote this test before the function, and it ended up being one of those times
        // where in trying to write the test, I just wrote the function logic instead.
        // I even copy-pasted some of it to the real thing!
        // This is not what I imagine effective property testing to be.
        proptest! {
            #[test]
            fn test_v1(
                resultant_goal in vec2(na::vec2(-1.0, -1.0), na::vec2(1.0, 1.0))
                    .prop_filter("Vector magnitudes must be 0 <= magnitude <= 1".to_owned(),
                                 |v| na::magnitude(v) <= 1.0),
                rot_angle in 0.0..(2.0 * std::f32::consts::PI),
                max_ang_vel in 0.01..2.0f32,
                max_force in 0.01..2.0f32
            ) {
                // case when angle of resultant goal equal to entity angle, i.e. it is facing the right direction already
                // then F = |goal| * max_force
                let boid = Entity {
                    rot: resultant_goal.clone(),
                    .. Default::default()
                };

                let updated = v1(ResultantGoal(resultant_goal), &boid, max_ang_vel, max_force);

                assert_approx_eq!(na::magnitude(&updated.forces[0]), na::magnitude(&resultant_goal) * max_force, 1e-3f32);

                // otherwise
                // F = angle_coefficient * |goal| * max_force

                let boid = Entity {
                    rot: na::rotate_vec2(&na::vec2(1.0, 0.0), rot_angle),
                    .. Default::default()
                };

                let updated = v1(ResultantGoal(resultant_goal), &boid, max_ang_vel, max_force);
                let angle = na::angle(&resultant_goal, &boid.rot);
                let angle_ratio = if angle <= PI { (PI - angle) / PI } else { angle % PI / PI };

                assert_approx_eq!(na::magnitude(&updated.forces[0]),
                                  angle_ratio * na::magnitude(&resultant_goal) * max_force,
                                  1e-3f32);

                let force = &updated.forces[0];
                assert!(na::magnitude(force) <= max_force);
                assert!(na::are_collinear2d(force, &boid.rot, 0.001));
                if force != &na::zero() { assert_approx_eq!(na::angle(force, &boid.rot), 0.0, 1e-3f32) };
                assert!(updated.angular_vel <= max_ang_vel);
            }
        }
    }
}
