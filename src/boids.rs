use crate::config;
use crate::physics;
use crate::physics::Entity;

pub struct World(pub Vec<Entity>);

impl World {
    pub fn map_with_rest_of_world(&self, f: impl Fn(&mut Entity, &[&Entity])) -> Self {
        let mut new_world = Vec::new();

        for (index, entity) in self.0.iter().enumerate() {
            let mut new_boid = entity.clone();

            let rest_left = self.0[0..index].iter();
            let rest_right = self.0[index + 1..self.0.len()].iter();
            let rest_of_world: Vec<&Entity> = rest_left.chain(rest_right).collect();

            f(&mut new_boid, &rest_of_world);
            new_world.push(new_boid);
        }

        Self(new_world)
    }

    pub fn to_refs(&self) -> Vec<&Entity> {
        self.0.iter().collect()
    }
}

pub fn step_world<F>(world: &World, config: &config::Config, goal_functions: &[F]) -> World
where
    F: Fn(&physics::Entity, &[&Entity]) -> goals::Goal,
{
    world.map_with_rest_of_world(|boid, entities| {
        let resultant_goal = goals::resultant_goal(
            boid,
            entities,
            goals::InfluenceRadius(config.influence_radius),
            goal_functions,
        );

        strategies::v1(boid, resultant_goal, config.max_ang_vel, config.max_force)
    })
}

pub fn origin_at_boid(boid: &Entity, boids: &[&Entity]) -> World {
    let mut result = Vec::new();
    for b in boids {
        let mut cloned = (*b).clone();
        cloned.pos -= boid.pos;
        result.push(cloned);
    }

    World(result)
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
        other_boids: &[&Entity],
        radius: InfluenceRadius,
        goal_functions: &[F],
    ) -> ResultantGoal
    where
        F: Fn(&Entity, &[&Entity]) -> Goal,
    {
        let influential_boids = region_of_influence(boid, other_boids, radius);

        let goals: Vec<na::Vec2> = goal_functions
            .iter()
            .map(|f| f(boid, &influential_boids.to_refs()).0)
            .collect();

        ResultantGoal(mean(&goals))
    }

    pub fn region_of_influence(
        boid: &Entity,
        boids: &[&Entity],
        InfluenceRadius(radius): InfluenceRadius,
    ) -> World {
        let boid_perspective_world = origin_at_boid(boid, boids);
        World(
            boid_perspective_world
                .0
                .into_iter()
                .filter(|b| na::magnitude(&b.pos) <= radius)
                .collect(),
        )
    }

    pub fn center_of_mass(boids: &[&Entity], InfluenceRadius(radius): InfluenceRadius) -> Goal {
        let positions: Vec<na::Vec2> = boids.iter().map(|boid| boid.pos).collect();
        Goal(mean(&positions) / radius)
    }

    pub fn keep_distance(
        boids: &[&Entity],
        InfluenceRadius(radius): InfluenceRadius,
        distance_coefficient: f32,
    ) -> Goal {
        let positions = boids.iter().map(|boid| boid.pos);
        let adjusted_radius = radius * distance_coefficient;

        Goal(
            positions
                .map(|pos| {
                    let magnitude = na::magnitude(&pos);
                    if magnitude > adjusted_radius {
                        na::zero()
                    } else {
                        (radius - na::magnitude(&pos)) * na::normalize(&-pos)
                    }
                })
                .sum(),
        )
    }

    pub fn same_direction(boids: &[&Entity], InfluenceRadius(radius): InfluenceRadius) -> Goal {
        let rotations: Vec<na::Vec2> = boids.iter().map(|boid| boid.rot).collect();

        Goal(mean(&rotations) / radius)
    }

    pub fn bound(boid: &Entity, dimensions: na::Vec2, min_dist: f32) -> Goal {
        let mut goals = vec![];

        let x_dist = (dimensions.x - boid.pos.x) / min_dist;
        let neg_x_dist = (-dimensions.x - boid.pos.x).abs() / min_dist;
        let y_dist = (dimensions.y - boid.pos.y) / min_dist;
        let neg_y_dist = (-dimensions.y - boid.pos.y).abs() / min_dist;

        if x_dist < 1.0 {
            goals.push(na::vec2(-(1.0 - x_dist), 0.0));
        }
        if neg_x_dist < 1.0 {
            goals.push(na::vec2(1.0 - neg_x_dist, 0.0));
        }
        if y_dist < 1.0 {
            goals.push(na::vec2(0.0, -(1.0 - y_dist)));
        }
        if neg_y_dist < 1.0 {
            goals.push(na::vec2(0.0, 1.0 - neg_y_dist));
        }

        Goal(goals.iter().sum())
    }

    pub fn static_goal(boid: &Entity, position: na::Vec2) -> Goal {
        let relative_position = position - boid.pos;
        Goal(na::normalize(&relative_position))
    }

    mod tests {
        use crate::boids::goals::*;
        use crate::test_utils::*;
        use nalgebra_glm as na;
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn test_region_of_influence(
                radius in 0.1..3.0f32,
                boids in proptest::collection::vec(entity_at_pos(na::vec2(-3.0, -3.0), na::vec2(3.0, 3.0)), 10..20)
            ) {
                let world = World(boids);
                let origin_boid = &world.0[0];
                let World(influential_boids) = region_of_influence(origin_boid, &world.to_refs(), InfluenceRadius(radius));

                for boid in influential_boids {
                    assert!(na::magnitude(&boid.pos) <= radius)
                }
            }
        }

        proptest! {
            #[test]
            fn test_center_of_mass(boids in proptest::collection::vec(entity_at_pos(na::vec2(-2.0, -2.0), na::vec2(2.0, 2.0)), 0..10)) {
                let origin_boid: Entity = Entity { pos: na::zero(), .. Default::default() };
                let world = World(boids);
                let influential_boids = region_of_influence(&origin_boid, &world.to_refs(), InfluenceRadius(2.0));
                let Goal(goal) = center_of_mass(&influential_boids.to_refs(), InfluenceRadius(2.0));

                assert!(na::magnitude(&goal) <= 1.0);
            }
        }
    }
}

/// Strategies translate goals into behaviour
pub mod strategies {
    use crate::boids::goals::ResultantGoal;
    use crate::physics;
    use crate::utilities::clockwise_angle_from_j_hat;
    use nalgebra_glm as na;
    use std::f32::consts::PI;

    const TAU: f32 = 2.0 * PI;

    pub fn clockwise_angle_relative_to(vec: &na::Vec2, base: &na::Vec2) -> f32 {
        let vec_angle = clockwise_angle_from_j_hat(&vec);
        let base_angle = clockwise_angle_from_j_hat(&base);

        let adjusted_angle = vec_angle - base_angle;
        if adjusted_angle < 0.0 {
            TAU + adjusted_angle
        } else {
            adjusted_angle
        }
    }

    pub fn steering_angle(rot: &na::Vec2, goal: &na::Vec2) -> f32 {
        let cw_angle = clockwise_angle_relative_to(rot, goal);

        if cw_angle > PI {
            -(TAU - cw_angle)
        } else {
            cw_angle
        }
    }

    pub fn v1(
        boid: &mut physics::Entity,
        ResultantGoal(g): ResultantGoal,
        max_ang_vel: f32,
        max_force: f32,
    ) {
        let angle = steering_angle(&g, &boid.rot);
        let angle_coefficient = -(angle / PI);
        let force_coefficient = (PI - angle.abs()) / PI;

        let force = physics::collinear_force(
            boid.rot,
            std::f32::consts::E.powf(6.0 * (force_coefficient - 1.0))
                * na::magnitude(&g)
                * max_force,
        );
        let ang_vel = angle_coefficient * max_ang_vel;

        physics::add_force(boid, force);
        boid.angular_vel = ang_vel;
        boid.area = 0.25 + (1.0 - force_coefficient) * 2.0;
    }

    mod tests {
        use crate::boids::strategies::*;
        use crate::physics::Entity;
        use crate::test_utils::*;
        use assert_approx_eq::*;
        use nalgebra_glm as na;
        use proptest::prelude::*;
        use std::f32::consts::PI;

        proptest! {
            #[test]
            fn test_angle_relative_bounded(
                vec_a in vec2(na::vec2(-1.0, -1.0), na::vec2(1.0, 1.0)),
            ) {
                let base = na::vec2(0.0, 1.0);
                assert!(clockwise_angle_relative_to(&vec_a, &base) <= 2.0 * PI);
                assert!(clockwise_angle_relative_to(&vec_a, &base) >= 0.0);

                assert!(clockwise_angle_relative_to(&base, &vec_a) <= 2.0 * PI);
                assert!(clockwise_angle_relative_to(&base, &vec_a) >= 0.0);
            }

            #[test]
            fn test_angle_relative_within_range(
                vec_a in vec2(na::vec2(-1.0, 0.01), na::vec2(1.0, 1.0)),
                vec_b in vec2(na::vec2(-1.0, -1.00), na::vec2(1.0, -0.01)),
            ) {
                let base = na::vec2(1.0, 0.0);
                assert!(clockwise_angle_relative_to(&vec_a, &base) > PI);
                assert!(clockwise_angle_relative_to(&vec_b, &base) < PI);
            }
        }

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
                let mut boid = Entity {
                    rot: resultant_goal.clone(),
                    .. Default::default()
                };

                v1(&mut boid, ResultantGoal(resultant_goal), max_ang_vel, max_force);

                assert_approx_eq!(na::magnitude(&boid.resultant_force), na::magnitude(&resultant_goal) * max_force, 1e-3f32);

                // otherwise
                // F = angle_coefficient * |goal| * max_force

                let mut boid = Entity {
                    rot: na::rotate_vec2(&na::vec2(1.0, 0.0), rot_angle),
                    .. Default::default()
                };

                v1(&mut boid, ResultantGoal(resultant_goal), max_ang_vel, max_force);
                let angle = na::angle(&resultant_goal, &boid.rot);
                let angle_ratio = if angle <= PI { (PI - angle) / PI } else { angle % PI / PI };

                assert_approx_eq!(na::magnitude(&boid.resultant_force),
                                  angle_ratio * na::magnitude(&resultant_goal) * max_force,
                                  1e-3f32);

                let force = &boid.resultant_force;
                assert!(na::magnitude(force) <= max_force);
                assert!(na::are_collinear2d(force, &boid.rot, 0.001));
                if force != &na::zero() { assert_approx_eq!(na::angle(force, &boid.rot), 0.0, 1e-3f32) };
                assert!(boid.angular_vel <= max_ang_vel);
            }
        }

        #[test]
        fn unit_test_v1() {
            for (g, expected_av, expected_f) in &[
                (na::vec2(0.5, 0.0), 0.5, 0.25),
                (na::vec2(0.0, -1.0), 1.0, 0.0),
            ] {
                let resultant_goal = ResultantGoal(*g);
                let rot = na::vec2(0.0, 1.0);

                let mut boid = Entity {
                    rot: rot,
                    ..Default::default()
                };

                let updated = v1(&mut boid, resultant_goal, 1.0, 1.0);
                let force = &boid.resultant_force;
                let ang_vel = boid.angular_vel;

                assert_approx_eq!(na::angle(force, &rot), 0.0, 1e-3f32);
                assert_approx_eq!(na::magnitude(force), expected_f);
                assert_approx_eq!(ang_vel, expected_av);
            }
        }
    }
}
