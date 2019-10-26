use nalgebra_glm as na;

use crate::physics::*;

pub struct World(pub Vec<Entity>);

impl World {
    pub fn map_with_rest_of_world<A>(&self, f: impl Fn(&Entity, &World) -> A) -> Vec<A> {
        let mut rest = self.0.clone();
        let mut before = vec![];
        let mut results = vec![];

        let mut next_boid = rest.pop();
        while next_boid.is_some() {
            let current_boid = next_boid.unwrap();
            let other_boids = World(
                before
                    .iter()
                    .chain(rest.iter())
                    .map(|e| e.clone())
                    .collect(),
            );

            results.push(f(&current_boid, &other_boids));

            before.push(current_boid);
            next_boid = rest.pop();
        }

        results
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

pub mod behaviours {
    use crate::logic::*;
    use crate::utilities::*;
    use nalgebra_glm as na;

    pub struct Goal(pub na::Vec2); // 0 <= magnitude <= 1
    pub struct ResultantGoal(pub na::Vec2); // unbounded

    #[derive(Clone, Copy)]
    pub struct InfluenceRadius(pub f32);

    pub const ACTIVE_BEHAVIOURS: Vec<fn(&World) -> Goal> =
        vec![|world| center_of_mass(world, InfluenceRadius(1.0))];

    pub fn resultant_goal(
        boid: &Entity,
        other_boids: &World,
        radius: InfluenceRadius,
    ) -> ResultantGoal {
        let influential_boids = region_of_influence(boid, other_boids, radius);

        let goals = ACTIVE_BEHAVIOURS.iter().map(|f| f(&influential_boids).0);
        ResultantGoal(goals.sum())
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
}