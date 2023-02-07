#include "pch.h"
#include "PhysicsWorld.h"

namespace drb {
	namespace physics {

		World::World(Uint32 maxBodies, Uint32 maxColliders, Uint32 targetSubsteps_)
			: bodies{}, 
			worldBody{},
			colliders{},
			contacts{},
			targetSubsteps{ targetSubsteps_ }
		{
			bodies.reserve(maxBodies);
			colliders.spheres.reserve(maxColliders);
			colliders.capsules.reserve(maxColliders);
			colliders.hulls.reserve(maxColliders);
		}


		void World::Step(Float32 dt)
		{
			// Broadphase Collision Detection
			// ...


			// Narrowphase Collision Detection + Contact Generation
			// ...


			// XPBD loop with substepping
			Float32 const h = dt / targetSubsteps;
			for (Uint32 i = 0; i < targetSubsteps; ++i)
			{
				for (auto&& rb : bodies) { rb.ProjectPositions(h); }

				// Solve Position constraints
				// - Collision Resolution

				for (auto&& rb : bodies) { rb.ProjectVelocities(h); }

				// Solve Velocity constraints
				// - Friction
				// - Damping
			}

			// Zero forces
			for (auto&& rb : bodies)
			{
				rb.accumulatedForces = Vec3(0);
				rb.accumulatedTorques = Vec3(0);
			}

			// Collision Callbacks
			// ...
		}


		RayCastHit World::RayCastQuery(Ray const& r)
		{
			// O(n^2) brute force -- can be simplified with broadphase structure
			RayCastHit result{};

			// Check rigidbodies
			for (auto&& rb : bodies)
			{
				Mat4 const rbTr = rb.GetTransformMatrix();

				rb.geometry->ForEachCollider([&]<Shape T>(CollisionShape<T> const& s)
				{
					AABB const b = MakeAABB(s.shape, rbTr * s.transform);
					if (auto cast = RayCast(r, b); cast.distance < result.info.distance) 
					{
						result.info = cast;
						result.body = &rb;
						result.shape = &s;
					}
				});
			}

			// Check static geometry
			for (auto&& s : colliders.spheres)
			{
				AABB const b = MakeAABB(s.shape, s.transform);
				if (auto cast = RayCast(r, b); cast.distance < result.info.distance)
				{
					result.info = cast;
					result.shape = &s;
				}
			}

			for (auto&& s : colliders.capsules)
			{
				AABB const b = MakeAABB(s.shape, s.transform);
				if (auto cast = RayCast(r, b); cast.distance < result.info.distance)
				{
					result.info = cast;
					result.shape = &s;
				}
			}

			for (auto&& s : colliders.hulls)
			{
				AABB const b = MakeAABB(s.shape, s.transform);
				if (auto cast = RayCast(r, b); cast.distance < result.info.distance)
				{
					result.info = cast;
					result.shape = &s;
				}
			}

			return result;
		}
	}
}