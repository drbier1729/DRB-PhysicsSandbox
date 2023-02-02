#include "pch.h"
#include "PhysicsWorld.h"

namespace drb {
	namespace physics {

		World::World(Uint32 maxBodies, Uint32 maxColliders, Uint32 targetSubsteps_)
			: bodies{}, sphColliders{}, capColliders{}, cvxColliders{}, meshColliders{}, targetSubsteps{ targetSubsteps_ }
		{
			bodies.reserve(maxBodies);
			sphColliders.reserve(maxColliders);
			capColliders.reserve(maxColliders);
			cvxColliders.reserve(maxColliders);
			meshColliders.reserve(maxColliders);
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

			// Check rigidbodies
			for (auto&& rb : bodies)
			{
				Mat4 const rbTr = rb.GetTransformMatrix();

				for (auto&& s : rb.spheres)
				{
					AABB const b = MakeAABB(s.shape, rbTr * s.transform);
					if (auto result = RayCast(r, b); result.hit) {
						return RayCastHit{
							.result = result,
							.body = &rb,
							.sph = &s
						};
					}
				}

				for (auto&& s : rb.capsules)
				{
					AABB const b = MakeAABB(s.shape, rbTr * s.transform);
					if (auto result = RayCast(r, b); result.hit) {
						return RayCastHit{
							.result = result,
							.body = &rb,
							.cap = &s
						};
					}
				}

				for (auto&& s : rb.hulls)
				{
					AABB const b = MakeAABB(s.shape, rbTr * s.transform);
					if (auto result = RayCast(r, b); result.hit) {
						return RayCastHit{
							.result = result,
							.body = &rb,
							.cvx = &s
						};
					}
				}
			}


			// Check static geometry

			for (auto&& s : sphColliders)
			{
				AABB const b = MakeAABB(s.shape, s.transform);
				if (auto result = RayCast(r, b); result.hit) {
					return RayCastHit{
						.result = result,
						.sph = &s
					};
				}
			}

			for (auto&& s : capColliders)
			{
				AABB const b = MakeAABB(s.shape, s.transform);
				if (auto result = RayCast(r, b); result.hit) {
					return RayCastHit{
						.result = result,
						.cap = &s
					};
				}
			}

			for (auto&& s : cvxColliders)
			{
				AABB const b = MakeAABB(s.shape, s.transform);
				if (auto result = RayCast(r, b); result.hit) {
					return RayCastHit{
						.result = result,
						.cvx = &s
					};
				}
			}

			return RayCastHit{};
		}
	}
}