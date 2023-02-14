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


#ifdef DRB_PHYSICS_XPBD
		void World::Step(Float32 dt)
		{
			// Broadphase Collision Detection
			// ...


			// Narrowphase Collision Detection + Contact Generation
			DetectCollisions();


			// XPBD loop with substepping
			Float32 const h = dt / targetSubsteps;
			for (Uint32 i = 0; i < targetSubsteps; ++i)
			{
				for (auto&& rb : bodies) { rb.ProjectPositions(h); }

				// Solve Position constraints
				// - Collision Resolution
				// - HeightMap constraint/collision

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
#else
		void World::Step(Float32 dt)
		{
			// Broadphase Collision Detection
			// ...


			// Narrowphase Collision Detection + Contact Generation
			DetectCollisions();

			// Sequential Impulses loop with substepping
			for (auto&& rb : bodies) { rb.ProjectForces(dt); }

			Float32 const h = dt / targetSubsteps;
			for (Uint32 i = 0; i < targetSubsteps; ++i)
			{
				// Solve Velocity constraints
				// - Collision Resolution
				// - HeightMap constraint/collision
			}

			for (auto&& rb : bodies) { rb.ProjectVelocities(dt); }

			// Collision Callbacks
			// ...
		}

#endif

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

		void World::DetectCollisions()
		{
			Uint32 const numBodies = static_cast<Uint32>(bodies.size());
			for (Uint32 i = 0; i < numBodies; ++i) 
			{
				Mat4 const trA   = bodies[i].GetTransformMatrix();
				auto const geomA = bodies[i].geometry;

				// Test against static geometry
				geomA->ForEachCollider([&]<Shape T>(CollisionShape<T> const& A) {

					Mat4 const shapeTrA = trA * A.transform;

					colliders.ForEachCollider([&]<Shape U>(CollisionShape<U> const& B) {

						ContactManifold const m = Collide(A.shape, shapeTrA, B.shape, B.transform);
						ManifoldKey const key(&bodies[i], &A, &worldBody, &B);

						if (m.numContacts > 0)
						{
							auto it = contacts.find(key);
							if (it != contacts.end())
							{
								it->second = m;
							}
							else
							{
								contacts.emplace(key, m);
							}
						}
						else
						{
							contacts.erase(key);
						}
					});
				});

				// Test against other rigidbodies
				for (Uint32 j = i + 1; j < numBodies; ++j)
				{
					Mat4 const trB   = bodies[j].GetTransformMatrix();
					auto const geomB = bodies[j].geometry;

					geomA->ForEachCollider([&]<Shape T>(CollisionShape<T> const& A) {
						
						Mat4 const shapeTrA = trA * A.transform;
						
						geomB->ForEachCollider([&]<Shape U>(CollisionShape<U> const& B) {
							
							Mat4 const shapeTrB = trB * B.transform;

							ContactManifold const m = Collide(A.shape, shapeTrA, B.shape, shapeTrB);
							ManifoldKey const key(&bodies[i], &A, &bodies[j], &B);
							
							if (m.numContacts > 0)
							{
								auto it = contacts.find(key);
								if (it != contacts.end()) 
								{
									it->second = m;
								}
								else
								{
									contacts.emplace(key, m);
								}
							}
							else 
							{
								contacts.erase(key);
							}
						});
					});
				}
			}
		}
	}
}