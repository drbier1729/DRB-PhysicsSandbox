#include "pch.h"
#include "PhysicsWorld.h"

#include "RayCast.h"
#include "PhysicsGeometryQueries.h"

namespace drb::physics {

	World::World(Uint32 maxBodies, Uint32 maxColliders, Uint32 targetSubsteps_)
		: bodies{},
		worldBody{},
		colliders{},
		bvhTree{},
		bodyProxies{},
		staticProxies{},
		contacts{},
		targetSubsteps{ targetSubsteps_ },
		locked { false }
	{
		bodies.reserve(maxBodies);
		colliders.Reserve(maxColliders);
		bodyProxies.reserve(maxBodies);
		bvhTree.Reserve(maxBodies);
	}

	void World::Init()
	{
		// Lock the World so no new objects can be added
		locked = true;

		for (auto&& rb : bodies)
		{
			if (rb.geometry)
			{
				// Make a new vector to hold the proxies and reserve
				// the correct number of slots for all geometry
				auto& rbProxiesVec = bodyProxies.emplace_back();
				rbProxiesVec.reserve(rb.geometry->Size());
			
				// Create proxies and add them to the BVH
				Mat4 const rbTr = rb.GetTransformMatrix();
				rb.geometry->ForEachCollider([&]<Shape T>(CollisionShape<T> const& col) {

					AABB const bounds = MakeAABB(col.shape, rbTr * col.transform);
					CollisionProxy& proxy = rbProxiesVec.emplace_back(CollisionProxy{ 
						.rb = &rb, 
						.shape = &col 
					});

					proxy.bvHandle = bvhTree.Insert(bounds, &proxy);
				});
			}
		}

		// Reserve enough space for all the proxies for static geometry
		staticProxies.reserve(colliders.Size());

		// Create the proxies and add them to the BVH
		colliders.ForEachCollider([this]<Shape T>(CollisionShape<T> const& col) {
			
			AABB const bounds = MakeAABB(col.shape, col.transform);
			CollisionProxy& proxy = staticProxies.emplace_back(CollisionProxy{
				.rb = &worldBody,
				.shape = &col
			});
			proxy.bvHandle = bvhTree.Insert(bounds, &proxy);
			bvhTree.SetMoved(proxy.bvHandle, false);
		});

		// No need to call Bake, but we'll still lock the static colliders
		colliders.locked = true;
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
		DetectCollisionsBroad();

		// Narrowphase Collision Detection + Contact Generation
		DetectCollisionsNarrow();

		// Sequential Impulses loop with substepping
		for (auto&& rb : bodies) { rb.ProjectForces(dt); }

		Float32 const h = dt / targetSubsteps;
		for (Uint32 i = 0; i < targetSubsteps; ++i)
		{
			// Solve Velocity constraints
			// - Collision and Resting Contact Resolution
			// - HeightMap constraint/collision
		}

		for (Uint32 i = 0; i < bodies.size(); ++i)
		{
			RigidBody& rb = bodies[i];

			if (not rb.geometry) { rb.ProjectVelocities(dt); }
			else
			{
				// Cache pre-update transform matrix
				Mat4 const preTr = rb.GetTransformMatrix();

				// Update positions and orientations
				rb.ProjectVelocities(dt);

				// Get the post-update transform matrix
				Mat4 const postTr = rb.GetTransformMatrix();

				// Update broadphase tree
				for (auto&& proxy : bodyProxies[i])
				{
					AABB const localBounds = MakeAABB(*proxy.shape);
					AABB const preBounds   = localBounds.Transformed(preTr);
					AABB const postBounds  = localBounds.Transformed(postTr);

					Vec3 const disp = postBounds.Center() - preBounds.Center();

					Bool const moved = bvhTree.MoveBoundingVolume(proxy.bvHandle, postBounds.Union(preBounds), disp);
					if (moved)
					{
						movedLastFrame.push_back(proxy);
					}
					else
					{
						bvhTree.SetMoved(proxy.bvHandle, false);
					}
				}
			}
		}

		// Collision Callbacks
		// ...
	}

#endif

	std::pair<CollisionProxy, CastResult> World::RayCastQuery(Ray const& r)
	{
		CastResult     result{};
		CollisionProxy proxy{};

		bvhTree.Query(r, [&result, &proxy](BVHNode const& node, CastResult const& tmpResult) -> Bool {
			
			if (tmpResult.distance < result.distance)
			{
				result = tmpResult;
				proxy = *static_cast<CollisionProxy const*>(node.bv.userData);
			}
			return true;
		});

		return { proxy, result };
	}

	void World::DetectCollisionsBroad()
	{
		potentialCollisions.clear();

		for (auto&& queryProxy : movedLastFrame)
		{
			BVHNode const* queryNode = bvhTree.Find(queryProxy.bvHandle);
			if (not queryNode) { continue; }

			BV const& bv = queryNode->bv;

			bvhTree.Query(bv.fatBounds, [this, &queryProxy, queryNode](BVHNode const& node) -> Bool {
				
				if (queryNode->index == node.index)
				{
                    // Avoid collisions with self
					return true;
				}
				if (node.moved && node.index > queryNode->index) 
				{
					// Both objects are moving -- avoid duplicate collisions
					return true;
				}

                CollisionProxy const proxy = *static_cast<CollisionProxy*>(node.bv.userData);
                if (proxy.rb == queryProxy.rb)
                {
                    // Avoid collisions between different parts of the same RigidBody
                    return true;
                }
                
				potentialCollisions.emplace_back(queryProxy, proxy);
				return true;
			});
		}

		for (auto&& queryProxy : movedLastFrame)
		{
			bvhTree.SetMoved(queryProxy.bvHandle, false);
		}
		movedLastFrame.clear();
	}

	void World::DetectCollisionsNarrow()
	{
		for (auto&& p : potentialCollisions)
		{
			auto const* shapeA = p.a.shape;
			auto const* shapeB = p.b.shape;
			ASSERT(shapeA && shapeB, "Neither shape should be nullptr");

			Mat4 const trA = p.a.rb->GetTransformMatrix();
			Mat4 const trB = p.b.rb->GetTransformMatrix();

			// TODO : do midphase pre-checks
			// - check for AABB intersection
			// - check cached Axis or Simplex

			ContactManifold const m = Collide(*shapeA, trA, *shapeB, trB);
			
			if (m.numContacts > 0)
			{
				auto it = contacts.find(p);
				if (it != contacts.end())
				{
					it->second = m; // it->second.Update(m);
				}
				else
				{
					contacts.emplace(p, m);
				}
			}
			else
			{
				contacts.erase(p);
			}
		}
		
	}
}