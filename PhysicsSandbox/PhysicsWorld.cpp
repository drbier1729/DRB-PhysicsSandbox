#include "pch.h"
#include "PhysicsWorld.h"

#include "RayCast.h"
#include "CollisionQueries.h"
#include "ContactSolver.h"

#include "DRBAssert.h"

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
		colliders.reserve(maxColliders);
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
				rbProxiesVec.reserve( rb.geometry->Size() );
			
				// Create proxies and add them to the BVH
				Mat4 const rbTr = rb.GetTransformMatrix();
				rb.geometry->ForEachCollider([&](Shape auto const& col) {

					AABB const bounds = col.Bounds().Transformed(rbTr);
					CollisionProxy& proxy = rbProxiesVec.emplace_back(CollisionProxy{ 
						.rb = &rb, 
						.shape = &col 
					});

					proxy.bvHandle = bvhTree.Insert(bounds, &proxy);
					bvhTree.SetMoved(proxy.bvHandle, false);
				});
			}
		}

		// Reserve enough space for all the proxies for static geometry
		staticProxies.reserve(colliders.size());

		// Create the proxies and add them to the BVH
		for (auto&& c : colliders)
		{
			auto& proxyArray = staticProxies.emplace_back();
			proxyArray.reserve(c.Size());

			c.ForEachCollider([this, &proxyArray](Shape auto const& col) {

				AABB const bounds = col.Bounds();
				CollisionProxy& proxy = proxyArray.emplace_back(CollisionProxy{
					.rb = &worldBody,
					.shape = &col
				});
				proxy.bvHandle = bvhTree.Insert(bounds, &proxy);
				bvhTree.SetMoved(proxy.bvHandle, false);
			});
		}
	}
#define DRB_PHYSICS_XPBD 1

#ifdef DRB_PHYSICS_XPBD
	void World::Step(Real dt)
	{
		// Broadphase Collision Detection
		// TODO: this may need to be more aggressive about generating
		//		potential collisions so that manifolds are not missed during
		//		substepping
		DetectCollisionsBroad(dt);

		// XPBD loop with substepping
		Real const h = dt / targetSubsteps;
		Real const invH = 1.0_r / h;
		for (Uint32 i = 0; i < targetSubsteps; ++i)
		{
			// Narrowphase Collision Detection + Contact Generation
			// TODO: this should be split into two parts:
			// - collision detection/manifold generation (happens outside substep)
			// - manifold update (happens inside the substep)
			DetectCollisionsNarrow();


			for (auto&& rb : bodies) { rb.ProjectPositions(h); }

			// Solve Position constraints
			// - Collision Resolution
			// - Distance Constraints
			// - Joint Constraints
			for (auto&& [p, m] : contacts) {
				SolveManifoldPositions(m, invH);
			}
			// ...

			for (auto&& rb : bodies) { rb.ProjectVelocities(invH); }

			// Solve Velocity constraints
			// - Friction
			// - Damping
			for (auto&& [p, m] : contacts) {
				SolveManifoldVelocities(m, h);
			}
			// ...
		}

		// Zero forces
		for (SizeT i = 0; auto&& rb : bodies)
		{
			rb.accumulatedForces = Vec3(0);
			rb.accumulatedTorques = Vec3(0);
		}

		// Collision Callbacks
		// ...
	}
#else
	void World::Step(Real dt)
	{
		// Broadphase Collision Detection
		DetectCollisionsBroad();

		// Narrowphase Collision Detection + Contact Generation
		DetectCollisionsNarrow();

		// Sequential Impulses loop with substepping
		for (auto&& rb : bodies) { rb.ProjectForces(dt); }

		Real const h = dt / targetSubsteps;
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
					AABB const preBounds  = std::visit([&preTr](auto&& s)  { return s->Bounds(preTr); }, proxy.shape);
					AABB const postBounds = std::visit([&postTr](auto&& s) { return s->Bounds(postTr); }, proxy.shape);

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

	void World::DetectCollisionsBroad(Real dt)
	{
		potentialCollisions.clear();

		// Update broad phase tree
		for (SizeT i = 0; auto && rb : bodies)
		{
			if (rb.geometry)
			{
				// Get the pre-update transform matrix
				Mat4 const preTr = rb.GetTransformMatrix();

				// Get the post-update transform matrix by projecting forward
				Vec3 const postPos = rb.position + rb.linearVelocity * dt;
				Quat const postRot = Normalize(rb.orientation + 0.5_r * Quat(0, rb.angularVelocity) * rb.orientation * dt);
				Mat4 const postTr  = glm::translate(Mat4(1), postPos) * glm::toMat4(postRot);

				// Update broadphase tree
				for (auto&& proxy : bodyProxies[i])
				{
					AABB const preBounds = std::visit([&preTr](auto&& s)   { return s->Bounds(preTr); }, proxy.shape);
					AABB const postBounds = std::visit([&postTr](auto&& s) { return s->Bounds(postTr); }, proxy.shape);

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
			++i;
		}

		// For every body that moved in the last frame, query the BVH for collisions
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

		// Push all new potential collisions to contacts list
		for (auto&& p : potentialCollisions)
		{
			auto it = contacts.find(p);
			if (it == contacts.end())
			{
				contacts.emplace(p, ContactManifold{.rbA = p.a.rb, .rbB = p.b.rb});
			}
		}

		// Reset moved flags on bounding volumes
		for (auto&& queryProxy : movedLastFrame)
		{
			bvhTree.SetMoved(queryProxy.bvHandle, false);
		}
		movedLastFrame.clear();
	}

	void World::DetectCollisionsNarrow()
	{
		for (auto it = contacts.begin(); it != contacts.end();)
		{
			// Decompose into key (CollisionPair) and value (ContactManifold)
			auto & [p, m] = *it;
			// Midphase pre-checks
			
			// 1) recheck fat AABBs for overlap -- if none, remove 
			// the contact manifold
			{
				AABB const fatBoundsA = bvhTree.Find(p.a.bvHandle)->bv.fatBounds;
				AABB const fatBoundsB = bvhTree.Find(p.b.bvHandle)->bv.fatBounds;
				if (not fatBoundsA.Intersects(fatBoundsB))
				{
					it = contacts.erase(it);
					continue;
				}
			}

			// TODO:
			// 2) check cached axis of least penetration or cached
			// GJK simplex -- if separation, update the manifold 
			// (but don't overwrite the old manifold points bc we 
			// may reuse them soon)
			//
			//  --> update separation normal
			//  --> update numContacts = 0
			//  --> update cached simplex
			//

			ConstShapePtr const shapeA = p.a.shape;
			ConstShapePtr const shapeB = p.b.shape;

			Mat4 const trA = p.a.rb->GetTransformMatrix();
			Mat4 const trB = p.b.rb->GetTransformMatrix();

			// Do the full narrow phase collision detection and update the manifold
			ContactManifold const newManifold = Collide(shapeA, trA, shapeB, trB);
			UpdateManifold(m, newManifold);
			
			++it;
		}
	}
}