#ifndef DRB_PHYSICSWORLD_H
#define DRB_PHYSICSWORLD_H

#include "PhysicsGeometry.h"
#include "PhysicsGeometryQueries.h"
#include "RigidBody.h"
#include "DynamicBVH.h"

namespace drb {
	namespace physics {

		class World
		{
		public:
			// DEBUG BEGIN
			friend class DebugRenderer;
			friend class WorldStateRecorder;
			// DEBUG END

		private:
			// Dynamic and kinematic objects -- capacity == maxBodies
			std::vector<RigidBody> bodies;
			
			// RigidBody used to handle collisions with static colliders
			RigidBody		       worldBody;

			// Static collision geometry -- capacity == maxColliders
			CollisionGeometry      colliders;

			// Dynamic AABB hierarchy for broadphase collision detection
			BVHierarchy			     bvhTree;
			std::vector<BV::Handle>  bodyBVHandles; // parallel with bodies array

			// Contacts between RigidBody collision geometries
			// (use std::map to maintain consistent solver order)
			// TODO: switch this to "Constraint Islands" approach
			std::map<ManifoldKey, ContactManifold> contacts;

			// Simulation variables
			Uint32 targetSubsteps;

		public:
			// -----------------------------------------------------------------
			// Constructors + Destructor
			// -----------------------------------------------------------------

			World(Uint32 maxBodies, Uint32 maxColliders, Uint32 targetSubsteps);
			// ... everything else defaulted
			

			// -----------------------------------------------------------------
			// Manipulators
			// -----------------------------------------------------------------
			
			void Init();

			void Step(Float32 dt);
			
			inline RigidBody& CreateRigidBody();

			template<Shape T>
			CollisionShape<T>& AddStaticCollider(T&& shape, CollisionShapeBase&& options = {});

			template<Shape T>
			CollisionShape<T>& AddStaticCollider(T const& shape, CollisionShapeBase const& options = {});

			inline void Clear();
			
			// For now this only checks against AABB bounding volumes, not actual
			// collision geometry
			RayCastHit RayCastQuery(Ray const& r);


			// -----------------------------------------------------------------
			// Accessors
			// -----------------------------------------------------------------
			
			// ...

		private:
			
			// Currently brute-force O(n^2) narrowphase -- very inefficient
			void DetectCollisions();

		};

	}
}

#include "PhysicsWorld.inl"

#endif

