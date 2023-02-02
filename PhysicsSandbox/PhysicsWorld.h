#ifndef DRB_PHYSICSWORLD_H
#define DRB_PHYSICSWORLD_H

#include "PhysicsGeometry.h"
#include "RigidBody.h"

namespace drb {
	namespace physics {

		struct RayCastHit
		{
			Ray::CastResult result = {};
			RigidBody* body = nullptr;
			CollisionShape<Sphere>* sph = nullptr;
			CollisionShape<Capsule>* cap = nullptr;
			CollisionShape<Convex>* cvx = nullptr;
			CollisionShape<Mesh>* mesh = nullptr;
		};

		class World
		{
		public:
			// Debugging only
			friend class DebugRenderer;
			friend class WorldStateRecorder;

		private:
			// Dynamic and kinematic objects -- capacity == maxBodies
			std::vector<RigidBody> bodies;
			
			// Static collision geometry -- capacity == maxColliders
			std::vector<CollisionShape<Sphere>>    sphColliders;
			std::vector<CollisionShape<Capsule>>   capColliders;
			std::vector<CollisionShape<Convex>>    cvxColliders;
			std::vector<CollisionShape<Mesh>>	   meshColliders;

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
			
			void Step(Float32 dt);
			
			inline RigidBody& CreateRigidBody();

			template<Shape T>
			inline CollisionShape<T>& AddStaticCollider(CollisionShape<T> const& shape);

			inline void Clear();
			
			// For now this only checks against AABB bounding volumes, not actual
			// collision geometry
			RayCastHit RayCastQuery(Ray const& r);

			// -----------------------------------------------------------------
			// Accessors
			// -----------------------------------------------------------------


		};

	}
}

#include "PhysicsWorld.inl"

#endif

