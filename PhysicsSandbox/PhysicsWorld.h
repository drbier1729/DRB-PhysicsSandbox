#ifndef DRB_PHYSICSWORLD_H
#define DRB_PHYSICSWORLD_H

#include "CollisionGeometry.h"
#include "GeometryQueryDataStructures.h"
#include "RigidBody.h"
#include "DynamicBVH.h"

namespace drb::physics {
	
	// Fwd decls
	struct Ray;

	// High-level class responsible for all stages of the simulation:
	// - Broad/Mid/Narrow phase collision detection
	// - Constraint and contact resolution
	// - RigidBody dynamics integration
	class World
	{ 
	private:
		using ProxyArray = std::vector<CollisionProxy>;

	public:
		// DEBUG BEGIN
		friend class DebugRenderer;
		friend class WorldStateRecorder;
		// DEBUG END

	private:
		// Dynamic and kinematic objects -- capacity == maxBodies
		std::vector<RigidBody>     bodies;
			
		// RigidBody used to handle collisions with static colliders
		RigidBody		           worldBody;

		// Static collision geometry -- capacity == maxColliders
		std::vector<CollisionGeometry> colliders;

		// Dynamic AABB hierarchy and supporting data for broadphase 
		// collision detection
		BVHierarchy			       bvhTree;
		std::vector<ProxyArray>    bodyProxies;    // parallel with bodies array
		std::vector<ProxyArray>    staticProxies;  // all proxies for static geometry
		ProxyArray                 movedLastFrame;
		std::vector<CollisionPair> potentialCollisions;
	
		// Contacts between RigidBody collision geometries
		// (use std::map to maintain consistent solver order)
		// TODO: switch this to "Constraint Graph + Islands" approach
		std::map<CollisionPair, ContactManifold> contacts;

		// Simulation variables
		Uint32 targetSubsteps;
		Bool   locked;

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
			
		inline RigidBody& CreateRigidBody(RigidBody const& source);
		inline RigidBody& CreateRigidBody(RigidBody&& source = {});

		inline World& CreateStaticCollisionGeometry(CollisionGeometry&& geom);
		inline World& CreateStaticCollisionGeometry(CollisionGeometry const& geom);

		inline void Clear();
			
		// For now this only checks against AABB bounding volumes, not actual
		// collision geometry
		std::pair<CollisionProxy, CastResult> RayCastQuery(Ray const& r);


		// -----------------------------------------------------------------
		// Accessors
		// -----------------------------------------------------------------
			
		// ...

	private:
		void DetectCollisionsBroad();
		void DetectCollisionsNarrow();

	};
}

#include "PhysicsWorld.inl"

#endif

