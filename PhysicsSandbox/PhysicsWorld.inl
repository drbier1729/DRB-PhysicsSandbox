#include "DRBAssert.h"

namespace drb::physics {

		inline RigidBody& World::CreateRigidBody(RigidBody&& source)
		{
			ASSERT(not locked, "Cannot create RigidBody when world is locked");
			return bodies.emplace_back(std::forward<RigidBody>(source));
		}
		
		inline RigidBody& World::CreateRigidBody(RigidBody const& source)
		{
			ASSERT(not locked, "Cannot create RigidBody when world is locked");
			return bodies.emplace_back(source);
		}

		inline World& World::CreateStaticCollisionGeometry(CollisionGeometry&& geom)
		{
			ASSERT(not locked, "Cannot add static colliders when world is locked");
			colliders.emplace_back(std::forward<CollisionGeometry>(geom));
			return *this;
		}

		inline World& World::CreateStaticCollisionGeometry(CollisionGeometry const& geom)
		{
			ASSERT(not locked, "Cannot add static colliders when world is locked");

			colliders.emplace_back(geom);
			return *this;
		}

		inline void World::Clear()
		{
			// Reverse order of construction
			// ...
		}

}