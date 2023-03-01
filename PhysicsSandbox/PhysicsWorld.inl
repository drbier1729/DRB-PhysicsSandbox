namespace drb {

	namespace physics {

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

		World& World::AddStaticCollider(Shape auto&& shape, CollisionShapeBase&& options)
		{
			ASSERT(not locked, "Cannot add static colliders when world is locked");

			colliders.AddCollider(std::forward<decltype(shape)>(shape), 
								  std::forward<CollisionShapeBase>(options));
			return *this;
		}

		World& World::AddStaticCollider(Shape auto const& shape, CollisionShapeBase const& options)
		{
			ASSERT(not locked, "Cannot add static colliders when world is locked");
		
			colliders.AddCollider(shape, options);
			return *this;
		}

		inline void World::Clear()
		{
			// Reverse order of construction
			// ...
		}
	}

}