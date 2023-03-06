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

		inline World& World::AddStaticCollider(Shape auto&& shape)
		{
			ASSERT(not locked, "Cannot add static colliders when world is locked");

			colliders.AddCollider(std::forward<decltype(shape)>(shape), 0.0f);
			return *this;
		}

		inline World& World::AddStaticCollider(Shape auto const& shape)
		{
			ASSERT(not locked, "Cannot add static colliders when world is locked");
		
			colliders.AddCollider(shape, 0.0f);
			return *this;
		}

		inline void World::Clear()
		{
			// Reverse order of construction
			// ...
		}
	}

}