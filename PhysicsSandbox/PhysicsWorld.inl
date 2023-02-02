namespace drb {

	namespace physics {

		inline RigidBody& World::CreateRigidBody()
		{
			ASSERT(bodies.size() < bodies.capacity(), "Capacity limit reached.");
			bodies.emplace_back();
			return bodies.back();
		}

		template<Shape T>
		inline CollisionShape<T>& World::AddStaticCollider(CollisionShape<T> const& shape)
		{
			if constexpr (std::is_same_v<T, Sphere>) 
			{
				ASSERT(sphColliders.size() < sphColliders.capacity(), "Capacity limit reached.");
				sphColliders.push_back(shape);
				return sphColliders.back();
			}
			else if constexpr (std::is_same_v<T, Capsule>) 
			{
				ASSERT(capColliders.size() < capColliders.capacity(), "Capacity limit reached.");
				capColliders.push_back(shape);
				return capColliders.back();
			}
			else if constexpr (std::is_same_v<T, Convex>) 
			{
				ASSERT(cvxColliders.size() < cvxColliders.capacity(), "Capacity limit reached.");
				cvxColliders.push_back(shape);
				return cvxColliders.back();
			}
			else if constexpr(std::is_same_v<T, Mesh>)
			{
				ASSERT(meshColliders.size() < meshColliders.capacity(), "Capacity limit reached.");
				meshColliders.push_back(shape);
				return meshColliders.back();
			}
			else
			{
				static_assert(std::false_type<T>::value, "Invalid shape type.");
			}
		}

		inline void World::Clear()
		{
			bodies.clear();
			sphColliders.clear();
			capColliders.clear();
			cvxColliders.clear();
			meshColliders.clear();
		}
	}

}