namespace drb {

	namespace physics {

		inline RigidBody& World::CreateRigidBody()
		{
			ASSERT(bodies.size() < bodies.capacity(), "Capacity limit reached.");
			bodies.emplace_back();
			return bodies.back();
		}

		template<Shape T>
		CollisionShape<T>& World::AddStaticCollider(T&& shape, CollisionShapeBase&& options)
		{
			if constexpr (std::is_same_v<T, Sphere>)
			{
				ASSERT(colliders.spheres.size() < colliders.spheres.capacity(), "Capacity limit reached.");
				colliders.spheres.emplace_back(std::forward<T>(shape), 
					std::forward<Mat4>(options.transform), 
					std::forward<Float32>(options.mass));
				return colliders.spheres.back();
			}
			else if constexpr (std::is_same_v<T, Capsule>)
			{
				ASSERT(colliders.capsules.size() < colliders.capsules.capacity(), "Capacity limit reached.");
				colliders.capsules.emplace_back(std::forward<T>(shape),
					std::forward<Mat4>(options.transform),
					std::forward<Float32>(options.mass));
				return colliders.capsules.back();
			}
			else if constexpr (std::is_same_v<T, Convex>)
			{
				ASSERT(colliders.hulls.size() < colliders.hulls.capacity(), "Capacity limit reached.");
				colliders.hulls.emplace_back(std::forward<T>(shape),
					std::forward<Mat4>(options.transform),
					std::forward<Float32>(options.mass));
				return colliders.hulls.back();
			}
			else
			{
				static_assert(std::false_type<T>::value, "Invalid shape type.");
			}
		}

		template<Shape T>
		CollisionShape<T>& World::AddStaticCollider(T const& shape, CollisionShapeBase const& options)
		{
			if constexpr (std::is_same_v<T, Sphere>)
			{
				ASSERT(colliders.spheres.size() < colliders.spheres.capacity(), "Capacity limit reached.");
				colliders.spheres.emplace_back(shape, options.transform, options.mass);
				return colliders.spheres.back();
			}
			else if constexpr (std::is_same_v<T, Capsule>)
			{
				ASSERT(colliders.capsules.size() < colliders.capsules.capacity(), "Capacity limit reached.");
				colliders.capsules.emplace_back(shape, options.transform, options.mass);
				return colliders.capsules.back();
			}
			else if constexpr (std::is_same_v<T, Convex>)
			{
				ASSERT(colliders.hulls.size() < colliders.hulls.capacity(), "Capacity limit reached.");
				colliders.hulls.emplace_back(shape, options.transform, options.mass);
				return colliders.hulls.back();
			}
			else
			{
				static_assert(std::false_type<T>::value, "Invalid shape type.");
			}
		}

		inline void World::Clear()
		{
			bodies.clear();
			colliders.spheres.clear();
			colliders.capsules.clear();
			colliders.hulls.clear();
		}
	}

}