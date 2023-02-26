namespace drb {

	namespace physics {

		inline RigidBody& World::CreateRigidBody()
		{
			ASSERT(bodies.size() < bodies.capacity(), "Capacity limit reached.");
			return bodies.emplace_back();
		}

		template<Shape T>
		CollisionShape<T>& World::AddStaticCollider(T&& shape, CollisionShapeBase&& options)
		{
			if constexpr (std::is_same_v<T, Sphere>)
			{
				ASSERT(colliders.spheres.size() < colliders.spheres.capacity(), "Capacity limit reached.");
				auto& c = colliders.spheres.emplace_back(std::forward<T>(shape), 
					std::forward<Mat4>(options.transform), 
					std::forward<Float32>(options.mass));

				bvhTree.Insert(MakeAABB(c.shape, c.transform), &c);
				return c;
			}
			else if constexpr (std::is_same_v<T, Capsule>)
			{
				ASSERT(colliders.capsules.size() < colliders.capsules.capacity(), "Capacity limit reached.");
				auto& c = colliders.capsules.emplace_back(std::forward<T>(shape),
					std::forward<Mat4>(options.transform),
					std::forward<Float32>(options.mass));

				bvhTree.Insert(MakeAABB(c.shape, c.transform), &c);
				return c;
			}
			else if constexpr (std::is_same_v<T, Convex>)
			{
				ASSERT(colliders.hulls.size() < colliders.hulls.capacity(), "Capacity limit reached.");
				auto& c = colliders.hulls.emplace_back(std::forward<T>(shape),
					std::forward<Mat4>(options.transform),
					std::forward<Float32>(options.mass));

				bvhTree.Insert(MakeAABB(c.shape, c.transform), &c);
				return c;
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
				auto& c = colliders.spheres.emplace_back(shape, options.transform, options.mass);

				bvhTree.Insert(MakeAABB(c.shape, c.transform), &c);
				return c;
			}
			else if constexpr (std::is_same_v<T, Capsule>)
			{
				ASSERT(colliders.capsules.size() < colliders.capsules.capacity(), "Capacity limit reached.");
				auto& c = colliders.capsules.emplace_back(shape, options.transform, options.mass);

				bvhTree.Insert(MakeAABB(c.shape, c.transform), &c);
				return c;
			}
			else if constexpr (std::is_same_v<T, Convex>)
			{
				ASSERT(colliders.hulls.size() < colliders.hulls.capacity(), "Capacity limit reached.");
				auto& c = colliders.hulls.emplace_back(shape, options.transform, options.mass);

				bvhTree.Insert(MakeAABB(c.shape, c.transform), &c);
				return c;
			}
			else
			{
				static_assert(std::false_type<T>::value, "Invalid shape type.");
			}
		}

		inline void World::Clear()
		{
			// Reverse order of construction
			bodyBVHandles.clear();
			bvhTree.Clear();
			colliders.hulls.clear();
			colliders.capsules.clear();
			colliders.spheres.clear();
			bodies.clear();
			
		}
	}

}