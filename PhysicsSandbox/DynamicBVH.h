#ifndef DRB_DYNAMICBVH_H
#define DRB_DYNAMICBVH_H

#include "AABB.h"

namespace drb::physics {
	
	// Fwd decl
	class RigidBody;
	struct Ray;
	struct Sphere;
	struct BVHNode;

	// User accessible data from the BVH
	struct BV
	{
		using Handle = Uint64;

		static constexpr Vec3    enlargeFactor = Vec3(0.1f);
		static constexpr Float32 displacementMultiplier = 4.0f;

		void* userData = nullptr; // ptr to CollisionGeometry or RigidBody, etc
		AABB  fatBounds = {};     // enlarged AABB

		BV() = default;
		BV(AABB const& aabb, void* data)
			: userData{ data },
			fatBounds{
				.max = aabb.max + enlargeFactor,
				.min = aabb.min - enlargeFactor
			}
		{}
	};

	// Dynamic binary tree Bounding Volume Hierarchy of enlarged AABB
	class BVHierarchy
	{
	private:
		class NodePool
		{
		private:
			BVHNode*  nodes;
			Uint32    firstFree;
			Uint32    size;
			Uint32    capacity;

		public:
			NodePool();
			~NodePool() noexcept;

			BVHNode&       operator[](Uint32 index);
			BVHNode const& operator[](Uint32 index) const;
			BVHNode*       Create(AABB const& aabb, void* userData = nullptr);
			void           Free(Uint32 nodeIndex);
			Uint32		   Size() const;
			Uint32		   Capacity() const;
		};

	private:
		NodePool tree    = {};
		Uint32   rootIdx = NullIdx;

	public:
		static constexpr Uint32 NullIdx = std::numeric_limits<Uint32>::max();

		BV::Handle  Insert(AABB const& aabb, void* userData);
		BV const*   Find(BV::Handle bvHandle) const;
		void	    Remove(BV::Handle bvHandle);

		// TODO
		Bool		MoveBV(BV::Handle, AABB const& aabb, Vec3 const& displacement);
		void	    Query(AABB const&   box, std::invocable<BV const&> auto callback) const;
		void	    Query(Sphere const& sph, std::invocable<BV const&> auto callback) const;
		void	    Query(Ray const&    ray, std::invocable<BV const&> auto callback) const;


	private:
		// Helpers
		Uint32	    Balance(Uint32 index);
		BVHNode*    FindBestSiblingFor(BVHNode const* n);
		void		RefitBVsFrom(Uint32 index);
		Bool	    ValidHandle(BV::Handle bvHandle, Uint32& indexOut) const;
	};

}

#include "DynamicBVH.inl"
#endif

