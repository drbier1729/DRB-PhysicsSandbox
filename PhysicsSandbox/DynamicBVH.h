#ifndef DRB_DYNAMICBVH_H
#define DRB_DYNAMICBVH_H

#include "AABB.h"

// Adapted from Box2D (see b2_dynamic_tree on GitHub, and Erin Catto's 2019 GDC talk)

namespace drb::physics {
	
	// Fwd decl
	class RigidBody;
	struct Ray;
	struct Sphere;

	// Internal class used by BVHierarchy
	struct BVHNode;

	// User accessible data from the BVH
	struct BV
	{
		using Handle = Uint64;

		static constexpr Float32 enlargeFactor = 0.1f;
		static constexpr Float32 displacementMultiplier = 4.0f;

		void* userData = nullptr; // ptr to CollisionGeometry or RigidBody, etc
		AABB  fatBounds = {};     // enlarged AABB

		BV() = default;
		BV(AABB const& aabb, void* data)
			: userData{ data },
			fatBounds{ aabb.Expanded(enlargeFactor) }
		{}
	};

	
	// Dynamic binary tree Bounding Volume Hierarchy of enlarged AABB
	class BVHierarchy
	{
	public:
		// DEBUG BEGIN
		friend class DebugRenderer;
		// DEBUG END

	private:
		class NodePool
		{
		public:
			friend class BVHierarchy;
		
		private:
			BVHNode* nodes;
			Uint32   firstFree;
			Uint32   size;
			Uint32   capacity;

		public:
			static constexpr Uint32 minCapacity = 16u;
			
			NodePool(); // default reserves minCapacity nodes
			~NodePool() noexcept;

			BVHNode&       operator[](Uint32 index);
			BVHNode const& operator[](Uint32 index) const;
			BVHNode*       Create(AABB const& aabb, void* userData = nullptr);
			void           Free(Uint32 nodeIndex);
			Uint32		   Size() const;
			Uint32		   Capacity() const;
			Uint32		   Reserve(Uint32 requestedCapacity); // returns actual new capacity
			void		   Clear(); // does not deallocate
		};

	private:
		NodePool tree    = {};
		Uint32   rootIdx = NullIdx;

	public:
		static constexpr Uint32 NullIdx = std::numeric_limits<Uint32>::max();

		BV::Handle  Insert(AABB const& aabb, void* userData = nullptr);
		BV const*   Find(BV::Handle bvHandle) const;
		void	    Remove(BV::Handle bvHandle);
		void		Reserve(Uint32 objectCount);
		void		Clear();

		// TODO
		Bool		MoveBoundingVolume(BV::Handle handle, AABB const& aabb, Vec3 const& displacement);
		void	    Query(AABB const&   box, std::invocable<BV const&> auto callback) const;
		void	    Query(Sphere const& sph, std::invocable<BV const&> auto callback) const;
		void	    Query(Ray const&    ray, std::invocable<BV const&> auto callback) const;

		void        ForEach(std::invocable<BV const&> auto fn);
	private:
		// Helpers
		Uint32	    Balance(Uint32 index);
		BVHNode*    FindBestSiblingFor(BVHNode const* n);
		void		RefitBVsFrom(Uint32 index);
		Bool        InsertLeaf(BVHNode* n);
		void        RemoveLeaf(Uint32 index);
		Bool	    ValidHandle(BV::Handle bvHandle, Uint32& indexOut) const;
	};
}

#include "DynamicBVH.inl"
#endif

