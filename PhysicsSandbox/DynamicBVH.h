#ifndef DRB_DYNAMICBVH_H
#define DRB_DYNAMICBVH_H

#include "AABB.h"
#include "RayCast.h"


namespace drb::physics {
	
	// Bounding volume
	struct BV
	{
		static constexpr Float32 enlargeFactor = 0.1f;
		static constexpr Float32 displacementMultiplier = 4.0f;

		void* userData = nullptr;
		AABB  fatBounds = {};     // enlarged AABB

		BV() = default;
		BV(AABB const& aabb, void* data)
			: userData{ data },
			fatBounds{ aabb.Expanded(enlargeFactor) }
		{}

		struct Info
		{
			Int32 index;
			Uint32 version; // this may overflow (very unlikely) so make it unsigned
		};
	};


	// Unique, versioned identifier for BVH nodes
	union BVHandle
	{
		Int64    handle;
		BV::Info info;

		BVHandle() = default;
		BVHandle(Int32 idx, Uint32 vr) : info{ idx, vr } {}

		// Implicit conversion to/from int
		BVHandle(Int64 val) : handle{ val } {}
		operator Int64() { return handle; }

		auto operator<=>(BVHandle const& other) const { return info.index <=> other.info.index; }
		auto operator==(BVHandle const& other) const  { return handle == other.handle; }
	};


	// Internal class used by BVHierarchy, but also used for queries
	struct BVHNode
	{
		static constexpr Int32 NullIdx = -1;

		BV bv = {};

		union {
			Int32 parent = NullIdx;
			Int32 nextFree;
		};

		Int32 children[2] = { NullIdx, NullIdx };

		// Free node = NullIdx, Leaf = 0 
		Int32 height = 0;

		// Used to check "Find" and "Remove" methods, and to generate a handle
		Int32  index   = NullIdx;
		Uint32 version = 0;

		// True if we removed and reinserted this node
		Bool moved = false;

		inline void Create(AABB const& aabb, void* userData);
		inline void Free(Int32 nextFreeIdx);
		inline Bool IsFree() const;
		inline Bool IsLeaf() const;
	};
	static_assert(sizeof(BVHNode) <= 64);



	// Callback signature for querying the BVHierarchy
	template<class Func>
	concept BVHIntersectionQuery = std::predicate<Func, BVHNode const&>;
	
	template<class Func>
	concept BVHRayCastQuery = std::predicate<Func, BVHNode const&, CastResult const&>;



	// Dynamic binary tree Bounding Volume Hierarchy of enlarged AABB
	// Adapted from Box2D (see b2_dynamic_tree on GitHub, and Erin 
	// Catto's 2019 GDC talk)
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
			Int32    firstFree;
			Int32    size;
			Int32    capacity;

		public:
			static constexpr Int32 minCapacity = 16u;
			
			NodePool(); // default reserves minCapacity nodes
			~NodePool() noexcept;

			NodePool(NodePool const&) = delete;
			NodePool& operator=(NodePool const&) = delete;
			NodePool(NodePool &&) = delete;
			NodePool& operator=(NodePool &&) = delete;

			BVHNode&       operator[](Int32 index);
			BVHNode const& operator[](Int32 index) const;
			Int32          Create(AABB const& aabb, void* userData = nullptr);
			void           Free(Int32 nodeIndex);
			Int32		   Size() const;
			Int32		   Capacity() const;
			Int32		   Reserve(Int32 requestedCapacity); // returns actual new capacity
			void		   Clear(); // does not deallocate
		};

	private:
		NodePool tree    = {};
		Int32    rootIdx = NullIdx;

	public:
		static constexpr Int32 NullIdx = -1;

		BVHandle       Insert(AABB const& aabb, void* userData = nullptr);
		BVHNode const* Find(BVHandle bvHandle) const;
		void		   Remove(BVHandle bvHandle);
		void		   SetMoved(BVHandle bvHandle, Bool val = true);
		Bool		   MoveBoundingVolume(BVHandle handle, AABB const& aabb, Vec3 const& displacement);
		void		   Reserve(Int32 objectCount);
		void		   Clear();
		void		   Query(AABB const& box, BVHIntersectionQuery auto queryCallback) const;
		void		   Query(Float32 radius, Vec3 const& center, BVHIntersectionQuery auto queryCallback) const;
		void		   Query(Ray const& ray, BVHRayCastQuery auto queryCallback) const;
		
		// DEBUG ONLY
		void ForEach(std::invocable<BV const&> auto fn);

	private:
		// Helpers
		Int32	    Balance(Int32 index);
		Int32       FindBestSiblingFor(Int32 index);
		void		RefitBVsFrom(Int32 index);
		Bool        InsertLeaf(Int32 index);
		void        RemoveLeaf(Int32 index);
		Bool	    ValidHandle(BVHandle bvHandle, Int32& indexOut) const;
	};
}

#include "DynamicBVH.inl"
#endif

