namespace drb::physics
{
	struct BVHNode
	{
		static constexpr Uint32 NullIdx = BVHierarchy::NullIdx;

		BV bv = {};

		union {
			Uint32 parent = NullIdx;
			Uint32 nextFree;
		};

		Uint32 children[2] = { NullIdx, NullIdx };

		// Free node = NullIdx, Leaf = 0 
		Uint32 height = 0;

		// Used to check "Find" and "Remove" methods
		Uint32 version = 0;
		Uint32 index = 0;

		// True if we need to remove and reinsert this node
		Bool   moved = false;

		inline void Create(AABB const& aabb, void* userData)
		{
			bv = BV{ aabb, userData };
			// do not set index -- this is updated when the NodePool grows
			// do not set version -- this is updated when node is freed
			height = 0;
			parent = NullIdx;
			children[0] = NullIdx;
			children[1] = NullIdx;
			moved = false;
		}
		inline void Free(Uint32 nextFreeIdx)
		{
			version++;
			height = NullIdx;
			nextFree = nextFreeIdx;
		}
		inline Bool IsFree() const { return height == NullIdx; }
		inline Bool IsLeaf() const { return children[0] == NullIdx; }
	};
	static_assert(sizeof(BVHNode) == 64);



	void BVHierarchy::Query(AABB const& box, std::invocable<BV const&> auto callback) const {}
	void BVHierarchy::Query(Sphere const& sph, std::invocable<BV const&> auto callback) const {}
	void BVHierarchy::Query(Ray const& ray, std::invocable<BV const&> auto callback) const {}


	void BVHierarchy::ForEach(std::invocable<BV const&> auto fn)
	{
		// Dumb iteration
		Uint32 const cap = tree.Capacity();
		for (Uint32 i = 0; i < cap; ++i)
		{
			if (tree.nodes[i].IsFree()) { continue; }
			fn(tree.nodes[i].bv);
		}
	}
}