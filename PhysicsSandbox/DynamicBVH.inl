namespace drb::physics
{

	inline void BVHNode::Create(AABB const& aabb, void* userData)
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
	
	inline void BVHNode::Free(Int32 nextFreeIdx)
	{
		version++;
		height = NullIdx;
		nextFree = nextFreeIdx;
	}

	inline Bool BVHNode::IsFree() const { return height == NullIdx; }
	inline Bool BVHNode::IsLeaf() const { return children[0] == NullIdx; }

	void BVHierarchy::Query(AABB const& box, BVHIntersectionQuery auto queryCallback) const
	{
		std::vector<Int32> stack{};
		stack.reserve(256);
		stack.push_back(rootIdx);

		Int32 currIdx = NullIdx;
		while (stack.size() > 0)
		{
			currIdx = stack.back();
			stack.pop_back();

			if (currIdx == NullIdx) { continue; }

			BVHNode const& curr = tree[currIdx];

			if (curr.bv.fatBounds.Intersects(box))
			{
				if (curr.IsLeaf())
				{
					if (not queryCallback(curr)) { return; }
				}
				else
				{
					stack.push_back(curr.children[0]);
					stack.push_back(curr.children[1]);
				}
			}
		}
	}

	void BVHierarchy::Query(Real r, Vec3 const& c, BVHIntersectionQuery auto queryCallback) const
	{
		Real const r2 = r * r;

		std::vector<Int32> stack{};
		stack.reserve(256);
		stack.push_back(rootIdx);

		Int32 currIdx = NullIdx;
		while (stack.size() > 0)
		{
			currIdx = stack.back();
			stack.pop_back();

			if (currIdx == NullIdx) { continue; }

			BVHNode const& curr = tree[currIdx];

			if (curr.bv.fatBounds.DistSquaredFromPoint(c) < r2)
			{
				if (curr.IsLeaf())
				{
					if (not queryCallback(curr)) { return; }
				}
				else
				{
					stack.push_back(curr.children[0]);
					stack.push_back(curr.children[1]);
				}
			}
		}
	}

	void BVHierarchy::Query(Ray const& ray, BVHRayCastQuery auto queryCallback) const
	{
		std::vector<Int32> stack{};
		stack.reserve(256);
		stack.push_back(rootIdx);

		Int32 currIdx = NullIdx;
		while (stack.size() > 0)
		{
			currIdx = stack.back();
			stack.pop_back();

			if (currIdx == NullIdx) { continue; }

			BVHNode const& curr = tree[currIdx];

			if (CastResult const r = ray.Cast(curr.bv.fatBounds); r.hit)
			{
				if (curr.IsLeaf())
				{
					if (not queryCallback(curr, r)) { return; }
				}
				else
				{
					stack.push_back(curr.children[0]);
					stack.push_back(curr.children[1]);
				}
			}
		}
	}


	// DEBUG ONLY
	void BVHierarchy::ForEach(std::invocable<BV const&> auto fn)
	{
		// Dumb iteration
		Int32 const cap = tree.Capacity();
		for (Int32 i = 0; i < cap; ++i)
		{
			if (tree.nodes[i].IsFree()) { continue; }
			fn(tree.nodes[i].bv);
		}
	}
}