#include "pch.h"
#include "DynamicBVH.h"
#include "DRBAssert.h"

namespace drb::physics {


	static inline AABB Union(AABB const& a, AABB const& b);

	BVHandle BVHierarchy::Insert(AABB const& aabb, void* userData)
	{
		Int32 newNodeIdx = tree.Create(aabb, userData);
		
		tree[newNodeIdx].moved = true;
		Bool const success = InsertLeaf(newNodeIdx);

		return success ?
			BVHandle{ newNodeIdx, tree[newNodeIdx].version } :
			BVHandle{ -1 };
	}

	void  BVHierarchy::Remove(BVHandle bvHandle)
	{
		if (Int32 index = NullIdx; ValidHandle(bvHandle, index))
		{
			RemoveLeaf(index);
			tree.Free(index);
		}
	}

	BVHNode const* BVHierarchy::Find(BVHandle bvHandle) const
	{
		if (Int32 index = NullIdx; ValidHandle(bvHandle, index))
		{
			return &tree[index];
		}
		return nullptr;
	}


	void BVHierarchy::SetMoved(BVHandle bvHandle, Bool val)
	{
		if (Int32 index = NullIdx; ValidHandle(bvHandle, index))
		{
			tree[index].moved = val;
		}
	}

	Bool BVHierarchy::MoveBoundingVolume(BVHandle handle, AABB const& aabb, Vec3 const& displacement)
	{
		if (Int32 index = NullIdx; ValidHandle(handle, index))
		{
			ASSERT(tree[index].IsLeaf(), "Cannot move internal nodes");

			Vec3 const d = BV::displacementMultiplier * displacement;

			AABB predictedAABB = aabb.Expanded(BV::enlargeFactor);
			Vec3 min = predictedAABB.Min(), max = predictedAABB.Max();
			if (d.x < 0.0) { min.x += d.x; }
			else { max.x += d.x; }
			if (d.y < 0.0) { min.y += d.y; }
			else { max.y += d.y; }
			if (d.z < 0.0) { min.z += d.z; }
			else { max.z += d.z; }
			predictedAABB.SetMinMax(min, max);

			AABB const& treeAABB = tree[index].bv.fatBounds;
			if (treeAABB.Contains(predictedAABB))
			{
				AABB const hugeAABB = predictedAABB.Expanded(BV::displacementMultiplier * BV::enlargeFactor);
				if (hugeAABB.Contains(treeAABB))
				{
					return false;
				}

				// The tree AABB needs to be shrunk
			}

			RemoveLeaf(index);

			tree[index].bv.fatBounds = predictedAABB;
			InsertLeaf(index);

			tree[index].moved = true;
			return true;
		}

		return false;
	}

	void BVHierarchy::Reserve(Int32 objectCount)
	{
		// reserve enough nodes for a tree with objectCount leaves
		tree.Reserve(objectCount * 2);
	}

	void BVHierarchy::Clear()
	{
		tree.Clear();
		rootIdx = NullIdx;
	}


	// -------------------------------------------------------------------------
	// Helpers
	// -------------------------------------------------------------------------

	BVHierarchy::NodePool::NodePool()
		: nodes{ nullptr },
		firstFree{ 0 },
		size{ 0 },
		capacity{ 0 }
	{
		Reserve(minCapacity);
	}

	BVHierarchy::NodePool::~NodePool() noexcept
	{
		std::free(nodes);
	}

	BVHierarchy::NodePool::NodePool(BVHierarchy::NodePool&& src) noexcept
		: nodes{src.nodes}, firstFree{src.firstFree}, size{src.size}, capacity{src.capacity}
	{
		src.nodes = nullptr;
		src.firstFree = NullIdx;
		src.size = 0;
		src.capacity = 0;
	}

	BVHierarchy::NodePool& BVHierarchy::NodePool::operator=(BVHierarchy::NodePool&& src) noexcept
	{
		if (this == std::addressof(src)) { return *this; }
		nodes = src.nodes;
		firstFree = src.firstFree;
		size = src.size;
		capacity = src.capacity;

		src.nodes = nullptr;
		src.firstFree = NullIdx;
		src.size = 0;
		src.capacity = 0;

		return *this;
	}

	void BVHierarchy::NodePool::Clear()
	{
		if (capacity == 0) { return; }

		// Reduce size
		size = 0;

		// Zero the nodes
		std::memset(nodes, 0, capacity * sizeof(BVHNode));

		// Reset free list
		for (Int32 i = 0; (i + 1) < capacity; ++i)
		{
			nodes[i].nextFree = i + 1;
			nodes[i].height = NullIdx;
			nodes[i].index = i;
		}
		nodes[capacity - 1].nextFree = NullIdx;
		nodes[capacity - 1].height = NullIdx;
		nodes[capacity - 1].index = capacity - 1;
		firstFree = 0;
	}

	BVHNode& BVHierarchy::NodePool::operator[](Int32 index)
	{
		ASSERT(0 <= index && index < capacity, "Index out of range");
		ASSERT(not nodes[index].IsFree(), "Node is dead");
		return nodes[index];
	}

	BVHNode const& BVHierarchy::NodePool::operator[](Int32 index) const
	{
		ASSERT(0 <= index && index < capacity, "Index out of range");
		ASSERT(not nodes[index].IsFree(), "Node is dead");
		return nodes[index];
	}

	Int32 BVHierarchy::NodePool::Create(AABB const& aabb, void* userData)
	{
		// Grow the pool if needed
		if (firstFree == NullIdx)
		{
			ASSERT(size == capacity, "Growing unecessarily");
			capacity = Reserve(capacity * 2);
		}

		Int32 const index = firstFree;
		ASSERT(0 <= index && index < capacity, "Bad index");

		BVHNode& newNode = nodes[index];
		firstFree = newNode.nextFree;
		++size;

		newNode.Create(aabb, userData);

		return index;
	}

	void  BVHierarchy::NodePool::Free(Int32 nodeIndex)
	{
		ASSERT(0 <= nodeIndex && nodeIndex < capacity, "Bad index");
		ASSERT(size > 0, "Empty array");

		nodes[nodeIndex].Free(firstFree);
		firstFree = nodeIndex;

		--size;
	}

	Int32 BVHierarchy::NodePool::Size() const { return size; }

	Int32 BVHierarchy::NodePool::Capacity() const { return capacity; }

	Int32 BVHierarchy::NodePool::Reserve(Int32 newCapacity)
	{
		static constexpr Int32 maxCapacity = 1024u * 1024u;

		if (capacity >= newCapacity || capacity == maxCapacity)
		{
			//ASSERT(0 <= capacity && capacity < newCapacity, "Reserving unecessarily");
			//ASSERT(0 <= capacity && capacity < maxCapacity, "Max capacity reached");
			return capacity;
		}

		if (capacity == 0)
		{
			capacity = glm::max(minCapacity, newCapacity);
			nodes = (BVHNode*)std::malloc(capacity * sizeof(BVHNode));

			if (not nodes)
			{
				ASSERT(nodes, "Bad alloc");
				return capacity;
			}
		}
		else
		{
			BVHNode* const oldNodes = nodes;
			Int32 const    oldCap = capacity;

			// Grow by at least by a factor of 2, or up to maxCapacity (which ever is less)
			capacity = glm::min(maxCapacity, glm::max(newCapacity, capacity * 2));
			nodes = (BVHNode*)std::malloc(capacity * sizeof(BVHNode));

			if (not nodes) {
				ASSERT(false, "Bad alloc");
				nodes = oldNodes;
				capacity = oldCap;
				return capacity;
			}

			// Copy the old nodes then free them
			std::memcpy(nodes, oldNodes, size * sizeof(BVHNode));
			std::free(oldNodes);
		}

		ASSERT(size < capacity && capacity > 0, "Something went wrong");

		// Zero the new nodes
		std::memset(nodes + size, 0, (capacity - size) * sizeof(BVHNode));

		// Set up free list
		for (Int32 i = size; (i + 1) < capacity; ++i)
		{
			nodes[i].nextFree = i + 1;
			nodes[i].height = NullIdx;
			nodes[i].index = i;
		}
		nodes[capacity - 1].nextFree = NullIdx;
		nodes[capacity - 1].height = NullIdx;
		nodes[capacity - 1].index = capacity - 1;
		firstFree = size;

		return capacity;
	}

	// Performs left or right rotation if subtree with root at index is
	// imbalanced. Returns the index of the new root of the subtree.
	Int32 BVHierarchy::Balance(Int32 aIdx)
	{
		ASSERT(aIdx < tree.Capacity(), "Index invalid");

		BVHNode& a = tree[aIdx];
		if (a.IsLeaf() || a.height < 2)
		{
			return aIdx;
		}

		BVHNode& b = tree[a.children[0]];
		BVHNode& c = tree[a.children[1]];

		Int32 const balance = c.height - b.height;

		if (balance > 1)
		{
			// Rotate c up
			BVHNode& cChild0 = tree[c.children[0]];
			BVHNode& cChild1 = tree[c.children[1]];

			// Swap a and c
			c.children[0] = a.index;
			c.parent = a.parent;
			a.parent = c.index;
			if (c.parent != NullIdx)
			{
				BVHNode& cParent = tree[c.parent];
				if (cParent.children[0] == a.index)
				{
					cParent.children[0] = c.index;
				}
				else
				{
					ASSERT(cParent.children[1] == a.index, "Parent is incorrect");
					cParent.children[1] = c.index;
				}
			}
			else
			{
				rootIdx = c.index;
			}

			// Rotate
			if (cChild0.height > cChild1.height)
			{
				c.children[1] = cChild0.index;
				a.children[1] = cChild1.index;

				cChild1.parent = a.index;

				a.bv.fatBounds = Union(b.bv.fatBounds, cChild1.bv.fatBounds);
				c.bv.fatBounds = Union(a.bv.fatBounds, cChild0.bv.fatBounds);

				a.height = 1 + glm::max(b.height, cChild1.height);
				c.height = 1 + glm::max(a.height, cChild0.height);
			}
			else
			{
				c.children[1] = cChild1.index;
				a.children[1] = cChild0.index;

				cChild0.parent = a.index;

				a.bv.fatBounds = Union(b.bv.fatBounds, cChild0.bv.fatBounds);
				c.bv.fatBounds = Union(a.bv.fatBounds, cChild1.bv.fatBounds);

				a.height = 1 + glm::max(b.height, cChild0.height);
				c.height = 1 + glm::max(a.height, cChild1.height);
			}

			return c.index;
		}
		else if (balance < -1)
		{
			// Rotate B up
			BVHNode& bChild0 = tree[b.children[0]];
			BVHNode& bChild1 = tree[b.children[1]];

			// Swap a and b
			b.children[0] = a.index;
			b.parent = a.parent;
			a.parent = b.index;

			if (b.parent != NullIdx)
			{
				BVHNode& bParent = tree[b.parent];
				if (bParent.children[0] == a.index)
				{
					bParent.children[0] = b.index;
				}
				else
				{
					ASSERT(bParent.children[1] == a.index, "Parent incorrect");
					bParent.children[1] = b.index;
				}
			}
			else
			{
				rootIdx = b.index;
			}

			// Rotate
			if (bChild0.height > bChild1.height)
			{
				b.children[1] = bChild0.index;
				a.children[0] = bChild1.index;
				bChild1.parent = a.index;

				a.bv.fatBounds = Union(c.bv.fatBounds, bChild1.bv.fatBounds);
				b.bv.fatBounds = Union(a.bv.fatBounds, bChild0.bv.fatBounds);

				a.height = 1 + glm::max(c.height, bChild1.height);
				b.height = 1 + glm::max(a.height, bChild0.height);
			}
			else
			{
				b.children[1] = bChild1.index;
				a.children[0] = bChild0.index;
				bChild0.parent = a.index;

				a.bv.fatBounds = Union(c.bv.fatBounds, bChild0.bv.fatBounds);
				b.bv.fatBounds = Union(a.bv.fatBounds, bChild1.bv.fatBounds);

				a.height = 1 + glm::max(c.height, bChild0.height);
				b.height = 1 + glm::max(a.height, bChild1.height);
			}

			return b.index;
		}

		return a.index;
	}

	Int32 BVHierarchy::FindBestSiblingFor(Int32 newNodeIndex)
	{
		AABB const nnBounds = tree[newNodeIndex].bv.fatBounds;

		Int32 idx = rootIdx;
		Int32 prevIdx = NullIdx;
		while (not tree[idx].IsLeaf() && idx != prevIdx)
		{
			prevIdx = idx;

			BVHNode& curr = tree[idx];

			Real const currSA = curr.bv.fatBounds.SurfaceArea();
			Real const combinedSA = Union(curr.bv.fatBounds, nnBounds).SurfaceArea();

			Real const inheritedCost = 2.0_r * (combinedSA - currSA);
			Real       directCost = 2.0_r * combinedSA;

			// Descend according to minimum cost, break if no children are better than this node
			for (Int8 i = 0; i < 2; ++i)
			{
				ASSERT(curr.children[i] != NullIdx, "Internal nodes must have 2 valid children");

				BVHNode const& child = tree[curr.children[i]];
				AABB const childBounds = Union(nnBounds, child.bv.fatBounds);

				Real childCost = 0.0_r;
				if (child.IsLeaf())
				{
					childCost = childBounds.SurfaceArea() + inheritedCost;
				}
				else
				{
					Real const oldSA = child.bv.fatBounds.SurfaceArea();
					Real const newSA = childBounds.SurfaceArea();
					childCost = (newSA - oldSA) + inheritedCost;
				}

				if (childCost < directCost)
				{
					directCost = childCost;
					idx = curr.children[i];
				}
			}
		}

		return idx;
	}

	void BVHierarchy::RefitBVsFrom(Int32 index)
	{
		ASSERT(0 <= index && index < tree.Capacity(), "Bad index");
		while (index != NullIdx)
		{
			index = Balance(index);

			BVHNode& curr = tree[index];
			ASSERT(0 <= curr.children[0] && curr.children[0] < tree.Capacity() &&
				0 <= curr.children[1] && curr.children[1] < tree.Capacity(),
				"Internal nodes must have two valid children");

			BVHNode const& child0 = tree[curr.children[0]];
			BVHNode const& child1 = tree[curr.children[1]];

			curr.height = 1 + glm::max(child0.height, child1.height);
			curr.bv.fatBounds = Union(child0.bv.fatBounds, child1.bv.fatBounds);

			index = curr.parent;
		}
	}

	Bool BVHierarchy::InsertLeaf(Int32 newNodeIdx)
	{
		// If this is the first node, it is the root
		if (tree.Size() == 1)
		{
			rootIdx = newNodeIdx;
			return true;
		}

		// ---------------------------------------------------------------------
		// 1. find the best sibling for the new leaf
		// ---------------------------------------------------------------------
		Int32 siblingIdx = FindBestSiblingFor(newNodeIdx);

		// ---------------------------------------------------------------------
		// 2. create the new parent
		// ---------------------------------------------------------------------
		Int32 oldParent = tree[siblingIdx].parent;

		Int32 newParent = tree.Create( Union(tree[newNodeIdx].bv.fatBounds, tree[siblingIdx].bv.fatBounds) );

		tree[newParent].parent = oldParent;
		tree[newParent].height = tree[siblingIdx].height + 1;

		if (oldParent != NullIdx)
		{
			// The sibling was not the root
			if (tree[oldParent].children[0] == siblingIdx)
			{
				tree[oldParent].children[0] = newParent;
			}
			else
			{
				tree[oldParent].children[1] = newParent;
			}
			tree[newParent].children[0] = siblingIdx;
			tree[newParent].children[1] = newNodeIdx;
			tree[siblingIdx].parent = newParent;
			tree[newNodeIdx].parent = newParent;
		}
		else
		{
			// The sibling was the root
			tree[newParent].children[0] = siblingIdx;
			tree[newParent].children[1] = newNodeIdx;
			tree[siblingIdx].parent = newParent;
			tree[newNodeIdx].parent = newParent;
			rootIdx = newParent;
		}

		// ---------------------------------------------------------------------
		// 3. walk up tree and refit BVs
		// ---------------------------------------------------------------------
		RefitBVsFrom(newParent);

		return true;
	}

	void BVHierarchy::RemoveLeaf(Int32 index)
	{
		if (index == rootIdx)
		{
			rootIdx = NullIdx;
		}
		else
		{
			BVHNode& curr = tree[index];

			Int32 const parentIdx = curr.parent;
			Int32 const grandparentIdx = tree[parentIdx].parent;
			Int32 const siblingIdx = (tree[parentIdx].children[0] == index) ?
				tree[parentIdx].children[1] :
				tree[parentIdx].children[0];

			if (grandparentIdx != NullIdx)
			{
				BVHNode& grandparent = tree[grandparentIdx];
				for (Int8 i = 0; i < 2; ++i)
				{
					if (grandparent.children[i] == parentIdx)
					{
						grandparent.children[i] = siblingIdx;
						break;
					}
				}
				tree[siblingIdx].parent = grandparentIdx;
				tree.Free(parentIdx);

				RefitBVsFrom(grandparentIdx);
			}
			else
			{
				rootIdx = siblingIdx;
				tree[siblingIdx].parent = NullIdx;
				tree.Free(parentIdx);
			}
		}
	}

	Bool BVHierarchy::ValidHandle(BVHandle bvHandle, Int32& indexOut) const
	{
		Int32 const idx = bvHandle.info.index;

		if (0 <= idx && idx < tree.Capacity())
		{
			indexOut = idx;
			return tree[idx].version == bvHandle.info.version &&
				tree[idx].height != NullIdx;
		}

		indexOut = NullIdx;
		return false;
	}

	static inline AABB Union(AABB const& a, AABB const& b)
	{
		return a.Union(b);
	}
}