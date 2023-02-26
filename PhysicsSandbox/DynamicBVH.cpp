#include "pch.h"
#include "DynamicBVH.h"

#include "PhysicsGeometry.h"

namespace drb::physics {

	static inline AABB Union(AABB const& a, AABB const& b);
	static inline BV::Handle MakeHandle(Uint32 index, Uint32 version);

	BV::Handle BVHierarchy::Insert(AABB const& aabb, void* userData) 
	{ 	
		BVHNode* newNode = tree.Create(aabb, userData);
		if (not newNode) 
		{
			ASSERT(false, "Failed to create node");
			return std::numeric_limits<BV::Handle>::max();
		}
		
		newNode->moved = true;
		Bool const success = InsertLeaf(newNode);

		return success ? 
			MakeHandle(newNode->index, newNode->version) : 
			std::numeric_limits<BV::Handle>::max();
	}
	
	void  BVHierarchy::Remove(BV::Handle bvHandle) 
	{
		if (Uint32 index = NullIdx; ValidHandle(bvHandle, index))
		{
			RemoveLeaf(index);
			tree.Free(index);
		}
	}

	BV const* BVHierarchy::Find(BV::Handle bvHandle) const
	{
		if (Uint32 index = NullIdx; ValidHandle(bvHandle, index))
		{
			return &tree[index].bv;
		}
		return nullptr;
	}

	Bool BVHierarchy::MoveBoundingVolume(BV::Handle handle, AABB const& aabb, Vec3 const& displacement)
	{
		if (Uint32 index = NullIdx; ValidHandle(handle, index))
		{
			ASSERT(tree[index].IsLeaf(), "Cannot move internal nodes");

			Vec3 const d = BV::displacementMultiplier * displacement;

			AABB predictedAABB = aabb.Expanded(BV::enlargeFactor);
			if (d.x < 0.0f) { predictedAABB.min.x += d.x; }
			else            { predictedAABB.max.x += d.x; }
			if (d.y < 0.0f) { predictedAABB.min.y += d.y; }
			else            { predictedAABB.max.y += d.y; }
			if (d.z < 0.0f) { predictedAABB.min.z += d.z; }
			else            { predictedAABB.max.z += d.z; }

			AABB const& treeAABB = tree[index].bv.fatBounds;
			if (treeAABB.Contains(predictedAABB))
			{
				AABB const hugeAABB = predictedAABB.Expanded(BV::displacementMultiplier * BV::enlargeFactor);
				if (hugeAABB.Contains(predictedAABB))
				{
					return false;
				}
			}

			RemoveLeaf(index);

			BVHNode& n = tree[index];
			n.bv.fatBounds = predictedAABB;

			InsertLeaf(&n);
			n.moved = true;			
		}

		return false;
	}

	void BVHierarchy::Reserve(Uint32 objectCount)
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
	{}

	BVHierarchy::NodePool::~NodePool() noexcept
	{
		std::free(nodes);
	}
	
	void BVHierarchy::NodePool::Clear()
	{
		if (capacity == 0) { return; }

		// Reduce size
		size = 0;

		// Zero the nodes
		std::memset(nodes, 0, capacity * sizeof(BVHNode));

		// Reset free list
		for (Uint32 i = 0; (i + 1) < capacity; ++i)
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

	BVHNode& BVHierarchy::NodePool::operator[](Uint32 index)
	{
		ASSERT(index < capacity, "Index out of range");
		ASSERT(not nodes[index].IsFree(), "Node is dead");
		return nodes[index];
	}

	BVHNode const& BVHierarchy::NodePool::operator[](Uint32 index) const
	{
		ASSERT(index < capacity, "Index out of range");
		ASSERT(not nodes[index].IsFree(), "Node is dead");
		return nodes[index];
	}

	BVHNode* BVHierarchy::NodePool::Create(AABB const& aabb, void* userData)
	{	
		// Grow the pool if needed
		if (firstFree == NullIdx)
		{
			ASSERT(size == capacity, "Growing unecessarily");
			capacity = Reserve(capacity * 2);
		}

		Uint32 const index = firstFree;
		ASSERT(index < capacity, "Bad index");

		BVHNode& newNode = nodes[index];
		firstFree = newNode.nextFree;
		++size;

		newNode.Create(aabb, userData);
		return &newNode;
	}

	void  BVHierarchy::NodePool::Free(Uint32 nodeIndex)
	{
		ASSERT(nodeIndex < capacity, "Bad index");
		ASSERT(size > 0, "Empty array");

		nodes[nodeIndex].Free(firstFree);
		firstFree = nodeIndex;

		--size;
	}

	Uint32 BVHierarchy::NodePool::Size() const { return size; }

	Uint32 BVHierarchy::NodePool::Capacity() const { return capacity; }

	Uint32 BVHierarchy::NodePool::Reserve(Uint32 newCapacity)
	{
		static constexpr Uint32 maxCapacity = 1024u * 1024u;
		
		if (capacity >= newCapacity || capacity == maxCapacity)
		{
			//ASSERT(capacity < newCapacity, "Reserving unecessarily");
			//ASSERT(capacity < maxCapacity, "Max capacity reached");
			return capacity;
		}

		if (capacity == 0)
		{
			ASSERT(size == 0, "Size invalid");

			capacity = glm::max(minCapacity, newCapacity);
			nodes    = (BVHNode*)std::malloc(capacity * sizeof(BVHNode));

			if (not nodes)
			{
				ASSERT(nodes, "Bad alloc");
				return capacity;
			}
		}
		else
		{
			BVHNode* const oldNodes = nodes;
			Uint32 const   oldCap   = capacity;

			// Grow by at least by a factor of 2, or up to maxCapacity (which ever is less)
			capacity = glm::min(maxCapacity, glm::max(newCapacity, capacity * 2));
			nodes    = (BVHNode*)std::malloc(capacity * sizeof(BVHNode));

			if (not nodes) {
				ASSERT(false, "Bad alloc");
				nodes    = oldNodes;
				capacity = oldCap;
				return capacity;
			}

			// Copy the old nodes then free them
			std::memcpy(nodes, oldNodes, size * sizeof(BVHNode));
			std::free(oldNodes);
		}

		// Zero the new nodes
		std::memset(nodes + size, 0, (capacity - size) * sizeof(BVHNode));

		// Set up free list
		for (Uint32 i = size; (i + 1) < capacity; ++i)
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
	Uint32 BVHierarchy::Balance(Uint32 aIdx)
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
			c.parent      = a.parent;
			a.parent      = c.index;
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

	BVHNode* BVHierarchy::FindBestSiblingFor(BVHNode const* newNode)
	{
		AABB const    nnBounds = newNode->bv.fatBounds;
		
		Uint32 idx = rootIdx;
		Uint32 prevIdx = NullIdx;
		while (not tree[idx].IsLeaf() && idx != prevIdx)
		{
			prevIdx = idx;

			BVHNode& curr = tree[idx];
			
			Float32 const currSA     = curr.bv.fatBounds.SurfaceArea();
			Float32 const combinedSA = Union(curr.bv.fatBounds, nnBounds).SurfaceArea();

			Float32 const inheritedCost = 2.0f * (combinedSA - currSA);
			Float32       directCost    = 2.0f * combinedSA;

			// Descend according to minimum cost, break if no children are better than this node
			for (Uint8 i = 0; i < 2; ++i)
			{
				ASSERT(curr.children[i] != NullIdx, "Internal nodes must have 2 valid children");

				BVHNode const& child   = tree[curr.children[i]];
				AABB const childBounds = Union(nnBounds, child.bv.fatBounds);

				Float32 childCost = 0.0f;
				if (child.IsLeaf())
				{
					childCost = childBounds.SurfaceArea() + inheritedCost;
				}
				else
				{
					Float32 const oldSA = child.bv.fatBounds.SurfaceArea();
					Float32 const newSA = childBounds.SurfaceArea();
					childCost = (newSA - oldSA) + inheritedCost;
				}

				if (childCost < directCost)
				{
					directCost = childCost;
					idx = curr.children[i];
				}
			}
		}

		return &tree[idx];
	}

	void BVHierarchy::RefitBVsFrom(Uint32 index)
	{
		while (index != NullIdx)
		{
			index = Balance(index);

			BVHNode& curr = tree[index];
			ASSERT(curr.children[0] < tree.Capacity() && curr.children[1] < tree.Capacity(), "Internal nodes must have two valid children");

			BVHNode const& child0 = tree[curr.children[0]];
			BVHNode const& child1 = tree[curr.children[1]];

			curr.height       = 1 + glm::max(child0.height, child1.height);
			curr.bv.fatBounds = Union(child0.bv.fatBounds, child1.bv.fatBounds);

			index = curr.parent;
		}
	}

	Bool BVHierarchy::InsertLeaf(BVHNode* newNode)
	{
		// If this is the first node, it is the root
		if (tree.Size() == 1)
		{
			rootIdx = newNode->index;
			return true;
		}

		// ---------------------------------------------------------------------
		// 1. find the best sibling for the new leaf
		// ---------------------------------------------------------------------
		BVHNode* sibling = FindBestSiblingFor(newNode);

		// ---------------------------------------------------------------------
		// 2. create the new parent
		// ---------------------------------------------------------------------
		Int32 oldParent = sibling->parent;

		BVHNode* newParent = tree.Create(Union(newNode->bv.fatBounds, sibling->bv.fatBounds));
		if (not newParent)
		{
			ASSERT(false, "Failed to create parent node.");
			tree.Free(newNode->index);
			return false;
		}

		newParent->parent = oldParent;
		newParent->height = sibling->height + 1;

		if (oldParent != NullIdx)
		{
			// The sibling was not the root
			if (tree[oldParent].children[0] == sibling->index)
			{
				tree[oldParent].children[0] = newParent->index;
			}
			else
			{
				tree[oldParent].children[1] = newParent->index;
			}
			newParent->children[0] = sibling->index;
			newParent->children[1] = newNode->index;
			sibling->parent = newParent->index;
			newNode->parent = newParent->index;
		}
		else
		{
			// The sibling was the root
			newParent->children[0] = sibling->index;
			newParent->children[1] = newNode->index;
			sibling->parent = newParent->index;
			newNode->parent = newParent->index;
			rootIdx = newParent->index;
		}

		// ---------------------------------------------------------------------
		// 3. walk up tree and refit BVs
		// ---------------------------------------------------------------------
		RefitBVsFrom(newNode->parent);
		
		return true;
	}

	void BVHierarchy::RemoveLeaf(Uint32 index)
	{
		if (index == rootIdx)
		{
			rootIdx = NullIdx;
		}
		else
		{
			BVHNode& curr = tree[index];

			Uint32 const parentIdx = curr.parent;
			Uint32 const grandparentIdx = tree[parentIdx].parent;
			Uint32 const siblingIdx = (tree[parentIdx].children[0] == index) ?
				tree[parentIdx].children[1] :
				tree[parentIdx].children[0];

			if (grandparentIdx != NullIdx)
			{
				BVHNode& grandparent = tree[grandparentIdx];
				for (Uint8 i = 0; i < 2; ++i)
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


	Bool BVHierarchy::ValidHandle(BV::Handle bvHandle, Uint32& indexOut) const
	{
		static constexpr auto mask32b = 0xFFFFFFFF;

		indexOut             = static_cast<Uint32>( bvHandle & mask32b );
		Uint32 const version = static_cast<Uint32>( bvHandle >> 32 );

		return indexOut < tree.Capacity() && 
			tree[indexOut].version == version && 
			tree[indexOut].height != NullIdx;
	}

	static inline AABB Union(AABB const& a, AABB const& b)
	{
		return a.Union(b);
	}

	static inline BV::Handle MakeHandle(Uint32 index, Uint32 version)
	{
		return static_cast<Uint64>(index) | (static_cast<Uint64>(version) << 32);
	}
}