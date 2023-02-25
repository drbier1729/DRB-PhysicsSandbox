#include "pch.h"
#include "DynamicBVH.h"

#include "PhysicsGeometry.h"

namespace drb::physics {

	static inline AABB Union(AABB const& a, AABB const& b);
	static inline BV::Handle MakeHandle(Uint32 index, Uint32 version);

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


	BV::Handle BVHierarchy::Insert(AABB const& aabb, void* userData) 
	{ 	
		BVHNode* newNode = tree.Create(aabb, userData);
		if (not newNode) 
		{
			ASSERT(false, "Failed to create node");
			return std::numeric_limits<BV::Handle>::max();
		}
		
		// If we just created the first node, it is the root
		if (tree.Size() == 1)
		{ 
			rootIdx = newNode->index;
			return MakeHandle(rootIdx, newNode->version); 
		}

		// ---------------------------------------------------------------------
		// 1. find the best sibling for the new leaf
		// ---------------------------------------------------------------------
		BVHNode* sibling = FindBestSiblingFor(newNode);

		// ---------------------------------------------------------------------
		// 2. create the new parent
		// ---------------------------------------------------------------------
		Int32 oldParent = sibling->parent;

		BVHNode* newParent = tree.Create( Union(newNode->bv.fatBounds, sibling->bv.fatBounds) );
		if (not newParent) 
		{
			ASSERT(false, "Failed to create parent node.");
			tree.Free(newNode->index);
			return std::numeric_limits<BV::Handle>::max();
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

		return MakeHandle(newNode->index, newNode->version);
	}
	
	void  BVHierarchy::Remove(BV::Handle bvHandle) 
	{
		if (Uint32 index = NullIdx; ValidHandle(bvHandle, index))
		{
			if (index == rootIdx)
			{
				rootIdx = NullIdx;
				return;
			}
			else
			{
				BVHNode& curr = tree[index];

				Uint32 const parentIdx      = curr.parent;
				Uint32 const grandparentIdx = tree[parentIdx].parent;
				Uint32 const siblingIdx     = (tree[parentIdx].children[0] == index) ?
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

	Bool BVHierarchy::MoveBV(BV::Handle, AABB const& aabb, Vec3 const& displacement)
	{
		ASSERT(false, "Not implemented");
		return false;
	}

	// -------------------------------------------------------------------------
	// Helpers
	// -------------------------------------------------------------------------

	BVHierarchy::NodePool::NodePool()
		: nodes{ (BVHNode*)std::malloc(sizeof(BVHNode) * 16) },
		firstFree{ NullIdx },
		size{ 0 },
		capacity{ 16 }
	{
		ASSERT(nodes, "Bad alloc");

		// Zero the new nodes
		std::memset(nodes, 0, capacity * sizeof(BVHNode));

		// Set up free list
		for (Uint32 i = 0; i < capacity - 1; ++i)
		{
			nodes[i].nextFree = i + 1;
			nodes[i].height = NullIdx;
			nodes[i].index = i;
		}
		nodes[capacity - 1].nextFree = NullIdx;
		nodes[capacity - 1].height = NullIdx;
		nodes[capacity - 1].index = capacity - 1;
	}

	BVHierarchy::NodePool::~NodePool() noexcept
	{
		std::free(nodes);
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

			static constexpr Uint32 maxCapacity = std::numeric_limits<Uint32>::max() / 2;
			
			if (capacity >= maxCapacity) {
				ASSERT(capacity < maxCapacity, "Growing will cause overflow");
				return nullptr;
			}

			BVHNode* const oldNodes = nodes;
			Uint32 const   oldCap   = capacity;
			
			capacity = (capacity == 0) ? 16 : capacity * 2;  // growth factor of 2
			nodes    = (BVHNode*)std::malloc(capacity * sizeof(BVHNode));
			
			if (not nodes) {
				ASSERT(false, "Bad alloc");
				nodes    = oldNodes;
				capacity = oldCap;
				return nullptr;
			}

			// Copy the old nodes then free them
			std::memcpy(nodes, oldNodes, size * sizeof(BVHNode));
			std::free(oldNodes);
		
			// Zero the new nodes
			std::memset(nodes + size, 0, (capacity - size) * sizeof(BVHNode));

			// Set up free list
			for (Uint32 i = size; (i + 1) < capacity; ++i)
			{
				nodes[i].nextFree = i + 1;
				nodes[i].height   = NullIdx;
				nodes[i].index    = i;
			}
			nodes[capacity - 1].nextFree = NullIdx;
			nodes[capacity - 1].height   = NullIdx;
			nodes[capacity - 1].index    = capacity - 1;

			firstFree = size;
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

	// Performs left or right rotation if subtree with root at index is
	// imbalanced. Returns the index of the new root of the subtree.
	Uint32 BVHierarchy::Balance(Uint32 index)
	{
		ASSERT(index < tree.Capacity(), "Index invalid");

		BVHNode& a = tree[index];
		if (a.IsLeaf() || a.height < 2)
		{
			return index;
		}

		ASSERT(false, "Impl not finished");
		return NullIdx;
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

			BVHNode& const child0 = tree[curr.children[0]];
			BVHNode& const child1 = tree[curr.children[1]];

			curr.height       = 1 + glm::max(child0.height, child1.height);
			curr.bv.fatBounds = Union(child0.bv.fatBounds, child1.bv.fatBounds);

			index = curr.parent;
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