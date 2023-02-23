#include "pch.h"
#include "DynamicBVH.h"

#include "PhysicsGeometry.h"

namespace drb::physics {

	inline static AABB Union(AABB const& a, AABB const& b)
	{
		return a.Union(b);
	}

	Int32 BVHierarchy::Insert(BV const& bv) 
	{ 	
		Node& newNode = CreateNode(bv);

		// If just created the first node, we can just return the root index
		if (size == 1) 
		{ 
			ASSERT(newNode.idx == rootIdx, "Internal error. New Node should have been the root.");
			return newNode.idx; 
		}

		// ---------------------------------------------------------------------
		// 1. find the best sibling for the new leaf
		// ---------------------------------------------------------------------
		Node& sibling = FindBestSiblingFor(newNode);

		// ---------------------------------------------------------------------
		// 2. create the new parent
		// ---------------------------------------------------------------------
		Int32 oldParent = sibling.parent;

		Node& newParent = CreateNode({
			.bounds = Union(bv.bounds, boundingVolumes[sibling.idx].bounds)
		});

		newParent.parent = oldParent;
		
		if (oldParent != Node::NullIdx)
		{
			// The sibling was not the root
			if (tree[oldParent].children[0] == sibling.idx)
			{
				tree[oldParent].children[0] = newParent.idx;
			}
			else
			{
				tree[oldParent].children[1] = newParent.idx;
			}
			newParent.children[0] = sibling.idx;
			newParent.children[1] = newNode.idx;
			sibling.parent = newParent.idx;
			newNode.parent = newParent.idx;
		}
		else
		{
			// The sibling was the root
			newParent.children[0] = sibling.idx;
			newParent.children[1] = newNode.idx;
			sibling.parent = newParent.idx;
			newNode.parent = newParent.idx;
			rootIdx = newParent.idx;
		}

		// ---------------------------------------------------------------------
		// 3. walk up tree and refit BVs
		// ---------------------------------------------------------------------
		Int32 index = newNode.parent;
		while (index != Node::NullIdx)
		{
			Int32 const child0 = tree[index].children[0];
			Int32 const child1 = tree[index].children[1];

			if (child0 != Node::NullIdx && child1 != Node::NullIdx)
			{
				BV const& childBV0 = boundingVolumes[child0];
				BV const& childBV1 = boundingVolumes[child1];

				boundingVolumes[index].bounds = Union(childBV0.bounds, childBV1.bounds);
			}
			else if (child0 != Node::NullIdx)
			{
				boundingVolumes[index].bounds = boundingVolumes[child0].bounds;
			}
			else if (child1 != Node::NullIdx)
			{
				boundingVolumes[index].bounds = boundingVolumes[child1].bounds;
			}
			else
			{
				ASSERT(false, "Parent node not attached to children");
			}

			index = tree[index].parent;
		}

		return newNode.idx;
	}
	
	void  BVHierarchy::Remove(Int32 bvHandle) 
	{

	}

	void BVHierarchy::Balance() 
	{
	
	}

	void  BVHierarchy::Query(AABB const& box,   std::vector<BV>& out) const {}
	void  BVHierarchy::Query(Sphere const& sph, std::vector<BV>& out) const {}
	void  BVHierarchy::Query(Ray const& ray,    std::vector<BV>& out) const {}


	// -------------------------------------------------------------------------
	// Helpers
	// -------------------------------------------------------------------------
	BVHierarchy::Node& BVHierarchy::CreateNode(BV const& bv)
	{
		// Empty tree
		if (firstFree == Node::NullIdx && size == 0)
		{
			ASSERT(boundingVolumes.size() == 0 && tree.size() == 0, "If we're in this scope, the vector should be empty.");

			size = 1;
			rootIdx = 0;
			boundingVolumes.emplace_back(bv);

			return tree.emplace_back(Node{ .idx = 0 });
		}

		++size;

		// Non empty tree, but no removals yet
		if (firstFree == Node::NullIdx)
		{
			Int32 const handle = static_cast<Int32>(boundingVolumes.size());
			boundingVolumes.emplace_back(bv);
			return tree.emplace_back(Node{ .idx = handle });
		}

		// Else: non empty tree with at least one removal
		Int32 const handle = firstFree;

		boundingVolumes[handle] = bv;
		tree[handle] = Node{ .idx = handle };

		// Look for the next free slot -- this can be accelerated
		// by using a FixedObjectPool or another data structure
		// that is better suited to handle this kind of pattern.
		Int32 const treeArraySize = static_cast<Int32>(tree.size());
		Bool found = false;
		do
		{
			firstFree = (firstFree + 1) % treeArraySize;

			if (tree[firstFree].idx == Node::NullIdx) { found = true; break; }

		} while (firstFree != handle);

		if (not found) { firstFree = Node::NullIdx; }

		return tree[handle];
	}


	BVHierarchy::Node& BVHierarchy::FindBestSiblingFor(Node const& newNode)
	{
		AABB const    newNodeBounds = boundingVolumes[newNode.idx].bounds;
		Float32 const newNodeSA     = newNodeBounds.SurfaceArea();

		//						     inheritedCost,  nodeIndex
		std::priority_queue<std::pair<   Float32,      Int32   >> pq{};
		pq.push({ 0.0f, rootIdx });

		Float32 bestCost = Union(newNodeBounds, boundingVolumes[rootIdx].bounds).SurfaceArea(); // root cost
		Int32   bestIdx  = rootIdx;

		while (not pq.empty())
		{
			// Pop the current node
			auto [currInhCost, currIdx] = pq.top();
			pq.pop();


			// Compute cost for current node
			Float32 const directCost = Union(newNodeBounds, boundingVolumes[currIdx].bounds).SurfaceArea(); // SA(newNode U currNode)
			Float32 const totalCost  = directCost + currInhCost;
			if (totalCost < bestCost)
			{
				bestCost = totalCost;
				bestIdx  = currIdx;
			}


			// Check if we can prune child subtrees, else push them to pq
			Float32 const currSA        = boundingVolumes[currIdx].bounds.SurfaceArea();
			Float32 const childInhCost  = totalCost - currSA; // This is (directCost - currSA) + currInhCost = dSA(curr) + dSA(curr's parent) + dSA(curr's grandparent) + ...

			if (newNodeSA + childInhCost < bestCost)
			{
				if (tree[currIdx].children[0] != Node::NullIdx)
				{
					pq.push({ childInhCost, tree[currIdx].children[0] });
				}
				if (tree[currIdx].children[1] != Node::NullIdx)
				{
					pq.push({ childInhCost, tree[currIdx].children[1] });
				}
			}
		}

		return tree[bestIdx];
	}
}