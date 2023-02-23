#ifndef DRB_DYNAMICBVH_H
#define DRB_DYNAMICBVH_H

#include "AABB.h"

namespace drb::physics {
	
	// Fwd decl
	class RigidBody;
	struct Ray;
	struct Sphere;

	// Could make this a template class which can accept
	// other shapes which satisfy a concept BoundingVolume
	struct BV
	{
		AABB  bounds      = {};
		Int32 ownerHandle = std::numeric_limits<Int32>::min(); // opaque handle to a RigidBody or CollisionGeometry object
	};

	class BVHierarchy
	{
	private:
		// Binary tree currently, but could add children
		// for a quad tree, etc
		struct Node 
		{
			static inline constexpr Int32 NullIdx = -1;

			Int32 idx         = NullIdx;
			Int32 parent      = NullIdx;
			Int32 children[2] = { NullIdx, NullIdx };

			inline Bool IsRoot() const  { return parent == NullIdx; }
			inline Bool IsLeaf() const  { return children[0] == NullIdx && children[1] == NullIdx; }
		};

	private:
		std::vector<BV>   boundingVolumes = {};
		std::vector<Node> tree            = {};
		Int32             rootIdx         = Node::NullIdx;
		Int32			  firstFree       = Node::NullIdx;
		Int32			  size            = 0;

	public:
		Int32 Insert(BV const& bv);
		
		// TODO
		void  Remove(Int32 bvHandle);
		void  Balance();
		void Query(AABB const& box, std::vector<BV>& out) const;
		void Query(Sphere const& sph, std::vector<BV>& out) const;
		void Query(Ray const&  ray, std::vector<BV>& out) const;


	private:
		// Helpers
		Node& CreateNode(BV const& bv = {});
		Node& FindBestSiblingFor(Node const& n);
	};

}
#endif

