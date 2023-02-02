#ifndef DRB_AABB_H
#define DRB_AABB_H

namespace drb {
	namespace physics {

		struct AABB
		{
			Vec3 min, max;
			
			inline Bool Intersects(AABB const& other) const;
			inline AABB Transformed(Mat3 const& orientation, Vec3 const& pos) const;
			inline AABB Union(AABB const& other) const;
			inline Vec3 Center() const;
			inline Vec3 Halfwidths() const;
		};
	}
}

#include "AABB.inl"

#endif

