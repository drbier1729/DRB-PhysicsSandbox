#ifndef DRB_AABB_H
#define DRB_AABB_H

namespace drb:: physics {

		struct AABB
		{
			// Default construct "inside out"
			Vec3 max{ std::numeric_limits<Float32>::lowest() };
			Vec3 min{ std::numeric_limits<Float32>::max() };

			inline Bool Intersects(AABB const& other) const;
			inline Bool Contains(AABB const& other) const;

			inline AABB Transformed(Mat3 const& orientation, Vec3 const& pos) const;
			inline AABB MovedBy(Vec3 const& displacement) const;
			inline AABB MovedTo(Vec3 const& newCenter) const;
			inline AABB Expanded(Float32 const scaleFactor) const;
			inline AABB Union(AABB const& other) const;

			inline Vec3 Center() const;
			inline Vec3 Halfwidths() const;
			inline Float32 SurfaceArea() const;

			
		};
}

#include "AABB.inl"

#endif

