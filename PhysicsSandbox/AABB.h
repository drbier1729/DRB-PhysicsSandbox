#ifndef DRB_AABB_H
#define DRB_AABB_H

namespace drb:: physics {

	struct AABBQuery
	{
		Real  penetration = std::numeric_limits<Real>::max();
		Int32 axis        = -1; // x == 0, y == 1, z == 2
	};

	struct AABB
	{
		// Default construct "inside out"
		Vec3 c{ 0.0_r };
		Vec3 e{ std::numeric_limits<Real>::lowest() };

		inline AABB& SetMinMax(Vec3 const& min, Vec3 const& max);

		inline Bool Intersects(AABB const& other) const;
		inline Bool Contains(AABB const& other) const;

		inline Real DistSquaredFromPoint(Vec3 const& pt) const;

		inline AABB Transformed(Quat const& rotation, Vec3 const& displacement) const;
		inline AABB Transformed(Mat3 const& rotation, Vec3 const& displacement) const;
		inline AABB Transformed(Mat4 const& transform) const;
		inline AABB MovedBy(Vec3 const& displacement) const;
		inline AABB MovedTo(Vec3 const& newCenter) const;
		inline AABB Rotated(Mat3 const& rotation) const;
		inline AABB Expanded(Real const scaleFactor) const;
		inline AABB Union(AABB const& other) const;

		inline Vec3 Center() const;
		inline Vec3 Halfwidths() const;
		inline Vec3 Max() const;
		inline Vec3 Min() const;
		inline Real SurfaceArea() const;
	};
}

#include "AABB.inl"

#endif

