#include "pch.h"
#include "RayCast.h"

#include "Math.h"
#include "AABB.h"
#include "DRBAssert.h"

namespace drb::physics {

	// See Ericson Ch. 5
	CastResult Ray::Cast(AABB const& aabb) const
	{
		Real tmin = 0.0_r;
		Real tmax = std::numeric_limits<Real>::max();

		Vec3 const min = aabb.Min(), max = aabb.Max();

		// For all three slabs
		for (Uint32 i = 0u; i < 3u; ++i)
		{
			if (EpsilonEqual(d[i], 0.0_r))
			{
				// Ray is parallel to slab. No hit if origin not within slab
				if (p[i] < min[i] || p[i] > max[i]) { return CastResult{}; }
			}
			else {
				// Compute intersection t value of ray with near and far plane of slab
				Real const ood = 1.0_r / d[i];
				Real t1 = (min[i] - p[i]) * ood;
				Real t2 = (max[i] - p[i]) * ood;

				// Make t1 be intersection with near plane, t2 with far plane
				if (t1 > t2) { std::swap(t1, t2); }

				// Compute the intersection of slab intersection intervals
				tmin = std::max(tmin, t1);
				tmax = std::min(tmax, t2);

				// Exit with no collision as soon as slab intersection becomes empty
				if (tmin > tmax) { return CastResult{}; }
			}
		}

		// Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin)
		return CastResult{
			.point = p + d * tmin,
			.distance = tmin,
			.hit = true
		};
	}


	CastResult Ray::Cast(Sphere const& sph, Vec3 const& sphCenter) const
	{
		ASSERT(false, "Not implemented");
		return CastResult{};
	}
}