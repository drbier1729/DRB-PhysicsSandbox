
namespace drb {
	namespace physics {

		// See Ericson Ch. 5
		inline CastResult RayCast(Ray const& r, AABB const& aabb)
		{
			Float32 tmin = 0.0f;
			Float32 tmax = std::numeric_limits<Float32>::max();

			// For all three slabs
			for (Uint32 i = 0u; i < 3u; ++i)
			{
				if (EpsilonEqual(r.d[i], 0.0f))
				{
					// Ray is parallel to slab. No hit if origin not within slab
					if (r.p[i] < aabb.min[i] || r.p[i] > aabb.max[i]) { return CastResult{}; }
				}
				else {
					// Compute intersection t value of ray with near and far plane of slab
					Float32 const ood = 1.0f / r.d[i];
					Float32 t1 = (aabb.min[i] - r.p[i]) * ood;
					Float32 t2 = (aabb.max[i] - r.p[i]) * ood;

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
				.point = r.p + r.d * tmin,
				.distance = tmin,
				.hit = true
			};
		}
		
		
		inline Float32 SignedDistance(Vec3 const& point, Plane const& plane)
		{
			return glm::dot(point, plane.n) - plane.d;
		}


		// See Ericson 5.3.1
		inline Bool Intersect(Segment const& seg, Plane const& plane, float& t, Vec3& q)
		{
			// Compute the t value for the directed line v intersecting the plane
			Vec3 const v = seg.e - seg.b;
			t = -SignedDistance(seg.b, plane) / glm::dot(plane.n, v);

			// If t in [0..1] compute and return intersection point
			if (t >= 0.0f && t <= 1.0f) 
			{
				q = seg.b + t * v;
				return true;
			}

			// Else no intersection
			return false;
		}


		// Classify point p to a plane thickened by a given thickness epsilon
		inline Side ClassifyPointToPlane(Vec3 const& p, Plane const& plane)
		{
			Float32 const dist = SignedDistance(p, plane);

			// Classify p based on the signed distance
			if (dist > Plane::thickness)  { return Side::Front; }
			if (dist < -Plane::thickness) { return Side::Back; }
			return Side::On;
		}
	}
}