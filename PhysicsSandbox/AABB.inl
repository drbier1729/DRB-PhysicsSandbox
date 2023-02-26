
namespace drb {

	namespace physics {

		inline Bool AABB::Intersects(AABB const& other) const
		{
			return not glm::any(glm::lessThan(max, other.min)) && 
				   not glm::any(glm::lessThan(other.max, min));
		}

		inline Bool AABB::Contains(AABB const& other) const
		{
			return glm::all(glm::lessThanEqual(min, other.min)) &&
				   glm::all(glm::lessThanEqual(other.max, max));
		}

		inline AABB AABB::Expanded(Float32 const scaleFactor) const
		{
			Vec3 const r{ scaleFactor };
			return AABB{ .max = max + r, .min = min - r };
		}

		inline AABB AABB::Union(AABB const& other) const
		{
			return AABB{ 
				.max = glm::max(max, other.max),
				.min = glm::min(min, other.min)
			};
		}

		inline AABB AABB::MovedBy(Vec3 const& displacement) const
		{
			return AABB{ .max = max + displacement, .min = min + displacement };
		}


		inline AABB AABB::MovedTo(Vec3 const& newCenter) const
		{
			return AABB{ .max = newCenter + Halfwidths(), .min = newCenter - Halfwidths() };
		}

		// See Realtime Collision Detection by Ericson, Ch 4
		inline AABB AABB::Transformed(Mat3 const& orientation, Vec3 const& pos) const
		{
			AABB result{ .max = pos, .min = pos };

			// Form extents by summing smaller and larger terms respectively
			for (auto i = 0; i < 3; i++) {
				for (auto j = 0; j < 3; j++) {
					
					Float32 const e = orientation[i][j] * min[j];
					Float32 const f = orientation[i][j] * max[j];
					
					if (e < f) 
					{
						result.min[i] += e;
						result.max[i] += f;
					}
					else 
					{
						result.min[i] += f;
						result.max[i] += e;
					}
				}
			}

			return result;
		}

		inline Vec3 AABB::Center() const
		{
			return 0.5f * (min + max);
		}

		inline Vec3 AABB::Halfwidths() const
		{
			return 0.5f * (max - min);
		}


		inline Float32 AABB::SurfaceArea() const
		{
			Vec3 const d = max - min;
			return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
		}
	}
}