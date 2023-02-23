
namespace drb {

	namespace physics {

		inline Bool AABB::Intersects(AABB const& other) const
		{
			if (max.x < other.min.x || min.x > other.max.x) { return false; }
			if (max.z < other.min.z || min.z > other.max.z) { return false; }
			if (max.y < other.min.y || min.y > other.max.y) { return false; }
			return true;
		}

		inline AABB AABB::Union(AABB const& other) const
		{
			return AABB{ 
				.max = glm::max(max, other.max),
				.min = glm::min(min, other.min)
			};
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