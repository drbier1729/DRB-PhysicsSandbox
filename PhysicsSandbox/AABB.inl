
namespace drb {

	namespace physics {

		inline Bool AABB::Intersects(AABB const& other) const
		{
			if (max.x < other.min.x || other.max.x < min.x) { return false; }
			if (max.y < other.min.y || other.max.y < min.y) { return false; }
			if (max.z < other.min.z || other.max.z < min.z) { return false; }
			return true;
			
			//return not glm::any(glm::lessThan(max, other.min)) && 
			//	   not glm::any(glm::lessThan(other.max, min));
		}

		inline Bool AABB::Contains(AABB const& other) const
		{
			return min.x <= other.min.x &&
				min.y <= other.min.y &&
				min.z <= other.min.z &&
				max.x >= other.max.x &&
				max.y >= other.max.y &&
				max.z >= other.max.z;

			//return glm::all(glm::lessThanEqual(min, other.min)) &&
			//	   glm::all(glm::lessThanEqual(other.max, max));
		}

		inline Float32 AABB::DistSquaredFromPoint(Vec3 const& pt) const
		{
			Float32 d2 = 0.0f;
			for (Uint32 i = 0; i < 3; ++i)
			{
				Float32 const v = pt[i];
				if (v < min[i]) { d2 += (min[i] - v) * (min[i] - v); }
				if (v > max[i]) { d2 += (v - max[i]) * (v - max[i]); }
			}
			return d2;
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

		inline AABB AABB::Transformed(Mat4 const& transform) const
		{
			return Transformed(Mat3(transform), transform[3]);
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