
namespace drb {

	namespace physics {

		inline Bool AABB::Intersects(AABB const& other) const
		{
			return glm::all(
					glm::lessThanEqual(
							glm::abs(c - other.c),
							e + other.e
						)
					);

			/*return not (glm::any(glm::lessThan(max, other.min)) ||
				         glm::any(glm::lessThan(other.max, min)) );*/
		}

		/*inline AABBQuery AABB::Collide(AABB const& other) const
		{
			AABBQuery result{};

			AABB const overlap = {
				.max = glm::min(max, other.max),
				.min = glm::max(min, other.min)
			};

			for (Int32 i = 0; i < 3; ++i)
			{
				Real const pen = overlap.max[i] - overlap.min[i];
				if (pen < result.penetration)
				{
					result.penetration = pen;
					result.axis = i;
				}
			}

			return result;
		}*/


		inline Bool AABB::Contains(AABB const& other) const
		{
			return glm::all(glm::lessThanEqual(Min(), other.Min())) &&
				glm::all(glm::greaterThanEqual(Max(), other.Max()));
		}

		inline Real AABB::DistSquaredFromPoint(Vec3 const& pt) const
		{
			Vec3 const min = Min();
			Vec3 const max = Max();
			Real d2 = 0.0f;
			for (Uint32 i = 0; i < 3; ++i)
			{
				Real const v = pt[i];
				if (v < min[i]) { d2 += (min[i] - v) * (min[i] - v); }
				if (v > max[i]) { d2 += (v - max[i]) * (v - max[i]); }
			}
			return d2;
		}

		inline Vec3 AABB::Min() const
		{
			return c - e;
		}

		inline Vec3 AABB::Max() const
		{
			return c + e;
		}

		inline AABB& AABB::SetMinMax(Vec3 const& min, Vec3 const& max)
		{
			c = 0.5_r * (max + min);
			e = 0.5_r * (max - min);
			return *this;
		}

		inline AABB AABB::Expanded(Real const scaleFactor) const
		{
			Vec3 const r{ scaleFactor };
			return AABB{ .c = c, .e = e + r };
		}

		inline AABB AABB::Union(AABB const& other) const
		{
			Vec3 const min = glm::min(Min(), other.Min());
			Vec3 const max = glm::max(Max(), other.Max());

			return AABB{}.SetMinMax(min, max);
		}

		inline AABB AABB::MovedBy(Vec3 const& displacement) const
		{
			return AABB{ .c = c + displacement, .e = e };
		}

		inline AABB AABB::MovedTo(Vec3 const& newCenter) const
		{
			return AABB{ .c = newCenter, .e = e };
		}

		inline AABB AABB::Transformed(Mat4 const& transform) const
		{
			return Transformed(Mat3(transform), transform[3]);
		}

		inline AABB AABB::Transformed(Quat const& orientation, Vec3 const& pos) const
		{
			return Transformed(glm::toMat3(orientation), pos);
		}

		inline AABB AABB::Rotated(Mat3 const& rotation) const
		{
			AABB result{ .c = Vec3(0.0_r), .e = Vec3(0.0_r) };
			for (auto i = 0; i < 3; i++) {
				for (auto j = 0; j < 3; j++) {
					result.c[i] += rotation[j][i] * c[j];
					result.e[i] += glm::abs(rotation[j][i]) * e[j];
				}
			}
			return result;
		}

		// See Realtime Collision Detection by Ericson, Ch 4
		inline AABB AABB::Transformed(Mat3 const& rotation, Vec3 const& disp) const
		{
			AABB result{ .c = disp, .e = Vec3(0.0_r) };
			for (auto i = 0; i < 3; i++) {
				for (auto j = 0; j < 3; j++) {
					result.c[i] += rotation[j][i] * c[j];
					result.e[i] += glm::abs(rotation[j][i]) * e[j];
				}
			}
			return result;
		}

		inline Vec3 AABB::Center() const
		{
			return c;
		}

		inline Vec3 AABB::Halfwidths() const
		{
			return e;
		}


		inline Real AABB::SurfaceArea() const
		{
			Vec3 const d = 2.0_r * e;
			return 2.0_r * (d.x * d.y + d.y * d.z + d.z * d.x);
		}
	}
}