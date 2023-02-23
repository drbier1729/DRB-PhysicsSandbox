#ifndef DRB_MATH_H
#define DRB_MATH_H

namespace drb {

		inline constexpr Int32 Sign(std::floating_point auto x) { return (x < 0) ? -1 : 1; }
		inline constexpr Int32 Sign(std::integral auto x) { return (x < 0) ? -1 : 1; }

		inline constexpr Bool CompareSigns(std::floating_point auto x, std::floating_point auto y)
		{
			return (x > 0 && y > 0) || (x < 0 && y < 0);
		}

		// from Realtime Collision Detection by Ericson, Ch.11
		inline Bool EpsilonEqual(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
		{
			return glm::abs(a - b) <= std::max({ glm::abs(a), glm::abs(b), 1.0f }) * glm::sqrt(epsilon);
		}
		
		inline Bool EpsilonEqual(double a, double b, double epsilon = std::numeric_limits<double>::epsilon())
		{
			return glm::abs(a - b) <= std::max({ glm::abs(a), glm::abs(b), 1.0 }) * glm::sqrt(epsilon);
		}

		inline Bool EpsilonEqual(Vec3 const& a, Vec3 const& b, float epsilon = std::numeric_limits<float>::epsilon())
		{
			return EpsilonEqual(a.x, b.x, epsilon) &&
				   EpsilonEqual(a.y, b.y, epsilon) &&
				   EpsilonEqual(a.z, b.z, epsilon);
		}

		inline Bool EpsilonEqual(Vec4 const& a, Vec4 const& b, float epsilon = std::numeric_limits<float>::epsilon())
		{
			return EpsilonEqual(a.x, b.x, epsilon) &&
				   EpsilonEqual(a.y, b.y, epsilon) &&
				   EpsilonEqual(a.z, b.z, epsilon) &&
				   EpsilonEqual(a.w, b.w, epsilon);
		}
		
		inline Bool EpsilonEqual(Quat const& a, Quat const& b, float epsilon = std::numeric_limits<float>::epsilon())
		{
			return EpsilonEqual(a.x, b.x, epsilon) &&
				   EpsilonEqual(a.y, b.y, epsilon) &&
				   EpsilonEqual(a.z, b.z, epsilon) &&
				   EpsilonEqual(a.w, b.w, epsilon);
		}

		inline Vec3 Normalize(Vec3 const& v) {
			Vec3 const normalized = glm::normalize(v);

			if (glm::any(glm::isnan(normalized)))
			{
				ASSERT(false, "Tried to normalize a zero vector");
				return Vec4(0);
			}
			return normalized;
		}
		
		inline Vec4 Normalize(Vec4 const& v) {
			Vec4 const normalized = glm::normalize(v);

			if (glm::any(glm::isnan(normalized)))
			{
				ASSERT(false, "Tried to normalize a zero vector");
				return Vec4(0);
			}
			return normalized;
		}
		
		inline Quat Normalize(Quat const& q) {
			Quat const normalized = glm::normalize(q);
			
			if (glm::any(glm::isnan(normalized))) {
				ASSERT(false, "Tried to normalize a zero quaternion");
				return Quat(1.0f, 0.0f, 0.0f, 0.0f);
			}

			return normalized;
		}

		inline Vec3 AnyUnitOrthogonalTo(Vec3 const& src) {	
			Vec3 other{};
			if (not EpsilonEqual(src.y, 0.0f) ||
				not EpsilonEqual(src.z, 0.0f)) {
				other = Vec3(1, 0, 0);
			}
			else {
				other = Vec3(0, 1, 0);
			}
			return glm::cross(other, src);
		}
		
		// Performs a x (b x c)
		inline Vec3 TripleProduct(Vec3 const& a, Vec3 const& b, Vec3 const& c) {
			return b * glm::dot(a, c) - c * glm::dot(a, b);
		}

		inline Mat3 UnitVectorToBasis3(Vec3 const& v) {
			ASSERT(EpsilonEqual(glm::length2(v), 1.0f), "v must be a unit vector");

			Vec3 const axis1 = AnyUnitOrthogonalTo(v);
			Vec3 const axis2 = glm::cross(v, axis1);

			return Mat3{
				v,
				axis1,
				axis2
			};
		}
		
		inline Mat4 UnitVectorToBasis4(Vec3 const& v) {
			ASSERT(EpsilonEqual(glm::length2(v), 1.0f), "v must be a unit vector");

			Vec3 const axis1 = AnyUnitOrthogonalTo(v);
			Vec3 const axis2 = glm::cross(v, axis1);

			return Mat4{
				Vec4(v, 0),
				Vec4(axis1,0),
				Vec4(axis2, 0),
				Vec4(0,0,0,1)
			};
		}
}

#endif

