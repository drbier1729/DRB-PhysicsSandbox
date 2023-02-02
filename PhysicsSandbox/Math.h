#ifndef DRB_MATH_H
#define DRB_MATH_H

namespace drb {

		inline constexpr Int32 Sign(std::floating_point auto x) { return (x < 0) ? -1 : 1; }
		inline constexpr Int32 Sign(std::integral auto x) { return (x < 0) ? -1 : 1; }

		// from Realtime Collision Detection by Ericson, Ch.11
		inline Bool EpsilonEqual(float a, float b)
		{
			static constexpr float epsilon = std::numeric_limits<float>::epsilon();

			return std::fabs(a - b) <= std::max({ std::fabs(a), std::fabs(b), 1.0f }) * std::sqrtf(epsilon);
		}
		
		inline Bool EpsilonEqual(double a, double b)
		{
			static constexpr double epsilon = std::numeric_limits<double>::epsilon();

			return std::fabs(a - b) <= std::max({ std::fabs(a), std::fabs(b), 1.0 }) * std::sqrt(epsilon);
		}

		inline Bool EpsilonEqual(Vec3 const& a, Vec3 const& b)
		{
			return EpsilonEqual(a.x, b.x) &&
				EpsilonEqual(a.y, b.y) &&
				EpsilonEqual(a.z, b.z);
		}

		inline Bool EpsilonEqual(Vec4 const& a, Vec4 const& b)
		{
			return EpsilonEqual(a.x, b.x) &&
				EpsilonEqual(a.y, b.y) &&
				EpsilonEqual(a.z, b.z) &&
				EpsilonEqual(a.w, b.w);
		}
		
		inline Bool EpsilonEqual(Quat const& a, Quat const& b)
		{
			return EpsilonEqual(a.x, b.x) &&
				EpsilonEqual(a.y, b.y) &&
				EpsilonEqual(a.z, b.z) &&
				EpsilonEqual(a.w, b.w);
		}

		inline Vec3 Normalize(Vec3 const& v) {
			if (EpsilonEqual(v, Vec3(0))) {
				ASSERT(false, "Tried to normalize a zero vector.");
				return Vec3(0);
			}
			return glm::normalize(v);
		}
		
		inline Vec4 Normalize(Vec4 const& v) {
			if (EpsilonEqual(v, Vec4(0))) {
				ASSERT(false, "Tried to normalize a zero vector.");
				return Vec4(0);
			}
			return glm::normalize(v);
		}
		
		inline Quat Normalize(Quat const& q) {
			Quat const normalized = glm::normalize(q);
			
			if (glm::any(glm::isnan(normalized))) {
				ASSERT(false, "Tried to normalize a zero quaternion");
				return Quat(1.0f, 0.0f, 0.0f, 0.0f);
			}

			return normalized;
		}
}

#endif

