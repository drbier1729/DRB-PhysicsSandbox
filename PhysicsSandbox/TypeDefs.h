#ifndef DRB_TYPEDEFS_H
#define DRB_TYPEDEFS_H

namespace drb {

	using Int8 =  std::int8_t;
	using Int16 = std::int16_t;
	using Int32 = std::int32_t;
	using Int64 = std::int64_t;
	using Uint8 = std::uint8_t;
	using Uint16 = std::uint16_t;
	using Uint32 = std::uint32_t;
	using Uint64 = std::uint64_t;
	using SizeT = std::size_t;
	using Byte = std::byte;

	using Bool = bool;

	using Float32 = float;
	using Float64 = double;
	using Real = Float32;

	using Vec2 = glm::tvec2<Real, glm::highp>;
	using Vec3 = glm::tvec3<Real, glm::highp>;
	using Vec4 = glm::tvec4<Real, glm::highp>;
	using Mat3 = glm::tmat3x3<Real, glm::highp>;
	using Mat4 = glm::tmat4x4<Real, glm::highp>;
	using Quat = glm::tquat<Real, glm::highp>;

	consteval Real operator ""_r(long double x) noexcept {
		return static_cast<Real>(x);
	}
}
#endif