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

	using Bool = bool;

	using Float32 = float;
	using Float64 = double;

	using Vec2 = glm::vec2;
	using Vec3 = glm::vec3;
	using Vec4 = glm::vec4;
	using Mat3 = glm::mat3;
	using Mat4 = glm::mat4;
	using Quat = glm::quat;

#define ASSERT(exp, msg) assert((exp) && (msg))
}

#endif