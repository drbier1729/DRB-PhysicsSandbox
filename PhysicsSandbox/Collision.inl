namespace drb {
	namespace Collision {

		// Helpers
		namespace detail {
			inline Vec3 GetAnyUnitOrthogonalTo(Vec3 const& src) {
				Vec3 other;
				if (glm::epsilonNotEqual(src.y, 0.0f, 0.0001f) ||
					glm::epsilonNotEqual(src.z, 0.0f, 0.0001f)) {
					other = Vec3(1, 0, 0);
				}
				else {
					other = Vec3(0, 1, 0);
				}
				return glm::normalize(glm::cross(other, src));
			}

			inline Mat4 BasisFromUnitVector(Vec3 const& v) {
				ASSERT(glm::epsilonEqual(glm::length2(v), 1.0f, 0.001f), "v must be a unit vector");

				Vec3 const axis1 = detail::GetAnyUnitOrthogonalTo(v);
				Vec3 const axis2 = glm::cross(v, axis1);

				return Mat4{
					Vec4(v,0),
					Vec4(axis1,0),
					Vec4(axis2,0),
					Vec4(0,0,0,1)
				};
			}
		}


		// Primitive methods
		inline LineSegment LineSegment::Transform(Mat4 const& transform) const {
			return LineSegment{
				.start = transform * Vec4(start, 1),
				.end = transform * Vec4(end, 1)
			};
		}

		inline Vec3 LineSegment::Midpoint() const {
			return 0.5f * (start + end);
		}

		inline AABB AABB::Transform(Mat4 const& transform) const {
			return AABB{
				.position = transform * Vec4(position, 1),
				.halfwidths = halfwidths
			};
		}

		inline Bool AABB::Intersect(AABB const& other) const {
			if (std::abs(position.x - other.position.x) > (halfwidths.x + other.halfwidths.x)) { return false; }
			if (std::abs(position.y - other.position.y) > (halfwidths.y + other.halfwidths.y)) { return false; }
			if (std::abs(position.z - other.position.z) > (halfwidths.z + other.halfwidths.z)) { return false; }
			return true;
		}

		inline ContactManifold AABB::Collide(AABB const& other) const {
			static constexpr Float32 y_bias = 0.01f;


			if (glm::all(glm::epsilonEqual(position, other.position, 0.001f))) {
				return ContactManifold{};
			}

			// Find penetration on all axes
			Vec3 pen = (halfwidths + other.halfwidths) - glm::abs(position - other.position);
			pen.y = std::max(pen.y - y_bias, 0.0f); // prioritize y-axis since ground collisions are most common

			// If there is no penetration on any axis, we have no contacts
			if (glm::any(glm::lessThan(pen, Vec3(0.0f)))) { return ContactManifold{}; }

			// Find the least penetration
			Float32 const min_pen = std::min({ pen.x, pen.y, pen.z });

			// Find axis of least penetration and contact point (could find 4, but one will do since AABBs don't rotate)
			Vec3 contact{};
			Vec3 normal{};
			if (min_pen == pen.y) {
				Float32 const sign = position.y < other.position.y ? 1.0f : -1.0f;
				normal = sign * Vec3(0, 1, 0);
				contact = Vec3{
					std::min(position.x + halfwidths.x, other.position.x + other.halfwidths.x),
					sign * (halfwidths.y - 0.5f * min_pen) + position.y,
					std::min(position.z + halfwidths.z, other.position.z + other.halfwidths.z)
				};
			}
			else if (min_pen == pen.x) {
				Float32 const sign = position.x < other.position.x ? 1.0f : -1.0f;
				normal = sign * Vec3(1, 0, 0);
				contact = Vec3{
					sign * (halfwidths.x - 0.5f * min_pen) + position.x,
					std::min(position.y + halfwidths.y, other.position.y + other.halfwidths.y),
					std::min(position.z + halfwidths.z, other.position.z + other.halfwidths.z),
				};
			}
			else {
				Float32 const sign = position.z < other.position.z ? 1.0f : -1.0f;
				normal = sign * Vec3(0, 0, 1);
				contact = Vec3{
					std::min(position.x + halfwidths.x, other.position.x + other.halfwidths.x),
					std::min(position.y + halfwidths.y, other.position.y + other.halfwidths.y),
					sign * (halfwidths.z - 0.5f * min_pen) + position.z
				};
			}

			ContactManifold result{ .normal = normal };
			result.contacts[result.num_contacts++] = {
				.position = contact,
				.penetration = min_pen
			};

			return result;
		}

		inline Plane Plane::Transform(Mat4 const& transform) const {
			Plane p{};
			p.normal = glm::normalize(Mat3(transform) * normal);
			p.d = glm::dot(Vec3(transform[3]), p.normal) + d;

			ASSERT(not glm::any(glm::isnan(p.normal)), "Bad transform matrix.");
			return p;
		}


		// Collider methods
		inline void Collider::UpdateWorldTransformFromBody(Mat4 const& body_tr) {
			local_to_world = body_tr * local_to_body;
		}

		inline Collider::Type Collider::GetType() const {
			return type;
		}

		inline Uint32 Collider::GetTypeIdx() const {
			return static_cast<Uint32>(type) - 1u;
		} // intentionally overflows NONE

		inline Mat3 Collider::GetInertiaTensorInSpace(Vec3 const& tran, Mat3 const& rot) const {
			// Rotated inertia tensor
			Mat3 I = rot * inertia_local * glm::transpose(rot);

			// Using Parallel Axis Theorem for translated inertia tensor
			I += (glm::dot(tran, tran) * Mat3(1) - glm::outerProduct(tran, tran));

			return I;
		}

		inline Vec3 Collider::LocalToBody(Vec3 const& pt) const {
			return Vec3(local_to_body * Vec4(pt, 1));
		}

		inline Vec3 Collider::LocalToBodyVec(Vec3 const& vec) const {
			return Mat3(local_to_body) * vec;
		}

		inline Vec3 Collider::BodyToLocal(Vec3 const& pt) const {
			return Vec3(glm::inverse(local_to_body) * Vec4(pt, 1));
		}

		inline Vec3 Collider::BodyToLocalVec(Vec3 const& vec) const {
			return glm::inverse(Mat3(local_to_body)) * vec;
		}

		inline Mat4 Collider::GetLocalToBodyTransform() const {
			return local_to_body;
		}

		inline Mat4 Collider::GetRelativeRotationMat4() const {
			Mat4 r = local_to_body;
			r[3] = Vec4(0, 0, 0, 1);
			return r;
		}

		inline Mat3 Collider::GetRelativeRotationMat3() const {
			return Mat3(local_to_body);
		}

		inline Vec3 Collider::GetRelativePosition() const {
			return Vec3(local_to_body[3]);
		}

		inline Mat3 Collider::GetInertiaTensorRelative() const {
			return GetInertiaTensorInSpace(GetRelativePosition(), GetRelativeRotationMat3());
		}

		inline Vec3 Collider::LocalToWorld(Vec3 const& pt) const {
			return Vec3(local_to_world * Vec4(pt, 1));
		}

		inline Vec3 Collider::LocalToWorldVec(Vec3 const& vec) const {
			return Mat3(local_to_world) * vec;
		}

		inline Vec3 Collider::WorldToLocal(Vec3 const& pt) const {
			return glm::inverse(local_to_world) * Vec4(pt, 1);
		}

		inline Vec3 Collider::WorldToLocalVec(Vec3 const& vec) const {
			return glm::inverse(Mat3(local_to_world)) * vec;
		}

		inline Mat4 Collider::GetLocalToWorldTransform() const {
			return local_to_world;
		}

		inline Mat3 Collider::GetWorldRotationMat3() const {
			return Mat3(local_to_world);
		}

		inline Mat4 Collider::GetWorldRotationMat4() const {
			Mat4 r = local_to_world;
			r[3] = Vec4(0, 0, 0, 1);
			return r;
		}

		inline Vec3 Collider::GetWorldPosition() const {
			return local_to_world[3];

		}

		inline Mat3 Collider::GetInertiaTensorWorldRotation() const {
			Mat3 const rot = Mat3(local_to_world);
			return rot * inertia_local * glm::transpose(rot);
		}

		inline void Collider::SetLocalToBodyTransform(Mat4 const& tr) {
			local_to_body = tr;
		}

		inline void Collider::SetRelativeRotation(Mat3 const& rot) {
			local_to_body[0] = Vec4(rot[0], 0);
			local_to_body[1] = Vec4(rot[1], 0);
			local_to_body[2] = Vec4(rot[2], 0);
		}

		inline void Collider::SetRelativePosition(Vec3 const& pos) {
			local_to_body[3] = Vec4(pos, 1);
		}


		// Sphere methods
		inline void Sphere::ComputeInertiaTensor() {
			inertia_local = 0.4f * radius * radius * Mat3(1);
		}

		inline Float32 Sphere::GetRadius() const {
			return radius;
		}

		inline void Sphere::SetRadius(Float32 r) {
			radius = r;
			ComputeInertiaTensor();
		}


		// Capsule methods
		inline LineSegment Capsule::GetCentralSegment() const {
			LineSegment ls{ .start = Vec3(0, -seg_halflength, 0), .end = Vec3(0, seg_halflength, 0) };
			return ls.Transform(GetLocalToWorldTransform());
		}

		inline void Capsule::ComputeInertiaTensor() {
			// Compute inertia tensor, assuming oriented vertically
			static constexpr Float32 two_pi = 6.283f;

			Float32 const rSq = radius * radius;
			Float32 const height = 2.0f * seg_halflength;

			// Cylinder volume
			Float32 const cV = two_pi * seg_halflength * rSq;

			// Hemisphere volume
			Float32 const hsV = two_pi * rSq * radius / 3.0f;

			Mat3 I(0);
			I[1][1] = rSq * cV * 0.5f;
			I[0][0] = I[2][2] = I[1][1] * 0.5f + cV * height * height / 12.0f;

			Float32 const temp0 = hsV * 2.0f * rSq / 5.0f;
			I[1][1] += temp0 * 2.0f;

			Float32 const temp1 = height * 0.5f;
			Float32 const temp2 = temp0 + hsV * temp1 * temp1 + 3.0f * height * radius / 8.0f;
			I[0][0] += temp2 * 2.0f;
			I[2][2] += temp2 * 2.0f;
			I[0][1] = I[0][2] = I[1][0] = I[1][2] = I[2][0] = I[2][1] = 0.0f;

			inertia_local = I;
		}

		inline Float32 Capsule::GetLength() const {
			return 2.0f * seg_halflength;
		}

		inline Float32 Capsule::GetRadius() const {
			return radius;
		}

		inline void Capsule::SetLength(Float32 len) {
			seg_halflength = 0.5f * len;
			ComputeInertiaTensor();
		}

		inline void Capsule::SetRadius(Float32 r) {
			radius = r;
			ComputeInertiaTensor();
		}


		// Hull methods
		inline Vec3 Hull::GetSupport(Vec3 const& directionL) const {
			Int32   max_index = -1;
			Float32 max_projection = std::numeric_limits<Float32>::lowest();

			Int32 const vert_count = static_cast<Int32>(vertices.size());

			for (Int32 i = 0; i < vert_count; ++i)
			{
				Float32 const projection = glm::dot(directionL, vertices[i]);
				if (projection > max_projection)
				{
					max_index = i;
					max_projection = projection;
				}
			}
			return vertices[max_index];
		}

		inline void Hull::Scale(Vec3 const& scale) {
			Scale(scale.x, scale.y, scale.z);
			ComputeInertiaTensor();
		}

		inline void Hull::Scale(Float32 x, Float32 y, Float32 z) {
			ASSERT(x > 0.001f && y > 0.001f && z > 0.001f, "Scale must be positive.");

			// Scale vertices
			for (auto&& v : vertices) {
				v.x *= x;
				v.y *= y;
				v.z *= z;
			}

			// Scale bounding box
			bounds.x *= x;
			bounds.y *= y;
			bounds.z *= z;

			// Scale face planes
			for (auto&& p : planes) {
				p.d = std::abs(x * p.normal.x + y * p.normal.y + z * p.normal.z);
			}
			ComputeInertiaTensor();
		}
	}
}