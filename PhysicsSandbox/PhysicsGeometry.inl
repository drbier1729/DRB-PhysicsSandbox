

namespace drb {
	namespace physics {

		inline Mat3 ComputeInertiaTensor(Sphere const& sph) {
			return  0.4f * sph.r * sph.r * Mat3(1);
		}

		inline Mat3 ComputeInertiaTensor(Capsule const& cap) {
			// Compute inertia tensor, assuming oriented vertically
			static constexpr Float32 two_pi = 6.283f;

			Float32 const rSq = cap.r * cap.r;
			Float32 const height = 2.0f * cap.h;

			// Cylinder volume
			Float32 const cV = two_pi * cap.h * rSq;

			// Hemisphere volume
			Float32 const hsV = two_pi * rSq * cap.r / 3.0f;

			Mat3 I(0);
			I[1][1] = rSq * cV * 0.5f;
			I[0][0] = I[2][2] = I[1][1] * 0.5f + cV * height * height / 12.0f;

			Float32 const temp0 = hsV * 2.0f * rSq / 5.0f;
			I[1][1] += temp0 * 2.0f;

			Float32 const temp1 = temp0 + (hsV * height * height) / 4.0f + (3.0f * height * cap.r) / 8.0f;
			I[0][0] += temp1 * 2.0f;
			I[2][2] += temp1 * 2.0f;
		
			return I;
		}
		
		inline Mat3 ComputeInertiaTensor(Convex const& hull) {
			Mat3 I(0);
			for (auto&& v : hull.verts) {
				I[0][0] += v.y * v.y + v.z * v.z;
				I[0][1] -= v.x * v.y;
				I[0][2] -= v.x * v.z;

				I[1][1] += v.x * v.x + v.z * v.z;
				I[1][2] -= v.y * v.z;

				I[2][2] += v.x * v.x + v.y * v.y;
			}

			I[1][0] = I[0][1];
			I[2][0] = I[0][2];
			I[2][1] = I[1][2];

			return I;
		}

		inline Convex MakeBox(Vec3 const& halfwidths) {

			return Convex{
				.bounds = AABB{.min = -halfwidths, .max = halfwidths},

				.verts = {
						Vec3{ -halfwidths.x, -halfwidths.y,  halfwidths.z }, // 0
						Vec3{  halfwidths.x, -halfwidths.y,  halfwidths.z }, // 1
						Vec3{  halfwidths.x,  halfwidths.y,  halfwidths.z }, // 2
						Vec3{ -halfwidths.x,  halfwidths.y,  halfwidths.z }, // 3
						Vec3{ -halfwidths.x, -halfwidths.y, -halfwidths.z }, // 4
						Vec3{  halfwidths.x, -halfwidths.y, -halfwidths.z }, // 5
						Vec3{  halfwidths.x,  halfwidths.y, -halfwidths.z }, // 6
						Vec3{ -halfwidths.x,  halfwidths.y, -halfwidths.z }  // 7
					},

				.edges = {
						{.next = 2, .twin = 1, .origin = 0, .face = 0}, //  0
						{.next = 8, .twin = 0, .origin = 1, .face = 1}, //  1

						{.next = 4, .twin = 3, .origin = 1, .face = 0}, //  2
						{.next = 13, .twin = 2, .origin = 2, .face = 2}, //  3

						{.next = 6, .twin = 5, .origin = 2, .face = 0}, //  4
						{.next = 21, .twin = 4, .origin = 3, .face = 3}, //  5

						{.next = 0, .twin = 7, .origin = 3, .face = 0}, //  6
						{.next = 14, .twin = 6, .origin = 0, .face = 4}, //  7

						{.next = 10, .twin = 9, .origin = 0, .face = 1},  //  8
						{.next = 7, .twin = 8, .origin = 4, .face = 4},  //  9

						{.next = 12, .twin = 11, .origin = 4, .face = 1},  // 10
						{.next = 17, .twin = 10, .origin = 5, .face = 5},  // 11

						{.next = 1, .twin = 13, .origin = 5, .face = 1},  // 12
						{.next = 18, .twin = 12, .origin = 1, .face = 2},  // 13

						{.next = 16, .twin = 15, .origin = 3, .face = 4},  // 14
						{.next = 5, .twin = 14, .origin = 7, .face = 3},  // 15

						{.next = 9, .twin = 17, .origin = 7, .face = 4},  // 16
						{.next = 23, .twin = 16, .origin = 4, .face = 5},  // 17

						{.next = 20, .twin = 19, .origin = 5, .face = 2},  // 18
						{.next = 11, .twin = 18, .origin = 6, .face = 5},  // 19

						{.next = 3, .twin = 21, .origin = 6, .face = 2},  // 20
						{.next = 22, .twin = 20, .origin = 2, .face = 3},  // 21

						{.next = 15, .twin = 23, .origin = 6, .face = 3},  // 22
						{.next = 19, .twin = 22, .origin = 7, .face = 5}   // 23   
					},

				.faces = {
					{.edge = 0,  .plane = {.n = Vec3(0,  0,  1), .d = halfwidths.z }},  // Front 0
					{.edge = 1,  .plane = {.n = Vec3(0, -1, 0),  .d = halfwidths.y }},  // Bottom 
					{.edge = 3,  .plane = {.n = Vec3(1,  0,  0), .d = halfwidths.x }},  // Right 2
					{.edge = 5,  .plane = {.n = Vec3(0,  1,  0), .d = halfwidths.y }},  // Top 3
					{.edge = 7,  .plane = {.n = Vec3(-1, 0,  0), .d = halfwidths.x }},  // Left 4
					{.edge = 11, .plane = {.n = Vec3(0,  0, -1), .d = halfwidths.z }},  // Back 5
				}
			};
		}


		inline Convex MakeTetrahedron(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2_, Vec3 const& p3_)
		{
			// Switch references s.t. triangle p1, p2, p3 is in counterclockwise order
			Vec3 const a = p2_ - p1, b = p3_ - p1;
			Bool const swap = glm::dot(glm::cross(a, b), p0) < 0.0f;
			Vec3 const& p2 = swap ? p3_ : p2_;
			Vec3 const& p3 = swap ? p2_ : p3_;

			return Convex{

				.bounds = { 
					.min = glm::min(p0, glm::min(p1, glm::min(p2, p3))), 
					.max = glm::max(p0, glm::max(p1, glm::max(p2, p3))) 
				},
				
				.verts = { p0, p1, p2, p3 },
				
				.edges = {
					{.next = 2, .twin = 1, .origin = 0, .face = 0}, //  0
					{.next = 9, .twin = 0, .origin = 1, .face = 2}, //  1
				
					{.next = 4, .twin = 3, .origin = 1, .face = 0}, //  2
					{.next = 11, .twin = 2, .origin = 2, .face = 3}, //  3

					{.next = 0, .twin = 5, .origin = 2, .face = 0}, //  4
					{.next = 6, .twin = 4, .origin = 0, .face = 1}, //  5
				
					{.next = 8, .twin = 7, .origin = 2, .face = 1}, //  6
					{.next = 3, .twin = 6, .origin = 3, .face = 3}, //  7

					{.next = 5, .twin = 9, .origin = 3, .face = 1}, //  8
					{.next = 10, .twin = 8, .origin = 0, .face = 2}, //  9

					{.next = 1, .twin = 11, .origin = 3, .face = 2}, //  10
					{.next = 7, .twin = 10, .origin = 1, .face = 3} //  11
				},

				.faces = {
					{.edge = 0, .plane = MakePlane(p0, p1, p2) },
					{.edge = 5, .plane = MakePlane(p0, p2, p3) },
					{.edge = 9, .plane = MakePlane(p0, p3, p1) },
					{.edge = 3, .plane = MakePlane(p3, p2, p1) },
				}
			};
		}

		inline AABB MakeAABB(Sphere const& sph, Mat4 const& tr)
		{
			Vec3 const center = tr[3];
			Vec3 const halfwidths = Vec3(sph.r);
			return AABB{ .min = center - halfwidths, .max = center + halfwidths };
		}

		inline AABB MakeAABB(Capsule const& cap, Mat4 const& tr)
		{
			AABB boundsLocal{
				.min = { -cap.r, -cap.h - cap.r, -cap.r },
				.max = { cap.r, cap.h + cap.r, cap.r }
			};

			Mat3 const rot = Mat3(tr);
			Vec3 const pos = tr[3];

			return boundsLocal.Transformed(rot, pos);
		}

		inline AABB MakeAABB(Convex const& cvx, Mat4 const& tr)
		{
			Mat3 const rot = Mat3(tr);
			Vec3 const pos = tr[3];

			return cvx.bounds.Transformed(rot, pos);
		}
	
		inline Plane MakePlane(Vec3 const& normal, Vec3 const& point) 
		{
			return Plane{ .n = normal, .d = glm::dot(normal, point) };
		}

		inline Plane MakePlane(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2)
		{
			Vec3 const a = Normalize(p1 - p0), b = Normalize(p2 - p0);

			Vec3 const n = glm::cross(a, b);

			return MakePlane(n, p0);
		}

		inline Float32 SignedDistance(Vec3 const& point, Plane const& plane) 
		{
			return glm::dot(point, plane.n) - plane.d;
		}

		inline Plane GetTransformed(Plane const& plane, Mat4 const& transform)
		{
			Vec3 const normal = Mat3(transform) * plane.n;
			return Plane{
				.n = normal,
				.d = glm::dot(Vec3(transform[3]), normal) + plane.d
			};
		}

		inline Segment GetCentralSegment(Capsule const& cap, Mat4 const& tr)
		{
			return Segment{
				.b = tr * Vec4(0, -cap.h, 0, 1),
				.e = tr * Vec4(0, cap.h, 0, 1)
			};
		}

		// See Ericson Ch. 5
		inline Ray::CastResult RayCast(Ray const& r, AABB const& aabb)
		{
			Float32 tmin = 0.0f;
			Float32 tmax = std::numeric_limits<Float32>::max();

			// For all three slabs
			for (Uint32 i = 0u; i < 3u; ++i) 
			{
				if (EpsilonEqual(r.d[i], 0.0f)) 
				{
					// Ray is parallel to slab. No hit if origin not within slab
					if (r.p[i] < aabb.min[i] || r.p[i] > aabb.max[i]) {	return Ray::CastResult{}; }
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
					if (tmin > tmax) { return Ray::CastResult{}; }
				}
			}
		
			// Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin)
			return Ray::CastResult{ 
				.point = r.p + r.d * tmin, 
				.distance = tmin,
				.hit = true
			};
		}
	}
}
