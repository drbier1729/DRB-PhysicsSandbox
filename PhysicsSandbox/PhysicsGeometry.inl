

namespace drb {
	namespace physics {

		template<Shape ShapeType>
		CollisionShape<ShapeType>::CollisionShape(ShapeType const& shape_, Mat4 const& transform_, Float32 mass_)
			: CollisionShapeBase{.transform = transform_, .mass = mass_, .type = ShapeType::type },
			shape { shape_ }
		{}

		template<Shape T>
		CollisionGeometry& CollisionGeometry::AddCollider(T&& shape, CollisionShapeBase&& options)
		{
			if constexpr (std::is_same_v<T, Sphere>) {
				spheres.emplace_back(
					std::forward<Sphere>(shape),
					std::forward<Mat4>(options.transform),
					std::forward<Float32>(options.mass));
			}
			else if constexpr (std::is_same_v<T, Capsule>) {
				capsules.emplace_back(
					std::forward<Capsule>(shape),
					std::forward<Mat4>(options.transform),
					std::forward<Float32>(options.mass));
			}
			else if constexpr (std::is_same_v<T, Convex>) {
				hulls.emplace_back(
					std::forward<Convex>(shape),
					std::forward<Mat4>(options.transform),
					std::forward<Float32>(options.mass));
			}
			else {
				static_assert(std::false_type<T>::value, "physics::Mesh not supported as RigidBody colliders");
			}

			return *this;
		}

		template<Shape T>
		CollisionGeometry& CollisionGeometry::AddCollider(T const& shape, CollisionShapeBase const& options)
		{
			if constexpr (std::is_same_v<T, Sphere>) {
				spheres.emplace_back(shape, options.transform, options.mass);
			}
			else if constexpr (std::is_same_v<T, Capsule>) {
				capsules.emplace_back(shape, options.transform, options.mass);
			}
			else if constexpr (std::is_same_v<T, Convex>) {
				hulls.emplace_back(shape, options.transform, options.mass);
			}
			else {
				static_assert(std::false_type<T>::value, "physics::Mesh not supported as RigidBody colliders");
			}

			return *this;
		}

		template<class Fn>
		void CollisionGeometry::ForEachCollider(Fn fn)
		{
			for (auto&& shape : spheres) {
				fn(shape);
			}
			for (auto&& shape : capsules) {
				fn(shape);
			}
			for (auto&& shape : hulls) {
				fn(shape);
			}
		}
		
		template<class Fn>
		void CollisionGeometry::ForEachCollider(Fn fn) const
		{
			for (auto&& shape : spheres) {
				fn(shape);
			}
			for (auto&& shape : capsules) {
				fn(shape);
			}
			for (auto&& shape : hulls) {
				fn(shape);
			}
		}

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
			
			for (auto&& v : hull.verts) 
			{
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
				.bounds = AABB{.max = halfwidths, .min = -halfwidths},

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

				.vertAdj = {
					0,
					1,
					3,
					5,
					9,
					11,
					20,
					15
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


		inline Convex MakeTetrahedron(Vec3 const& p0_, Vec3 const& p1_, Vec3 const& p2_, Vec3 const& p3_)
		{			
			// Centroid
			Vec3 const c = 0.25f * (p0_ + p1_ + p2_ + p3_);

			// Check if we need to swap the vertex order in order for
			// triangle p1, p2, p3 to be in counterclockwise order
			Vec3 const a = p2_ - p1_, b = p3_ - p1_;
			Bool const swap = glm::dot(glm::cross(a, b), p0_) < 0.0f;

			// Shift all points s.t. centroid is at origin, swapping order if needed
			Vec3 const p0 = p0_ - c;
			Vec3 const p1 = p1_ - c;
			Vec3 const p2 = (swap ? p3_ : p2_) - c;
			Vec3 const p3 = (swap ? p2_ : p3_) - c;

			return Convex{

				.bounds = { 
					.max = glm::max(p0, glm::max(p1, glm::max(p2, p3))),
					.min = glm::min(p0, glm::min(p1, glm::min(p2, p3))) 
				},

				.verts = { p0, p1, p2, p3},
				
				.vertAdj = {
					0,
					1,
					3,
					4
				},

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
			return AABB{ .max = center + halfwidths, .min = center - halfwidths };
		}

		inline AABB MakeAABB(Capsule const& cap, Mat4 const& tr)
		{
			AABB boundsLocal{
				.max = { cap.r, cap.h + cap.r, cap.r },
				.min = { -cap.r, -cap.h - cap.r, -cap.r }
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
	

		inline Polygon FaceAsPolygon(Convex const& hull, Mat4 const& tr, Convex::FaceID face)
		{
			ASSERT(face < hull.faces.size(), "Index out of range");
		
			Polygon poly{};

			ForEachEdgeOfFace(hull, face, [&](Convex::HalfEdge e) {
				poly.verts.emplace_back(tr * Vec4(hull.verts[e.origin], 1));
			});

			return poly;
		}


		inline void ForEachOneRingNeighbor(Convex const& hull, Convex::EdgeID start, std::invocable<Convex::HalfEdge> auto fn)
		{
			ASSERT(start < hull.edges.size(), "Index out of range");

			Convex::HalfEdge const firstNeighbor = hull.edges[hull.edges[start].twin];
			
			Convex::HalfEdge neighbor = firstNeighbor;
			do {
				// Call function
				fn( neighbor );

				// Traverse to next neighbor
				neighbor = hull.edges[hull.edges[neighbor.next].twin];

			} while (neighbor != firstNeighbor);
		}

		inline void ForEachEdgeOfFace(Convex const& hull, Convex::FaceID face, std::invocable<Convex::HalfEdge> auto fn)
		{
			ASSERT(face < hull.faces.size(), "Index out of range");

			Convex::HalfEdge const start = hull.edges[hull.faces[face].edge];

			Convex::HalfEdge e = start;
			do {
				fn( e );
				e = hull.edges[e.next];
			} while (e != start);
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

		inline Plane Transformed(Plane const& plane, Mat4 const& transform)
		{
			Vec3 const normal = Mat3(transform) * plane.n;
			return Plane{
				.n = normal,
				.d = glm::dot(Vec3(transform[3]), normal) + plane.d
			};
		}

		inline Segment Transformed(Segment const& seg, Mat4 const& transform)
		{
			return Segment{
				.b = transform * Vec4(seg.b, 1.0f),
				.e = transform * Vec4(seg.e, 1.0f)
			};
		}


		inline Segment CentralSegment(Capsule const& cap, Mat4 const& tr)
		{
			return Segment{
				.b = tr * Vec4(0, -cap.h, 0, 1),
				.e = tr * Vec4(0, cap.h, 0, 1)
			};
		}
	}
}
