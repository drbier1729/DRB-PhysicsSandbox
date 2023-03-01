#include "pch.h"
#include "PhysicsGeometry.h"

#include "StackAllocator.h"
#include "PhysicsGeometryQueries.h"

namespace drb {
	namespace physics {

		Float32 Plane::thickness = 0.1f;

		namespace util {
			// This is useful bc CollisionShapes will often/always be fixed to a RigidBody and the
			// origin (in the local space of the RigidBody) will be the RigidBody's center of mass.
			template<Shape T>
			static Mat3 InertiaTensorAboutOrigin(CollisionShape<T> const& s)
			{
				// Rotate the body-space inertia tensor
				Mat3 const rot = Mat3(s.transform);
				Mat3 I = rot * ComputeInertiaTensor(s.shape) * glm::transpose(rot);

				// Use Parallel Axis Theorem to translate the inertia tensor
				Vec3 const disp = -s.transform[3]; // displacement vector from COM to origin
				I += glm::dot(disp, disp) * Mat3(1) - glm::outerProduct(disp, disp);

				// Assuming uniform density, total mass will factor out
				return s.mass * I;
			}
		}


		// See Ericson 8.3.4 (Sutherland-Hodgman clipping with fat planes)
		// TODO : verify that this maintains ordering of original poly (i.e. counterclockwise)
		void SplitPolygon(Polygon const& poly, Plane const& plane, Polygon& front, Polygon& back)
		{
			if (poly.verts.empty()) { return; }

			// Test all edges (a, b) starting with edge from last to first vertex
			auto const numVerts = poly.verts.size();

			Vec3 a = poly.verts.back();
			auto aSide = ClassifyPointToPlane(a, plane);

			// Loop over all edges given by vertex pair (n-1, n)
			for (auto n = 0ull; n < numVerts; n++)
			{
				Vec3 const b = poly.verts[n];
				auto const bSide = ClassifyPointToPlane(b, plane);

				if (bSide == Side::Front)
				{
					if (aSide == Side::Back)
					{
						// Edge (a, b) straddles, output intersection point to both sides
						// Consistently clip edge as ordered going from in front -> back
						Vec3 i{};
						Float32 t{};
						Intersect(Segment{ .b = b, .e = a }, plane, t, i);

						ASSERT(ClassifyPointToPlane(i, plane) == Side::On, "Intersection point must be on plane");

						front.verts.push_back(i);
						back.verts.push_back(i);
					}
					// In all three cases, output b to the front side
					front.verts.push_back(b);
				}
				else if (bSide == Side::Back)
				{
					if (aSide == Side::Front)
					{
						// Edge (a, b) straddles plane, output intersection point
						Vec3 i{};
						Float32 t{};
						Intersect(Segment{ .b = a, .e = b }, plane, t, i);

						ASSERT(ClassifyPointToPlane(i, plane) == Side::On, "Intersection point must be on plane");

						front.verts.push_back(i);
						back.verts.push_back(i);
					}
					else if (aSide == Side::On)
					{
						// Output a when edge (a, b) goes from ‘on’ to ‘behind’ plane
						back.verts.push_back(a);
					}
					// In all three cases, output b to the back side
					back.verts.push_back(b);
				}
				else
				{
					// b is on the plane. In all three cases output b to the front side
					front.verts.push_back(b);

					// In one case, also output b to back side
					if (aSide == Side::Back)
					{
						back.verts.push_back(b);
					}
				}

				// Keep b as the starting point of the next edge
				a = b;
				aSide = bSide;
			}
		}


		void Convex2::AllocateData(VertID numVerts, EdgeID numEdges, FaceID numFaces)
		{
			ASSERT(0 < numVerts, "Number of verts invalid");
			ASSERT(0 < numEdges, "Number of edges invalid");
			ASSERT(0 < numFaces, "Number of faces invalid");
			ASSERT(numVerts + numFaces == 2ull + numEdges / 2, "Invalid Euler characteristic");

			// Maximum padding that might be required for alignement within block storage
			static auto constexpr pad = alignof(Vec3) + alignof(EdgeID) + alignof(HalfEdge) + alignof(Face);

			// Raw sizes of each type going into the arrays
			static auto constexpr hSize = sizeof(Header);
			auto const vSize = numVerts * sizeof(Vec3);
			auto const vaSize = numVerts * sizeof(EdgeID);
			auto const eSize = numEdges * sizeof(HalfEdge);
			auto const fSize = numFaces * sizeof(Face);

			// Total capacity required (without accounding for alignment)
			auto const cap = hSize + vSize + vaSize + eSize + fSize;

			// Allocate a block for storage. Let it be owned by
			// a StackAllocator for now.
			StackAllocator a{ std::malloc(cap + pad), cap + pad };

			// Linearly allocate from our block storage for each array
			Header* h          = a.Construct<Header>();
			std::byte* verts   = static_cast<std::byte*>(a.Alloc(vSize, alignof(Vec3)));
			std::byte* vertAdj = static_cast<std::byte*>(a.Alloc(vaSize, alignof(EdgeID)));
			std::byte* edges   = static_cast<std::byte*>(a.Alloc(eSize, alignof(HalfEdge)));
			std::byte* faces   = static_cast<std::byte*>(a.Alloc(fSize, alignof(Face)));

			// Make sure we were successful with ALL allocations
			if (not (h && verts && vertAdj && edges && faces))
			{
				// Memory freed by ~StackAllocator()
				return;
			}

			// At this point we can transfer ownership of the block from 
			// the StackAllocator to this object. We keep the Arena (simple
			// POD with a pointer, size, and capacity) around for the next
			// computations.
			data = h;
			Arena arena = a.Release();

			// Set up offsets into block storage
			data->vertsOffset = static_cast<Int16>(verts - arena.mem);
			data->vertAdjOffset = static_cast<Int16>(vertAdj - arena.mem);
			data->numVerts = numVerts;
			data->edgesOffset = static_cast<Int16>(edges - arena.mem);
			data->numEdges = numEdges;
			data->facesOffset = static_cast<Int16>(faces - arena.mem);
			data->numFaces = numFaces;
			
			// Record how much memory we actually needed from the block storage
			// (this allows copies of this object to allocate only what they
			// need, and it's nice to have, and it makes sizeof(Header) == 16).
			data->memUsed = static_cast<Int16>( arena.size );
		}

		Convex2::Convex2(Vec3 const* verts, EdgeID const* vertAdj, VertID numVerts, 
						 HalfEdge const* edges,                    EdgeID numHalfEdges, 
						 Face const* faces,                        FaceID numFaces)
			
			: data{ nullptr },
			centroid{},
			orientation{},
			bounds{}
		{
			AllocateData(numVerts, numHalfEdges, numFaces);
			if (not data) { throw std::bad_alloc{}; }

			std::memcpy(GetRawVerts(),    verts,      numVerts * sizeof(Vec3));
			std::memcpy(GetRawVertAdjs(), vertAdj,    numVerts * sizeof(EdgeID));
			std::memcpy(GetRawEdges(),    edges,      numHalfEdges * sizeof(HalfEdge));
			std::memcpy(GetRawFaces(),    faces,      numFaces * sizeof(Face));
		}

		Convex2::~Convex2() noexcept
		{
			std::free(data);
		}

		Convex2::Convex2(Convex2 const& src)
			: data{nullptr}, 
			centroid{src.centroid}, 
			orientation{src.orientation}, 
			bounds{src.bounds}
		{
			if (src.data)
			{
				void* memory = (Header*)std::malloc(src.data->memUsed);
				if (not memory) { throw std::bad_alloc{}; }

				std::memcpy(memory, src.data, src.data->memUsed);
				data = (Header*)memory;
			}
		}

		Convex2::Convex2(Convex2&& src) noexcept
			: data{ src.data },
			centroid{ src.centroid },
			orientation{ src.orientation },
			bounds{ src.bounds }
		{
			src.data = nullptr;
		}

		Convex2& Convex2::operator=(Convex2 const& other)
		{
			if (this == &other) { return *this; }

			std::free(data);
			if (other.data)
			{
				Header* newData = static_cast<Header*>( std::malloc(other.data->memUsed) );
				if (not newData) { throw std::bad_alloc{}; }

				std::memcpy(newData, other.data, other.data->memUsed);
				data = newData;
			}
			else
			{
				data = nullptr;
			}

			centroid = other.centroid;
			orientation = other.orientation;
			bounds = other.bounds;

			return *this;
		}
		Convex2& Convex2::operator=(Convex2&& other) noexcept
		{
			if (this == &other) { return *this; }
			
			std::free(data);
			data = other.data;
			centroid = other.centroid;
			orientation = other.orientation;
			bounds = other.bounds;

			other.data = nullptr;

			return *this;
		}

		Convex2  Convex2::MakeBox(Vec3 const& halfwidths)
		{
			Vec3 const verts[] = {
						Vec3{ -halfwidths.x, -halfwidths.y,  halfwidths.z }, // 0
						Vec3{  halfwidths.x, -halfwidths.y,  halfwidths.z }, // 1
						Vec3{  halfwidths.x,  halfwidths.y,  halfwidths.z }, // 2
						Vec3{ -halfwidths.x,  halfwidths.y,  halfwidths.z }, // 3
						Vec3{ -halfwidths.x, -halfwidths.y, -halfwidths.z }, // 4
						Vec3{  halfwidths.x, -halfwidths.y, -halfwidths.z }, // 5
						Vec3{  halfwidths.x,  halfwidths.y, -halfwidths.z }, // 6
						Vec3{ -halfwidths.x,  halfwidths.y, -halfwidths.z }  // 7
			};
			
			static constexpr EdgeID vertAdj[] = { 0, 1,	3, 5, 9, 11, 20, 15	};

			static constexpr HalfEdge halfEdges[] = { 
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
			};

			Face const faces[] = {
					{.edge = 0,  .plane = {.n = Vec3(0,  0,  1), .d = halfwidths.z }},  // Front 0
					{.edge = 1,  .plane = {.n = Vec3(0, -1, 0),  .d = halfwidths.y }},  // Bottom 
					{.edge = 3,  .plane = {.n = Vec3(1,  0,  0), .d = halfwidths.x }},  // Right 2
					{.edge = 5,  .plane = {.n = Vec3(0,  1,  0), .d = halfwidths.y }},  // Top 3
					{.edge = 7,  .plane = {.n = Vec3(-1, 0,  0), .d = halfwidths.x }},  // Left 4
					{.edge = 11, .plane = {.n = Vec3(0,  0, -1), .d = halfwidths.z }},  // Back 5
			};

			Convex2 box{ verts, vertAdj, 8, halfEdges, 24, faces, 6 };
			box.bounds = { .max = halfwidths, .min = -halfwidths };

			return box;
		}

		Convex2  Convex2::MakeTetrahedron(Vec3 const& p0_, Vec3 const& p1_, Vec3 const& p2_, Vec3 const& p3_)
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

			Vec3 const verts[] = { p0, p1, p2, p3 };

			static constexpr EdgeID vertAdj[] = { 0, 1, 3, 4 };

			static constexpr HalfEdge halfEdges[] = {
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
			};

			Face const faces[] = {
					{.edge = 0, .plane = MakePlane(p0, p1, p2) },
					{.edge = 5, .plane = MakePlane(p0, p2, p3) },
					{.edge = 9, .plane = MakePlane(p0, p3, p1) },
					{.edge = 3, .plane = MakePlane(p3, p2, p1) } 
			};

			Convex2 tet{ verts, vertAdj, 4, halfEdges, 12, faces, 4 };
			tet.bounds = {
					.max = glm::max(p0, glm::max(p1, glm::max(p2, p3))),
					.min = glm::min(p0, glm::min(p1, glm::min(p2, p3)))
			};

			return tet;
		}
		

		AABB MakeAABB(Mesh const& sph, Mat4 const& tr)
		{
			ASSERT(false, "Not implemented");
			return AABB{};
		}

		void CollisionGeometry::Bake()
		{
			locked = true;

			Float32 mass = 0.0f;

			// Compute mass and center of mass
			for (auto&& s : spheres) {
				mass += s.mass;
				com += mass * Vec3(s.transform[3]);
			}
			for (auto&& c : capsules) {
				mass += c.mass;
				com += mass * Vec3(c.transform[3]);
			}
			for (auto&& h : hulls) {
				mass += h.mass;
				com += mass * Vec3(h.transform[3]);
			}
			ASSERT(mass > 0.0f, "Total mass must be > 0.");
			invMass = 1.0f / mass;
			com *= invMass;

			// Shift all collider transforms s.t. COM is at origin
			for (auto&& s : spheres) {
				s.transform[3] -= Vec4(com, 0.0f);
			}
			for (auto&& c : capsules) {
				c.transform[3] -= Vec4(com, 0.0f);
			}
			for (auto&& h : hulls) {
				h.transform[3] -= Vec4(com, 0.0f);
			}

			// Compute local inertia tensor
			Mat3 I{ 0.0f };
			for (auto&& s : spheres) {
				I += util::InertiaTensorAboutOrigin(s);
			}
			for (auto&& c : capsules) {
				I += util::InertiaTensorAboutOrigin(c);
			}
			for (auto&& h : hulls) {
				I += util::InertiaTensorAboutOrigin(h);
			}
			invInertia = glm::inverse(I);

			// Compute bounds
			for (auto&& s : spheres) {
				bounds = bounds.Union( MakeAABB(s.shape, s.transform) );
			}
			for (auto&& c : capsules) {
				bounds = bounds.Union(MakeAABB(c.shape, c.transform));
			}
			for (auto&& h : hulls) {
				bounds = bounds.Union(MakeAABB(h.shape, h.transform));
			}
		}			
	}
}
