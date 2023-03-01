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


		Convex2::Header* Convex2::AllocateData(SizeT numVerts, SizeT numEdges, SizeT numFaces)
		{
			ASSERT(0 < numVerts && numVerts <= MAX_INDEX, "Number of verts out of range");
			ASSERT(0 < numEdges && numEdges <= MAX_INDEX, "Number of edges out of range");
			ASSERT(0 < numFaces && numFaces <= MAX_INDEX, "Number of faces out of range");
			ASSERT(numVerts + numFaces == 2ull + numEdges, "Invalid Euler characteristic");

			auto constexpr hSize = sizeof(Header);
			auto const     vSize = numVerts * sizeof(Vec3);
			auto const     vaSize = numVerts * sizeof(EdgeID);
			auto const     eSize = numEdges * sizeof(HalfEdge);
			auto const     fSize = numFaces * sizeof(Face);
			auto const     cap = hSize + vSize + vaSize + eSize + fSize;
			auto const     pad = alignof(Vec3) + alignof(EdgeID) + alignof(HalfEdge) + alignof(Face);

			void* memory = std::malloc(cap + pad);
			if (not memory)
			{
				return nullptr;
			}
			std::memset(memory, 0, cap);

			StackAllocator a{ memory, cap };

			Header* h = (Header*)a.Alloc(hSize, alignof(Header));
			h->verts = std::span{ (Vec3*)a.Alloc(vSize, alignof(Vec3)),         numVerts };
			h->vertAdj = std::span{ (EdgeID*)a.Alloc(vaSize, alignof(EdgeID)),  numVerts };
			h->edges = std::span{ (HalfEdge*)a.Alloc(eSize, alignof(HalfEdge)), numEdges };
			h->faces = std::span{ (Face*)a.Alloc(fSize, alignof(Face)),         numFaces };
			h->memUsed = a.Release().size;

			if (not h || 
				h->verts.empty() ||
				h->vertAdj.empty() ||
				h->edges.empty() ||
				h->faces.empty())
			{
				std::free(memory);
				return nullptr;
			}

			return h;
		}

		Convex2::Convex2(SizeT numVerts, SizeT numEdges, SizeT numFaces)
			: data{ AllocateData(numVerts, numEdges, numFaces) }, 
			centroid{}, 
			orientation{}, 
			bounds {}
		{
			if (not data) { throw std::bad_alloc{}; }
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
				data = AllocateData(src.NumVerts(), src.NumEdges(), src.NumFaces());
				if (not data) { throw std::bad_alloc{}; }

				std::copy(src.data->verts.begin(),   src.data->verts.end(),   data->verts.begin());
				std::copy(src.data->vertAdj.begin(), src.data->vertAdj.end(), data->vertAdj.begin());
				std::copy(src.data->edges.begin(),   src.data->edges.end(),   data->edges.begin());
				std::copy(src.data->faces.begin(),   src.data->faces.end(),   data->faces.begin());
				data->memUsed = src.data->memUsed;
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
				data = AllocateData(other.NumVerts(), other.NumEdges(), other.NumFaces());
				if (not data) { throw std::bad_alloc{}; }

				std::copy(other.data->verts.begin(),   other.data->verts.end(), data->verts.begin());
				std::copy(other.data->vertAdj.begin(), other.data->vertAdj.end(), data->vertAdj.begin());
				std::copy(other.data->edges.begin(),   other.data->edges.end(), data->edges.begin());
				std::copy(other.data->faces.begin(),   other.data->faces.end(), data->faces.begin());
				data->memUsed = other.data->memUsed;
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
