#include "pch.h"
#include "CollisionGeometry.h"

#include "StackAllocator.h"
#include "Math.h"

namespace drb::physics {


	// -------------------------------------------------------------------------
	// Capsule
	// -------------------------------------------------------------------------
	
	Capsule::Capsule(Segment const& centralSeg, Float32 radius)
		: seg{ centralSeg }, 
		h{ 0.5f * glm::length(centralSeg.e - centralSeg.b)}, 
		r{ radius }
	{}

	Capsule::Capsule(Quat const& orientation, Vec3 pos, Float32 segmentHalfLength, Float32 radius)
		: seg{ }, h{segmentHalfLength}, r{radius}
	{
		Vec3 const dir = glm::rotate(orientation, Vec3(0, 1, 0));
		seg.b = pos - h * dir;
		seg.e = pos + h * dir;
	}

	Capsule::Capsule(Mat3 const& orientation, Vec3 pos, Float32 segmentHalfLength, Float32 radius)
		: Capsule{ glm::toQuat(orientation), pos, segmentHalfLength, radius }
	{}

	Segment const& Capsule::CentralSegment() const
	{
		return seg;
	}

	Float32  Capsule::SegmentHalfLength() const
	{
		return h;
	}

	Vec3 Capsule::Direction() const
	{
		return Normalize(seg.e - seg.b);
	}

	void Capsule::SetCentralSegment(Segment const& newSeg)
	{
		seg = newSeg;
		h = 0.5f * glm::length(newSeg.e - newSeg.b);
	}

	void Capsule::SetHalfLength(Float32 newH)
	{
		h = newH;
		Vec3 const dir = Direction();
		Vec3 const c = Position();
		seg.b = c - newH * dir;
		seg.e = c + newH * dir;
	}

	void Capsule::SetDirection(Vec3 const& newDir)
	{
		ASSERT(EpsilonEqual(glm::length2(newDir), 1.0f), "New direction is not normalized");

		Quat const dq{ Direction(), newDir };
		Vec3 const newC = glm::rotate(dq, Position());
		seg.b = newC - h * newDir;
		seg.e = newC + h * newDir;
	}

	// -------------------------------------------------------------------------
	// Box
	// -------------------------------------------------------------------------

	Box::Box(Vec3 const& e, Vec3 const& p, Mat3 const& o)
		: extents{e}, position{p}, orientation{o}, _pad{}
	{}
	
	Box::Box(Vec3 const& e, Vec3 const& p, Quat const& o)
		: extents{e}, position{p}, orientation{ glm::toMat3( o )}, _pad{}
	{}

	Polygon Box::FaceAsPolygon(Face face) const
	{
		Float32 const sign = (static_cast<Int32>(face) >= 3) ? -1.0f : 1.0f;
		Int32 const axis = static_cast<Int32>(face) % 3;
		Int32 const j = (axis + 1) % 3;
		Int32 const k = (j + 1) % 3;

		Polygon result{
			.verts = {	sign * extents, 
					    sign * extents, 
						sign * extents, 
						sign * extents 
			}
		};
		
		for (Int32 i = 0; i < 4; ++i) {
			// Wind counter clockwise around verts
			// (j, k) = (+, +) --> (-, +) --> (-, -) --> (-, +)
			result.verts[i][j] *= (i == 0 || i == 3) ? 1.0f : -1.0f;
			result.verts[i][k] *= (i < 2)            ? 1.0f : -1.0f;
		}

		return result;
	}

	Segment Box::EdgeAsSegment(Face adjFace1, Face adjFace2) const
	{
		Int32 const faceIdx1 = static_cast<Int32>(adjFace1);
		Int32 const faceIdx2 = static_cast<Int32>(adjFace2);

		Float32 const sign1 = (faceIdx1 >= 3) ? -1.0f : 1.0f;
		Int32   const axis1 =  faceIdx1 % 3;
		
		Float32 const sign2 = (faceIdx2 >= 3) ? -1.0f : 1.0f;
		Int32   const axis2 =  faceIdx2 % 3;

		ASSERT(axis1 != axis2, "Faces are parallel, not adjacent");

		Int32 const axis3 = (axis1 + axis2 == 1) ? 2 :
								((axis1 + axis2 == 2) ? 1 : 0);

		Vec3 e{ extents };
		e[axis1] *= sign1;
		e[axis2] *= sign2;

		Vec3 b{ e };
		b[axis3] *= -1.0f;

		return Segment{
			.b = b,
			.e = e
		};
	}

	Vec3 Box::FaceNormal(Face face) const
	{
		Int32 const faceIdx = static_cast<Int32>(face);
		Float32 const sign = (faceIdx >= 3) ? -1.0f : 1.0f;

		Vec3 result{ 0.0f };
		result[faceIdx % 3] = sign;
		
		return result;
	}

	Box::Face Box::SupportingFace(Vec3 const& localDir) const
	{
		static constexpr Int32 numFaces = static_cast<Int32>(Face::COUNT); // 6 

		Face best{};
		Float32 bestProj = std::numeric_limits<Float32>::lowest();
		for (Int32 i = 0; i < numFaces; ++i)
		{
			Face const f = static_cast<Face>(i);
			Float32 const dot = glm::dot(FaceNormal(f), localDir);
			if (dot > bestProj) 
			{
				bestProj = dot;
				best = f;
			}
		}
		return best;
	}


	Segment Box::SupportingEdgeWithDirection(Int32 axisIdx, Vec3 const& localDir) const
	{
		// We test two adjacent vertices on the face given by axisIdx, starting 
		// with extents
		Vec3 verts[2] = {
			extents,
			extents
		};

		// For the second vert, we need to do some index trickery
		Int32 const j = (axisIdx + 1) % 3;
		verts[1][j] *= -1.0f;

		// Choose the vert with the best (abs) projection onto dir
		Int32 vIdx = 0;
		Float32 proj = glm::dot(verts[0], localDir);
		if (Float32 const proj1 = glm::dot(verts[1], localDir);
			glm::abs(proj1) > glm::abs(proj))
		{
			proj = proj1;
			vIdx = 1;
		}

		// If the projection was negative, flip to the opposite vertex 
		// on the same face (e.g. diagonal from this vert)
		if (proj < 0.0f)
		{
			Int32 const k = (j + 1) % 3;
			verts[vIdx][j] *= -1.0f;
			verts[vIdx][k] *= -1.0f;
		}

		// Get the adjacent vertex on the opposite face in axisIdx direction
		Segment result
		{
			.b = verts[vIdx],
			.e = verts[vIdx]
		};
		result.e[axisIdx] *= -1.0f;

		return result;
	}

	Segment Box::SupportingEdge(Vec3 const& localDir) const
	{
		// Identify the most orthogonal axis to localDir
		Int32 axisIdx = 0;
		{
			Float32 bestProj = glm::abs(localDir[0]);
			Float32 proj{};
			for (Int32 i = 1; i < 2; ++i)
			{
				proj = glm::abs(localDir[i]);
				if (proj < bestProj) 
				{
					bestProj = proj;
					axisIdx = i;
				}
			}
		}

		return SupportingEdgeWithDirection(axisIdx, localDir);
	}

	// -------------------------------------------------------------------------
	// Convex
	// -------------------------------------------------------------------------
	
	Polygon Convex::FaceAsPolygon(FaceID face, Mat4 const& tr_) const
	{
		ASSERT(face < NumFaces(), "Index out of range");

		Mat4 const tr = tr_ * Transform();

		Polygon poly{};
		ForEachEdgeOfFace(face, [&](Convex::HalfEdge e) {
			Vec4 const v = Vec4(GetVert(e.origin), 1);
			poly.verts.emplace_back(tr * v);
		});

		return poly;
	}
	
	void Convex::AllocateData(VertID numVerts, EdgeID numEdges, FaceID numFaces)
	{
		ASSERT(0 < numVerts, "Number of verts invalid");
		ASSERT(0 < numEdges, "Number of edges invalid");
		ASSERT(0 < numFaces, "Number of faces invalid");
		ASSERT(numVerts + numFaces == 2 + numEdges / 2, "Invalid Euler characteristic");

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
		Header* h = a.Construct<Header>();
		std::byte* verts = static_cast<std::byte*>(a.Alloc(vSize, alignof(Vec3)));
		std::byte* vertAdj = static_cast<std::byte*>(a.Alloc(vaSize, alignof(EdgeID)));
		std::byte* edges = static_cast<std::byte*>(a.Alloc(eSize, alignof(HalfEdge)));
		std::byte* faces = static_cast<std::byte*>(a.Alloc(fSize, alignof(Face)));

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
		data->memUsed = static_cast<Int16>(arena.size);
	}

	Convex::Convex(Vec3 const* verts, EdgeID const* vertAdj, VertID numVerts,
		HalfEdge const* edges, EdgeID numHalfEdges,
		Face const* faces, FaceID numFaces)

		: data{ nullptr },
		position{ 0.0f },
		orientation{1, 0, 0, 0},
		bounds{}
	{
		AllocateData(numVerts, numHalfEdges, numFaces);
		if (not data) { throw std::bad_alloc{}; }

		Vec3* rawVerts = GetRawVerts();
		std::memcpy(rawVerts, verts, numVerts * sizeof(Vec3));
		std::memcpy(GetRawVertAdjs(), vertAdj, numVerts * sizeof(EdgeID));
		std::memcpy(GetRawEdges(), edges, numHalfEdges * sizeof(HalfEdge));
		std::memcpy(GetRawFaces(), faces, numFaces * sizeof(Face));

		for (VertID i = 0; i != numVerts; ++i)
		{
			Vec3 const& v = *(rawVerts + i);
			data->localCentroid += v;
			bounds.max = glm::max(bounds.max, v);
			bounds.min = glm::max(bounds.min, v);
		}
		data->localCentroid *= 1.0f / numVerts;
	}

	Convex::~Convex() noexcept
	{
		std::free(data);
	}

	Convex::Convex(Convex const& src)
		: data{ nullptr },
		position{ src.position },
		orientation{ src.orientation },
		bounds{ src.bounds }
	{
		if (src.data)
		{
			void* memory = (Header*)std::malloc(src.data->memUsed);
			if (not memory) { throw std::bad_alloc{}; }

			std::memcpy(memory, src.data, src.data->memUsed);
			data = (Header*)memory;
		}
	}

	Convex::Convex(Convex&& src) noexcept
		: data{ src.data },
		position{ src.position },
		orientation{ src.orientation },
		bounds{ src.bounds }
	{
		src.data = nullptr;
	}

	Convex& Convex::operator=(Convex const& other)
	{
		if (this == &other) { return *this; }

		std::free(data);
		if (other.data)
		{
			Header* newData = static_cast<Header*>(std::malloc(other.data->memUsed));
			if (not newData) { throw std::bad_alloc{}; }

			std::memcpy(newData, other.data, other.data->memUsed);
			data = newData;
		}
		else
		{
			data = nullptr;
		}

		position = other.position;
		orientation = other.orientation;
		bounds = other.bounds;

		return *this;
	}
	Convex& Convex::operator=(Convex&& other) noexcept
	{
		if (this == &other) { return *this; }

		std::free(data);
		data = other.data;
		position = other.position;
		orientation = other.orientation;
		bounds = other.bounds;

		other.data = nullptr;

		return *this;
	}

	Convex  Convex::MakeBox(Vec3 const& halfwidths, Mat4 const& tr)
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

		static constexpr EdgeID vertAdj[] = { 0, 1,	3, 5, 9, 11, 20, 15 };

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

		Convex box{ verts, vertAdj, 8, halfEdges, 24, faces, 6 };
		box.bounds = { .max = halfwidths, .min = -halfwidths };
		box.position = tr[3];
		box.orientation = Quat(tr);
		return box;
	}

	Convex  Convex::MakeTetrahedron(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2_, Vec3 const& p3_, Mat4 const& tr)
	{
		// Centroid
		Vec3 const c = 0.25f * (p0 + p1 + p2_ + p3_);

		// Check if we need to swap the vertex order in order for
		// triangle p1, p2, p3 to be in counterclockwise order
		Vec3 const a = p2_ - p1, b = p3_ - p1;
		Bool const swap = glm::dot(glm::cross(a, b), p0) < 0.0f;
		Vec3 const& p2 = (swap ? p3_ : p2_);
		Vec3 const& p3 = (swap ? p2_ : p3_);

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
				{.edge = 0, .plane = Plane::Make(p0, p1, p2) },
				{.edge = 5, .plane = Plane::Make(p0, p2, p3) },
				{.edge = 9, .plane = Plane::Make(p0, p3, p1) },
				{.edge = 3, .plane = Plane::Make(p3, p2, p1) }
		};

		Convex tet{ verts, vertAdj, 4, halfEdges, 12, faces, 4 };
		tet.bounds = {
				.max = glm::max(p0, glm::max(p1, glm::max(p2, p3))),
				.min = glm::min(p0, glm::min(p1, glm::min(p2, p3)))
		};
		tet.data->localCentroid = c;
		tet.position = Vec3(tr[3]);
		tet.orientation = Quat(tr);

		return tet;
	}

	// -------------------------------------------------------------------------
	// Collision Geometry
	// -------------------------------------------------------------------------

	void CollisionGeometry::Bake()
	{
		// Add colliders, and compute mass and center of mass
		Float32 mass = 0.0f;
		Vec3 com{ 0.0f };
		for (auto&& s : settings)
		{
			mass += s.relativeMass;
			com += s.relativeMass * s.centroid;
		}

		// Compute inertia tensors
		if (mass > 0.0f) {

			invMass = 1.0f / mass;
			centerOfMass = com * invMass;

			Mat3 I{ 0.0f };

			// Awkward way to iterate but we gotta...
			SizeT nSphs = 0, nCaps = 0, nBox = 0, nCvx = 0;
			for (auto&& s : settings)
			{
				switch (s.type)
				{
				break; case ColliderType::Sphere:
				{
					ASSERT(nSphs < spheres.size(), "Out of range");
					I += s.relativeMass * spheres[nSphs++].InertiaTensorAbout(centerOfMass);
				}
				break; case ColliderType::Capsule:
				{
					ASSERT(nCaps < capsules.size(), "Out of range");
					I += s.relativeMass * capsules[nCaps++].InertiaTensorAbout(centerOfMass);
				}
				break; case ColliderType::Box:
				{
					ASSERT(nBox < boxes.size(), "Out of range");
					I += s.relativeMass * boxes[nBox++].InertiaTensorAbout(centerOfMass);
				}
				break; case	ColliderType::Convex:
				{
					ASSERT(nCvx < hulls.size(), "Out of range");
					I += s.relativeMass * hulls[nCvx++].InertiaTensorAbout(centerOfMass);
				}
				}
			}
			invInertia = glm::inverse(I);
		}

		// Compute bounding boxes
		ForEachCollider([this](Shape auto& shape)
		{
			bounds = bounds.Union(shape.Bounds());
		});

		// Clear our settings since this is no longer modifiable
		settings.clear();
		settings.shrink_to_fit();

		locked = true;
	}
}