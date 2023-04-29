
#include "DRBAssert.h"

namespace drb::physics {

	// -------------------------------------------------------------------------
	// Sphere
	// -------------------------------------------------------------------------

	inline Mat3 Sphere::InertiaTensorAbout(Vec3 const& pt) const
	{
		// Compute local inertia tensor with center of mass at origin
		Mat3 I = 0.4_r * r * r * Mat3(1);

		// Use Parallel Axis Theorem to translate the inertia tensor
		Vec3 const disp = pt - c;        // displacement vector from center to pt
		I += glm::length2(disp) * Mat3(1) - glm::outerProduct(disp, disp);

		return I;
	}

	inline Vec3 Sphere::Position() const
	{
		return c;
	}

	inline void Sphere::SetPosition(Vec3 const& newC)
	{
		c = newC;
	}

	inline Quat Sphere::Orientation() const
	{
		return Quat(1, 0, 0, 0);
	}

	inline void Sphere::SetOrientation(Quat const& newO)
	{
		// do nothing
	}

	inline Mat4 Sphere::Transform() const
	{
		return glm::translate(Mat4(1), c);
	}

	inline void Sphere::SetTransform(Mat4 const& newTr)
	{
		c = newTr[3];
	}

	inline AABB Sphere::Bounds(Mat4 const& worldTr) const
	{
		Vec3 const extents = Vec3(r);
		Vec3 const worldC = worldTr * Vec4(c, 1);
		return AABB{ .c = worldC, .e = extents};
	}


	// -------------------------------------------------------------------------
	// Capsule
	// -------------------------------------------------------------------------

	inline Mat3 Capsule::InertiaTensorAbout(Vec3 const& pt) const
	{
		// Compute inertia tensor, assuming oriented vertically and
		// center is at local origin
		static constexpr Real two_pi = 6.283_r;

		Real const rSq = r * r;
		Real const height = 2.0_r * h;

		// Cylinder volume
		Real const cV = two_pi * h * rSq;

		// Hemisphere volume
		Real const hsV = two_pi * rSq * r / 3.0_r;

		Mat3 I(0);
		I[1][1] = rSq * cV * 0.5_r;
		I[0][0] = I[2][2] = I[1][1] * 0.5_r + cV * height * height / 12.0_r;

		Real const temp0 = hsV * 2.0_r * rSq / 5.0_r;
		I[1][1] += temp0 * 2.0_r;

		Real const temp1 = temp0 + (hsV * height * height) / 4.0_r + (3.0_r * height * r) / 8.0_r;
		I[0][0] += temp1 * 2.0_r;
		I[2][2] += temp1 * 2.0_r;

		// Use Parallel Axis Theorem to translate the inertia tensor
		Vec3 const disp = pt - Position(); // displacement vector from center to pt
		I += glm::length2(disp) * Mat3(1) - glm::outerProduct(disp, disp);

		return I;
	}

	inline Vec3 Capsule::Position() const
	{
		return 0.5_r * (seg.b + seg.e);
	}

	inline void Capsule::SetPosition(Vec3 const& newC)
	{
		Vec3 const dir = Direction();
		seg.b = newC - h * dir;
		seg.e = newC + h * dir;
	}

	inline Quat Capsule::Orientation() const
	{
		return Quat(Vec3(0, 1, 0), 0.5_r * (seg.e - seg.b));
	}

	inline void Capsule::SetOrientation(Quat const& newO)
	{
		Vec3 const c = Position();
		seg.b = glm::rotate(newO, seg.b - c) + c;
		seg.e = glm::rotate(newO, seg.e - c) + c;
	}

	inline Mat4 Capsule::Transform() const
	{
		Quat const o = Orientation();
		Vec3 const c = Position();
		return glm::translate(Mat4(1), c) * glm::toMat4(o);
	}

	inline void Capsule::SetTransform(Mat4 const& newTr)
	{
		seg = seg.Transformed(newTr);
	}

	inline AABB Capsule::Bounds(Mat4 const& worldTr) const
	{
		Vec3 const extents = Vec3(r, h + r, r);
		AABB const boundsLocal = AABB{ .e = extents };

		return boundsLocal.Transformed(worldTr * Transform());
	}


	// -------------------------------------------------------------------------
	// Box
	// -------------------------------------------------------------------------
	inline Mat3 Box::InertiaTensorAbout(Vec3 const& pt) const
	{
		static constexpr Real oneTwelfth = 1.0_r / 12.0_r;

		// Compute local inertia tensor
		Real const w2 = oneTwelfth * extents.x * extents.x;
		Real const h2 = oneTwelfth * extents.y * extents.y;
		Real const d2 = oneTwelfth * extents.z * extents.z;
		Mat3 I{
			h2 + d2, 0, 0,
			0, w2 + d2, 0,
			0, 0, w2 + h2
		};

		// Rotate the inertia tensor based on local orientation
		I = orientation * I * glm::transpose(orientation);

		// Use Parallel Axis Theorem to translate the inertia tensor
		Vec3 const disp = pt - position;        // displacement vector from center to pt
		I += glm::length2(disp) * Mat3(1) - glm::outerProduct(disp, disp);

		return I;
	}

	inline Vec3 Box::Position() const
	{
		return position;
	}

	inline void Box::SetPosition(Vec3 const& newPosition)
	{
		position = newPosition;
	}

	inline Quat Box::Orientation() const
	{
		return orientation;
	}

	inline void Box::SetOrientation(Quat const& newOrientation)
	{
		orientation = glm::toMat3(newOrientation);
	}

	inline Mat4 Box::Transform() const
	{
		return Mat4{ 
			Vec4(orientation[0], 0),
			Vec4(orientation[1], 0),
			Vec4(orientation[2], 0),
			Vec4(position, 1)
		};
	}

	inline void Box::SetTransform(Mat4 const& newTransform)
	{
		orientation = newTransform;
		position    = newTransform[3];
	}

	inline AABB Box::Bounds(Mat4 const& worldTr) const
	{
		return AABB{ .e = extents }.Transformed(worldTr * Transform());
	}


	// -------------------------------------------------------------------------
	// Convex
	// -------------------------------------------------------------------------
	inline Mat3 Convex::InertiaTensorAbout(Vec3 const& pt) const
	{
		// NOTE: this computes the inertia tensor assuming
		// the shape consists of connected point masses,
		// not a solid shape

		Mat3 I(0);

		auto const verts = GetVerts();
		for (auto&& v : verts)
		{
			Vec3 const r = (orientation * v + position) - pt;
			I[0][0] += r.y * r.y + r.z * r.z;
			I[0][1] -= r.x * r.y;
			I[0][2] -= r.x * r.z;

			I[1][1] += r.x * r.x + r.z * r.z;
			I[1][2] -= r.y * r.z;

			I[2][2] += r.x * r.x + r.y * r.y;
		}

		I[1][0] = I[0][1];
		I[2][0] = I[0][2];
		I[2][1] = I[1][2];

		return I;
	}

	inline Vec3 Convex::Position() const
	{
		return position;
	}

	inline void Convex::SetPosition(Vec3 const& newC)
	{
		position = newC;
	}

	inline Quat Convex::Orientation() const
	{
		return orientation;
	}

	inline void Convex::SetOrientation(Quat const& newO)
	{
		orientation = newO;
	}

	inline Mat4 Convex::Transform() const
	{
		return glm::translate(Mat4(1), position) * glm::toMat4(orientation);
	}
	

	inline void Convex::SetTransform(Mat4 const& newTr)
	{
		position = newTr[3];
		orientation = Quat(newTr);
	}

	inline AABB Convex::Bounds(Mat4 const& worldTr) const
	{
		return bounds.Transformed(Transform()).Transformed(worldTr);
	}

	inline Mat4 Convex::ModelTransform() const
	{
		Mat4 const tr = Transform();
		return data ? 
			tr * glm::translate(Mat4(1), -data->localCentroid) :
			Transform();
	}

	inline Vec3 const& Convex::GetVert(VertID index) const
	{
		ASSERT(data, "No data allocated");
		ASSERT(0 <= index && index < data->numVerts, "Index out of range");
		Vec3 const* verts = GetRawVerts();
		return *(verts + index);
	}

	inline Convex::EdgeID Convex::GetOneEdgeFrom(VertID index) const
	{
		ASSERT(data, "No data allocated");
		ASSERT(0 <= index && index < data->numVerts, "Index out of range");
		EdgeID const* vertAdj = GetRawVertAdjs();
		return *(vertAdj + index);
	}

	inline Convex::VertID Convex::NumVerts() const
	{
		return data ? static_cast<VertID>(data->numVerts) : 0;
	}

	inline Convex::HalfEdge const& Convex::GetEdge(EdgeID index) const
	{
		ASSERT(data, "No data allocated");
		ASSERT(0 <= index && index < data->numEdges, "Index out of range");
		HalfEdge const* edges = GetRawEdges();
		return *(edges + index);
	}

	inline Convex::EdgeID Convex::NumHalfEdges() const
	{
		return data ? static_cast<EdgeID>(data->numEdges) : 0;
	}

	inline Convex::Face const& Convex::GetFace(FaceID index) const
	{
		ASSERT(data, "No data allocated");
		ASSERT(0 <= index && index < data->numFaces, "Index out of range");
		Face const* faces = GetRawFaces();
		return *(faces + index);
	}

	inline Convex::FaceID Convex::NumFaces() const
	{
		return data ? static_cast<FaceID>(data->numFaces) : 0;
	}

	inline std::span<Vec3 const> Convex::GetVerts() const
	{
		ASSERT(data, "No data allocated");
		return std::span{ GetRawVerts(), static_cast<SizeT>(data->numVerts) };
	}

	inline std::span<Convex::EdgeID const> Convex::GetVertAdjs() const
	{
		ASSERT(data, "No data allocated");
		return std::span{ GetRawVertAdjs(), static_cast<SizeT>(data->numVerts) };
	}

	inline std::span<Convex::HalfEdge const> Convex::GetEdges() const
	{
		ASSERT(data, "No data allocated");
		return std::span{ GetRawEdges(), static_cast<SizeT>(data->numEdges) };
	}

	inline std::span<Convex::Face const> Convex::GetFaces() const
	{
		ASSERT(data, "No data allocated");
		return std::span{ GetRawFaces(), static_cast<SizeT>(data->numFaces) };
	}

	inline Vec3* Convex::GetRawVerts()
	{
		ASSERT(data, "No data allocated");
		Vec3* verts = reinterpret_cast<Vec3*>(reinterpret_cast<std::byte*>(data) + data->vertsOffset);
		return verts;
	}

	inline Convex::EdgeID* Convex::GetRawVertAdjs()
	{
		ASSERT(data, "No data allocated");
		EdgeID* vertAdj = reinterpret_cast<EdgeID*>(reinterpret_cast<std::byte*>(data) + data->vertAdjOffset);
		return vertAdj;
	}

	inline Convex::HalfEdge* Convex::GetRawEdges()
	{
		ASSERT(data, "No data allocated");
		HalfEdge* edges = reinterpret_cast<HalfEdge*>(reinterpret_cast<std::byte*>(data) + data->edgesOffset);
		return edges;
	}

	inline Convex::Face* Convex::GetRawFaces()
	{
		ASSERT(data, "No data allocated");
		Face* faces = reinterpret_cast<Face*>(reinterpret_cast<std::byte*>(data) + data->facesOffset);
		return faces;
	}

	inline Vec3 const* Convex::GetRawVerts() const
	{
		ASSERT(data, "No data allocated");
		Vec3 const* verts = reinterpret_cast<Vec3 const*>(reinterpret_cast<std::byte*>(data) + data->vertsOffset);
		return verts;
	}

	inline Convex::EdgeID const* Convex::GetRawVertAdjs() const
	{
		ASSERT(data, "No data allocated");
		EdgeID const* vertAdj = reinterpret_cast<EdgeID const*>(reinterpret_cast<std::byte*>(data) + data->vertAdjOffset);
		return vertAdj;
	}

	inline Convex::HalfEdge const* Convex::GetRawEdges() const
	{
		ASSERT(data, "No data allocated");
		HalfEdge const* edges = reinterpret_cast<HalfEdge const*>(reinterpret_cast<std::byte*>(data) + data->edgesOffset);
		return edges;
	}

	inline Convex::Face const* Convex::GetRawFaces() const
	{
		ASSERT(data, "No data allocated");
		Face const* faces = reinterpret_cast<Face const*>(reinterpret_cast<std::byte*>(data) + data->facesOffset);
		return faces;
	}

	void Convex::ForEachOneRingNeighbor(EdgeID start, std::invocable<HalfEdge> auto&& fn) const
	{
		ASSERT(start < NumHalfEdges(), "Index out of range");

		HalfEdge const* edgesRaw = GetRawEdges();
		Convex::HalfEdge const firstNeighbor = edgesRaw[edgesRaw[start].twin];

		Convex::HalfEdge neighbor = firstNeighbor;
		do {
			fn(neighbor);

			// Traverse to next neighbor
			neighbor = edgesRaw[edgesRaw[neighbor.next].twin];

		} while (neighbor != firstNeighbor);
	}

	void Convex::ForEachEdgeOfFace(FaceID face, std::invocable<HalfEdge> auto&& fn) const
	{
		ASSERT(face < NumFaces(), "Index out of range");

		HalfEdge const* edgesRaw     = GetRawEdges();
		Face const* facesRaw         = GetRawFaces();
		Convex::HalfEdge const start = edgesRaw[ facesRaw[face].edge ];

		Convex::HalfEdge e = start;
		do {
			fn(e);

			// Traverse to next edge
			e = edgesRaw[e.next];

		} while (e != start);
	}
		

	void CollisionGeometry::ForEachCollider(ConstShapeFn auto&& fn) const
	{
		for (auto&& shape : spheres) {
			fn(shape);
		}
		for (auto&& shape : capsules) {
			fn(shape);
		}
		for (auto&& shape : boxes) {
			fn(shape);
		}
		for (auto&& shape : hulls) {
			fn(shape);
		}
	}

	void CollisionGeometry::ForEachCollider(ShapeFn auto&& fn)
	{
		for (auto&& shape : spheres) {
			fn(shape);
		}
		for (auto&& shape : capsules) {
			fn(shape);
		}
		for (auto&& shape : boxes) {
			fn(shape);
		}
		for (auto&& shape : hulls) {
			fn(shape);
		}
	}

	CollisionGeometry& CollisionGeometry::AddCollider(Shape auto const& collider, Real mass)
	{
		if (locked)
		{
			ASSERT(not locked, "Bake has already been called");
			return *this;
		}

		using T = std::remove_cvref_t<decltype(collider)>;

		ColliderSettings s{ .type = T::type, .relativeMass = mass, .centroid = collider.Position() };

		if constexpr (std::same_as<T, Sphere>)
		{
			s.index = spheres.size();
			spheres.emplace_back(collider);
		}
		else if constexpr (std::same_as<T, Capsule>)
		{
			s.index = capsules.size();
			capsules.emplace_back(collider);
		}
		else if constexpr (std::same_as<T, Box>)
		{
			s.index = boxes.size();
			boxes.emplace_back(collider);
		}
		else if constexpr (std::same_as<T, Convex>)
		{
			s.index = hulls.size();
			hulls.emplace_back(collider);
		}
		else
		{
			static_assert(std::false_type<T>::value, "Invalid type");
		}

		settings.emplace_back(std::move(s));
		return *this;
	}

	CollisionGeometry& CollisionGeometry::AddCollider(Shape auto&& collider, Real mass)
	{
		if (locked)
		{
			ASSERT(not locked, "Bake has already been called");
			return *this;
		}

		using T = std::remove_cvref_t<decltype(collider)>;

		ColliderSettings s{ .type = T::type, .relativeMass = mass, .centroid = collider.Position() };

		if constexpr (std::same_as<T, Sphere>)
		{
			s.index = spheres.size();
			spheres.emplace_back(std::forward<T>(collider));
		}
		else if constexpr (std::same_as<T, Capsule>)
		{
			s.index = capsules.size();
			capsules.emplace_back(std::forward<T>(collider));
		}
		else if constexpr (std::same_as<T, Box>)
		{
			s.index = boxes.size();
			boxes.emplace_back(std::forward<T>(collider));
		}
		else if constexpr (std::same_as<T, Convex>)
		{
			s.index = hulls.size();
			hulls.emplace_back(std::forward<T>(collider));
		}
		else
		{
			static_assert(std::false_type<T>::value, "Invalid type");
		}

		settings.emplace_back(std::move(s));
		return *this;
	}

	inline SizeT CollisionGeometry::Size() const
	{
		return spheres.size() + capsules.size() + boxes.size() + hulls.size();
	}

	inline void  CollisionGeometry::Reserve(SizeT newCapacities)
	{
		spheres.reserve(newCapacities);
		capsules.reserve(newCapacities);
		boxes.reserve(newCapacities);
		hulls.reserve(newCapacities);
	}

	inline Mat3 const& CollisionGeometry::InverseInertia() const { return invInertia; }

	inline Real     CollisionGeometry::InverseMass() const       { return invMass; }
	
	inline Vec3 const& CollisionGeometry::CenterOfMass() const   { return centerOfMass; }
	
	inline AABB const& CollisionGeometry::Bounds() const         { return bounds; }
}