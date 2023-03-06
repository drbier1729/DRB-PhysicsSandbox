namespace drb::physics {

	// -------------------------------------------------------------------------
	// Sphere
	// -------------------------------------------------------------------------

	inline Mat3 Sphere::InertiaTensorAbout(Vec3 const& pt) const
	{
		// Compute local inertia tensor with center of mass at origin
		Mat3 I = 0.4f * r * r * Mat3(1);

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
		Vec3 const worldC = c + Vec3(worldTr[3]);
		return AABB{ .max = worldC + extents, .min = worldC - extents};
	}
	// -------------------------------------------------------------------------
	// Capsule
	// -------------------------------------------------------------------------

	inline Mat3 Capsule::InertiaTensorAbout(Vec3 const& pt) const
	{
		// Compute inertia tensor, assuming oriented vertically and
		// center is at local origin
		static constexpr Float32 two_pi = 6.283f;

		Float32 const rSq = r * r;
		Float32 const height = 2.0f * h;

		// Cylinder volume
		Float32 const cV = two_pi * h * rSq;

		// Hemisphere volume
		Float32 const hsV = two_pi * rSq * r / 3.0f;

		Mat3 I(0);
		I[1][1] = rSq * cV * 0.5f;
		I[0][0] = I[2][2] = I[1][1] * 0.5f + cV * height * height / 12.0f;

		Float32 const temp0 = hsV * 2.0f * rSq / 5.0f;
		I[1][1] += temp0 * 2.0f;

		Float32 const temp1 = temp0 + (hsV * height * height) / 4.0f + (3.0f * height * r) / 8.0f;
		I[0][0] += temp1 * 2.0f;
		I[2][2] += temp1 * 2.0f;

		// Use Parallel Axis Theorem to translate the inertia tensor
		Vec3 const disp = pt - Position(); // displacement vector from center to pt
		I += glm::length2(disp) * Mat3(1) - glm::outerProduct(disp, disp);

		return I;
	}

	inline Vec3 Capsule::Position() const
	{
		return 0.5f * (seg.b + seg.e);
	}

	inline void Capsule::SetPosition(Vec3 const& newC)
	{
		Vec3 const dir = Direction();
		seg.b = newC - h * dir;
		seg.e = newC + h * dir;
	}

	inline Quat Capsule::Orientation() const
	{
		return Quat(Vec3(0, 1, 0), 0.5f * (seg.e - seg.b));
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
		AABB const boundsLocal{ .max = extents, .min = -extents };

		return boundsLocal.Transformed(worldTr * Transform());
	}

	// -------------------------------------------------------------------------
	// Convex
	// -------------------------------------------------------------------------
	inline Mat3 Convex::InertiaTensorAbout(Vec3 const& pt) const
	{
		Mat3 I(0);

		auto const verts = GetVerts();
		for (auto&& v : verts)
		{
			Vec3 const r = v - pt; // relative position from pt to v
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
		// Orientation is applied to the object with its centroid at the origin
		return glm::translate(Mat4(1), Centroid()) * glm::toMat4(orientation) * glm::translate(Mat4(1), -LocalCentroid());
	}

	inline void Convex::SetTransform(Mat4 const& newTr)
	{
		position = newTr[3];
		orientation = Quat(newTr);
	}

	inline AABB Convex::Bounds(Mat4 const& worldTr) const
	{
		return bounds.Transformed(worldTr * Transform());
	}

	inline Vec3 Convex::LocalCentroid() const
	{
		return data ? data->localCentroid : Vec3(0);
	}

	inline Vec3 Convex::Centroid() const
	{
		return data ? position + data->localCentroid : position;
	}

	inline void Convex::SetCentroid(Vec3 const& newC)
	{
		position = data ? newC - data->localCentroid : newC;
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

	void Convex::ForEachOneRingNeighbor(EdgeID start, std::invocable<HalfEdge> auto fn) const
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

	void Convex::ForEachEdgeOfFace(FaceID face, std::invocable<HalfEdge> auto fn) const
	{
		ASSERT(face < NumFaces(), "Index out of range");

		HalfEdge const* edgesRaw           = GetRawEdges();
		Face const* facesRaw               = GetRawFaces();
		Convex::HalfEdge const start = edgesRaw[ facesRaw[face].edge ];

		Convex::HalfEdge e = start;
		do {
			fn(e);

			// Traverse to next edge
			e = edgesRaw[e.next];

		} while (e != start);
	}


	// -------------------------------------------------------------------------
	// Collider
	// -------------------------------------------------------------------------

	inline ColliderType Collider::Type() const
	{
		return type;
	}

	inline Float32 Collider::Mass() const
	{
		return mass;
	}

	template<class RetType, class Fn, class ... Args>
	RetType Collider::CallOnShape(Fn func, Args && ... args) const
	{
		switch (type) 
		{
		break; case ColliderType::Sphere: { 
			return func(static_cast<CollisionShape<Sphere> const*>(this)->shape, std::forward<Args>(args)...);
		}
		break; case ColliderType::Capsule: {
			return func(static_cast<CollisionShape<Capsule> const*>(this)->shape, std::forward<Args>(args)...);
		} 
		break; case ColliderType::Convex: {
			return func(static_cast<CollisionShape<Convex> const*>(this)->shape, std::forward<Args>(args)...);
		}
		break; default: { ASSERT(false, "Bad type"); }
		}
		
		return RetType{};
	}


#define CONVERT_TO_SHAPE_AND_CALL_CONST_METHOD(retType, method, ...) \
switch (type) {\
	break; case ColliderType::Sphere: { \
		return static_cast<CollisionShape<Sphere> const*>(this)->shape.##method##(__VA_ARGS__); \
	} \
	break; case ColliderType::Capsule: { \
		return static_cast<CollisionShape<Capsule> const*>(this)->shape.##method##(__VA_ARGS__); \
	} \
	break; case ColliderType::Convex: { \
		return static_cast<CollisionShape<Convex> const*>(this)->shape.##method##(__VA_ARGS__); \
	} \
	break; default: { ASSERT(false, "Bad type"); } }\
 return retType{};

#define CONVERT_TO_SHAPE_AND_CALL_METHOD(retType, method, ...) \
switch (type) {\
	break; case ColliderType::Sphere: { \
		return static_cast<CollisionShape<Sphere>*>(this)->shape.##method##(__VA_ARGS__); \
	} \
	break; case ColliderType::Capsule: { \
		return static_cast<CollisionShape<Capsule>*>(this)->shape.##method##(__VA_ARGS__); \
	} \
	break; case ColliderType::Convex: { \
		return static_cast<CollisionShape<Convex>*>(this)->shape.##method##(__VA_ARGS__); \
	} \
	break; default: { ASSERT(false, "Bad type"); } }\
 return retType{};

	Mat3 Collider::InertiaTensorAbout(Vec3 const& pt) const
	{
		CONVERT_TO_SHAPE_AND_CALL_CONST_METHOD(Mat3, InertiaTensorAbout, pt)
	}

	Vec3 Collider::Position() const
	{
		CONVERT_TO_SHAPE_AND_CALL_CONST_METHOD(Vec3, Position)
	}

	void Collider::SetPosition(Vec3 const& newPos)
	{
		CONVERT_TO_SHAPE_AND_CALL_METHOD(void, SetPosition, newPos)
	}

	Quat Collider::Orientation() const
	{
		CONVERT_TO_SHAPE_AND_CALL_CONST_METHOD(Quat, Orientation)
	}

	void Collider::SetOrientation(Quat const& newOrientation)
	{
		CONVERT_TO_SHAPE_AND_CALL_METHOD(void, SetOrientation, newOrientation)
	}
	Mat4 Collider::Transform() const
	{
		CONVERT_TO_SHAPE_AND_CALL_CONST_METHOD(Mat4, Transform)
	}
	void Collider::SetTransform(Mat4 const& newTransform)
	{
		CONVERT_TO_SHAPE_AND_CALL_METHOD(void, SetTransform, newTransform)
	}
	AABB Collider::Bounds(Mat4 const& worldTr) const
	{
		CONVERT_TO_SHAPE_AND_CALL_CONST_METHOD(AABB, Bounds, worldTr)
	}

#undef CONVERT_TO_SHAPE_AND_CALL_METHOD
#undef CONVERT_TO_SHAPE_AND_CALL_CONST_METHOD

	CollisionGeometry& CollisionGeometry::AddCollider(Shape auto&& shape, Float32 mass)
	{
		ASSERT(not locked, "Cannot add colliders after calling Bake");

		using T = std::remove_cvref_t<decltype(shape)>;

		if constexpr (std::is_same_v<T, Sphere>) {
			spheres.emplace_back(std::forward<Sphere>(shape), std::forward<Float32>(mass));
		}
		else if constexpr (std::is_same_v<T, Capsule>) {
			capsules.emplace_back(std::forward<Capsule>(shape), std::forward<Float32>(mass));
		}
		else if constexpr (std::is_same_v<T, Convex>) {
			hulls.emplace_back(std::forward<Convex>(shape), std::forward<Float32>(mass));
		}
		else {
			static_assert(std::false_type<T>::value, "Not supported");
		}

		return *this;
	}

	CollisionGeometry& CollisionGeometry::AddCollider(Shape auto const& shape, Float32 mass)
	{
		ASSERT(not locked, "Cannot add colliders after calling Bake");
		
		using T = std::remove_cvref_t<decltype(shape)>;

		if constexpr (std::is_same_v<T, Sphere>) {
			spheres.emplace_back(shape, mass);
		}
		else if constexpr (std::is_same_v<T, Capsule>) {
			capsules.emplace_back(shape, mass);
		}
		else if constexpr (std::is_same_v<T, Convex>) {
			hulls.emplace_back(shape, mass);
		}
		else {
			static_assert(std::false_type<T>::value, "Not supported");
		}

		return *this;
	}

	template<class Fn>
	void CollisionGeometry::ForEachCollider(Fn fn)
	{
		ASSERT(not locked, "Cannot modify colliders after calling Bake");
		
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


	inline Int32 CollisionGeometry::Size() const
	{
		return static_cast<Int32>(spheres.size() + capsules.size() + hulls.size());
	}

	inline void CollisionGeometry::Reserve(Int32 newCap)
	{
		spheres.reserve(newCap);
		capsules.reserve(newCap);
		hulls.reserve(newCap);
	}
}