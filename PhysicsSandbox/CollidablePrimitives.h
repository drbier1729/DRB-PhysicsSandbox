//#pragma once
//#include "StringID.h"
//
//// Fwd decls
//struct RigidBody;
//struct Point;
//struct Sphere;
//struct Capsule;
//struct Plane;
//struct Box;
//struct Assembly;
//
//////////////////////////////////////////////////////////////////////////////////
//// Abstract Primitive Class
//////////////////////////////////////////////////////////////////////////////////
//
//enum class CollidableType
//{
//		Point,
//		Sphere,
//		Capsule,
//		Plane,
//		Box,
//		Assembly,
//		COUNT
//};
//
//
//struct CollidablePrimitive
//{
//	RigidBody* owner = nullptr;
//	Mat4 offset = Mat4(1);
//	Mat4 transform = Mat4(1);
//
//	virtual ~CollidablePrimitive() noexcept = default;
//
//	// Collision dispatcher. Returns true if any contacts were generated.
//	virtual Bool Collide(CollidablePrimitive*) = 0;
//
//	// Collision implementations. Returns true if any contacts were generated.
//	virtual Bool Collide(Point*) = 0;
//	virtual Bool Collide(Sphere*) = 0;
//	virtual Bool Collide(Capsule*) = 0;
//	virtual Bool Collide(Plane*) = 0;
//	virtual Bool Collide(Box*) = 0;
//	virtual Bool Collide(Assembly*) = 0;
//
//	// Type info (for serialization and debugging)
//	virtual CollidableType GetType() const noexcept = 0;
//	virtual const char* GetName() const noexcept = 0;
//	virtual StringID GetNameSID() const noexcept = 0;
//
//	// Calculates the transform based on offset and owner's position and 
//	// orientation. This should be called any time the owner may have moved,
//	// or the offset matrix has been changed.
//	void ComputeTransform() noexcept;
//
//	// Sets the owner of this primitive and computes internal data (normalizes 
//	// owner orientation, computes AABB bounds for owner, and computes this 
//	// primitive's transform matrix.
//	void SetOwner(RigidBody* rb) noexcept;
//
//	virtual Vec3 ComputeAABB() const noexcept {
//		return Vec3(std::numeric_limits<Float32>::max());
//	}
//
//	virtual void DebugDraw(Uint32 shader_id) const noexcept = 0;
//};
//
//// Concept defn
//template<typename T>
//concept Collidable = std::derived_from<T, CollidablePrimitive> && 
//	requires {
//		{ T::type } -> std::convertible_to<CollidableType>;
//		std::same_as<std::remove_cvref_t<decltype(T::type)>, CollidableType>;
//		
//		{ T::name } -> std::convertible_to<const char*>;
//		std::same_as<std::remove_cvref_t<decltype(T::name)>, const char*>;
//		
//		{ T::name_sid } -> std::convertible_to<StringID>;
//		std::same_as<std::remove_cvref_t<decltype(T::name_sid)>, StringID>;
//	};
//
///* Macro to simplify boilerplate for concrete primitive classes. Defines:
//*	- name, name_sid, and type static constexpr members along with virtual accessors
//*	- Collide virtual method overrides for all primitive types
//*	- DebugDraw virtual method override
//*	- Default ctor
//*/
//#define MAKE_COLLIDABLE(shape) static constexpr CollidableType type = CollidableType::##shape; \
//inline CollidableType GetType() const noexcept override { return type; } \
//static constexpr const char* name = #shape; \
//inline const char* GetName() const noexcept override { return name; } \
//static constexpr StringID name_sid = #shape##_sid; \
//inline StringID GetNameSID() const noexcept override { return name_sid; } \
//shape##() = default; \
//inline Bool Collide(CollidablePrimitive* second) override { return second->Collide(static_cast<shape*>(this)); } \
//Bool Collide(Point* first) override; \
//Bool Collide(Sphere* first) override; \
//Bool Collide(Capsule* first) override; \
//Bool Collide(Plane* first) override; \
//Bool Collide(Box* first) override; \
//Bool Collide(Assembly* first) override;\
//void DebugDraw(Uint32 shader_id) const noexcept override;
//
//
//////////////////////////////////////////////////////////////////////////////////
//// Concrete Primitive Classes
//////////////////////////////////////////////////////////////////////////////////
//
//struct Point final : public CollidablePrimitive 
//{
//	inline Vec3 ComputeAABB() const noexcept override {
//		return Vec3(0.5f);
//	}
//
//	MAKE_COLLIDABLE(Point)
//};
//
//struct Sphere final : public CollidablePrimitive
//{
//	explicit Sphere(Float32 r) : radius{ r } {}
//
//	Float32 radius = 1.0f;
//
//	inline Vec3 ComputeAABB() const noexcept override {
//		return Vec3(radius);
//	}
//
//	MAKE_COLLIDABLE(Sphere)
//};
//
//struct Capsule final : public CollidablePrimitive
//{
//	explicit Capsule(Vec3 const& start_pt, Vec3 const& end_pt, Float32 r) : p0{ start_pt }, p1{ end_pt }, radius{ r } {}
//
//	// Default = unit length with radius 1 
//	Vec3 p0{0, 0 ,0}, p1{0, 1, 0};
//	Float32 radius = 1.0f;
//
//	MAKE_COLLIDABLE(Capsule)
//};
//
//struct Plane final : public CollidablePrimitive
//{
//	explicit Plane(Vec3 const& normal, Float32 dist) : normal_dir{ normal }, signed_dist{ dist } {}
//	
//	// Default = xz plane
//	Vec3 normal_dir = Vec3(0, 1, 0);
//	Float32 signed_dist = 0.0f;
//
//	inline Vec3 ComputeAABB() const noexcept override {
//		static constexpr auto maxf = std::numeric_limits<Float32>::max();
//		if (normal_dir == Vec3(1, 0, 0)) { return Vec3(1,maxf, maxf); }
//		if (normal_dir == Vec3(0, 1, 0)) { return Vec3(maxf, 1, maxf); }
//		if (normal_dir == Vec3(0, 0, 1)) { return Vec3(maxf, maxf, 1); }
//		return Vec3(maxf, maxf, maxf);
//	}
//
//	MAKE_COLLIDABLE(Plane)
//};
//
//struct Box final : public CollidablePrimitive
//{
//	explicit Box(Vec3 const& halfwidths) : halfwidth_extents(halfwidths) {}
//
//	// Default = cube with "radius" 1
//	Vec3 halfwidth_extents = Vec3(1.0f);
//
//	struct Vertices {
//		Vec3 v[8] = {};
//		inline Vec3& operator[](Uint32 i) { return v[i]; }
//		inline Vec3 const& operator[](Uint32 i) const { return v[i]; }
//	};
//
//	inline void GenerateVertices(Vertices& outV) const noexcept {
//		outV.v[0] = GenVert(transform, halfwidth_extents, 1.f, 1.f, 1.f);
//		outV.v[1] = GenVert(transform, halfwidth_extents,  1.f, -1.f,  1.f);
//		outV.v[2] = GenVert(transform, halfwidth_extents,  1.f,  1.f, -1.f);
//		outV.v[3] = GenVert(transform, halfwidth_extents,  1.f, -1.f, -1.f);
//		outV.v[4] = GenVert(transform, halfwidth_extents, -1.f,  1.f,  1.f);
//		outV.v[5] = GenVert(transform, halfwidth_extents, -1.f, -1.f,  1.f);
//		outV.v[6] = GenVert(transform, halfwidth_extents, -1.f,  1.f, -1.f);
//		outV.v[7] = GenVert(transform, halfwidth_extents, -1.f, -1.f, -1.f);
//	}
//
//	inline Float32 ProjectOntoAxis(Vec3 const& axis) const {
//		Vec4 axis_(axis, 1);
//		return halfwidth_extents.x * glm::abs(glm::dot(axis_, glm::column(transform, 0))) +
//			halfwidth_extents.y * glm::abs(glm::dot(axis_, glm::column(transform, 1))) +
//			halfwidth_extents.z * glm::abs(glm::dot(axis_, glm::column(transform, 2)));
//	}
//
//	inline Vec3 ComputeAABB() const noexcept override {
//		return Vec3(ProjectOntoAxis(Vec3(1, 0, 0)),
//					ProjectOntoAxis(Vec3(0, 1, 0)),
//					ProjectOntoAxis(Vec3(0, 0, 1)));
//	}
//
//	MAKE_COLLIDABLE(Box)
//
//private:
//	inline static Vec3 GenVert(Mat4 const& tr, Vec3 const& halfwidths, Float32 i, Float32 j, Float32 k) {
//		return Vec3(tr * Vec4(i * halfwidths.x, j * halfwidths.y, k * halfwidths.z, 1.f));
//	}
//};
//
//// Structure for connecting multiple primitives together
//struct Assembly final : CollidablePrimitive
//{
//	struct PrimitiveInSet
//	{
//		CollidablePrimitive* primitive;
//		Mat4 offset_from_assembly;
//	};
//
//	Vector<PrimitiveInSet> primitives;
//
//	MAKE_COLLIDABLE(Assembly)
//};
//
//#undef MAKE_COLLIDABLE
//
//// Check to ensure concrete primitives satisfy Collidable concept
//static_assert(Collidable<Point>);
//static_assert(Collidable<Sphere>);
//static_assert(Collidable<Capsule>);
//static_assert(Collidable<Plane>);
//static_assert(Collidable<Box>);
//static_assert(Collidable<Assembly>);
//
//
