#ifndef DRB_COLLISIONGEOMETRY_H
#define DRB_COLLISIONGEOMETRY_H

#include "AABB.h"
#include "GeometryPrimitives.h"

#define DRB_SHAPE_INTERFACE() \
inline Mat3 InertiaTensorAbout(Vec3 const& pt) const; \
inline Vec3 Position() const; \
inline void SetPosition(Vec3 const& newPosition); \
inline Quat Orientation() const; \
inline void SetOrientation(Quat const& newOrientation); \
inline Mat4 Transform() const; \
inline void SetTransform(Mat4 const& newTransform); \
inline AABB Bounds(Mat4 const& worldTransform = Mat4(1)) const

namespace drb::physics {

	enum class ColliderType
	{
		NONE = 0,
		Sphere = 1,
		Capsule = 2,
		Convex = 3,
		Mesh = 4
	};

	struct Sphere
	{
		static constexpr auto type = ColliderType::Sphere;

		Vec3    c; // world space centroid
		Float32 r; // radius

		DRB_SHAPE_INTERFACE();
	};


	// The default capsule is oriented with central 
	// segment with direction (0, 1, 0). The 
	// Capsule::Orientation method returns the 
	// orientation relative to this default.
	struct Capsule
	{
		static constexpr auto type = ColliderType::Capsule;

	private:
		Segment seg; // world space central segment
		Float32 h;   // half-length of central segment

	public:
		Float32 r;   // radius

	public:
		Capsule(Segment const& centralSeg, Float32 radius);
		Capsule(Quat const& orientation, Vec3 pos, Float32 segmentHalfLength, Float32 radius);
		Capsule(Mat3 const& orientation, Vec3 pos, Float32 segmentHalfLength, Float32 radius);
		
		Capsule() = default;
		Capsule(Capsule const&) = default;
		Capsule(Capsule &&) = default;
		Capsule& operator=(Capsule const&) = default;
		Capsule& operator=(Capsule &&) = default;
		~Capsule() noexcept = default;

		Segment const& CentralSegment() const;
		Float32        SegmentHalfLength() const;
		Vec3		   Direction() const;

		void		   SetCentralSegment(Segment const& newSeg);
		void		   SetHalfLength(Float32 h);
		void		   SetDirection(Vec3 const& dir);

		DRB_SHAPE_INTERFACE();
	};


	// This assumes (but does not check):
	// -- local-space centroid is at the origin
	// -- no coplanar (or nearly coplanar) faces
	// -- halfedges are stored adjacent to their twin
	// -- face plane normals point outward
	// -- each face has counterclockwise winding of vertices
	// -- each vertex, edge, and face is unique
	// -- the hull is, indeed, a convex hull
	struct Convex
	{
		static constexpr auto type = ColliderType::Convex;

		// ------------------
		// Types & Constants
		// ------------------
		using IndexT = Uint8;
		using EdgeID = IndexT;
		using FaceID = IndexT;
		using VertID = IndexT;

		struct HalfEdge {
			EdgeID next = INVALID_INDEX;
			EdgeID twin = INVALID_INDEX;
			VertID origin = INVALID_INDEX;
			FaceID face = INVALID_INDEX;

			constexpr bool operator==(HalfEdge const&) const = default;
		};

		struct Face {
			EdgeID edge = INVALID_INDEX;
			Plane  plane = {};

			constexpr bool operator==(Face const&) const = default;
		};

	private:
		struct Header
		{
			Vec3 localCentroid;

			Int16 vertsOffset;
			Int16 vertAdjOffset;
			Int16 numVerts;

			Int16 edgesOffset;
			Int16 numEdges;

			Int16 facesOffset;
			Int16 numFaces;

			Int16 memUsed;

			unsigned char pad_[4];
		};

	public:
		static constexpr IndexT MAX_INDEX = std::numeric_limits<IndexT>::max() - 1;
		static constexpr IndexT MAX_SIZE = std::numeric_limits<IndexT>::max();
		static constexpr IndexT INVALID_INDEX = MAX_SIZE;

		// ------------------
		// Fields
		// ------------------
	private:
		Header*  data = nullptr;
		AABB     bounds{};        // in local space

	public:
		Vec3     position{};      // world space
		Quat     orientation{};   // world space

		// ------------------
		// Methods
		// ------------------
	public:
		// Constructors + Destructor
		Convex() = default;

		Convex(Vec3 const* verts, EdgeID const* vertAdj, VertID numVerts,
			   HalfEdge const* edges,                    EdgeID numHalfEdges,
			   Face const* faces,                        FaceID numFaces);

		Convex(Convex const& src);
		Convex& operator=(Convex const& other);

		Convex(Convex&& src) noexcept;
		Convex& operator=(Convex&& other) noexcept;

		~Convex() noexcept;

		// Access to centroid data
		inline Vec3		                 LocalCentroid() const;
		inline Vec3						 Centroid() const;
		inline void						 SetCentroid(Vec3 const& newC);

		// Indexed access to individual data elements
		inline Vec3 const&               GetVert(VertID index) const;
		inline EdgeID                    GetOneEdgeFrom(VertID index) const;
		inline VertID                    NumVerts() const;

		inline HalfEdge const&           GetEdge(EdgeID index) const;
		inline EdgeID                    NumHalfEdges() const;

		inline Face const&               GetFace(FaceID index) const;
		inline FaceID                    NumFaces() const;

		// Access to underlying arrays
		inline std::span<Vec3 const>     GetVerts() const;
		inline std::span<EdgeID const>   GetVertAdjs() const;
		inline std::span<HalfEdge const> GetEdges() const;
		inline std::span<Face const>     GetFaces() const;

		// Iteration methods
		void							 ForEachOneRingNeighbor(EdgeID start, std::invocable<HalfEdge> auto fn) const;
		void							 ForEachEdgeOfFace(FaceID face, std::invocable<HalfEdge> auto fn) const;
		
		// Situtational methods
		Polygon							 FaceAsPolygon(FaceID face, Mat4 const& tr = Mat4(1)) const;


		// "Maker" static methods
		static Convex                    MakeBox(Vec3 const& halfwidths, Mat4 const& transform = Mat4(1));
		static Convex                    MakeTetrahedron(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2, Vec3 const& p3, Mat4 const& transform = Mat4(1));

		DRB_SHAPE_INTERFACE();

	private:
		// Helpers
		void AllocateData(VertID numV, EdgeID numE, FaceID numF);

		inline Vec3* GetRawVerts();
		inline EdgeID* GetRawVertAdjs();
		inline HalfEdge* GetRawEdges();
		inline Face* GetRawFaces();
		inline Vec3 const* GetRawVerts() const;
		inline EdgeID const* GetRawVertAdjs() const;
		inline HalfEdge const* GetRawEdges() const;
		inline Face const* GetRawFaces() const;

		// Make sure that the offset type (Int16) is large enough for our maximum data
		static constexpr SizeT MAX_DATA_BYTES = Convex::MAX_SIZE * (sizeof(Vec3) + sizeof(Convex::EdgeID) + sizeof(Convex::HalfEdge) + sizeof(Convex::Face)) + sizeof(Convex::Header) + alignof(Vec3) + alignof(Convex::EdgeID) + alignof(Convex::HalfEdge) + alignof(Convex::Face) + alignof(Convex::Header);
		static_assert(Convex::MAX_DATA_BYTES < std::numeric_limits<Int16>::max());
	};


	static_assert(sizeof(Sphere) == 16);
	static_assert(sizeof(Capsule) == 32);
	static_assert(sizeof(Convex) == 64);


	template<typename T>
	concept Shape = std::same_as<T, Sphere> ||
		std::same_as<T, Capsule> ||
		std::same_as<T, Convex>;


	struct Collider
	{
	protected:
		ColliderType type = ColliderType::NONE;
		Float32 mass;
	public:
		Collider(ColliderType type, Float32 mass = 1.0f) : type{ type }, mass{ mass } {}

		inline ColliderType Type() const;
		inline Float32 Mass() const;

		template<class RetType, class Fn, class ... Args>
		RetType CallOnShape(Fn func, Args && ... args) const;

		DRB_SHAPE_INTERFACE();
	};


	template<Shape ShapeType>
	struct CollisionShape : public Collider
	{
		CollisionShape(ShapeType const& shape, Float32 mass = 1.0f)
			: Collider{ShapeType::type, mass}, shape{shape} 
		{}

		ShapeType shape;
	};


	struct CollisionGeometry
	{
		std::vector<CollisionShape<Sphere>>  spheres;
		std::vector<CollisionShape<Capsule>> capsules;
		std::vector<CollisionShape<Convex>>  hulls;

		Mat3    invInertia;
		Float32 invMass;
		Vec3    centerOfMass; // in local space
		AABB    bounds;
		Bool    locked;

		// Must be called by user after all colliders have been added. Computes 
		// the mass, center of mass, and local inertia tensor, plus a local space
		// bounding box.
		void Bake();

		CollisionGeometry& AddCollider(Shape auto&& shape, Float32 mass = 0.0f);

		CollisionGeometry& AddCollider(Shape auto const& shape, Float32 mass = 0.0f);

		template<class Fn>
		void ForEachCollider(Fn fn);

		template<class Fn>
		void ForEachCollider(Fn fn) const;

		inline Int32 Size() const;
		inline void  Reserve(Int32 newCap);
	};
}

#undef DRB_SHAPE_INTERFACE

#include "CollisionGeometry.inl"
#endif
