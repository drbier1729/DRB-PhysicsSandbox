#ifndef DRB_COLLISIONGEOMETRY_H
#define DRB_COLLISIONGEOMETRY_H

#include "AABB.h"
#include "GeometryPrimitives.h"

// Pseudo-polymorphic shape interface
#define DRB_SHAPE_INTERFACE() \
inline Mat3 InertiaTensorAbout(Vec3 const& pt) const; \
inline Vec3 Position() const; \
inline void SetPosition(Vec3 const& newPosition); \
inline Vec3 Centroid() const; \
inline void SetCentroid(Vec3 const& newCentroid); \
inline Quat Orientation() const; \
inline void SetOrientation(Quat const& newOrientation); \
inline Mat4 Transform() const; \
inline void SetTransform(Mat4 const& newTransform); \
inline AABB Bounds(Mat4 const& worldTransform = Mat4(1)) const; \

namespace drb::physics {

	// -------------------------------------------------------------------------
	// Collision Shape Types
	// -------------------------------------------------------------------------
	
	enum class ColliderType
	{
		NONE = 0,
		Sphere = 1,
		Capsule = 2,
		Box = 3,
		Convex = 4
	};

	struct Sphere
	{
		friend struct ColliderSettings;
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
		friend struct ColliderSettings;
		static constexpr auto type = ColliderType::Capsule;

	private:
		Segment seg; // world space central segment
		Float32 h;   // half-length of central segment

	public:
		Float32 r;   // radius

	public:
    	Capsule() = default;
		Capsule(Segment const& centralSeg, Float32 radius);
		Capsule(Quat const& orientation, Vec3 pos, Float32 segmentHalfLength, Float32 radius);
		Capsule(Mat3 const& orientation, Vec3 pos, Float32 segmentHalfLength, Float32 radius);


		Segment const& CentralSegment() const;
		Float32        SegmentHalfLength() const;
		Vec3		   Direction() const;

		void		   SetCentralSegment(Segment const& newSeg);
		void		   SetHalfLength(Float32 h);
		void		   SetDirection(Vec3 const& dir);

		DRB_SHAPE_INTERFACE();
	};


	// Oriented-Bounding Box data structure
	struct Box
	{
		friend struct ColliderSettings;
		static constexpr auto type = ColliderType::Box;

		enum class Face
		{
			Right  = 0, // +x direction in local space
			Top    = 1, // +y 
			Front  = 2, // +z
			Left   = 3, // -x
			Bottom = 4, // -y
			Back   = 5, // -z
			COUNT  = 6
		};

		Vec3 extents;     // halfwidth extents
		Vec3 position;    // center in world space
		Mat3 orientation; // orientation in world space

	private:
		char _pad[4];     // padding (can be used for something else...)

	public:
		Box() = default;
		Box(Vec3 const& halfwidths, Vec3 const& position = Vec3(0), Mat3 const& orientation = Mat3(1));
		Box(Vec3 const& halfwidths, Vec3 const& position, Quat const& orientation);
		
		// Return values are in local space
		Polygon FaceAsPolygon(Face face) const;
		Vec3    FaceNormal(Face face) const;
		Face    SupportingFace(Vec3 const& localDir) const;
		Segment SupportingEdge(Vec3 const& localDir) const;
		Segment SupportingEdgeWithDirection(Int32 axisDir, Vec3 const& localDir) const;
		Segment EdgeAsSegment(Face adjFace1, Face adjFace2) const;

		DRB_SHAPE_INTERFACE();
	};


	// This assumes (but does not check):
	// -- no coplanar (or nearly coplanar) faces
	// -- halfedges are stored adjacent to their twin
	// -- face plane normals point outward
	// -- each face has counterclockwise winding of vertices
	// -- each vertex, edge, and face is unique
	// -- the hull is, indeed, a convex hull
	// -- vertices have been sensibly "merged"
	struct Convex
	{
		friend struct ColliderSettings;
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
			HalfEdge const* edges, EdgeID numHalfEdges,
			Face const* faces, FaceID numFaces);

		Convex(Convex const& src);
		Convex& operator=(Convex const& other);

		Convex(Convex&& src) noexcept;
		Convex& operator=(Convex&& other) noexcept;

		~Convex() noexcept;

		// Access to centroid data
		inline Vec3		                 LocalCentroid() const;

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

		inline Vec3 const*               GetRawVerts() const;
		inline EdgeID const*             GetRawVertAdjs() const;
		inline HalfEdge const*           GetRawEdges() const;
		inline Face const*               GetRawFaces() const;

		// Iteration methods
		void							 ForEachOneRingNeighbor(EdgeID start, std::invocable<HalfEdge> auto&& fn) const;
		void							 ForEachEdgeOfFace(FaceID face, std::invocable<HalfEdge> auto&& fn) const;
		
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

		// Make sure that the offset type (Int16) is large enough for our maximum data
		static constexpr SizeT MAX_DATA_BYTES = Convex::MAX_SIZE * (sizeof(Vec3) + sizeof(Convex::EdgeID) + sizeof(Convex::HalfEdge) + sizeof(Convex::Face)) + sizeof(Convex::Header) + alignof(Vec3) + alignof(Convex::EdgeID) + alignof(Convex::HalfEdge) + alignof(Convex::Face) + alignof(Convex::Header);
		static_assert(Convex::MAX_DATA_BYTES < std::numeric_limits<Int16>::max());
	};

	static_assert(sizeof(Sphere) == 16);
	static_assert(sizeof(Capsule) == 32);
	static_assert(sizeof(Box) == 64);
	static_assert(sizeof(Convex) == 64);

	// -------------------------------------------------------------------------
	// Concepts
	// -------------------------------------------------------------------------
	template<typename T>
	concept Shape = std::same_as<T, Sphere> ||
		std::same_as<T, Capsule> ||
		std::same_as<T, Box> ||
		std::same_as<T, Convex>;

	template<typename Fn>
	concept ConstShapeFn = std::invocable<Fn, Sphere const&>&&
		std::invocable<Fn, Capsule const&>&&
		std::invocable<Fn, Box const&>&&
		std::invocable<Fn, Convex const&>;

	template<typename Fn>
	concept ShapeFn = std::invocable<Fn, Sphere&>&&
		std::invocable<Fn, Capsule&>&&
		std::invocable<Fn, Box&>&&
		std::invocable<Fn, Convex&>;


	// -------------------------------------------------------------------------
	// Pseudo-polymorphic access
	// -------------------------------------------------------------------------
	using ConstShapePtr = std::variant<Sphere const*, Capsule const*, Box const*, Convex const*>;
	

	// -------------------------------------------------------------------------
	// Pseudo-polymorphic construction
	// -------------------------------------------------------------------------
	struct ColliderSettings
	{
		friend struct CollisionGeometry;

		SizeT        index = 0;
		ColliderType type = ColliderType::NONE;
		Float32	     relativeMass = 0.0f;
		Vec3		 centroid = {};
	};


	// -------------------------------------------------------------------------
	// Assembly of Colliders
	// -------------------------------------------------------------------------
	struct CollisionGeometry : public std::enable_shared_from_this<CollisionGeometry>
	{
		friend class World;
		#ifdef _DEBUG
		friend class DebugRenderer;
		#endif

	private:
		std::vector<Sphere>			  spheres{};
		std::vector<Capsule>		  capsules{};
		std::vector<Box>			  boxes{};
		std::vector<Convex>			  hulls{};
		std::vector<ColliderSettings> settings{};

		Mat3    invInertia{0.0f};
		Float32 invMass{ 0.0f };
		Vec3    centerOfMass{ 0.0f }; // in local space
		AABB    bounds{};
		Bool    locked = false;

	public:		
		// Accessors
		void               ForEachCollider(ConstShapeFn auto&& fn) const;
					      
		inline Mat3 const& InverseInertia() const;
		inline Float32     InverseMass() const;
		inline Vec3 const& CenterOfMass() const;
		inline AABB const& Bounds() const;
		inline SizeT       Size() const;

		// Manipulators
		inline void        Reserve(SizeT newCapacities);

		CollisionGeometry& AddCollider(Shape auto const& collider, Float32 mass = 0.0f);
		CollisionGeometry& AddCollider(Shape auto && collider, Float32 mass = 0.0f);
		void			   Bake(); // Must be called after all colliders have been added

	private:
		void ForEachCollider(ShapeFn auto&& fn);
	};
	
}

#undef DRB_SHAPE_INTERFACE

#include "CollisionGeometry.inl"
#endif
