#ifndef DRB_PHYSICSGEOMETRY_H
#define DRB_PHYSICSGEOMETRY_H

#include "Math.h"
#include "AABB.h"

namespace drb {
	namespace physics {
		
		// ---------------------------------------------------------------------
		// PRIMITIVES
		// ---------------------------------------------------------------------

		struct Plane
		{
			Vec3    n = Vec3(1,0,0); // unit normal
			Float32 d = 0.0f;        // signed distance

			constexpr bool operator==(Plane const&) const = default;

			// point p is on the plane if dot(p, n) - d = 0
		};
		inline Plane MakePlane(Vec3 const& normal, Vec3 const& point);
		inline Plane MakePlane(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2); // assumes points are ordered counter-clockwise
		inline Float32 SignedDistance(Vec3 const& point, Plane const& plane);
		inline Plane GetTransformed(Plane const& plane, Mat4 const& transform);



		struct Segment
		{
			Vec3 b = Vec3(0),     // begin point
				 e = Vec3(1,0,0); // end point
		};



		struct Ray
		{
			Vec3 p = Vec3(0),		// origin point
				 d = Vec3(1, 0, 0); // (normalized) direction

			struct CastResult
			{
				Vec3    point    = Vec3(std::numeric_limits<Float32>::max());
				Float32 distance = std::numeric_limits<Float32>::max();
				Bool	hit      = false;
			};
		};
		inline Ray::CastResult RayCast(Ray const& r, AABB const& aabb);


		//void SplitPolygonSH(Polygon& poly, Plane plane, Polygon** frontPoly, Polygon** backPoly);



		// ---------------------------------------------------------------------
		// IDENTIFIERS
		// ---------------------------------------------------------------------
		
		// Used to identify which face, edge, or vertex of colliding objects are
		// in contact. This is really only relevant for Convex and Mesh. For
		// Sphere and Capsule, we just always use 0 as the FeatureID.
		struct Feature
		{
			enum class Type : unsigned {
				Face = 0,
				Edge = 1,
				Vert = 2
			};

			unsigned index = 0;
			Type     type = Type::Face;
		};



		enum class ColliderType
		{
			NONE = 0,
			Sphere = 1,
			Capsule = 2,
			Convex = 3,
			Mesh = 4
		};


		// ---------------------------------------------------------------------
		// COLLIDERS
		// ---------------------------------------------------------------------

		struct Sphere
		{
			Float32 r = 1.0f; // radius

			static constexpr auto type = ColliderType::Sphere;
		};
		inline Mat3 ComputeInertiaTensor(Sphere const& sph);
		inline AABB MakeAABB(Sphere const& sph, Mat4 const& tr);


		
		struct Capsule // assume oriented vertically with y up by default
		{
			Float32 r = 1.0f; // radius
			Float32 h = 0.5f; // half-length of central segment

			static constexpr auto type = ColliderType::Capsule;
		};
		inline Mat3 ComputeInertiaTensor(Capsule const& cap);
		inline Segment GetCentralSegment(Capsule const& cap, Mat4 const& tr);
		inline AABB MakeAABB(Capsule const& cap, Mat4 const& tr);


		
		struct Convex // assume origin in local space is at centroid
		{
			struct HalfEdge {
				Uint8 next = 0;
				Uint8 twin = 0;
				Uint8 origin = 0;
				Uint8 face = 0;

				constexpr bool operator==(HalfEdge const&) const = default;
			};

			struct Face {
				Uint8 edge = 0;
				Plane plane = {};

				constexpr bool operator==(Face const&) const = default;
			};

			AABB					 bounds{}; // AABB in local space -- used to quickly recompute AABB/OBB in world space
			std::vector<Vec3>		 verts{};  // in local space
			std::vector<HalfEdge>	 edges{};  // stored s.t. each edge is adjacent to its twin
			std::vector<Face>		 faces{};

			static constexpr auto MAX_EDGES = std::numeric_limits<Uint8>::max();
			static constexpr auto type = ColliderType::Convex;
		};
		inline Convex MakeBox(Vec3 const& halfwidths);
		inline Convex MakeTetrahedron(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2, Vec3 const& p3);
		inline Mat3   ComputeInertiaTensor(Convex const& hull);
		inline AABB   MakeAABB(Convex const& hull, Mat4 const& tr);



		struct Mesh
		{
			Bool isHeightfield = false;

			static constexpr auto type = ColliderType::Mesh;
		};
		AABB MakeAABB(Mesh const& mesh, Mat4 const& tr);



		template<typename T>
		concept Shape = std::same_as<T, Sphere> ||
			std::same_as<T, Capsule> ||
			std::same_as<T, Convex> ||
			std::same_as<T, Mesh>;



		template<Shape ShapeType>
		struct CollisionShape
		{
			ShapeType shape{};
			Mat4      transform = Mat4(1);
			Float32   mass = 1.0f;
		};
	}
}

#include "PhysicsGeometry.inl"
#endif 

