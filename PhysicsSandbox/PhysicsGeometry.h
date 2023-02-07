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

			static Float32 thickness;
		};
		inline Plane MakePlane(Vec3 const& normal, Vec3 const& point);
		inline Plane MakePlane(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2); // assumes points are ordered counter-clockwise
		inline Plane Transformed(Plane const& plane, Mat4 const& transform);


		struct Polygon
		{
			// oriented counterclockwise
			std::vector<Vec3> verts;
		};
		void SplitPolygon(Polygon const& poly, Plane const& plane, Polygon& frontPoly, Polygon& backPoly);


		struct Segment
		{
			Vec3 b = Vec3(0),     // begin point
				 e = Vec3(1,0,0); // end point
		};
		inline Segment Transformed(Segment const& seg, Mat4 const& transform);



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


		// ---------------------------------------------------------------------
		// IDENTIFIERS
		// ---------------------------------------------------------------------

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
		inline Segment CentralSegment(Capsule const& cap, Mat4 const& tr);
		inline AABB MakeAABB(Capsule const& cap, Mat4 const& tr);


		// assume...
		// - centroid is at the origin in local space
		// - closed and convex (i.e. this is a polyhedron)
		// - no coplanar faces
		struct Convex 
		{
			struct HalfEdge {
				Uint8 next = MAX_EDGES;
				Uint8 twin = MAX_EDGES;
				Uint8 origin = MAX_EDGES;
				Uint8 face = MAX_EDGES;

				constexpr bool operator==(HalfEdge const&) const = default;
			};

			struct Face {
				Uint8 edge = MAX_EDGES;
				Plane plane = {};

				constexpr bool operator==(Face const&) const = default;
			};

			AABB					 bounds{};   // AABB in local space -- used to quickly recompute AABB/OBB in world space
			std::vector<Vec3>		 verts{};    // in local space
			std::vector<HalfEdge>	 edges{};    // stored s.t. each edge is adjacent to its twin
			std::vector<Face>		 faces{};

			static constexpr auto MAX_EDGES = std::numeric_limits<Uint8>::max();
			static constexpr auto type = ColliderType::Convex;
		};
		inline Convex  MakeBox(Vec3 const& halfwidths);
		inline Convex  MakeTetrahedron(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2, Vec3 const& p3); // note that this will shift the points s.t. the centroid is at (0,0,0)
		inline Mat3    ComputeInertiaTensor(Convex const& hull);
		inline void    ForEachOneRingNeighbor(Convex const& hull, Convex::HalfEdge const& start, std::invocable<Convex::HalfEdge> auto fn);
		inline AABB    MakeAABB(Convex const& hull, Mat4 const& tr);
		inline Polygon FaceAsPolygon(Convex const& hull, Mat4 const& tr, Convex::Face const& face);


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


		struct CollisionShapeBase {
			Mat4         transform = Mat4(1);
			Float32      mass = 1.0f;
			ColliderType type = ColliderType::NONE;
		};


		template<Shape ShapeType>
		struct CollisionShape final : public CollisionShapeBase
		{
			CollisionShape(ShapeType const& shape, Mat4 const& transform, Float32 mass = 1.0f);
			ShapeType shape;
		};

		struct CollisionGeometry final
		{
			std::vector<CollisionShape<Sphere>>  spheres    = {};
			std::vector<CollisionShape<Capsule>> capsules   = {};
			std::vector<CollisionShape<Convex>>  hulls      = {};
			Mat3								 invInertia = Mat3(1);
			Float32                              invMass    = 1.0f;
			Vec3								 com        = {};

			// -----------------------------------------------------------------
			// Setup Methods
			// -----------------------------------------------------------------

			// must be called by user after all colliders have been added. computes 
			// the mass, center of mass, and local inertia tensor. This will also
			// set the local position of each collider such that the center of 
			// mass of the body is at the origin, but saves the coords of the COM
			// in the original model space.
			void Bake();

			template<Shape T>
			CollisionGeometry& AddCollider(T&& shape, CollisionShapeBase&& options = {});

			template<Shape T>
			CollisionGeometry& AddCollider(T const& shape, CollisionShapeBase const& options = {});

			template<class Fn>
			void ForEachCollider(Fn fn);

			template<class Fn>
			void ForEachCollider(Fn fn) const;
		};
	}
}

#include "PhysicsGeometry.inl"
#endif 

