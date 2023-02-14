#ifndef DRB_PHYSICSGEOMETRYQUERIES_H
#define DRB_PHYSICSGEOMETRYQUERIES_H

#include "PhysicsGeometry.h"
#include "GeometryQueryDataStructures.h"

namespace drb {
	namespace physics {

		// ---------------------------------------------------------------------
		// COLLISION FUNCTIONS
		// ---------------------------------------------------------------------

		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB);

		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB);

		ContactManifold Collide(Convex const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
		ContactManifold Collide(Convex const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
		ContactManifold Collide(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
		ContactManifold Collide(Convex const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB);

		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB); // not implemented - asserts

		// ---------------------------------------------------------------------
		// OTHER QUERIES
		// ---------------------------------------------------------------------
		
		inline CastResult      RayCast(Ray const& r, AABB const& aabb);

		inline Bool            Intersect(Segment const& seg, Plane const& plane, float& t, Vec3& q);
		
		inline Side			   ClassifyPointToPlane(Vec3 const& p, Plane const& plane);
		
		inline Float32         SignedDistance(Vec3 const& point, Plane const& plane);
	}
}

#include "PhysicsGeometryQueries.inl"

#endif
