//#ifndef DRB_PHYSICSGEOMETRYQUERIES_H
//#define DRB_PHYSICSGEOMETRYQUERIES_H
//
//#include "CollisionGeometry.h"
//#include "GeometryQueryDataStructures.h"
//
//namespace drb {
//	namespace physics {
//
//		// ---------------------------------------------------------------------
//		// COLLISION FUNCTIONS
//		// ---------------------------------------------------------------------
//
//		// Top level collision function -- casts the argument shapes to the appropriate
//		// types then calls one of the overloads below. Note that since CollisionShapeBase
//		// holds a local transform, the trA and trB arguments will be multiplied into
//		// A.transform and B.transform.
//		ContactManifold Collide(CollisionShapeBase const& A, Mat4 const& trA, CollisionShapeBase const& B, Mat4 const& trB);
//
//		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
//		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
//		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
//		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB);
//
//		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
//		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
//		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
//		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB);
//
//		ContactManifold Collide(Convex const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
//		ContactManifold Collide(Convex const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
//		ContactManifold Collide(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
//		ContactManifold Collide(Convex const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB);
//
//		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
//		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
//		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
//		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB); // not implemented - asserts
//
//		// ---------------------------------------------------------------------
//		// OTHER QUERIES
//		// ---------------------------------------------------------------------
//
//		inline Bool            Intersect(Segment const& seg, Plane const& plane, float& t, Vec3& q);
//		
//		inline Side			   ClassifyPointToPlane(Vec3 const& p, Plane const& plane);
//		
//		inline Float32         SignedDistance(Vec3 const& point, Plane const& plane);
//	}
//}
//
//#include "PhysicsGeometryQueries.inl"
//
//#endif
