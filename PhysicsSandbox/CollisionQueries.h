#ifndef DRB_COLLISIONQUERIES_H
#define DRB_COLLISIONQUERIES_H

#include "CollisionGeometry.h"
#include "GeometryQueryDataStructures.h"

namespace drb::physics {

	// Top level collision function
	ContactManifold Collide(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);

	// Sub functions which the user may want to call. The tr arguments allow
	// the caller to specify additional transformations applied to the shapes.

	// NOTE: These do not work right now! They should return normal in world space, not local space
	// of A
	ContactManifold Collide(Sphere const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB);
	ContactManifold Collide(Sphere const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
	ContactManifold Collide(Sphere const& A, Mat4 const& trA, Box const& B, Mat4 const& trB);
	ContactManifold Collide(Sphere const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
	
	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Box const& B, Mat4 const& trB);
	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
	
	ContactManifold Collide(Box const& A, Mat4 const& trA, Box const& B, Mat4 const& trB);
	ContactManifold Collide(Box const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);

	// This is the only one that works
	ContactManifold Collide(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
}

#endif
