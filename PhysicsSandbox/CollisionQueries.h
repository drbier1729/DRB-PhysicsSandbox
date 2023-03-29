#ifndef DRB_COLLISIONQUERIES_H
#define DRB_COLLISIONQUERIES_H

#include "CollisionGeometry.h"
#include "GeometryQueryDataStructures.h"

namespace drb::physics {

	// Top level collision function
	ContactManifold Collide(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);

	// Sub functions which the user may want to call. The disp and tr arguments allow
	// the caller to specify additional transformations applied to the shapes.
	ContactManifold Collide(Sphere const& A, Vec3 const& dispA, Sphere const& B, Vec3 const& dispB);
	ContactManifold Collide(Sphere const& A, Vec3 const& dispA, Capsule const& B, Mat4 const& trB);
	ContactManifold Collide(Sphere const& A, Vec3 const& dispA, Box const& B, Mat4 const& trB);
	ContactManifold Collide(Sphere const& A, Vec3 const& dispA, Convex const& B, Mat4 const& trB);
	
	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB);
	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Box const& B, Mat4 const& trB);
	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
	
	ContactManifold Collide(Box const& A, Mat4 const& trA, Box const& B, Mat4 const& trB);
	ContactManifold Collide(Box const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);

	ContactManifold Collide(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
}

#endif
