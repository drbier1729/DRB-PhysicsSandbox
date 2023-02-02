#ifndef DRB_PHYSICSGEOMETRYQUERIES_H
#define DRB_PHYSICSGEOMETRYQUERIES_H

#include "PhysicsGeometry.h"

namespace drb {
	namespace physics {


		// ---------------------------------------------------------------------
		// DATA STRUCTURES
		// ---------------------------------------------------------------------

		struct Contact 
		{
			Feature featureA = {}, featureB = {};

			Vec3    position = {};	 // in world coordinates
			Float32 penetration = std::numeric_limits<Float32>::lowest(); // greater than 0 indicates overlap

			Float32 impulseN = 0.0f, impulseT = 0.0f; // normal and tangent impulses
			Float32 massN = 0.0f, massT = 0.0f;		  // normal and tangent effective masses
			Float32 bias = 0.0f;
		};

		struct ContactManifold 
		{
			static constexpr Uint32 MAX_CONTACTS = 4u;

			Contact contacts[MAX_CONTACTS] = {};
			Uint32  numContacts = 0;
			Vec3    normal = {}; // Always points from object A toward object B. Unit length.
		};


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
	}
}


#endif
