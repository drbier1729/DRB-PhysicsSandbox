#ifndef DRB_PHYSICSGEOMETRYQUERIES_H
#define DRB_PHYSICSGEOMETRYQUERIES_H

#include "PhysicsGeometry.h"

namespace drb {
	namespace physics {

		// fwd decls
		class RigidBody;

		// ---------------------------------------------------------------------
		// DATA STRUCTURES
		// ---------------------------------------------------------------------

		// Used to identify which face, edge, or vertex of colliding objects are
		// in contact. This is really only relevant for Convex and Mesh. For
		// Sphere and Capsule, we just always use 0 as the FeatureID.
		struct Feature
		{
			enum class Type : Uint16 {
				NONE = 0,
				Face = 1,
				Edge = 2,
				Vert = 3
			};

			Int16 index = -1;
			Type  type  = Type::NONE;
		};


		enum class Side
		{
			Back = -1,
			On = 0,
			Front = 1
		};


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


		// Used as a key to uniquely identify a contact manifold
		struct ManifoldKey
		{
			RigidBody* a, * b;
			CollisionShapeBase* aShape, * bShape;

			ManifoldKey(RigidBody* a_, CollisionShapeBase* aShape_, RigidBody* b_, CollisionShapeBase* bShape_);
		};


		struct RayCastHit
		{
			Ray::CastResult           info = {};
			RigidBody*			      body = nullptr;
			CollisionShapeBase const* shape = nullptr;
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
	

		// ---------------------------------------------------------------------
		// OTHER QUERIES
		// ---------------------------------------------------------------------
		
		inline Ray::CastResult RayCast(Ray const& r, AABB const& aabb);

		inline Bool            Intersect(Segment const& seg, Plane const& plane, float& t, Vec3& q);
		
		inline Side			   ClassifyPointToPlane(Vec3 const& p, Plane const& plane);
		
		inline Float32         SignedDistance(Vec3 const& point, Plane const& plane);
	}
}

#include "PhysicsGeometryQueries.inl"

#endif
