#ifndef DRB_GEOMETRYQUERYDATASTRUCTURES_H
#define DRB_GEOMETRYQUERYDATASTRUCTURES_H

namespace drb::physics {
	// fwd decls
	class RigidBody;
	struct CollisionShapeBase;
	struct Ray;

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
		Type  type = Type::NONE;
	};


	enum class Side
	{
		Back = -1,
		On = 0,
		Front = 1
	};


	struct Contact
	{
		Vec3    position = {};	 // in world coordinates
		Float32 penetration = std::numeric_limits<Float32>::lowest(); // greater than 0 indicates overlap

		// For sequential impulses
		Float32 impulseN = 0.0f, impulseT = 0.0f; // normal and tangent impulses
		Float32 massN = 0.0f, massT = 0.0f;		  // normal and tangent effective masses
		Float32 bias = 0.0f;
	};


	struct ContactManifold
	{
		static constexpr Uint32 MAX_CONTACTS = 4u;

		Feature featureA{}, featureB{}; // either face or edge or vertex ... TODO : should be tracked per Contact?
		Contact contacts[MAX_CONTACTS] = {};
		Uint32  numContacts = 0;
		Vec3    normal{}; // Always points from object A toward object B. Unit length.
	};


	// Used as a key to uniquely identify a contact manifold
	struct ManifoldKey
	{
		RigidBody* a, * b;
		CollisionShapeBase const* aShape, * bShape;

		ManifoldKey(RigidBody* a_, CollisionShapeBase const* aShape_, RigidBody* b_, CollisionShapeBase const* bShape_);
	};
	inline constexpr bool operator<(ManifoldKey const& A, ManifoldKey const& B) {
		if (A.a < B.a) { return true; }
		if (A.a == B.a && A.b < B.b) { return true; }
		if (A.a == B.a && A.b == B.b && A.aShape < B.aShape) { return true; }
		if (A.a == B.a && A.b == B.b && A.aShape == B.aShape && A.bShape < B.bShape) { return true; }
		return false;
	}


	struct ClosestPointsQuery {
		Vec3 ptA = Vec3(NAN);							  // pt on shape a in world space
		Vec3 ptB = Vec3(NAN);							  // pt on shape b in world space
		Float32 d2 = std::numeric_limits<Float32>::max(); // sqr distance
	};


	struct CastResult
	{
		Vec3    point = Vec3(std::numeric_limits<Float32>::max());
		Float32 distance = std::numeric_limits<Float32>::max();
		Bool	hit = false;
	};


	struct RayCastHit
	{
		CastResult info = {};
		RigidBody* body = nullptr;
		CollisionShapeBase const* shape = nullptr;
	};
}

#endif
