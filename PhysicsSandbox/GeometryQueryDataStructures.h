#ifndef DRB_GEOMETRYQUERYDATASTRUCTURES_H
#define DRB_GEOMETRYQUERYDATASTRUCTURES_H


#include "CollisionGeometry.h"
#include "Constraints.h"

namespace drb::physics {
	// Fwd decls
	class RigidBody;
	
	// ---------------------------------------------------------------------
	// DATA STRUCTURES
	// ---------------------------------------------------------------------

	enum class Side
	{
		Back = -1,
		On = 0,
		Front = 1
	};

	// Used to identify which face, edge, or vertex of colliding objects are
	// in contact. This is really only relevant for Box and Convex. For
	// Sphere and Capsule, we just always use 0 as the Feature.
	struct Feature
	{
		enum class Type : Uint16 {
			NONE = 0,
			Face = 1,
			Edge = 2,
			Vert = 3
		};

		Int16 index = 0;
		Type  type = Type::NONE;

		// These are only relevant for comparing features of the same geometric object
		friend constexpr bool operator==(Feature const&, Feature const&) = default;
		friend constexpr auto operator<=>(Feature const&, Feature const&) = default;
	};

	struct FeaturePair
	{
		Feature fA, fB;
		friend constexpr bool operator==(FeaturePair const&, FeaturePair const&) = default;
		friend constexpr auto operator<=>(FeaturePair const&, FeaturePair const&) = default;
	};


	struct Contact
	{
		Vec3 position = {};	 // in world coordinates
		Real penetration = std::numeric_limits<Real>::lowest(); // greater than 0 indicates overlap

		// For sequential impulses
		Real impulseN = 0.0_r, impulseT = 0.0_r; // normal and tangent impulses
		Real massN = 0.0_r, massT = 0.0_r;		  // normal and tangent effective masses
		Real bias = 0.0_r;
	};


	struct ContactManifold_OLD
	{
		static constexpr Uint32 MAX_CONTACTS = 8u;

		Feature featureA{}, featureB{}; // either face or edge or vertex ... TODO : should be tracked per Contact?
		Contact contacts[MAX_CONTACTS] = {};
		Uint32  numContacts = 0;
		Vec3    normal{}; // Always points from object A toward object B. Unit length.
	};


	struct ContactManifold
	{
		static constexpr Uint32 MAX_CONTACTS = 8u;
		using ContactArray = std::array<CollisionConstraint, MAX_CONTACTS>;
		using RealArray = std::array<Real, MAX_CONTACTS>;
		
		RigidBody*   rbA = nullptr, * rbB = nullptr;
		ContactArray contacts = {};
		Int32        numContacts = 0;
		Vec3         normal{}; // Always points from object A toward object B. Unit length. World space.
	};


	struct CollisionProxy
	{
		Int64			bvHandle     = -1;
		RigidBody*      rb           = nullptr;
		ConstShapePtr   shape        = {};

		auto operator<=>(CollisionProxy const& other) const = default;
	};
	

	// Pair of potentially colliding objects. Used as a key to uniquely identify a contact manifold
	struct CollisionPair
	{
		CollisionProxy a, b;
		CollisionPair(CollisionProxy const& a, CollisionProxy const& b);
			
		auto operator<=>(CollisionPair const& other) const = default;
	};


	struct ClosestPointsQuery {
		Vec3 ptA = Vec3(std::numeric_limits<Real>::max());	// pt on shape a in world space
		Vec3 ptB = Vec3(std::numeric_limits<Real>::max());	// pt on shape b in world space
		Real d2  = std::numeric_limits<Real>::max();		// sqr distance.  (d2 < 0)  -->  A and B intersecting
	};
}

#endif
