#ifndef DRB_SATGJK_H
#define DRB_SATGJK_H

#include "GeometryQueryDataStructures.h"
#include "CollisionGeometry.h"

// Note that none of these functions enquire about the local transform
// of the shapes -- it is assumed that their world space transform is
// described entirely by their vertex data and the matrices passed
// in as arguments.
namespace drb::physics::util {

	// -----------------------------------------------------------------
	// Separating Axis Test
	// -----------------------------------------------------------------
	struct FaceQuery {
		Float32 separation = std::numeric_limits<Float32>::lowest();
		Vec3    normal = {};
		Int16   index = -1;
	};

	struct EdgeQuery {
		Float32 separation = std::numeric_limits<Float32>::lowest();
		Vec3	normal = {};
		Int16   indexA = -1;
		Int16   indexB = -1;
	};

	// Note that these need to be called sequentially for the full SAT: 
	//	QueryFace(A, B)
	//	QueryFace(B, A)
	//	QueryEdge(A, B)
	FaceQuery SATQueryFaceDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
	EdgeQuery SATQueryEdgeDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);

	// This is a check for a single axis -- could have been the cached separating axis
	// from a previous SAT query. Axis must be in world space and oriented pointing A->B
	Float32	  SeparationOnAxis(Vec3 const& axis, Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);


	// -----------------------------------------------------------------
	// GJK
	// -----------------------------------------------------------------

	// Should be private to GJK algorithm but can be cached. Don't call any of the member
	// functions outside GJK computations
	struct Simplex
	{
		enum class Type : Uint8 {
			NONE = 0,
			Pnt = 1,
			Seg = 2,
			Tri = 3,
			Tet = 4
		};

		Vec3 vertsA[4] = {};   // world space positions of simplex points on A
		Vec3 vertsB[4] = {};   // world space positions of simplex points on B

		Vec4 lambdas = {};

		// Used for hill climbing. Edges give more info
		// than verts since each edge has an origin vert.
		Convex::EdgeID edgeIdxsA[4] = {};
		Convex::EdgeID edgeIdxsB[4] = {};
		Convex::EdgeID bestA = Convex::INVALID_INDEX, 
					   bestB = Convex::INVALID_INDEX;
				
		Uint8 size = 0;
		Bool  containsOrigin = false;

		inline Type GetType() const;
		inline void PushVert(Vec3 const& vA, Convex::EdgeID eIdxA, Vec3 const& vB, Convex::EdgeID eIdxB);
		inline void ExtractWitnessPoints(Vec3& witnessA, Vec3& witnessB) const;
		void ComputeBarycentricCoords(Vec3 const& disp);
	};


	// A support function takes an object and a given direction (in the local
	// space of the object) as args, and returns the most extreme point of
	// the object in that direction.
	template<class Shape>
	using SupportFn = std::function<Vec3(Shape const&, Vec3 const&)>;


	// For all GJK tests below, an optional parameter "seed" can be passed in to 
	// accelerate the routine. If seed is not nullptr, then its data will be set by
	// the function s.t. it can be passed in again on the next call for these
	// two objects. This is only helpful when at least one object is of type Convex
	// because it relies on vertex IDs to build a new simplex based on the old
	// simplex's features.


	// General GJK intersection test which can be called for custom shape types.
	// Returns the distance between shapes A and B. If the result is negative, the 
	// value is not meaningful except to indicate that the objects are intersecting.
	template<class ShapeA, class ShapeB>
	ClosestPointsQuery GJK(ShapeA const& A, Mat4 const& trA, SupportFn<ShapeA> supportFunctionA,
				            ShapeB const& B, Mat4 const& trB, SupportFn<ShapeB> supportFunctionB,
							Simplex * seed = nullptr);
			
	// Specialized GJK: Convex-Convex -- NOT IMPLEMENTED
	ClosestPointsQuery GJK(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB, Simplex* seed = nullptr);

	// Specialized GJK: Point-Convex -- PARTIALLY IMPLEMENTED
	// Issues:
	// - No S3D method yet
	// - Witness points "jump" sometimes 
	//		repro: use sphere against tet or capsule against box and move in the y direction.
	//		the witness point will "jump" to hull.verts[ hull.edges[0] ] when the sphere/cap
	//		is about halfway up the side of the convex hull
	ClosestPointsQuery GJK(Vec3 const& A, Convex const& B, Mat4 const& trB, Simplex* seed = nullptr);

	// Specialized GJK: Segment-Convex
	ClosestPointsQuery GJK(Segment const& A, Convex const& B, Mat4 const& trB, Simplex* seed = nullptr);
		
}

#include "SATGJK.inl"

#endif