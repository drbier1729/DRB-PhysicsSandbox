#include "pch.h"
#include "SATGJK.h"

#include "PhysicsGeometry.h"
#include "PhysicsGeometryQueries.h"

namespace drb::physics::util {
	
	// -------------------------------------------------------------------------
	// Helpers
	// -------------------------------------------------------------------------
	
	// Return value for GetSupport___ functions
	struct CvxSupport
	{
		Vec3 pt;
		Convex::HalfEdge e;
	};

	// Return value for NearestSimplex
	struct GJKWorkingPair
	{
		Simplex simplex{};
		Vec3 searchDir{};
	};

	// If this uses hill climbing, returns the same as GetSupportHillClimbing, else returns a support point
	// and the "e" field is left defaulted. Start hint is an optional argument specifying where hill climbing
	// should begin. Note that startHint is assumed to be a valid edge from hull, but this is not checked!
	static inline CvxSupport GetSupport(Convex const& hull, Vec3 const& dir, Convex::HalfEdge startHint = {});

	// Returns support point in local space and an edge with the support point as its origin. See comment for
	// "GetSupport" for more details.
	static inline CvxSupport GetSupportHillClimbing(Convex const& hull, Vec3 const& dir, Convex::HalfEdge startHint);
	
	// Computes the signed distance from plane to the closest point on the hull
	static inline Float32 Project(const Plane& plane, const Convex& hull);
	
	// Does some Dirk Gregorius magic -- see his GDC talk on SAT
	static inline Float32 Project(const Vec3& p1, const Vec3& e1, const Vec3& p2, const Vec3& e2, const Vec3& c1, Vec3& out_normal);
	
	// Checks whether edges defined by a->b and c->d form a face on the Minkowski difference of their respective
	// convex hulls
	static inline Bool IsMinkowskiFace(Vec3 const& a, Vec3 const& b, Vec3 const& b_x_a, Vec3 const& c, Vec3 const& d, Vec3 const& d_x_c);
	
	// This is the "Distance Subalgorithm" part of GJK -- see "Improving the GJK" by Montanari et al.
	static inline GJKWorkingPair NearestSimplex(Simplex const& s);


	// -------------------------------------------------------------------------
	// SAT Implementation
	// -------------------------------------------------------------------------

	// axis pointing from A->B in world space
	Float32	  SeparationOnAxis(Vec3 const& axis, Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
	{
		Vec3 const axisLocalA = glm::inverse(trA) * Vec4(axis, 0);
		Vec3 const axisLocalB = glm::inverse(trB) * Vec4(axis, 0);

		auto [ptA, eA] = GetSupport(A, axisLocalA);
		auto [ptB, eB] = GetSupport(B, -axisLocalB);

		return glm::dot( Vec3(trB * Vec4(ptB, 1) - trA * Vec4(ptA, 1)), axis );
	}

	FaceQuery SATQueryFaceDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
	{
		// We perform all computations in local space of B
		Mat4 const transform = glm::inverse(trB) * trA;

		FaceQuery result{};

		Int32 const face_count = static_cast<Int32>(A.faces.size());
		for (Int32 i = 0; i < face_count; ++i)
		{
			Plane const p = Transformed(A.faces[i].plane, transform);

			Float32 const separation = Project(p, B);
			if (separation > result.separation)
			{
				result.index = i;
				result.separation = separation;
				result.normal = Mat3(trB) * p.n; // transform normal back into world space, pointing from A toward B
			}
		}
		return result;
	}


	EdgeQuery SATQueryEdgeDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
	{
		// We perform all computations in local space of B
		Mat4 const transform = glm::inverse(trB) * trA;

		// Get the centroid of A in the local space of B
		Vec3 const cA = transform[3];

		// Find axis of minimum penetration
		EdgeQuery result{};

		Int32 const edgeCountA = static_cast<Int32>(A.edges.size());
		Int32 const edgeCountB = static_cast<Int32>(B.edges.size());

		for (Int32 i = 0; i < edgeCountA; i += 2)
		{
			Convex::HalfEdge const& edgeA = A.edges[i];
			Convex::HalfEdge const& twinA = A.edges[i + 1];
			ASSERT(edgeA.twin == i + 1 && twinA.twin == i, "A is invalid.");

			// EdgeA segment and vector
			Segment const eSegA = Transformed(
				{
					.b = A.verts[edgeA.origin],
					.e = A.verts[twinA.origin]
				}, 
				transform);
			Vec3 const eA = eSegA.e - eSegA.b;

			// Normals of faces adjacent to edgeA
			Vec3 const uA = Mat3(transform) * A.faces[edgeA.face].plane.n;
			Vec3 const vA = Mat3(transform) * A.faces[twinA.face].plane.n;

			for (Int32 j = 0; j < edgeCountB; j += 2)
			{
				Convex::HalfEdge const& edgeB = B.edges[j];
				Convex::HalfEdge const& twinB = B.edges[j + 1];
				ASSERT(edgeB.twin == j + 1 && twinB.twin == j, "B is invalid.");

				// EdgeB segment and vector
				Segment const eSegB = { .b = B.verts[edgeB.origin], .e = B.verts[twinB.origin] };
				Vec3 const eB = eSegB.e - eSegB.b;

				// Normals of faces adjacent to edgeB
				Vec3 const uB = B.faces[edgeB.face].plane.n;
				Vec3 const vB = B.faces[twinB.face].plane.n;

				if (IsMinkowskiFace(uA, vA, -eA, -uB, -vB, -eB))
				{
					Vec3 normal{};
					Float32 const separation = Project(eSegA.b, eA, eSegA.b, eB, cA, normal);
					if (separation > result.separation)
					{
						result.indexA = i;
						result.indexB = j;
						result.separation = separation;
						result.normal = Mat3(trB) * normal; // transform back into world space
					}
				}
			}
		}

		return result;
	}


	// -------------------------------------------------------------------------
	// GJK Implementation
	// -------------------------------------------------------------------------

	// GJK: Point-Convex
	Simplex GJK(Vec3 const& A, Convex const& B, Mat4 const& trB,
		Vec3 const& dir, Simplex const& initialSimplex)
	{
		ASSERT(false, "Not implemented");
		return Simplex{};
	}

	// GJK: Segment-Convex
	Simplex GJK(Segment const& A, Convex const& B, Mat4 const& trB,
		Vec3 const& dir, Simplex const& initialSimplex)
	{
		ASSERT(false, "Not implemented");
		return Simplex{};
	}

	// GJK: Convex-Convex
	Simplex GJK(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB,
		Vec3 const& dir, Simplex const& initialSimplex)
	{
		ASSERT(false, "Not implemented");
		return Simplex{};


		//ASSERT(A.verts.size() > 0 && B.verts.size() > 0, "One of the hulls is empty");

		//// current simplex and search direction
		//GJKWorkingPair curr{};
		//if (initialSimplex.GetType() != Simplex::Type::NONE)
		//{
		//	curr = NearestSimplex(initialSimplex);
		//}
		//else
		//{
		//	curr.searchDir = -(GetSupport(A, dir) - GetSupport(B, -dir));
		//	curr.simplex.PushVert(-curr.searchDir);
		//}

		//while (not curr.simplex.containsOrigin)
		//{
		//	Vec3 const a = GetSupport(A, curr.searchDir) - GetSupport(B, -curr.searchDir);
		//	if (glm::dot(a, curr.searchDir) < 0.0f)
		//	{
		//		// return the partially constructed, but failing, simplex
		//		return curr.simplex;
		//	}

		//	// Generate a new simplex and continue
		//	curr.simplex.PushVert(a);
		//	curr = NearestSimplex(curr.simplex);
		//}
		//return curr.simplex;
	}


	// -------------------------------------------------------------------------
	// Helper Implementations
	// -------------------------------------------------------------------------

	// dir should be in hull's local space
	static inline CvxSupport GetSupport(Convex const& hull, Vec3 const& dir, Convex::HalfEdge hillClimbingHint) 
	{
		// TODO : Arbitrary threshold... needs to be tested and tuned. Cameron suggests,
		// "As a rough rule of thumb, this difference becomes measurable at around 10 
		// vertices per hull, and important at about 20 vertices per hull." (see
		// S. Cameron 1998 -- Enhanced GJK, and Ericson Ch.9.5.4)
		static constexpr Uint32 hillClimbingThreshold = 8;

		if (hull.verts.size() > hillClimbingThreshold) 
		{
			if (hillClimbingHint.origin == Convex::INVALID_INDEX) { 
				hillClimbingHint = hull.edges[0]; 
			}
			
			return GetSupportHillClimbing(hull, dir, hillClimbingHint);
		}
		else
		{
			Vec3    bestVert = Vec3(std::numeric_limits<Float32>::lowest());
			Float32 maxProjection = std::numeric_limits<Float32>::lowest();

			for (auto&& v : hull.verts)
			{
				Float32 const projection = glm::dot(dir, v);
				if (projection > maxProjection)
				{
					bestVert = v;
					maxProjection = projection;
				}
			}

			return CvxSupport{ .pt = bestVert, .e = Convex::HalfEdge{} };
		}
	}


	// dir should be in hull's local space
	static inline CvxSupport GetSupportHillClimbing(Convex const& hull, Vec3 const& dir, Convex::HalfEdge e)
	{
		ASSERT(hull.edges.size() > 0, "Hull is empty");

		// edge = start
		// while (no neighbors more extreme):
		//	-> Inspect all neighbor vertices of e.origin
		//	-> Move to the neighbor whose projection onto dir is maximal, or exit if none found
		
		Float32 maxProj = std::numeric_limits<Float32>::lowest();
		Uint8 lastVisitedVertex = Convex::INVALID_INDEX;
		Bool found = true;

		while (found) 
		{
			found = false;
			ForEachOneRingNeighbor(hull, e, [&](Convex::HalfEdge const& neighbor)
			{
				if (neighbor.origin != lastVisitedVertex) 
				{
					Vec3 const    v = hull.verts[neighbor.origin];
					Float32 const proj = glm::dot(dir, v);

					if (proj > maxProj)
					{
						e = neighbor;
						maxProj = proj;
						found = true;
					}
				}
			});

			// Document the vertex we just found so we don't check it again
			// during next iteration -- note that this does not prevent
			// cycles due to coplanar faces, so we must disallow coplanar
			// faces during Convex creation.
			lastVisitedVertex = e.origin;
		}

		return CvxSupport{ .pt = hull.verts[e.origin], .e = e };
	}


	// TODO : this can be made faster using "Improved GJK" by M. Montanari et al.
	static inline GJKWorkingPair NearestSimplex(Simplex const& s)
	{
		ASSERT(false, "Not implemented");
		return { Simplex{}, Vec3{} };
	};


	// This function computes the distance between a plane and the hull assuming the plane
	// is in the local space of the hull
	static inline Float32 Project(Plane const& plane, Convex const& hull)
	{
		auto [support, edge] = GetSupport(hull, -plane.n);
		return SignedDistance(support, plane);
	}


	// This function is used only within SATQueryEdgeDirections
	static inline Float32 Project(Vec3 const& p1, Vec3 const& e1,
		Vec3 const& p2, Vec3 const& e2,
		Vec3 const& c1,
		Vec3& outNormal)
	{
		// Build search direction
		Vec3 const e1_x_e2 = glm::cross(e1, e2);

		// Skip near parallel edges: |e1 x e2| = sin(alpha) * |e1| * |e2|
		static constexpr Float32 k_tolerance = 0.005f;
		Float32 const L = glm::length(e1_x_e2);
		if (L < k_tolerance * std::sqrt(glm::length2(e1) * glm::length2(e2))) {
			outNormal = Vec3(NAN);
			return std::numeric_limits<Float32>::lowest();
		}

		// Ensure consistent normal orientation (A -> B)
		outNormal = e1_x_e2 / L;
		if (glm::dot(outNormal, p1 - c1) < 0.0f) {
			outNormal *= -1.0f;
		}

		// s = Dot(n, p2) - d = Dot(n, p2) - Dot(n, p1) = Dot(n, p2 - p1) 
		return glm::dot(outNormal, p2 - p1);
	}


	// This function is used only within SATQueryEdgeDirections
	static inline Bool IsMinkowskiFace(Vec3 const& a, Vec3 const& b,
		Vec3 const& b_x_a,
		Vec3 const& c, Vec3 const& d,
		Vec3 const& d_x_c)
	{
		// Test if arcs AB and CD intersect on the unit sphere 
		Float32 const CBA = glm::dot(c, b_x_a);
		Float32 const DBA = glm::dot(d, b_x_a);
		Float32 const ADC = glm::dot(a, d_x_c);
		Float32 const BDC = glm::dot(b, d_x_c);

		return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
	}
}