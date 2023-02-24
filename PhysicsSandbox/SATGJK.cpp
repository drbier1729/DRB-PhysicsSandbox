#include "pch.h"
#include "SATGJK.h"

#include "PhysicsGeometry.h"

namespace drb::physics::util {
	
	// -------------------------------------------------------------------------
	// Helpers
	// -------------------------------------------------------------------------
	
	struct CvxSupport
	{
		Vec3           pt   = {};
		Float32        proj = std::numeric_limits<Float32>::lowest();
		Convex::EdgeID e    = Convex::INVALID_INDEX;
	};

	// See "Improving the GJK algorithm for faster and more reliable distance 
	// queries between convex objects" by Montanari, Petrinic, and Barbieri,
	// and Montanari's OpenGJK
	class SimplexSolver
	{
	private:
		// Configuration Space vertices of CSO.
		// vert[3] is oldest, vert[0] is most recently added.
		Vec3  verts[4];

		// Configuration Space search direction toward origin
		Vec3  dir;

		// Indirect indexing into verts to allow for an update to 
		// the original simplex using this proxy, and for removal
		// of vertices without changing verts -- which means
		// all it takes to recover from a temporary removal
		// is to reset this small array to its state before
		// the call to RemoveVert
		//		verts[ idxs[i] ] = ith vertex of reduced simplex
		Uint8 idxs[4];

		// How many total points we have -- max 4, min 1
		Uint8 size;
		
	public:
		// Constructors
		SimplexSolver() = default;
		SimplexSolver(Simplex& s, Vec3& d);

	private:
		// Helpers
		void               Solve3D();
		void               Solve2D();
		void               Solve1D();
		
		inline void        RemoveVert(Uint8 idx);	
		inline void        RemoveVerts(Uint8 idxA, Uint8 idxB);	
		
		inline void        UpdateSimplex(Simplex& s) const;
	};

	// If this uses hill climbing, returns the same as GetSupportHillClimbing, else returns a support point
	// and the "e" field is left defaulted. Start hint is an optional argument specifying where hill climbing
	// should begin.
	static inline CvxSupport GetSupport(Convex const& hull, Vec3 const& dir, Convex::EdgeID startHint = 0);

	// Returns support point in local space and an edge with the support point as its origin. See comment for
	// "GetSupport" for more details.
	static inline CvxSupport GetSupportHillClimbing(Convex const& hull, Vec3 const& dir, Convex::EdgeID startHint = 0);
	
	// Does some Dirk Gregorius magic -- see his GDC talk on SAT
	static inline Float32 Project(const Vec3& p1, const Vec3& e1, const Vec3& p2, const Vec3& e2, const Vec3& c1, Vec3& out_normal);
	
	// Checks whether edges defined by a->b and c->d form a face on the Minkowski difference of their respective
	// convex hulls
	static inline Bool IsMinkowskiFace(Vec3 const& a, Vec3 const& b, Vec3 const& b_x_a, Vec3 const& c, Vec3 const& d, Vec3 const& d_x_c);

	// Used internally by SimplexSolver -- these are named inappropriately but I'm not sure
	// exactly what is going on here in the OpenGJK code... see hff_ functions
	static inline Bool OriginInFrontOfFace(Vec3 const& p, Vec3 const& q, Vec3 const& r);
	static inline Bool OriginInFrontOfEdge(Vec3 const& p, Vec3 const& q, Vec3 const& r);
	static inline Bool OriginInFrontOfVert(Vec3 const& p, Vec3 const& q);

	// Used in SimplexSolver
	static inline Vec3 ProjectOriginOnLine(Vec3 const& ptOnLine0, Vec3 const& ptOnLine1);
	static inline Vec3 ProjectOriginOnPlane(Vec3 const& ptOnPlane0, Vec3 const& ptOnPlane1, Vec3 const& ptOnPlane2);

	// -------------------------------------------------------------------------
	// SAT Implementation
	// -------------------------------------------------------------------------

	// axis pointing from A->B in world space
	Float32	  SeparationOnAxis(Vec3 const& axis, Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
	{
		Vec3 const axisLocalA = glm::inverse(Mat3(trA)) * axis;
		Vec3 const axisLocalB = glm::inverse(Mat3(trB)) * axis;

		auto [ptA, projA, eA] = GetSupport(A, axisLocalA);
		auto [ptB, projB, eB] = GetSupport(B, -axisLocalB);

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

			auto const [support, proj, eID] = GetSupport(B, -p.n);
			Float32 const separation = (-proj) - p.d;
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
					Float32 const separation = Project(eSegA.b, eA, eSegB.b, eB, cA, normal);
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
	ClosestPointsQuery GJK(Vec3 const& A_, Convex const& B, Mat4 const& trB, Simplex* seed)
	{
		ASSERT(B.verts.size() > 0, "Hull must be non empty");

		// Arbitrary -- could be set externally if we want
		static constexpr Float32 tol = 1.0e-8f;
		static constexpr Float32 tol2 = tol * tol;

		// If we were provided with a seed, we can operate on that simplex, 
		// otherwise we'll just use a local one
		Simplex localSimplex{};
		Simplex& simplex = seed ? *seed : localSimplex;

		// Transform A into local space of B so we can do fewer matrix
		// multiplications in the main loop. Note that in this special
		// case ALL COMPUTATIONS ARE IN LOCAL SPACE OF B
		Vec3 const A = glm::inverse(trB) * Vec4(A_, 1);

		// If we didn't receive a seed, or we received an empty seed, 
		// initialize our simplex
		if (simplex.GetType() == Simplex::Type::NONE)
		{
			simplex.lambdas[0] = 1.0f;
			simplex.bestA = 0;
			simplex.bestB = 0;

			Vec3 const& initVertB = B.verts[B.edges[0].origin];
			simplex.PushVert(A, 0, initVertB, 0);
		}
		// Otherwise, update seed simplex with the new world positions of verts
		else
		{
			for (Uint8 i = 0; i < simplex.size; ++i)
			{
				// Vertex positions in local space of B
				simplex.vertsA[i] = A;
				
				// simplex.vertsB already in local space
			}
		}

		// Check for 1 degenerate case: A is at the centroid of B
		Float32 const d2 = glm::length2(A);
		if (EpsilonEqual(d2, 0.0f, tol))
		{
			return ClosestPointsQuery{
				.ptA = A_,
				.ptB = trB * Vec4(B.verts[0], 1),
				.d2 = -1.0f
			};
		}

		// Values used in main loop
		Vec3    disp    = A;
		Vec3    oldDisp = Vec3(NAN);
		Float32 oldD2   = std::numeric_limits<Float32>::max();

		// supportA always == A
		CvxSupport supportB{};		

		ClosestPointsQuery result{
			.ptA = A,
			.ptB = Vec3(0),
			.d2  = d2
		};

		// Main loop
		Int32 maxIters = static_cast<Int32>(B.verts.size());
		while (maxIters-- > 0)
		{
			ASSERT(simplex.GetType() != Simplex::Type::NONE, "Invalid simplex");

			// Compute next support points
			// supportA = GetSupport(A, invRotA *  -disp, simplex.bestA);
			supportB = GetSupport(B, /* invRotB* */ disp, simplex.bestB);


			// Test first exit conditions: new point is already in the simplex or 
			// dir is invalid, indicating no intersection
			Float32 const gVal = result.d2 - glm::dot(disp, A - supportB.pt);
			if (gVal <= tol * result.d2 || gVal < tol2) { break; }
			
			// This is similar to the exit condition in Box2D, but seems to be
			// unecessary in many other implementations (e.g. OpenGJK, Jolt Physics,
			// Cameron's Enhanced GJK)
			Bool duplicate = false;
			for (Uint8 i = 0; i < simplex.size; ++i)
			{
				if (supportB.e == simplex.edgeIdxsB[i])
				{
					duplicate = true;
					break;
				}
			}
			if (duplicate) { break; }


			// Push new support point to the simplex
			simplex.bestB = supportB.e;
			simplex.PushVert(A, 0, supportB.pt, supportB.e);


			// Reduce simplex and compute new search direction
			if (simplex.size > 1)
			{
				oldDisp = disp;
				SimplexSolver solve{ simplex, disp };
			}
			else
			{
				ASSERT(false, "Invalid simplex. Must have at least 2 points during solver iterations");
				break;
			}


			// Record last d2 value and update new d2 value
			oldD2 = result.d2;
			result.d2 = glm::length2(disp);


			// Test second exit conditions: we are no longer converging
			if (not (result.d2 < oldD2))
			{
				// Reset to one step prior in case of NAN values
				disp = oldDisp;
				result.d2 = oldD2;
				break;
			}


			// Test third exit conditions: we were unable to reduce our simplex from
			// a tetrahedron OR our d2 value is very small, indicating an intersection
			{
				Float32 norm2Max = std::numeric_limits<Float32>::lowest();
				for (Uint8 i = 0; i < simplex.size; ++i)
				{
					Float32 const norm2 = glm::length2(A - simplex.vertsB[i]);
					if (norm2 > norm2Max)
					{
						norm2Max = norm2;
					}
				}
				if (simplex.size == 4 || result.d2 <= tol2 * norm2Max)
				{
					simplex.containsOrigin = true;
					break;
				}
			}
		}

		if (simplex.containsOrigin)
		{
			// If our objects are overlapping, this distance should not be used
			// so mark it as a garbage negative value to indicate intersection.
			result.d2 = -1.0f;
		}

		simplex.ComputeBarycentricCoords(disp);
		simplex.ExtractWitnessPoints(result.ptA, result.ptB);

		// Return to world space coords
		result.ptA = A_;
		result.ptB = trB * Vec4(result.ptB, 1);
		return result;
	}

	// GJK: Segment-Convex
	ClosestPointsQuery GJK(Segment const& A_, Convex const& B, Mat4 const& trB, Simplex* seed)
	{
		ASSERT(B.verts.size() > 0, "Hull must be non empty");

		// Arbitrary -- could be set externally if we want
		static constexpr Float32 tol = 1.0e-8f;
		static constexpr Float32 tol2 = tol * tol;

		// If we were provided with a seed, we can operate on that simplex, 
		// otherwise we'll just use a local one
		Simplex localSimplex{};
		Simplex& simplex = seed ? *seed : localSimplex;

		// Transform A into local space of B so we can do fewer matrix
		// multiplications in the main loop. Note that in this special
		// case ALL COMPUTATIONS ARE IN LOCAL SPACE OF B
		Segment const A = Transformed(A_, glm::inverse(trB));

		// If we didn't receive a seed, or we received an empty seed, initialize our simplex
		if (simplex.GetType() == Simplex::Type::NONE)
		{
			simplex.lambdas[0] = 1.0f;
			simplex.bestA = 0;
			simplex.bestB = 0;

			Vec3 const& initVertB = B.verts[B.edges[0].origin];
			simplex.PushVert(A.b, 0, initVertB, 0);
		}
		// Otherwise, update seed simplex with the new world positions of verts
		else
		{
			for (Uint8 i = 0; i < simplex.size; ++i)
			{
				// Vertex positions in local space of B
				simplex.vertsA[i] = simplex.edgeIdxsA[i] == 0 ? A.b : A.e;

				// simplex.vertsB already in local space
			}
		}

		// Values used in main loop
		Mat3 const invRotB = glm::transpose(Mat3(trB));
		
		Vec3    disp    = A.b - simplex.vertsB[0];
		Vec3    oldDisp = disp;
		Float32 oldD2   = std::numeric_limits<Float32>::max();

		Float32    projA{};
		Uint8	   idxA{};
		Vec3       supportA{};
		CvxSupport supportB{};

		ClosestPointsQuery result{
			.ptA = A.b,
			.ptB = simplex.vertsB[0],
			.d2 = glm::length2(disp)
		};

		// Main loop
		Int32 maxIters = static_cast<Int32>(B.verts.size());
		while (maxIters-- > 0)
		{
			ASSERT(simplex.GetType() != Simplex::Type::NONE, "Invalid simplex");

			// Compute next support points
			projA = glm::dot(A.b, -disp);
			idxA = 0;
			supportA = A.b;
			if (Float32 const tmp = glm::dot(A.e, -disp); tmp > projA)
			{
				projA = tmp;
				idxA = 1;
				supportA = A.e;
			}
			supportB = GetSupport(B, /* invRotB* */ disp, simplex.bestB);


			// Test first exit conditions: new point is already in the simplex or 
			// dir is invalid, indicating no intersection
			Float32 const gVal = result.d2 - glm::dot(disp, supportA - supportB.pt);
			if (gVal <= tol * result.d2 || gVal < tol2) { 
				break; }

			// This is similar to the exit condition in Box2D, but seems to be
			// unecessary in many other implementations (e.g. OpenGJK, Jolt Physics,
			// Cameron's Enhanced GJK)
			Bool duplicate = false;
			for (Uint8 i = 0; i < simplex.size; ++i)
			{
				if (idxA == simplex.edgeIdxsA[i] && supportB.e == simplex.edgeIdxsB[i])
				{
					duplicate = true;
					break;
				}
			}
			if (duplicate) { 
				break; }


			// Push new support point to the simplex
			simplex.bestB = supportB.e;
			simplex.PushVert(supportA, idxA, supportB.pt, supportB.e);


			// Reduce simplex and compute new search direction
			if (simplex.size > 1)
			{
				oldDisp = disp;
				SimplexSolver solve{ simplex, disp };
			}
			else
			{
				ASSERT(false, "Invalid simplex. Must have at least 2 points during solver iterations");
				break;
			}


			// Record last d2 value and update new d2 value
			oldD2 = result.d2;
			result.d2 = glm::length2(disp);


			// Test second exit conditions: we are no longer converging
			if (not (result.d2 < oldD2))
			{
				// Reset to one step prior in case of NAN values
				disp = oldDisp;
				result.d2 = oldD2;
				break;
			}


			// Test third exit conditions: we were unable to reduce our simplex from
			// a tetrahedron OR our d2 value is very small, indicating an intersection
			{
				Float32 norm2Max = std::numeric_limits<Float32>::lowest();
				for (Uint8 i = 0; i < simplex.size; ++i)
				{
					Float32 const norm2 = glm::length2(simplex.vertsA[i] - simplex.vertsB[i]);
					if (norm2 > norm2Max)
					{
						norm2Max = norm2;
					}
				}
				if (simplex.size == 4 || result.d2 <= tol2 * norm2Max)
				{
					simplex.containsOrigin = true;
					break;
				}
			}
		}

		if (simplex.containsOrigin)
		{
			// If our objects are overlapping, this distance should not be used
			// so mark it as a garbage negative value to indicate intersection.
			result.d2 = -1.0f;
		}

		simplex.ComputeBarycentricCoords(disp);
		simplex.ExtractWitnessPoints(result.ptA, result.ptB);

		// Return to world space coords
		result.ptA = trB * Vec4(result.ptA, 1);
		result.ptB = trB * Vec4(result.ptB, 1);
		return result;
	}

	// GJK: Convex-Convex -- See S. Cameron "Enhanced GJK" and Monanari "Improved GJK" -- NOT DONE
	ClosestPointsQuery GJK(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB, Simplex* seed)
	{
		ASSERT(false, "Not implemented");


		ASSERT(A.verts.size() > 0 && B.verts.size() > 0, "Hulls must be non empty");

		// Arbitrary -- could be set externally if we want
		static constexpr Float32 tolerance = 1.0e-8f;

		// If we were provided with a seed, we can operate on that simplex, 
		// otherwise we'll just use a local one
		Simplex localSimplex{};
		Simplex& simplex = seed ? *seed : localSimplex;

		// If we didn't receive a seed, or we received an empty seed, initialize our simplex
		if (simplex.GetType() == Simplex::Type::NONE) 
		{
			simplex.lambdas[0] = 1.0f;
			simplex.bestA = 0;
			simplex.bestB = 0;

			Vec3 const& initVertA = A.verts[A.edges[0].origin];
			Vec3 const& initVertB = B.verts[B.edges[0].origin];
			simplex.PushVert(trA * Vec4(initVertA, 1), 0, trB * Vec4(initVertB, 1), 0);
		}
		// Otherwise, update seed simplex with the new world positions of verts
		else
		{
			for (Uint8 i = 0; i < simplex.size; ++i)
			{
				// Local space vertex positions
				Vec3 const& vA = A.verts[ A.edges[simplex.edgeIdxsA[i]].origin ];
				Vec3 const& vB = B.verts[ B.edges[simplex.edgeIdxsB[i]].origin ];

				// World space vertex positions
				simplex.vertsA[i] = trA * Vec4(vA, 1);
				simplex.vertsB[i] = trB * Vec4(vB, 1);
			}
		}

		// Values used in main loop
		Mat3 const invRotA = glm::transpose(Mat3(trA));
		Mat3 const invRotB = glm::transpose(Mat3(trB));
		Vec3 const centroidA = trA[3];
		Vec3 const centroidB = trB[3];

		Vec3      dir = centroidB - centroidA; // from A toward B
		Float32 oldD2 = std::numeric_limits<Float32>::max();

		CvxSupport supportA{}, supportB{};

		ClosestPointsQuery result{ 
			.ptA = centroidA,
			.ptB = centroidB,
			.d2 = glm::length2(dir) 
		};

		// Main loop
		Int32 maxIters = static_cast<Int32>(A.verts.size() * B.verts.size());
		while (maxIters-- > 0)
		{
			ASSERT(simplex.GetType() != Simplex::Type::NONE, "Invalid simplex");

			// Compute next support points
			supportA = GetSupport(A,   invRotA * dir,  simplex.bestA);
			supportB = GetSupport(B, -(invRotB * dir), simplex.bestB);

			// Test first exit condition: new point is already in the simplex or dir is invalid
			Float32 const exceedRelTol = result.d2 - (supportA.proj - supportB.proj);
			if (result.d2 < tolerance || EpsilonEqual(exceedRelTol, 0.0f, tolerance)) { break; }

			// Push new point to the simplex
			simplex.bestA = supportA.e;
			simplex.bestB = supportB.e;
			simplex.PushVert(trA * Vec4(supportA.pt, 1), supportA.e, 
					         trB * Vec4(supportB.pt, 1), supportB.e);

			// Reduce the simplex
			//GJKDistanceSubAlgorithm(simplex);

			// Update our query result and search direction
			simplex.ExtractWitnessPoints(result.ptA, result.ptB);
			dir       = result.ptA - result.ptB; // from B toward A
			oldD2     = result.d2;
			result.d2 = glm::length2(dir);

			// Test second exit condition: we were unable to reduce our simplex from
			// a tetrahedron OR our d2 value is very small, indicating an intersection
			if (simplex.size == 4 || EpsilonEqual(result.d2, 0.0f, tolerance))
			{
				simplex.containsOrigin = true;
				break;
			}
			
			// Test error condition: we're no longer converging.
			if (oldD2 >= result.d2)
			{
				ASSERT(false, "Something went wrong: we're no longer converging.");
				break;
			}
		}


		if (simplex.containsOrigin)
		{
			// If our objects are overlapping, this distance should not be used
			// so mark it as a garbage negative value to indicate intersection.
			result.d2 = -1.0f;
		}

		return result;
	}


	// -------------------------------------------------------------------------
	// Helper Implementations
	// -------------------------------------------------------------------------

	void Simplex::ComputeBarycentricCoords(Vec3 const& disp)
	{
		// Compute lambdas from simplex points and disp (projected origin 
		// onto simplex in Config Space)

		switch (size)
		{
		break; case 1:
		{
			lambdas[0] = 1.0f;
		}
		break; case 2:
		{
			// Project simplex onto Cartesian axis with maximal length
			// then compute lambdas on this axis
			Vec3 const s0 = vertsA[0] - vertsB[0];
			Vec3 const s1 = vertsA[1] - vertsB[1];

			Uint8 x = 0;
			Float32 maxLength = 0.0f;
			for (Uint8 i = 0; i < 3; ++i)
			{
				Float32 const len = s0[i] - s1[i];
				if (glm::abs(len) > glm::abs(maxLength))
				{
					maxLength = len;
					x = i;
				}
			}

			// Solve M * l = p, implicitly
			lambdas[0] = -(s1[x] - disp[x]) / maxLength;
			lambdas[1] =  (s0[x] - disp[x]) / maxLength;
		}
		break; case 3:
		{
			// Project simplex onto Cartesian plane with maximal area
			// then compute lambdas on this plane
			Vec3 const s0 = vertsA[0] - vertsB[0];
			Vec3 const s1 = vertsA[1] - vertsB[1];
			Vec3 const s2 = vertsA[2] - vertsB[2];

			Float32 maxArea = 0.0f;
			Uint8 x = 0, y = 1;
			Uint8 j = 1, k = 2;
			for (Uint8 i = 0; i < 3; ++i)
			{
				Float32 const area = s0[j] * s1[k] - s1[j] * s0[k]
								   + s1[j] * s2[k] - s2[j] * s1[k]
					  			   + s2[j] * s0[k] - s0[j] * s2[k];
				if (glm::abs(area) > glm::abs(maxArea))
				{
					maxArea = area;
					x = j;
					y = k;
				}

				j = k;
				k = i;
			}

			Mat3 const M = {
				Vec3(s0[x], s0[y], 1),
				Vec3(s1[x], s1[y], 1),
				Vec3(s2[x], s2[y], 1)
			};
			Vec3 const dispProj = Vec3(disp[x], disp[y], 1);

			// Solve M * l = p
			lambdas = Vec4(glm::inverse(M) * dispProj, 0.0f);
		}
		break; case 4:
		{
			// Solve M * l = p
			Mat4 const M = {
				Vec4(vertsA[0] - vertsB[0], 1),
				Vec4(vertsA[1] - vertsB[1], 1),
				Vec4(vertsA[2] - vertsB[2], 1),
				Vec4(vertsA[3] - vertsB[3], 1),
			};
			lambdas = glm::inverse(M) * Vec4(disp, 1);
		}
		}
	}

	// dir should be in hull's local space
	static inline CvxSupport GetSupport(Convex const& hull, Vec3 const& dir, Convex::EdgeID hillClimbingHint) 
	{
		// // TODO : Arbitrary threshold... needs to be tested and tuned. Cameron suggests,
		// // "As a rough rule of thumb, this difference becomes measurable at around 10 
		// // vertices per hull, and important at about 20 vertices per hull." (see
		// // S. Cameron 1998 -- Enhanced GJK, and Ericson Ch.9.5.4)
		static constexpr Uint8 hillClimbingThreshold = 10;

		Uint8 const numVerts = static_cast<Uint8>(hull.verts.size());
		if (numVerts > hillClimbingThreshold) 
		{			
			return GetSupportHillClimbing(hull, dir, hillClimbingHint);
		}

		// Else: use brute force

		Convex::VertID bestVert = Convex::INVALID_INDEX;
		Float32 maxProjection   = std::numeric_limits<Float32>::lowest();	
		Convex::VertID current  = 0;
		for (auto && v : hull.verts)
		{
			Float32 const projection = glm::dot(dir, v);
			if (projection > maxProjection)
			{
				bestVert = current;
				maxProjection = projection;
			}

			++current;
		}

		return CvxSupport{ .pt = hull.verts[bestVert], .proj = maxProjection, .e = hull.vertAdj[bestVert] };
	}


	// dir should be in hull's local space
	static inline CvxSupport GetSupportHillClimbing(Convex const& hull, Vec3 const& dir, Convex::EdgeID e)
	{
		Convex::EdgeID const numEdges = static_cast<Convex::EdgeID>(hull.edges.size());

		ASSERT(numEdges > 0, "Hull is empty");
		ASSERT(e < numEdges, "Index out of range.");

		// edge = start
		// while (no neighbors more extreme):
		//	-> Inspect all neighbor vertices of e.origin
		//	-> Move to the neighbor whose projection onto dir is maximal, or exit if none found
		
		Float32 maxProj = std::numeric_limits<Float32>::lowest();
		Convex::VertID lastVisitedVertex = Convex::INVALID_INDEX;

		Bool found = true;
		Convex::EdgeID iterCount = 0; // shouldn't be needed, but used as a safety precaution to avoid infinite loops
		while(found && iterCount++ < numEdges)
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
							e = hull.edges[neighbor.twin].twin; // awkward way of extracting the EdgeID of neighbor
							maxProj = proj;
							found = true;
						}
					}
				});

			// Record the vertex we just found so we don't check it again
			// during next iteration -- note that this does not prevent
			// cycles due to coplanar faces, so we must disallow coplanar
			// faces during Convex creation.
			lastVisitedVertex = hull.edges[e].origin;
		}

		// If, on our last iteration, we still found a better vertex, then hill-climbing got 
		// stuck somewhere -- something was wrong with our hull geometry (possibly coplanar faces).
		ASSERT(not found, "Hill climbing failed to terminate.");

		return CvxSupport{
			.pt = hull.verts[lastVisitedVertex],
			.proj = maxProj,
			.e = e 
		};
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
		if (L < k_tolerance * glm::sqrt(glm::length2(e1) * glm::length2(e2))) {
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

	SimplexSolver::SimplexSolver(Simplex& s, Vec3& d)
		: verts{}, 
		dir{ d },
		idxs{ },
		size{ s.size }
	{
		Uint8 idx = size - 1;
		for (Uint8 i = 0; i < size; ++i)
		{
			// order by most recently added
			verts[i] = s.vertsA[idx] - s.vertsB[idx];
			idxs[i]  = idx--;
		}

		// Signed Volumes method
		using ST = Simplex::Type;
		switch (s.GetType())
		{
		break; case ST::Tet:
		{
			Solve3D();
		}
		break; case ST::Tri:
		{
			Solve2D();
		}
		break; case ST::Seg:
		{
			Solve1D();
		}
		break; default:
		{
			ASSERT(false, "Simplex is invalid (has type Pnt or NONE)");
		}
		}

		UpdateSimplex(s);
		d = dir;
	}
	
	void SimplexSolver::Solve1D()
	{
		ASSERT(size == 2, "Can only be called on a Segment");
	
		Vec3 const& s0 = verts[0];
		Vec3 const& s1 = verts[1];

		if (OriginInFrontOfVert(s0, s1))
		{
			dir = ProjectOriginOnLine(s0, s1);
			// region { 0, 1 }
		}
		else
		{
			dir = s0;
			RemoveVert(1);
			// region { 0 }
		}
	}
	

	void SimplexSolver::Solve2D()
	{
		ASSERT(size == 3, "Can only be called on a Triangle");

		Vec3 const& s0 = verts[0];
		Vec3 const& s1 = verts[1];
		Vec3 const& s2 = verts[2];

		Bool const testVert01 = OriginInFrontOfVert(s0, s1); // origin in front of vert 0 (in direction of edge 01)
		Bool const testVert02 = OriginInFrontOfVert(s0, s2); // origin in front of vert 0 (in direction of edge 02)

		if (testVert01)
		{
			Bool const testEdge12 = OriginInFrontOfEdge(s0, s1, s2); // origin in front of edge 01 (outward from triangle 012)
			if (not testEdge12)
			{
				if (testVert02)
				{
					Bool const testEdge21 = OriginInFrontOfEdge(s0, s2, s1); // origin in front of edge 02 (outward from triangle 021) 
					if (not testEdge21)
					{
						dir = ProjectOriginOnPlane(s0, s1, s2);
						// region { 0, 1, 2 }
					}
					else
					{
						dir = ProjectOriginOnLine(s0, s2);
						RemoveVert(1);
						// region { 0, 2 }
					}
				}
				else
				{
					dir = ProjectOriginOnPlane(s0, s1, s2);
					// region { 0, 1, 2 }
				}
			}
			else
			{
				dir = ProjectOriginOnLine(s0, s1);
				RemoveVert(2);
				// region { 0, 1 }
			}
		}
		else if (testVert02)
		{
			Bool const testEdge21 = OriginInFrontOfEdge(s0, s2, s1); // origin in front of edge 02 (outward from triangle 021)
			if (not testEdge21)
			{
				dir = ProjectOriginOnPlane(s0, s1, s2);
				// region { 0, 1, 2 }
			}
			else
			{
				dir = ProjectOriginOnLine(s0, s2);
				RemoveVert(1);
				// region { 0, 2 }
			}
		}
		else
		{
			dir = s0;
			size = 1; // implicitly remove verts 1 and 2
			// region { 0 }
		}
	}
	
#ifndef DRB_SOLVE_3D
	void SimplexSolver::Solve3D()
	{
		ASSERT(false, "Not implemented");
	}
#else
	// TODO : what the absolute FUCK is happening in this function...
	void SimplexSolver::Solve3D()
	{
		ASSERT(size == 4, "Can only be called on a Tetrahedron");

		Vec3 const& s0 = Vert(0);
		Vec3 const& s1 = Vert(1);
		Vec3 const& s2 = Vert(2);
		Vec3 const& s3 = Vert(3);

		Vec3 const edge01 = s1 - s0;
		Vec3 const edge02 = s2 - s0;
		Vec3 const edge03 = s3 - s0;

		// What the hell is this?
		Bool const testLine[4] = {   
			false,
			Discard1D(s0, s1),
			Discard1D(s0, s2),
			Discard1D(s0, s3),
		};

		Uint32 const dotTotal = testLine[1] + testLine[2] + testLine[3];
		if (dotTotal == 0)
		{
			dir = s0;
			size = 1;
			// region { 0 }
		}
		else
		{
			Float32 const det023 = glm::determinant(Mat3(edge02, edge03, edge01));
			Bool const det023Pos = (det023 > 0.0f);

			Bool const testPlane023 = ( Discard3D(s0, s2, s3) == det023Pos );
			Bool const testPlane031 = ( Discard3D(s0, s3, s1) == det023Pos );
			Bool const testPlane012 = ( Discard3D(s0, s1, s2) == det023Pos );

			switch (testPlane023 + testPlane031 + testPlane012)
			{
				// case 3: nothing to do! region { 0, 1, 2, 3 }

				break; case 2:
				{
					// One triangle facing the origin. Figure out which one
					// and remove the point not on it.
					if      (not testPlane023) { RemoveVert(1); }
					else if (not testPlane031) { RemoveVert(2); }
					else if (not testPlane012) { RemoveVert(3); }
					
					Solve2D();
				}
				break; case 1:
				{
					// Two triangles face the origin. Don't know what the fuck
					// Montanari is doing here or why...
					Uint8 i = 0, j = 0, k = 0;
					if      (testPlane023) { i = 2; j = 3; k = 1; } // keep s2 and s3, remove s1
					else if (testPlane031) { i = 3; j = 1; k = 2; } // keep s3 and s1, remove s2 
					else  /*testPlane012*/ { i = 1; j = 2; k = 3; } // keep s1 and s2, remove s3 

					Vec3 const& si = Vert(i);
					Vec3 const& sj = Vert(j);
					Vec3 const& sk = Vert(k);

					// Now a bunch more dumb shit tests
					if (dotTotal == 1)
					{
						if      (testLine[k])
						{
							if (Discard2D(s0, sk, si))
							{
								dir = ProjectOriginOnPlane(s0, si, sk);
								RemoveVert(j);
								// region { 0, i, k }
							}
							else if (Discard2D(s0, sk, sj))
							{
								dir = ProjectOriginOnPlane(s0, sj, sk);
								RemoveVert(i);
								// region { 0, j, k }
							}
							else
							{
								dir = ProjectOriginOnLine(s0, sk);
								RemoveVerts(i, j);
								// region { 0, k }
							}
						}
						else if (testLine[i])
						{
							if (Discard2D(s0, si, sk))
							{
								dir = ProjectOriginOnPlane(s0, si, sk);
								RemoveVert(i);
								// region { 0, i, k }
							}
							else
							{
								dir = ProjectOriginOnLine(s0, si);
								RemoveVerts(j, k);
								// region { 0, i }
							}
						}
						else // testLine[j]
						{
							if (Discard2D(s0, sj, sk))
							{
								dir = ProjectOriginOnPlane(s0, sj, sk);
								RemoveVert(i);
								// region { 0, j, k }
							}
							else
							{
								dir = ProjectOriginOnLine(s0, sj);
								RemoveVerts(i, k);
								// region { 0, j }
							}
						}
					}
					else if (dotTotal == 2)
					{
						if      (testLine[i])
						{
							if (Discard2D(s0, sk, si))
							{
								if (Discard2D(s0, si, sk))
								{
									dir = ProjectOriginOnPlane(s0, si, sk);
									RemoveVert(j);
									// region { 0, i, k }
								}
								else
								{
									dir = ProjectOriginOnLine(s0, sk);
									RemoveVerts(i, j);
									// region { 0, k }
								}
							}
							else
							{
								if (Discard2D(s0, sk, sj))
								{
									dir = ProjectOriginOnPlane(s0, sj, sk);
									RemoveVert(i);
									// region { 0, j, k }
								}
								else
								{
									dir = ProjectOriginOnLine(s0, sk);
									RemoveVerts(i, j);
									// region { 0, k }
								}
							}
						}
						else if (testLine[j])
						{
							if (Discard2D(s0, sk, sj))
							{
								if (Discard2D(s0, sj, sk))
								{
									dir = ProjectOriginOnPlane(s0, sj, sk);
									RemoveVert(i);
									// region { 0, j, k }
								}
								else
								{
									dir = ProjectOriginOnLine(s0, sj);
									RemoveVerts(i, k);
									// region { 0, j }
								}
							}
							else
							{
								if (Discard2D(s0, sk, si))
								{
									dir = ProjectOriginOnPlane(s0, si, sk);
									RemoveVert(j);
									// region { 0, i, k }
								}
								else
								{
									dir = ProjectOriginOnLine(s0, sk);
									RemoveVerts(i, j);
									// region { 0, k }
								}
							}
						}
						else
						{
							ASSERT(false, "Unknown Error... because OpenGJK makes no damn sense");
						}
					}
					else if (dotTotal == 3)
					{
						Bool const discard2_sik = not Discard2D(s0, si, sk);
						Bool const discard2_sjk = not Discard2D(s0, sj, sk);
						Bool const discard2_ski = not Discard2D(s0, sk, si);
						Bool const discard2_skj = not Discard2D(s0, sk, sj);

						if (discard2_ski && discard2_skj)
						{
							dir = ProjectOriginOnLine(s0, sk);
							RemoveVerts(i, j);
							// region { 0, k }
						}
						else if (discard2_ski)
						{
							if (discard2_sjk)
							{
								dir = ProjectOriginOnLine(s0, sj);
								RemoveVerts(i, k);
								// region { 0, j }
							}
							else
							{
								dir = ProjectOriginOnPlane(s0, sk, sj);
								RemoveVert(i);
								// region { 0, j, k }
							}
						}
						else
						{
							if (discard2_sik)
							{
								dir = ProjectOriginOnLine(s0, si);
								RemoveVerts(j, k);
								// region { 0, i }
							}
							else
							{
								dir = ProjectOriginOnPlane(s0, sk, si);
								RemoveVert(j);
								// region { 0, i, k }
							}
						}
					}
				}
				break; case 0:
				{
					// And even more dumb shit tests........!!!!!
					if      (dotTotal == 1)
					{
						Uint8 i = 0, j = 0, k = 0;
						if      (testLine[2]) { i = 2; j = 3; k = 1; }
						else if (testLine[3]) { i = 3; j = 1; k = 2; }
						else				  { i = 1; j = 2; k = 3; } 

						Vec3 const& si = Vert(i);
						Vec3 const& sj = Vert(j);
						Vec3 const& sk = Vert(k);

						if (Discard2D(s0, si, sj))
						{
							dir = ProjectOriginOnPlane(s0, si, sj);
							RemoveVert(k);
							// region { 0, i, j }
						}
						else if (Discard2D(s0, si, sk))
						{
							dir = ProjectOriginOnPlane(s0, si, sk);
							RemoveVert(j);
							// region { 0, i, k }
						}
						else
						{
							dir = ProjectOriginOnLine(s0, si);
							RemoveVerts(j, k);
							// region { 0, i }
						}
					}
					else if (dotTotal == 2)
					{
						Uint8 i = 0, j = 0, k = 0;
						if      (not testLine[2]) { i = 2; j = 3; k = 1; }
						else if (not testLine[3]) { i = 3; j = 1; k = 2; }
						else                      { i = 1; j = 2; k = 3; }

						Vec3 const& si = Vert(i);
						Vec3 const& sj = Vert(j);
						Vec3 const& sk = Vert(k);

						if (Discard2D(s0, sj, sk))
						{
							if (Discard2D(s0, sk, sj))
							{
								dir = ProjectOriginOnPlane(s0, sj, sk);
								RemoveVert(i);
								// region { 0, j, k }
							}
							else if (Discard2D(s0, sk, si))
							{
								dir = ProjectOriginOnPlane(s0, si, sk);
								RemoveVert(j);
								// region { 0, i, k }
							}
							else
							{
								dir = ProjectOriginOnLine(s0, sk);
								RemoveVerts(i, j);
								// region { 0, k }
							}
						}
						else if (Discard2D(s0, sj, si))
						{
							dir = ProjectOriginOnPlane(s0, si, sj);
							RemoveVert(k);
							// region { 0, i, j }
						}
						else
						{
							dir = ProjectOriginOnLine(s0, sj);
							RemoveVerts(i, k);
							// region { 0, j }
						}
					}
				}
			}
		}
	}
#endif

	inline void SimplexSolver::RemoveVert(Uint8 i)
	{
		ASSERT(i < size && i != 0, "Invalid index");
		ASSERT(size != 0, "Cannot remove from an empty proxy");

		for (; i < size - 1; ++i)
		{
			verts[i] = verts[i + 1];
			idxs[i] = idxs[i + 1];
		}

		size--;
	}
	
	inline void SimplexSolver::RemoveVerts(Uint8 idxA, Uint8 idxB)
	{
		ASSERT(idxA < size && idxA != 0, "Invalid index");
		ASSERT(idxB < size && idxB != 0, "Invalid index");
		ASSERT(idxA != idxB, "Cannot remove the same vert twice");
		ASSERT(size > 1, "Cannot remove two verts from proxy of size 1 or 0");

		switch (idxA + idxB)
		{
			break; case 3: // remove verts 1 and 2
			{
				verts[1] = verts[3];
				idxs[1] = idxs[3];
			}
			break; case 4: // remove verts 1 and 3
			{
				verts[1] = verts[2];
				idxs[1] = idxs[2];
			}
			// case 5: remove verts 2 and 3... nothing to do
		}

		size -= 2;
	}

	inline void SimplexSolver::UpdateSimplex(Simplex& s) const
	{
		Uint8 const last = size - 1;
		for (Uint8 i = 0; i < size; ++i)
		{
			Uint8 const idx = idxs[last - i];
			
			// Remove vertices
			s.vertsA[i]    = s.vertsA[idx];
			s.vertsB[i]    = s.vertsB[idx];
			s.edgeIdxsA[i] = s.edgeIdxsA[idx];
			s.edgeIdxsB[i] = s.edgeIdxsB[idx];
		}

		s.size = size;
	}

    // Returns true if the origin is "in front of" plane pqr 
	// (outward from the tetrahedron with CCW triangle faces)
	static inline Bool OriginInFrontOfFace(Vec3 const& p, Vec3 const& q, Vec3 const& r)
	{
		Vec3 const n = glm::cross(q - p, r - p);
		return glm::dot(p, n) < 0.0f;
	}

    // Returns true if the origin is "in front of" the edge pq of 
    // triangle pqr (outward from the CCW triangle)
	static inline Bool OriginInFrontOfEdge(Vec3 const& p, Vec3 const& q, Vec3 const& r)
	{
		Vec3 const pq = q - p;
		Vec3 const n = TripleProduct(pq, pq, r - p);
		return glm::dot(p, n) < 0.0f;
	}

    // Returns true if origin is "in front of" the segment pq
    // (in the direction of pq)
	static inline Bool OriginInFrontOfVert(Vec3 const& p, Vec3 const& q)
	{
		return glm::dot(p, q - p) < 0.0f;
	}

	static inline Vec3 ProjectOriginOnLine(Vec3 const& p, Vec3 const& q)
	{
		Vec3 const qp = p - q;
		Float32 const t = glm::dot(p, qp) / glm::length2(qp);
		return p - qp * t;
	}

	static inline Vec3 ProjectOriginOnPlane(Vec3 const& p, Vec3 const& q, Vec3 const& r)
	{
		Vec3 const n = glm::cross(p - q, p - r);	
		Float32 const t = glm::dot(n, p) / glm::length2(n);
		return n * t;
	}
}