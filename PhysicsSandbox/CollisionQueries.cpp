#include "pch.h"
#include "CollisionQueries.h"

#include "Math.h"
#include "GeometryPrimitiveQueries.h"
#include "SATGJK.h"

#define COLLIDE_FCN_NOT_IMPLEMENTED return ContactManifold{};

namespace drb::physics {

	namespace util {
		// These functions "extract" the correct shape types from A and B then call the 
		// correct Collide overload		
		static inline ContactManifold CollideSphereSphere(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		static inline ContactManifold CollideSphereCapsule(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		static inline ContactManifold CollideSphereBox(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		static inline ContactManifold CollideSphereConvex(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		
		static inline ContactManifold CollideCapsuleCapsule(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		static inline ContactManifold CollideCapsuleConvex(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		static inline ContactManifold CollideCapsuleBox(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		
		static inline ContactManifold CollideBoxBox(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		static inline ContactManifold CollideBoxConvex(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);
		
		static inline ContactManifold CollideConvexConvex(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB);

		// Helper to create the contact manifold for face collisions found during SAT 
		static ContactManifold GenerateFaceContact(FaceQuery const& fq, Convex const& reference, Mat4 const& refTr, Convex const& incident, Mat4 const& incTr);

		// Converts a polygon to a contact manifold the uses the best 4 (or fewer) points
		// This has a side effect of projecting incFacePoly onto refFacePlane
		static	ContactManifold ReduceContactSet(Polygon& incFacePoly, Plane const& refFacePlane, Mat4 const& refTr);

		// Helper to flip the normal and features of a manifold
		static inline void Flip(ContactManifold& m);
	}


	ContactManifold Collide(ConstShapePtr A, Mat4 const& trA, ConstShapePtr B, Mat4 const& trB)
	{
		using CollideFcn = decltype(&util::CollideSphereSphere);

		static constexpr CollideFcn dispatchTable[4][4] = {
			{&util::CollideSphereSphere,  &util::CollideSphereCapsule,  &util::CollideSphereBox,  &util::CollideSphereConvex},
			{nullptr,                     &util::CollideCapsuleCapsule, &util::CollideCapsuleBox, &util::CollideCapsuleConvex},
			{nullptr,                     nullptr,                      &util::CollideBoxBox,     &util::CollideBoxConvex},
			{nullptr,                     nullptr,                      nullptr,                  &util::CollideConvexConvex}
		};

		auto GetTypeID = [](auto&& s) -> Int16 {
			return static_cast<Int16>(s->type) - 1;
		};

		Int16 const iA = std::visit(GetTypeID, A);
		Int16 const iB = std::visit(GetTypeID, B);

		if (iA < 0 || iA > 4 || iB < 0 || iB > 4) 
		{ 
			ASSERT(0 <= iA && iA < 4, "Invalid type");
			ASSERT(0 <= iB && iB < 4, "Invalid type"); 
			return ContactManifold{}; 
		}


		if (iA <= iB) 
		{
			return dispatchTable[iA][iB](A, trA, B, trB);
		}
		
		// else swap args then flip the result
		ContactManifold m = dispatchTable[iB][iA](B, trB, A, trA);
		util::Flip(m);
		return m;
	}


	ContactManifold Collide(Sphere const& A, Vec3 const& dispA, Sphere const& B, Vec3 const& dispB)
	{
		ContactManifold result{};

		// Get world-space centers
		Vec3 const cA = dispA + A.Position();
		Vec3 const cB = dispB + B.Position();

		// Compute displacement between centers
		Vec3 const disp  = cB - cA;
		Float32 const d2 = glm::length2(disp);

		// Test if centers are at the same point
		if (EpsilonEqual(d2, 0.0f, 1.0e-12f))
		{
			result.normal = Vec3(1, 0, 0);// arbitrary direction 
		}
		else
		{
			Float32 const dist = glm::sqrt(d2);
			Float32 const pen = (A.r + B.r) - dist;
			if (pen > 0.0f) {

				result.normal = disp / dist;

				result.contacts[result.numContacts++] = Contact{
					// contact position is halfway between the surface points of two spheres
					.position = result.normal * (A.r - 0.5f * pen) + cA,
					.penetration = pen,
				};
			}
		}

		return result;
	}

	ContactManifold Collide(Sphere const& A, Vec3 const& dispA, Capsule const& B, Mat4 const& trB)
	{
		Segment const segB = B.CentralSegment().Transformed(trB);
		Vec3 const cA = dispA + A.Position();

		// Closest point to a on internal line segment of b
		Vec3 const p = ClosestPoint(segB, cA);

		// Construct a sphere on the fly, centered at the closest point, p,
		// to sphere A on central segment of capsule B with radius of capsule B
		Sphere const S{ p, B.r};

		return Collide(A, cA, S, Vec3(0));
	}

	ContactManifold Collide(Sphere const& A, Vec3 const& dispA, Box const& B, Mat4 const& trB)
	{
		COLLIDE_FCN_NOT_IMPLEMENTED
	}

	ContactManifold Collide(Sphere const& A, Vec3 const& dispA, Convex const& B, Mat4 const& trB_)
	{
		COLLIDE_FCN_NOT_IMPLEMENTED

		Vec3 const cA = dispA + A.Position();
		Mat4 const trB = trB_ * B.Transform();

		auto const gjkResult = util::GJK(cA, B, trB);

		Vec3 const normal = Normalize(gjkResult.ptB - gjkResult.ptA);

		/*if (gjkResult.d2 > A.r * A.r)
		{
			return ContactManifold{
				.normal = normal
			};
		}
		else if (gjkResult.d2 > 0.0f)
		{
			return ContactManifold{
				.contacts = Contact{
					.position = gjkResult.ptA + normal * A.r,
					.penetration = A.r - glm::sqrt(gjkResult.d2),
				},
				.numContacts = 1,
				.normal = normal,
			};
		}
		else // deep collision
		{
			// find best separating axis (face normal of B)
		}
		*/

		// DEBUG
		return ContactManifold{
				.contacts = {
					Contact{
						.position = gjkResult.ptA,
						.penetration = 0.0f,
					},
					Contact{
						.position = gjkResult.ptB,
						.penetration = 0.0f,
					},
				},
				.numContacts = 2,
				.normal = normal,
		};
		// END DEBUG
	}


	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB)
	{
		static constexpr Float32 tol = 1.0e-12f;

		Segment const segA = A.CentralSegment().Transformed(trA);
		Segment       segB = B.CentralSegment().Transformed(trB); // not const b/c may swap b with e later

		Vec3 const    vecA  = segA.Vector();
		Vec3 const    vecB  = segB.Vector();
		Float32 const mag2A = segA.Length2();
		Float32 const mag2B = segB.Length2();

		// Check for capsules degenerating to spheres. If they do,
		// we can just do a sphere-sphere or sphere-capsule collision.
		Bool const AisSphere = mag2A < tol;
		Bool const BisSphere = mag2B < tol;
		if (AisSphere)
		{
			Sphere const sA{ segA.b, A.r};
			if (BisSphere) // Both spheres
			{
				Sphere const sB{ segB.b, B.r };
				return Collide(sA, Vec3(0), sB, Vec3(0)); // Sphere-Sphere
			}

			// Else: only A is a sphere
			return Collide(sA, Vec3(0), B, trB); // Sphere-Capsule
		}
		else if (BisSphere)
		{
			Sphere const sB{ segB.b, B.r };
			ContactManifold m = Collide(sB, Vec3(0), A, trA); // Sphere-Capsule
			util::Flip(m);
			return m;
		}
		// Else: neither capsule is degenerate


		// Detect parallel capsules
		Float32 const AdotB = glm::dot(vecA, vecB);
		Bool const parallel = glm::abs(AdotB * AdotB - mag2A * mag2B) < tol;

		if (not parallel)
		{
			// Find closest points on central segments of each capsule
			auto const [pA, pB, d2] = ClosestPointsNonDegenerate(segA, segB, vecA, vecB, mag2A, mag2B);
			if (d2 > (A.r + B.r) * (A.r + B.r)) {
				return ContactManifold{
					.normal = EpsilonEqual(d2, 0.0f, tol) ? Vec3(1,0,0) : (pB - pA) / glm::sqrt(d2)
				};
			}

			// Construct two spheres on the fly centered at closest points between
			// central segments and with radii equal to that of the capsules
			Sphere const sA{ pA, A.r }, sB{ pB, B.r };
			return Collide(sA, Vec3(0), sB, Vec3(0)); // Sphere-Sphere
		}
		// Else: capsules are parallel

		// Need to find two contact points, clipping the central
		// segments against each other. To do this, first we'll
		// project beginning of segB onto the line defined by segA.
		Vec4 const pLineA0 = ClosestPointOnLine(segB.b, vecA, segA.b);

		// To clip this point, we'll clamp the "t" param from
		// the above computation, then use this to compute 
		// the point on segA
		Float32 const t0 = glm::clamp(pLineA0.w, 0.0f, 1.0f);
		Vec3 const pA0 = vecA * t0 + segA.b;

		// For the other point, we'll just handle the clamping directly
		// within the utility function
		Vec4 const pA1 = ClosestPoint(segA, segB.e);

		// Test if we've clipped down to a single endpoint
		if (EpsilonEqual(t0, pA1.w, tol))
		{
			// Do sphere capsule collision
			Sphere const sA{ pA0, A.r };
			return Collide(sA, Vec3(0), B, trB); // Sphere-Capsule
		}
		// Else: we actually do have 2 points

		// The axis of collision MUST be orthogonal to both capsule 
		// segments and can be trivially obtained from our line proj
		Vec3 const axis = segB.b - Vec3(pLineA0);
		Float32 const d2 = glm::length2(axis);

		// Check for overlapping segments
		if (EpsilonEqual(d2, 0.0f, tol))
		{
			return ContactManifold{
				.normal = Vec3(1,0,0)
			};
		}

		// Check for collision
		Float32 const dist = glm::sqrt(d2);
		Vec3 const normal = axis / dist;
		Float32 const pen = A.r + B.r - dist;
		if (pen > 0.0f)
		{
			// Collision!
			return ContactManifold{
				.contacts = {
					Contact{
						.position = normal * (A.r - 0.5f * pen) + pA0,
						.penetration = pen
					},
					Contact{
						.position = normal * (A.r - 0.5f * pen) + Vec3(pA1),
						.penetration = pen
					}
				},
				.numContacts = 2,
				.normal = normal
			};
		}
		// Else: no collision

		return ContactManifold{
			.normal = normal
		};
	}

	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Box const& B, Mat4 const& trB)
	{
		COLLIDE_FCN_NOT_IMPLEMENTED
	}

	ContactManifold Collide(Capsule const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB_)
	{
		COLLIDE_FCN_NOT_IMPLEMENTED

		Mat4 const trB = trB_ * B.Transform();

		Segment const segA = A.CentralSegment().Transformed(trA);
		auto const gjkResult = util::GJK(segA, B, trB);

		Vec3 const normal = Normalize(gjkResult.ptB - gjkResult.ptA);

		/*if (gjkResult.d2 > A.r * A.r)
		{
			return ContactManifold{
				.normal = normal
			};
		}
		else if (gjkResult.d2 > 0.0f)
		{
			return ContactManifold{
				.contacts = Contact{
					.position = gjkResult.ptA + normal * A.r,
					.penetration = A.r - glm::sqrt(gjkResult.d2),
				},
				.numContacts = 1,
				.normal = normal,
			};
		}
		else // deep collision
		{
			// find best separating axis (face normal of B)
		}
		*/

		// DEBUG
		return ContactManifold{
				.contacts = {
					Contact{
						.position = gjkResult.ptA,
						.penetration = 0.0f,
					},
					Contact{
						.position = gjkResult.ptB,
						.penetration = 0.0f,
					},
				},
				.numContacts = 2,
				.normal = normal,
		};
		// END DEBUGs
	}


	ContactManifold Collide(Box const& A, Mat4 const& trA_, Box const& B, Mat4 const& trB_)
	{
		// These epsilon values are used to prevent issues with parallel axes
		static constexpr Float32 epsilon = 1.0e-4f;
		static constexpr Mat3 epsilonMat3 = Mat3{ Vec3(epsilon), Vec3(epsilon), Vec3(epsilon) };

		// These values are used to make Face A preferable to Face B, and both faces
		// preferable to an Edge Pair when deciding what contact to generate.
		// Since values are negative, this multiplier makes values closer
		// to zero, therefore more preferable.
		static constexpr Float32 linearSlop = 0.005f;
		static constexpr Float32 relEdgeTolerance = 0.90f;
		static constexpr Float32 relFaceTolerance = 0.98f;
		static constexpr Float32 absTolerance = 0.5f * linearSlop;

		// Perform computations in local space of A
		Mat4 const trA = trA_ * A.Transform();
		Mat4 const trB = trB_ * B.Transform();
		Mat3 const rotA = trA;
		Mat3 const rotB = trB;
		Vec3 const posA = trA[3];
		Vec3 const posB = trB[3];

		Mat3 const R = glm::transpose(rotA) * rotB;                                           // expresses B in coordinate frame of A. transpose (inverse) goes the other way from A's coords to B's.
		Mat3 const absR = Mat3(glm::abs(R[0]), glm::abs(R[1]), glm::abs(R[2])) + epsilonMat3; // epsilon used to handle nearly parallel edges
		Vec3 const worldT = posB - posA;
		Vec3 const t = glm::transpose(rotA) * worldT;

		// Helper to go from an axis index to a vector in local space of A
		auto IndexToAxis = [&](Int32 axisIndex) -> Vec3 {
			switch (axisIndex) {
			break; case 0: { return Vec3(1, 0, 0); }
			break; case 1: { return Vec3(0, 1, 0); }
			break; case 2: { return Vec3(0, 0, 1); }
			break; case 3: { return R * Vec3(1, 0, 0); }
			break; case 4: { return R * Vec3(0, 1, 0); }
			break; case 5: { return R * Vec3(0, 0, 1); }

			break; case 6: { return glm::normalize(glm::cross(Vec3(1, 0, 0), R * Vec3(1, 0, 0))); }
			break; case 7: { return glm::normalize(glm::cross(Vec3(1, 0, 0), R * Vec3(0, 1, 0))); }
			break; case 8: { return glm::normalize(glm::cross(Vec3(1, 0, 0), R * Vec3(0, 0, 1))); }
			break; case 9: { return glm::normalize(glm::cross(Vec3(0, 1, 0), R * Vec3(1, 0, 0))); }
			break; case 10: { return glm::normalize(glm::cross(Vec3(0, 1, 0), R * Vec3(0, 1, 0))); }
			break; case 11: { return glm::normalize(glm::cross(Vec3(0, 1, 0), R * Vec3(0, 0, 1))); }
			break; case 12: { return glm::normalize(glm::cross(Vec3(0, 0, 1), R * Vec3(1, 0, 0))); }
			break; case 13: { return glm::normalize(glm::cross(Vec3(0, 0, 1), R * Vec3(0, 1, 0))); }
			break; case 14: { return glm::normalize(glm::cross(Vec3(0, 0, 1), R * Vec3(0, 0, 1))); }
			}
			return Vec3(0);
		};

		// Helper to obtain clipped incident face in the local space of the reference box
		auto ClipIncidentToReference = [](Box const& ref, Box const& inc, Int32 refAxis, Box::Face incFace, Mat4 const& incToRef) -> Polygon
		{
			Polygon result = inc.FaceAsPolygon(incFace);
			for (auto&& v : result.verts)
			{
				v = incToRef * Vec4(v, 1);
			}

			Polygon front{}, back{};
			Int32 const j = (refAxis + 1) % 3;
			Int32 const k = (j + 1) % 3;
			Plane clipPlane{ .n = Vec3(0), .d = 0 };

			{
				clipPlane.n[j] = 1.0f;
				clipPlane.d = ref.extents[j];

				result.Split(clipPlane, front, back);
				std::swap(result.verts, back.verts);

				back.verts.clear();
				clipPlane.n[j] = 0.0f;
			}
			{
				clipPlane.n[k] = 1.0f;
				clipPlane.d = ref.extents[k];

				result.Split(clipPlane, front, back);
				std::swap(result.verts, back.verts);

				back.verts.clear();
				clipPlane.n[k] = 0.0f;
			}
			{
				clipPlane.n[j] = -1.0f;
				clipPlane.d = ref.extents[j];

				result.Split(clipPlane, front, back);
				std::swap(result.verts, back.verts);

				back.verts.clear();
				clipPlane.n[j] = 0.0f;
			}
			{
				clipPlane.n[k] = -1.0f;
				clipPlane.d = ref.extents[k];

				result.Split(clipPlane, front, back);
				std::swap(result.verts, back.verts);
			}
			return result;
		};


		// We keep track of the axis of greatest separation
		Int32   bestFaceAxisA = -1;
		Int32   bestFaceAxisB = -1;
		Int32   bestEdgeAxisA = -1, bestEdgeAxisB = -1;
		Float32 bestFaceSepA = std::numeric_limits<Float32>::lowest();
		Float32 bestFaceSepB = std::numeric_limits<Float32>::lowest();
		Float32 bestEdgeSep = std::numeric_limits<Float32>::lowest();

		ContactManifold result{};

		// Tests for face normals of A
		Vec3 const faceSepsA = glm::abs(t) - (A.extents + absR * B.extents);
		for (Int32 i = 0; i < 3; ++i) {
			if (faceSepsA[i] > 0.0f) {
				Vec3 localDir{ 0.0f };
				localDir[i] = 1.0f;

				result.normal = rotA * localDir;
				if (glm::dot(result.normal, worldT) < 0.0f) {
					result.normal *= -1.0f;
				}
				return result;
			}
			else if (faceSepsA[i] > bestFaceSepA) {
				bestFaceSepA = faceSepsA[i];
				bestFaceAxisA = i;
			}
		}

		// Tests for face normals of B
		Vec3 const faceSepsB = glm::abs(glm::transpose(R) * t) - (glm::transpose(absR) * A.extents + B.extents);
		for (Int32 i = 0; i < 3; ++i) {
			if (faceSepsB[i] > 0.0f) {
				Vec3 localDir{ 0.0f };
				localDir[i] = 1.0f;

				result.normal = rotB * localDir;
				if (glm::dot(result.normal, worldT) < 0.0f) {
					result.normal *= -1.0f;
				}
				return result;
			}
			else if (faceSepsB[i] > bestFaceSepB) {
				bestFaceSepB = faceSepsB[i];
				bestFaceAxisB = i;
			}
		}

		// Tests for axisA x axisB
		{
			Int32 m = 1, n = 2;
			for (Int32 a = 0; a < 3; ++a)
			{
				Vec3 localA{ 0.0f };
				localA[a] = 1.0f;

				Int32 j = 1, k = 2;

				for (Int32 b = 0; b < 3; ++b)
				{
					Float32 const ra = A.extents[m] * absR[b][n] + A.extents[n] * absR[b][m];
					Float32 const rb = B.extents[j] * absR[k][a] + B.extents[k] * absR[j][a];

					Float32 const sep = glm::abs(t[n] * R[b][m] - t[m] * R[b][n]) - (ra + rb);

					Vec3 localB{ 0.0f };
					localB[b] = 1.0f;

					if (sep > 0.0f) {
						result.normal = glm::cross(rotA * localA, rotB * localB);
						if (glm::dot(result.normal, worldT) < 0.0f) {
							result.normal *= -1.0f;
						}
						return result;
					}
					else if (sep > bestEdgeSep) {

						// If edges are not parallel, record the axis
						if (glm::abs(glm::dot(R * localB, localA)) + epsilon < 1.0f) {
							bestEdgeSep = sep;
							bestEdgeAxisA = a;
							bestEdgeAxisB = b;
						}
					}
					j = k;
					k = b;
				}
				m = n;
				n = a;
			}
		}


		Bool const edgeContact = bestEdgeSep > (relEdgeTolerance * glm::max(bestFaceSepA, bestFaceSepB) + absTolerance);
		Bool const faceBContact = bestFaceSepB > (relFaceTolerance * bestFaceSepA + absTolerance);

		Vec3 normalLocalA{};

		if (edgeContact) {
			// Compute contact normal in local space of A
			normalLocalA = IndexToAxis((bestEdgeAxisA * 3 + bestEdgeAxisB) + 6);
			if (glm::dot(normalLocalA, t) < 0.0f) { // flip s.t. pointing A->B
				normalLocalA *= -1.0f;
			}

			// Identify which edges (in world space) are in contact
			Segment const edgeA = A.SupportingEdgeWithDirection(bestEdgeAxisA, normalLocalA)
								   .Transformed(trA);
			Segment const edgeB = B.SupportingEdgeWithDirection(bestEdgeAxisB, glm::transpose(R) * -normalLocalA)
								   .Transformed(trB);

			// Compute closest points in world space
			auto const closestPts = ClosestPoints(edgeA, edgeB);

			// Build a contact point at midpoint between the two closest points
			result.contacts[result.numContacts++] = Contact{
				.position = 0.5f * (closestPts.ptA + closestPts.ptB),
				.penetration = -bestEdgeSep
			};
		}
		else {
			if (faceBContact) {
				// Compute normal in local space of A
				normalLocalA = IndexToAxis(bestFaceAxisB + 3);
				if (glm::dot(normalLocalA, t) < 0.0f) { // flip s.t. pointing A->B
					normalLocalA *= -1.0f;
				}
				Vec3 const normalLocalB = glm::transpose(R) * normalLocalA;

				auto const incidentFace  = A.SupportingFace(normalLocalA);
				Polygon clipFace = ClipIncidentToReference(B, A, bestFaceAxisB, incidentFace, glm::inverse(trB) * trA);
				Int32 const numClipVerts = static_cast<Int32>(clipFace.verts.size());

				ASSERT(0 <= numClipVerts && numClipVerts <= 8, "Invalid contact point count");

				// Compute penetration depths and project clip verts onto ref plane
				Plane const refFacePlane{ .n = -normalLocalB, .d = B.extents[bestFaceAxisB] };
				Float32 depths[8] = {};
				for (Int32 i = 0; i < numClipVerts; ++i) {
					depths[i] = SignedDistance(refFacePlane, clipFace.verts[i]);
					clipFace.verts[i] -= depths[i] * refFacePlane.n;
				}

				// Finally, generate the contact points
				for (Int32 i = 0; i < numClipVerts; ++i) {
					if (depths[i] < epsilon) {
						result.contacts[result.numContacts++] = Contact{
							.position = trB * Vec4(clipFace.verts[i], 1),
							.penetration = -depths[i]
						};
					}
				}
			}
			else { // Face A is reference

				// Compute normal in local space of A, and in world space, pointing A->B
				normalLocalA = IndexToAxis(bestFaceAxisA);
				if (glm::dot(normalLocalA, t) < 0.0f) {
					normalLocalA *= -1.0f;
				}
				Vec3 const normalLocalB = glm::transpose(R) * normalLocalA;

				auto const incidentFace = B.SupportingFace(-normalLocalB);
				Polygon clipFace = ClipIncidentToReference(A, B, bestFaceAxisA, incidentFace, glm::inverse(trA) * trB);
				Int32 const numClipVerts = static_cast<Int32>(clipFace.verts.size());

				ASSERT(0 <= numClipVerts && numClipVerts <= 8, "Invalid contact point count");

				// Compute penetration depths and project clip verts onto ref plane
				Plane const refFacePlane{ .n = normalLocalA, .d = A.extents[bestFaceAxisA] };
				Float32 depths[8] = {};
				for (Int32 i = 0; i < numClipVerts; ++i) {
					depths[i] = SignedDistance(refFacePlane, clipFace.verts[i]);
					clipFace.verts[i] -= depths[i] * refFacePlane.n;
				}

				// Finally, generate the contact points
				for (Int32 i = 0; i < numClipVerts; ++i) {
					if (depths[i] < epsilon) {
						result.contacts[result.numContacts++] = Contact{
							.position = trA * Vec4(clipFace.verts[i], 1),
							.penetration = -depths[i]
						};
					}
				}
			}
		}

		result.normal = rotA * normalLocalA;

		ASSERT(glm::epsilonEqual(glm::length2(result.normal), 1.0f, epsilon), "Invalid normal");
		return result;
	}

	ContactManifold Collide(Box const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
	{
		COLLIDE_FCN_NOT_IMPLEMENTED
	}


	ContactManifold Collide(Convex const& A, Mat4 const& trA_, Convex const& B, Mat4 const& trB_)
	{
		Mat4 const trA = trA_ * A.Transform();
		Mat4 const trB = trB_ * B.Transform();

		// Test faces of A
		util::FaceQuery fqA = util::SATQueryFaceDirections(A, trA, B, trB);
		if (fqA.separation > 0.0f)
		{
			return ContactManifold{

				.normal = fqA.normal

			}; // no contacts, but track the separating normal
		}

		// Test faces of B
		util::FaceQuery fqB = util::SATQueryFaceDirections(B, trB, A, trA);
		if (fqB.separation > 0.0f)
		{
			return ContactManifold{

				.normal = -1.0f * fqB.normal  // normal was pointing from B -> A, so flip it

			}; // no contacts, but track the separating normal
		}

		// Test edge pairs
		util::EdgeQuery eq = util::SATQueryEdgeDirections(A, trA, B, trB);
		if (eq.separation > 0.0f)
		{
			return ContactManifold{

				.normal = eq.normal

			}; // no contacts, but track the separating normal
		}

		// Now generate the contact manifold!
		ContactManifold result{ };

		// These values are used to make Face A preferable to Face B, and both faces
		// preferable to an Edge Pair when deciding what contact to generate.
		// Since values are negative, this multiplier makes values closer
		// to zero, therefore more preferable.
		static constexpr Float32 linearSlop = 0.005f;
		static constexpr Float32 relEdgeTolerance = 0.90f;
		static constexpr Float32 relFaceTolerance = 0.98f;
		static constexpr Float32 absTolerance = 0.5f * linearSlop;


		Bool const edgeContact = eq.separation > relEdgeTolerance * std::max(fqA.separation, fqB.separation) + absTolerance;
		Bool const faceBContact = fqB.separation > relFaceTolerance * fqA.separation + absTolerance;

		// Create Edge-Edge contact
		if (edgeContact)
		{
			auto const edgesA = A.GetEdges();
			auto const vertsA = A.GetVerts();
			auto const edgesB = B.GetEdges();
			auto const vertsB = B.GetVerts();

			// First find closest points on the two witness edges
			Convex::HalfEdge const edgeA = edgesA[eq.indexA];
			Convex::HalfEdge const twinA = edgesA[edgeA.twin];
			Segment const edgeSegA = Segment{ .b = vertsA[edgeA.origin], .e = vertsA[twinA.origin] }.Transformed(trA);

			Convex::HalfEdge const edgeB = edgesB[eq.indexB];
			Convex::HalfEdge const twinB = edgesB[edgeB.twin];
			Segment const edgeSegB = Segment{ .b = vertsB[edgeB.origin], .e = vertsB[twinB.origin] }.Transformed(trB);

			ClosestPointsQuery closestPts = ClosestPoints(edgeSegA, edgeSegB);

			// Then build the contact with position at midpoint between closest points
			result.contacts[result.numContacts++] = Contact{
				.position = 0.5f * (closestPts.ptA + closestPts.ptB),
				.penetration = -eq.separation
			};
			result.normal = eq.normal;
			result.featureA = { .index = eq.indexA, .type = Feature::Type::Edge };
			result.featureB = { .index = eq.indexB, .type = Feature::Type::Edge };
		}
		else
		{
			if (faceBContact)
			{
				result = util::GenerateFaceContact(fqB, B, trB, A, trA);
				util::Flip(result);
			}
			else
			{
				result = util::GenerateFaceContact(fqA, A, trA, B, trB);
			}
		}

		return result;
	}

	namespace util {

		static inline ContactManifold CollideSphereSphere(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Sphere const*>(A_);
			auto const* B = std::get<Sphere const*>(B_);
			return Collide(*A, trA[3], *B, trB[3]);
		}

		static inline ContactManifold CollideSphereCapsule(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Sphere const*>(A_);
			auto const* B = std::get<Capsule const*>(B_);
			return Collide(*A, trA[3], *B, trB);
		}

		static inline ContactManifold CollideSphereBox(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Sphere const*>(A_);
			auto const* B = std::get<Box const*>(B_);
			return Collide(*A, trA[3], *B, trB);
		}

		static inline ContactManifold CollideSphereConvex(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Sphere const*>(A_);
			auto const* B = std::get<Convex const*>(B_);
			return Collide(*A, trA[3], *B, trB);
		}

		static inline ContactManifold CollideCapsuleCapsule(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Capsule const*>(A_);
			auto const* B = std::get<Capsule const*>(B_);
			return Collide(*A, trA, *B, trB);
		}
		
		static inline ContactManifold CollideCapsuleBox(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Capsule const*>(A_);
			auto const* B = std::get<Box const*>(B_);
			return Collide(*A, trA, *B, trB);
		}

		static inline ContactManifold CollideCapsuleConvex(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Capsule const*>(A_);
			auto const* B = std::get<Convex const*>(B_);
			return Collide(*A, trA, *B, trB);
		}

		static inline ContactManifold CollideBoxBox(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Box const*>(A_);
			auto const* B = std::get<Box const*>(B_);
			return Collide(*A, trA, *B, trB);
		}
		
		static inline ContactManifold CollideBoxConvex(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Box const*>(A_);
			auto const* B = std::get<Convex const*>(B_);
			return Collide(*A, trA, *B, trB);
		}
		
		static inline ContactManifold CollideConvexConvex(ConstShapePtr A_, Mat4 const& trA, ConstShapePtr B_, Mat4 const& trB)
		{
			auto const* A = std::get<Convex const*>(A_);
			auto const* B = std::get<Convex const*>(B_);
			return Collide(*A, trA, *B, trB);
		}

		static inline void Flip(ContactManifold& m)
		{
			m.normal *= -1.0f;
			std::swap(m.featureA, m.featureB);
		}


		// See Dirk Gregorius "Robust Contact Manifolds" GDC
		static ContactManifold GenerateFaceContact(FaceQuery const& fq, Convex const& reference, Mat4 const& refTr, Convex const& incident, Mat4 const& incTr)
		{
			Mat4 const incToRefLocal = glm::inverse(refTr) * incTr;

			ASSERT(fq.index >= 0 && fq.index <= Convex::MAX_INDEX, "Invalid index");
			Convex::FaceID const refFaceIdx = static_cast<Convex::FaceID>(fq.index);
			Convex::Face const& refFace = reference.GetFace(refFaceIdx);

			// Find the incident face
			Convex::FaceID incFaceIdx = Convex::INVALID_INDEX;
			Float32 minDotProd = std::numeric_limits<Float32>::max();
			auto const incidentFaces = incident.GetFaces();
			Int16 const numIncidentFaces = incident.NumFaces();
			for (SizeT i = 0; i < numIncidentFaces; ++i)
			{
				Vec3    const n   = incidentFaces[i].plane.Transformed(incToRefLocal).n;
				Float32 const dot = glm::dot(n, refFace.plane.n);
				if (dot < minDotProd)
				{
					minDotProd = dot;
					incFaceIdx = static_cast<Convex::FaceID>(i);
				}
			}

			// Clip the incident face against edge planes of reference face
			Polygon incFacePoly = incident.FaceAsPolygon(incFaceIdx, incToRefLocal);
			Polygon frontPoly{}, backPoly{};
			reference.ForEachEdgeOfFace(refFaceIdx, [&](Convex::HalfEdge edge) {

				// Create a plane orthogonal to refFace and containing
				// endpoints of edge
				Convex::HalfEdge const twin = reference.GetEdge( edge.twin );
				Vec3 const p0 = reference.GetVert(edge.origin);
				Vec3 const p1 = reference.GetVert(twin.origin);
				Vec3 const outwardNormal = Normalize(glm::cross(p1 - p0, refFace.plane.n));
				Plane const edgePlane = Plane::Make(outwardNormal, p0); // normal pointing "outward" away from face center

				// Split incident face on edgePlane
				incFacePoly.Split(edgePlane, frontPoly, backPoly);

				// Save the points behind or on the edgePlane
				std::swap(incFacePoly.verts, backPoly.verts);
				backPoly.verts.clear();
			});

			// Keep only points in clipped incident face which are behind or on reference face plane
			std::erase_if(incFacePoly.verts, [&refFace](Vec3 const& p) {
				return Classify(refFace.plane, p) == Side::Front;
			});


			// Reduce the manifold to at most 4 points
			ContactManifold m = ReduceContactSet(incFacePoly, refFace.plane, refTr);

			m.featureA = { .index = fq.index,   .type = Feature::Type::Face };
			m.featureB = { .index = incFaceIdx, .type = Feature::Type::Face };
			m.normal = Normalize(Mat3(refTr) * refFace.plane.n);

			return m;
		}


		// Reduce the clipped and projected incident face to at most 4 contact 
		// points. This is a bit of a process -- see D.G.'s GDC talk...
		static ContactManifold ReduceContactSet(Polygon& incFacePoly, Plane const& refFacePlane, Mat4 const& refTr)
		{
			ContactManifold m{};

			// Possible that clipping has removed ALL points...
			Int32 const numCandidates = static_cast<Int32>(incFacePoly.verts.size());
			if (numCandidates <= 0) { return m; }

			static constexpr Uint32 maxCandidates = 64;
			ASSERT(numCandidates < maxCandidates, "Too many potential contacts. You probably want to reduce the complexity of your collision geometry.");

			// First, project contact points onto reference face, and identify the deepest 
			// point (which is needed for CCD) -- this will be our first contact point
			Float32 depths[maxCandidates] = {};
			Uint32  p0Idx = 0;
			Float32 deepest = std::numeric_limits<Float32>::max();
			for (Int32 i = 0; i < numCandidates; ++i) {

				Float32 const depth = SignedDistance(refFacePlane, incFacePoly.verts[i]);
				if (depth < deepest)
				{
					deepest = depth;
					p0Idx = i;
				}

				// Save the depth then project onto ref face
				depths[i] = depth;
				incFacePoly.verts[i] -= depth * refFacePlane.n;
			}
			Vec3 const p0 = incFacePoly.verts[p0Idx];
			m.contacts[m.numContacts++] = Contact{
				.position = refTr * Vec4(p0, 1.0f),
				.penetration = -deepest
			};

			// If we already only have at most 4 contacts, just make the 
			// manifold directly from incFacePoly
			if (0 <= numCandidates && numCandidates <= 4)
			{
				Uint32 const N = static_cast<Uint32>(numCandidates);
				Uint32 curr = (p0Idx + 1) % N;
				while (curr != p0Idx)
				{
					m.contacts[m.numContacts++] = Contact{
						.position = refTr * Vec4(incFacePoly.verts[curr], 1.0f),
						.penetration = -depths[curr]
					};

					curr++;
					curr %= N;
				}
				return m;
			}

			// If we have more than 4 potential candidate points, we'll first 
			// identify the candidate furthest away from p0 that is not adjacent 
			// to p0 (in order to prevent p0->p1 from being on the edge of
			// manifold -- this is needed to effectively find points 3 and 4).
			Int32   p1Idx = -1;
			Float32 furthest = std::numeric_limits<Float32>::lowest();

			Int32 const end = (p0Idx - 1 + numCandidates) % numCandidates; // previous vert adjacent to p0
			Int32       curr = (p0Idx + 1) % numCandidates;				   // next vert adjacent to p0
			while (curr != end)
			{
				Float32 const dist2 = glm::distance2(p0, incFacePoly.verts[curr]);
				if (dist2 > furthest)
				{
					furthest = dist2;
					p1Idx = curr;
				}

				curr++;
				curr %= numCandidates;
			}

			if (p1Idx < 0) { return m; }
			//ASSERT(p1Idx >= 0, "Invalid index");
			Vec3 const p1 = incFacePoly.verts[p1Idx];
			m.contacts[m.numContacts++] = Contact{
				.position = refTr * Vec4(p1, 1.0f),
				.penetration = -depths[p1Idx]
			};

			// 3rd and 4th points are tricky! We want to maximize the manifold
			// area without comprimising stability.

			// For point 3, look for a point on the side of line p0->p1 that gives
			// us the maximal triangle area.
			Int32   p2Idx = -1;
			Float32 maxTriArea = 0.0f;
			for (Int32 i = 0; i < numCandidates; ++i)
			{
				if (i == p0Idx || i == p1Idx) { continue; }

				Vec3 const u = p0 - incFacePoly.verts[i];
				Vec3 const v = p1 - incFacePoly.verts[i];
				Float32 const area = glm::dot(glm::cross(u, v), refFacePlane.n);
				if (area > maxTriArea)
				{
					maxTriArea = area;
					p2Idx = i;
				}
			}

			if (p2Idx < 0) { return m; }
			//ASSERT(p2Idx >= 0, "Invalid index");
			Vec3 const p2 = incFacePoly.verts[p2Idx];
			m.contacts[m.numContacts++] = Contact{
				.position = refTr * Vec4(p2, 1.0f),
				.penetration = -depths[p2Idx]
			};


			// Finally, we identify the point on the opposite side of 
			// edge p0-p1 from p2 that maximizes the area of the quad
			// by looking for the point that would give us the most 
			// negative area when a triangle is formed with a pair of 
			// our current points
			Int32   p3Idx = -1;
			Float32 minTriArea = 0.0f;
			for (Int32 i = 0; i < numCandidates; ++i)
			{
				if (i == p0Idx || i == p1Idx || i == p2Idx) { continue; }

				Vec3 const u = p0 - incFacePoly.verts[i];
				Vec3 const v = p1 - incFacePoly.verts[i];
				Vec3 const w = p2 - incFacePoly.verts[i];

				// Test triangle p0p1p3
				Float32 area = glm::dot(glm::cross(u, v), refFacePlane.n);
				if (area < minTriArea)
				{
					minTriArea = area;
					p3Idx = i;
				}

				// Test triangle p1p2p3
				area = glm::dot(glm::cross(v, w), refFacePlane.n);
				if (area < minTriArea)
				{
					minTriArea = area;
					p3Idx = i;
				}

				// Test triangle p2p0p3
				area = glm::dot(glm::cross(w, u), refFacePlane.n);
				if (area < minTriArea)
				{
					minTriArea = area;
					p3Idx = i;
				}
			}

			if (p3Idx < 0) { return m; }
			//ASSERT(p3Idx >= 0, "Invalid index");
			Vec3 const p3 = incFacePoly.verts[p3Idx];
			m.contacts[m.numContacts++] = Contact{
				.position = refTr * Vec4(p3, 1.0f),
				.penetration = -depths[p3Idx]
			};

			return m;
		}
	}
}