#include "pch.h"
#include "PhysicsGeometryQueries.h"

#include "PhysicsGeometry.h"
#include "SATGJK.h"

//#define COLLIDE_FCN_NOT_IMPLEMENTED ASSERT(false, "Not yet implemented."); return ContactManifold{};
#define COLLIDE_FCN_NOT_IMPLEMENTED return ContactManifold{};


namespace drb::physics {

		namespace util {

			// -----------------------------------------------------------------
			// HELPERS
			// -----------------------------------------------------------------

			// Returns closest point (in world space) on segment ls to point pt
			// Parameter "t" is stored in the 4th(w) coord of result, which allows
			// easy checking for if the result is an endpoint of ls.
			static inline Vec4               ClosestPoint(Vec3 const& pt, Segment const& ls);

			// Similar to above, but returns the point on the line (not segment!) defined by
			// the two points given by ls. Note the length of the direction vector will be used 
			// to parameterize the result point and will affect "t".
			static inline Vec4               ClosestPointOnLine(Vec3 const& pt, Segment const& ls);

			// Same as above, but representing a line as a point and direction vector (with
			// length > 0). Note the length of the direction vector will be used to parameterize
			// the result point and will affect "t".
			static inline Vec4				 ClosestPointOnLine(Vec3 const& pt, Vec3 const& lineDir, Vec3 const& linePt);

			// Returns closest points (in world space) on each segment to the other segment, and the
			// square distance between them
			static        ClosestPointsQuery ClosestPoints(Segment const& A, Segment const& B);

			// Same as above but does not check if A or B degenerate to a point, and requires
			// precomputation of their direction vectors (dA and dB) and their square magnitudes 
			// (mag2A and mag2B).
			static        ClosestPointsQuery ClosestPointsNonDegenerate(Segment const& A, Segment const& B, Vec3 const& dA, Vec3 const& dB, Float32 mag2A, Float32 mag2B);

			// Helper to swap the arguments A and B and call the appropriate collision function
			static inline ContactManifold    CollideFlipped(auto const& A, Mat4 const& trA, auto const& B, Mat4 const& trB);

			// Helper to handle when normal is oriented the wrong way for a manifold
			static inline void               Flip(ContactManifold& manifold);

			// Helper to create the contact manifold for face collisions found during SAT 
			static		  ContactManifold	 GenerateFaceContact(FaceQuery const& fq, Convex const& reference, Mat4 const& refTr, Convex const& incident, Mat4 const& incTr);
		
			// Converts a polygon to a contact manifold the uses the best 4 (or fewer) points
			// This has a side effect of projecting incFacePoly onto refFacePlane
			static		  ContactManifold    ReduceContactSet(Polygon& incFacePoly, Plane const& refFacePlane, Mat4 const& refTr);

			// The actual computation of the collision -- useful because internally we often have the center of the sphere
			// and do not want to convert a point to a matrix only to convert back to a point again
			static inline ContactManifold	 CollideSphereSphereImpl(Sphere const& A, Vec3 const& cA, Sphere const& B, Vec3 const& cB);
			static inline ContactManifold    CollideSphereCapImpl(Sphere const& A, Vec3 const& cA, Capsule const& B, Mat4 const& trB);
		}



		// ---------------------------------------------------------------------
		// TOP LEVEL DISPATCH
		// ---------------------------------------------------------------------
		
#define DEFN_COLLIDE_FCN(T, U) static ContactManifold Collide##T##U##(CollisionShapeBase const& A, Mat4 const& trA, CollisionShapeBase const& B, Mat4 const& trB) \
		{ \
			CollisionShape<T> const& castA = static_cast<CollisionShape<T> const&>(A); \
			CollisionShape<U> const& castB = static_cast<CollisionShape<U> const&>(B); \
			return Collide(castA.shape, trA * castA.transform, castB.shape, trB * castB.transform); \
		}
		DEFN_COLLIDE_FCN(Sphere, Sphere)
		DEFN_COLLIDE_FCN(Sphere, Capsule)
		DEFN_COLLIDE_FCN(Sphere, Convex)
		DEFN_COLLIDE_FCN(Capsule, Sphere)
		DEFN_COLLIDE_FCN(Capsule, Capsule)
		DEFN_COLLIDE_FCN(Capsule, Convex)
		DEFN_COLLIDE_FCN(Convex, Sphere)
		DEFN_COLLIDE_FCN(Convex, Capsule)
		DEFN_COLLIDE_FCN(Convex, Convex)

#undef DEFN_COLLIDE_FCN
			
		ContactManifold Collide(CollisionShapeBase const& A, Mat4 const& trA, CollisionShapeBase const& B, Mat4 const& trB)
		{
			using Fn = decltype(&CollideCapsuleCapsule);
			static constexpr Fn fcnTable[3][3] = {
				{&CollideSphereSphere,  &CollideSphereCapsule,  &CollideSphereConvex},
				{&CollideCapsuleSphere, &CollideCapsuleCapsule, &CollideCapsuleConvex},
				{&CollideConvexSphere,  &CollideConvexCapsule,  &CollideConvexConvex}
			};

			Int32 const idxA = static_cast<Int32>(A.type) - 1;
			Int32 const idxB = static_cast<Int32>(B.type) - 1;

			ASSERT(0 <= idxA && idxA < 3, "Index A out of range. Bad Type.");
			ASSERT(0 <= idxB && idxB < 3, "Index B out of range. Bad Type.");

			return fcnTable[idxA][idxB](A, trA, B, trB);
		}


		// ---------------------------------------------------------------------
		// SPHERE
		// ---------------------------------------------------------------------

		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB)
		{
			return util::CollideSphereSphereImpl(A, trA[3], B, trB[3]);
		}


		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB)
		{
			return util::CollideSphereCapImpl(A, trA[3], B, trB);
		}


		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
		{
			COLLIDE_FCN_NOT_IMPLEMENTED

			Vec3 const cA = trA[3];
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


		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB)
		{
			COLLIDE_FCN_NOT_IMPLEMENTED
		}


		// ---------------------------------------------------------------------
		// CAPSULE
		// ---------------------------------------------------------------------

		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB)
		{
			static constexpr Float32 tol = 1.0e-12f;

			Segment const segA = CentralSegment(A, trA);
			Segment segB = CentralSegment(B, trB); // not const b/c may swap b with e later

			Vec3 const vecA = segA.e - segA.b;
			Vec3 const vecB = segB.e - segB.b;
			Float32 const mag2A = glm::length2(vecA);
			Float32 const mag2B = glm::length2(vecB);

			// Check for capsules degenerating to spheres. If they do,
			// we can just do a sphere-sphere or sphere-capsule collision.
			Bool const AisSphere = mag2A < tol;
			Bool const BisSphere = mag2B < tol;
			if (AisSphere)
			{
				Sphere const sA{ A.r };
				if (BisSphere) // Both spheres
				{
					Sphere const sB{ B.r };
					return util::CollideSphereSphereImpl(sA, segA.b, sB, segB.b);
				}

				// Else: only A is a sphere
				return util::CollideSphereCapImpl(sA, segA.b, B, trB);
			}
			else if (BisSphere)
			{
				Sphere const sB{ B.r };
				ContactManifold m = util::CollideSphereCapImpl(sB, segB.b, A, trA);
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
				auto const [pA, pB, d2] = util::ClosestPointsNonDegenerate(segA, segB, vecA, vecB, mag2A, mag2B);
				if (d2 > (A.r + B.r) * (A.r + B.r)) {
					return ContactManifold{
						.normal = EpsilonEqual(d2, 0.0f, tol) ? Vec3(1,0,0) : (pB - pA) / glm::sqrt(d2)
					};
				}

				// Construct two spheres on the fly centered at closest points between
				// central segments and with radii equal to that of the capsules
				Sphere const sA{ A.r }, sB{ B.r };
				return util::CollideSphereSphereImpl(sA, pA, sB, pB);
			}
			// Else: capsules are parallel
			
			// Need to find two contact points, clipping the central
			// segments against each other. To do this, first we'll
			// project beginning of segB onto the line defined by segA.
			Vec4 const pLineA0 = util::ClosestPointOnLine(segB.b, vecA, segA.b);
			
			// To clip this point, we'll clamp the "t" param from
			// the above computation, then use this to compute 
			// the point on segA
			Float32 const t0 = glm::clamp(pLineA0.w, 0.0f, 1.0f);
			Vec3 const pA0 = vecA * t0 + segA.b;
			
			// For the other point, we'll just handle the clamping directly
			// within the utility function
			Vec4 const pA1 = util::ClosestPoint(segB.e, segA);

			// Test if we've clipped down to a single endpoint
			if (EpsilonEqual(t0, pA1.w, tol))
			{
				// Do sphere capsule collision
				Sphere const sA{ A.r };
				return util::CollideSphereCapImpl(sA, pA0, B, trB);
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


		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
		{
			COLLIDE_FCN_NOT_IMPLEMENTED

			Segment const segA = CentralSegment(A, trA);
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


		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB)
		{
			COLLIDE_FCN_NOT_IMPLEMENTED
		}


		// ---------------------------------------------------------------------
		// CONVEX
		// ---------------------------------------------------------------------

		ContactManifold Collide(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
		{
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
				// First find closest points on the two witness edges
				Convex::HalfEdge const edgeA = A.edges[eq.indexA];
				Convex::HalfEdge const twinA = A.edges[edgeA.twin];
				Segment const edgeSegA = Transformed({ .b = A.verts[edgeA.origin], .e = A.verts[twinA.origin] }, trA);

				Convex::HalfEdge const edgeB = B.edges[eq.indexB];
				Convex::HalfEdge const twinB = B.edges[edgeB.twin];
				Segment const edgeSegB = Transformed({ .b = B.verts[edgeB.origin], .e = B.verts[twinB.origin] }, trB);

				ClosestPointsQuery closestPts = util::ClosestPoints(edgeSegA, edgeSegB);

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
		

		ContactManifold Collide(Convex const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB)
		{
			COLLIDE_FCN_NOT_IMPLEMENTED
		}


		// ---------------------------------------------------------------------
		// MESH
		// ---------------------------------------------------------------------

		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Mesh const& B, Mat4 const& trB)
		{
			ASSERT(false, "Intentionally not implemented b/c Mesh colliders are required to be static.");
			return ContactManifold{};
		}


		// ---------------------------------------------------------------------
		// FLIPPED ARGS
		// ---------------------------------------------------------------------
		
		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB)
		{
			return util::CollideFlipped(A, trA, B, trB);
		}

		ContactManifold Collide(Convex const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB)
		{
			return util::CollideFlipped(A, trA, B, trB);
		}

		ContactManifold Collide(Convex const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB)
		{
			return util::CollideFlipped(A, trA, B, trB);
		}

		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB)
		{
			return util::CollideFlipped(A, trA, B, trB);
		}

		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB)
		{
			return util::CollideFlipped(A, trA, B, trB);
		}

		ContactManifold Collide(Mesh const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
		{
			return util::CollideFlipped(A, trA, B, trB);
		}


		// ---------------------------------------------------------------------
		// Helper Implementations
		// ---------------------------------------------------------------------

		namespace util {

			static inline Vec4 ClosestPoint(Vec3 const& pt, Segment const& ls) {
				Vec3 const s = ls.e - ls.b;
				ASSERT(glm::length2(s) > 1.0e-12f, "Line segment is poorly defined. Start == End.");

				// Project pt onto ls, computing parameterized position d(t) = start + t*(end - start)
				Float32 t = glm::dot(pt - ls.b, s) / glm::length2(s);

				// If outside segment, clamp t (and therefore d) to the closest endpoint
				t = glm::clamp(t, 0.0f, 1.0f);

				// Compute projected position from the clamped t
				return { ls.b + t * s, t };
			}
			
			static inline Vec4 ClosestPointOnLine(Vec3 const& pt, Segment const& ls) {
				Vec3 const s = ls.e - ls.b;
				ASSERT(glm::length2(s) > 1.0e-12f, "Line segment is poorly defined. Start == End.");

				return ClosestPointOnLine(pt, s, ls.b);
			}
			
			static inline Vec4 ClosestPointOnLine(Vec3 const& pt, Vec3 const& lineDir, Vec3 const& linePt) {
				ASSERT(glm::length2(lineDir) > 1.0e-12f, "Line is poorly defined. Start == End.");

				// Project pt onto ls, computing parameterized position d(t) = start + t*(end - start)
				Float32 const t = glm::dot(pt - linePt, lineDir) / glm::length2(lineDir);
				
				// Compute projected position from the clamped t
				return { linePt + t * lineDir, t };
			}


			// See Ericson Realtime Collision Detection Ch 5.1.9
			static ClosestPointsQuery ClosestPoints(Segment const& A, Segment const& B)
			{
				static constexpr Float32 tol = 1.0e-4f;

				Vec3 const dA = A.e - A.b; // Direction vector of segment a
				Vec3 const dB = B.e - B.b; // Direction vector of segment b
				Vec3 const r = A.b - B.b;
				Float32 const LA = glm::length2(dA);  // Squared length of segment a, always nonnegative
				Float32 const LB = glm::length2(dB);  // Squared length of segment b, always nonnegative
				Float32 const f = glm::dot(dB, r);

				Float32 t{}, s{};

				// Check if either or both segments degenerate into points
				if (LA <= tol && LB <= tol) {
					// Both segments degenerate into points
					return ClosestPointsQuery{ A.b, B.b, glm::distance2(A.b, B.b) };
				}
				if (LA <= tol) {
					// First segment degenerates into a point
					t = f / LB;
					t = glm::clamp(t, 0.0f, 1.0f);
				}
				else {
					Float32 c = glm::dot(dA, r);
					if (LB <= tol) {
						// Second segment degenerates into a point
						t = 0.0f;
						s = glm::clamp(-c / LA, 0.0f, 1.0f);
					}
					else {
						Float32 d = glm::dot(dA, dB);
						Float32 denom = LA * LB - d * d; // Always nonnegative
						if (denom != 0.0f) {
							s = glm::clamp((d * f - c * LB) / denom, 0.0f, 1.0f);
						}
						else {
							s = 0.0f;
						}

						t = (d * s + f) / LB;

						if (t < 0.0f) {
							t = 0.0f;
							s = glm::clamp(-c / LA, 0.0f, 1.0f);
						}
						else if (t > 1.0f) {
							t = 1.0f;
							s = glm::clamp((d - c) / LA, 0.0f, 1.0f);
						}
					}
				}
				Vec3 const c1 = A.b + dA * s;
				Vec3 const c2 = B.b + dB * t;

				return ClosestPointsQuery{
					.ptA = c1,
					.ptB = c2,
					.d2 = glm::distance2(c1, c2)
				};
			}

			static ClosestPointsQuery ClosestPointsNonDegenerate(Segment const& A, Segment const& B, Vec3 const& dA, Vec3 const& dB, Float32 mag2A, Float32 mag2B)
			{
				Vec3 const r = A.b - B.b;
				Float32 const c = glm::dot(dA, r);
				Float32 const d = glm::dot(dA, dB);
				Float32 const denom = mag2A * mag2B - d * d; // Always nonnegative
				Float32 const f = glm::dot(dB, r);

				Float32 s = EpsilonEqual(denom, 0.0f) ?  // Parallel line segs
						0.0f :
						glm::clamp((d * f - c * mag2B) / denom, 0.0f, 1.0f);
				
				Float32 t = (d * s + f) / mag2B;

				if (t < 0.0f) {
					t = 0.0f;
					s = glm::clamp(-c / mag2A, 0.0f, 1.0f);
				}
				else if (t > 1.0f) {
					t = 1.0f;
					s = glm::clamp((d - c) / mag2A, 0.0f, 1.0f);
				}

				Vec3 const c1 = A.b + dA * s;
				Vec3 const c2 = B.b + dB * t;

				return ClosestPointsQuery{
					.ptA = c1,
					.ptB = c2,
					.d2 = glm::distance2(c1, c2)
				};
			}


			static inline void Flip(ContactManifold& m)
			{
				m.normal *= -1.0f;
				std::swap(m.featureA, m.featureB);
			}


			static inline ContactManifold CollideFlipped(auto const& A, Mat4 const& trA, auto const& B, Mat4 const& trB)
			{
				ContactManifold m = Collide(B, trB, A, trA);
				Flip(m);
				return m;
			}


			// See Dirk Gregorius "Robust Contact Manifolds" GDC
			static ContactManifold GenerateFaceContact(FaceQuery const& fq, Convex const& reference, Mat4 const& refTr, Convex const& incident, Mat4 const& incTr)
			{
				Mat4 const incToRefLocal = glm::inverse(refTr) * incTr;
				
				ASSERT(fq.index >= 0 && fq.index <= Convex::MAX_EDGES, "Invalid index");
				Convex::FaceID const refFaceIdx = static_cast<Convex::FaceID>(fq.index);
				Convex::Face const& refFace = reference.faces[refFaceIdx];
				
				
				// Find the incident face
				Convex::FaceID incFaceIdx = Convex::INVALID_INDEX;
				Float32 minDotProd = std::numeric_limits<Float32>::max();
				for (Uint8 i = 0; i < incident.faces.size(); ++i)
				{
					Vec3 const    n   = Transformed(incident.faces[i].plane, incToRefLocal).n;
					Float32 const dot = glm::dot(n, refFace.plane.n);
					if (dot < minDotProd)
					{
						minDotProd = dot;
						incFaceIdx = i;
					}
				}
				
				// Clip the incident face against edge planes of reference face
				Polygon incFacePoly = FaceAsPolygon(incident, incToRefLocal, incFaceIdx);
				Polygon frontPoly{}, backPoly{};
				ForEachEdgeOfFace(reference, refFaceIdx, [&](Convex::HalfEdge edge) {

					// Create a plane orthogonal to refFace and containing
					// endpoints of edge
					Convex::HalfEdge const twin = reference.edges[edge.twin];
					Vec3 const p0 = reference.verts[edge.origin];
					Vec3 const p1 = reference.verts[twin.origin];
					Vec3 const outwardNormal = Normalize(glm::cross(p1 - p0, refFace.plane.n));
					Plane const edgePlane = MakePlane(outwardNormal, p0); // normal pointing "outward" away from face center

					// Split incident face on edgePlane
					SplitPolygon(incFacePoly, edgePlane, frontPoly, backPoly);

					// Save the points behind or on the edgePlane
					std::swap(incFacePoly.verts, backPoly.verts);
					backPoly.verts.clear();
				});

				// Keep only points in clipped incident face which are behind or on reference face plane
				std::erase_if(incFacePoly.verts, [&refFace](Vec3 const& p) {
					return ClassifyPointToPlane(p, refFace.plane) == Side::Front;
				});


				// Reduce the manifold to at most 4 points
				ContactManifold m = ReduceContactSet(incFacePoly, refFace.plane, refTr);
				
				m.featureA = { .index = fq.index,   .type = Feature::Type::Face };
				m.featureB = { .index = incFaceIdx, .type = Feature::Type::Face };
				m.normal   = Normalize(Mat3(refTr) * refFace.plane.n);
				
				return m;
			}


			// Reduce the clipped and projected incident face to at most 4 contact 
			// points. This is a bit of a process -- see D.G.'s GDC talk...
			static ContactManifold ReduceContactSet(Polygon& incFacePoly, Plane const& refFacePlane, Mat4 const& refTr)
			{
				ContactManifold m{};
				
				// Possible that clipping has removed ALL points...
				Int32 const numCandidates = static_cast<Int32>(incFacePoly.verts.size());
				if (numCandidates == 0) { return m; }

				static constexpr Uint32 maxCandidates = 64;
				ASSERT(numCandidates < maxCandidates, "Too many potential contacts. You probably want to reduce the complexity of your collision geometry.");

				// First, project contact points onto reference face, and identify the deepest 
				// point (which is needed for CCD) -- this will be our first contact point
				Float32 depths[maxCandidates] = {};
				Uint32  p0Idx = 0;
				Float32 deepest = std::numeric_limits<Float32>::max();
				for (Int32 i = 0; i < numCandidates; ++i) {

					Float32 const depth = SignedDistance(incFacePoly.verts[i], refFacePlane);
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
				if (numCandidates <= 4)
				{
					Uint32 curr = (p0Idx + 1) % numCandidates;
					while (curr != p0Idx)
					{
						m.contacts[m.numContacts++] = Contact{
							.position = refTr * Vec4(incFacePoly.verts[curr], 1.0f),
							.penetration = -depths[curr]
						};

						curr++;
						curr %= numCandidates;
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

			static inline ContactManifold CollideSphereSphereImpl(Sphere const& A, Vec3 const& cA, Sphere const& B, Vec3 const& cB)
			{
				ContactManifold result{};

				Vec3 const disp = cB - cA;
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


			static inline ContactManifold CollideSphereCapImpl(Sphere const& A, Vec3 const& cA, Capsule const& B, Mat4 const& trB)
			{
				Segment const segB = CentralSegment(B, trB);

				// Closest point to a on internal line segment of b
				Vec3 const p = util::ClosestPoint(cA, segB);

				// Construct a sphere on the fly, centered at the closest point, p,
				// to sphere A on central segment of capsule B with radius of capsule B
				Sphere const S{ B.r };

				return CollideSphereSphereImpl(A, cA, S, p);
			}

		} //  namespace util
} // namespace drb::physics

#undef COLLIDE_FCN_NOT_IMPLEMENTED