#include "pch.h"
#include "PhysicsGeometryQueries.h"

#include "PhysicsGeometry.h"
#include "SATGJK.h"

#define COLLIDE_FCN_NOT_IMPLEMENTED ASSERT(false, "Not yet implemented."); return ContactManifold{};

namespace drb {
	namespace physics {

		namespace util {

			// -----------------------------------------------------------------
			// HELPERS
			// -----------------------------------------------------------------
			
			struct ClosestPointsQuery {
				Vec3 ptA = Vec3(NAN);							  // pt on shape a in world space
				Vec3 ptB = Vec3(NAN);							  // pt on shape b in world space
				Float32 d2 = std::numeric_limits<Float32>::max(); // sqr distance
			};

			// Returns closest point (in world space) on segment ls to point pt
			static inline Vec3               ClosestPoint(Vec3 const& pt, Segment const& ls);

			// Returns closest points (in world space) on each segment to the other segment, and the
			// square distance between them
			static        ClosestPointsQuery ClosestPoints(Segment const& A, Segment const& B);

			// Helper to swap the arguments A and B and call the appropriate collision function
			static inline ContactManifold    CollideFlipped(auto const& A, Mat4 const& trA, auto const& B, Mat4 const& trB);

			// Helper to handle when normal is oriented the wrong way for a manifold
			static inline void               Flip(ContactManifold& manifold);

			// Helper to create the contact manifold for face collisions found during SAT 
			static		  ContactManifold	 GenerateFaceContact(FaceQuery const& fq, Convex const& reference, Mat4 const& refTr, Convex const& incident, Mat4 const& incTr);
		}


		// ---------------------------------------------------------------------
		// MANIFOLD
		// ---------------------------------------------------------------------

		ManifoldKey::ManifoldKey(RigidBody* a_, CollisionShapeBase* aShape_, RigidBody* b_, CollisionShapeBase* bShape_)
			: a{ a_ }, aShape{ aShape_ }, b{ b_ }, bShape{ bShape_ }
		{
			ASSERT(a != b, "RigidBodies a and b must be unique.");

			if (a > b)
			{
				std::swap(a, b);
				std::swap(aShape, bShape);
			}
		}


		// ---------------------------------------------------------------------
		// SPHERE
		// ---------------------------------------------------------------------

		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Sphere const& B, Mat4 const& trB)
		{
			ContactManifold result{};

			Vec3 const posA = trA[3];
			Vec3 const posB = trB[3];

			Float32 const dist = glm::distance(posA, posB);
			if (dist < 0.0001f) {
				return result;
			}

			Float32 const pen = (A.r + B.r) - dist;
			if (pen > 0.0f) {

				result.normal = (posB - posA) / dist;

				result.contacts[result.numContacts++] = Contact{
					// contact position is halfway between the surface points of two spheres
					.position = result.normal * (A.r - 0.5f * pen) + posA,
					.penetration = pen,
				};
			}

			return result;
		}


		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Capsule const& B, Mat4 const& trB)
		{
			Vec3 const posA = trA[3];
			Segment const segB = CentralSegment(B, trB);

			// Closest point to a on internal line segment of b
			Vec3 const p = util::ClosestPoint(posA, segB);

			// Construct a sphere on the fly, centered at the closest point to sphere A 
			// on central segment of capsule B with radius of capsule B
			Sphere const S{ B.r };
			Mat4 const trS = glm::translate(Mat4(1), p);

			return Collide(A, trA, S, trS);
		}


		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
		{
			COLLIDE_FCN_NOT_IMPLEMENTED

			// Use GJK between A's center and B
			// -> if (center and B are not intersecting): generate contact if distance < A's radius
			// -> else: use SAT to generate contact
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
			Segment const segA = CentralSegment(A, trA);
			Segment const segB = CentralSegment(B, trB);

			auto const [pA, pB, d2] = util::ClosestPoints(segA, segB);
			
			if (d2 > (A.r + B.r) * (A.r + B.r)) {
				return ContactManifold{
					.normal = d2 > 0.0001f ? (pB - pA) / glm::sqrt(d2) : Vec3(NAN)
				};
			}

			// Construct two spheres on the fly centered at closest points between
			// central segments and with radii equal to that of the capsules
			Sphere const sA{ A.r }, sB{ B.r };
			Mat4   const trSA = glm::translate(Mat4(1), pA), trSB = glm::translate(Mat4(1), pB);

			return Collide(sA, trSA, sB, trSB);
		}


		ContactManifold Collide(Capsule const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
		{
			COLLIDE_FCN_NOT_IMPLEMENTED
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

			// This is a value used to make Face A preferable to Face B, and both faces
			// preferable to an Edge Pair when deciding what contact to generate.
			static constexpr Float32 bias = 0.98f;

			Bool const faceContactA = fqA.separation * bias > eq.separation;
			Bool const faceContactB = fqB.separation * bias > eq.separation;

			if (faceContactA && faceContactB) // Create Face-Face contacts
			{
				if (fqA.separation * bias > fqB.separation)
				{
					result = util::GenerateFaceContact(fqA, A, trA, B, trB);
				}
				else
				{
					result = util::GenerateFaceContact(fqB, B, trB, A, trA);
					util::Flip(result);
				}
			}
			else // Create Edge-Edge contact
			{
				// First find closest points on the two witness edges
				Convex::HalfEdge const edgeA = A.edges[eq.indexA];
				Convex::HalfEdge const twinA = A.edges[edgeA.twin];
				Segment const edgeSegA = Transformed({ .b = A.verts[edgeA.origin], .e = A.verts[twinA.origin] }, trA);
				
				Convex::HalfEdge const edgeB = B.edges[eq.indexB];
				Convex::HalfEdge const twinB = B.edges[edgeB.twin];
				Segment const edgeSegB = Transformed({ .b = B.verts[edgeB.origin], .e = B.verts[twinB.origin] }, trB);

				util::ClosestPointsQuery closestPts = util::ClosestPoints(edgeSegA, edgeSegB);

				// Then build the contact with position at midpoint between closest points
				result.contacts[result.numContacts++] = Contact{
					.position = 0.5f * (closestPts.ptA + closestPts.ptB),
					.penetration = -eq.separation
				};
				result.normal = eq.normal;
				result.featureA = { .index = eq.indexA, .type = Feature::Type::Edge };
				result.featureB = { .index = eq.indexB, .type = Feature::Type::Edge };
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

			static inline Vec3 ClosestPoint(Vec3 const& pt, Segment const& ls) {
				Vec3 const s = ls.e - ls.b;
				ASSERT(glm::length2(s) > 0.0001f, "Line segment is poorly defined. Start == End.");

				// Project pt onto ls, computing parameterized position d(t) = start + t*(end - start)
				Float32 t = glm::dot(pt - ls.b, s) / glm::length2(s);

				// If outside segment, clamp t (and therefore d) to the closest endpoint
				t = glm::clamp(t, 0.0f, 1.0f);

				// Compute projected position from the clamped t
				return ls.b + t * s;
			}


			// See Ericson Realtime Collision Detection Ch 5.1.9
			static ClosestPointsQuery ClosestPoints(Segment const& A, Segment const& B)
			{
				Vec3 const dA = A.e - A.b; // Direction vector of segment a
				Vec3 const dB = B.e - B.b; // Direction vector of segment b
				Vec3 const r = A.b - B.b;
				Float32 const LA = glm::length2(dA);  // Squared length of segment a, always nonnegative
				Float32 const LB = glm::length2(dB);  // Squared length of segment b, always nonnegative
				Float32 const f = glm::dot(dB, r);

				Float32 t{}, s{};

				// Check if either or both segments degenerate into points
				if (LA <= 0.0001f && LB <= 0.0001f) {
					// Both segments degenerate into points
					return ClosestPointsQuery{ A.b, B.b, glm::distance2(A.b, B.b) };
				}
				if (LA <= 0.0001f) {
					// First segment degenerates into a point
					t = f / LB;
					t = glm::clamp(t, 0.0f, 1.0f);
				}
				else {
					Float32 c = glm::dot(dA, r);
					if (LB <= 0.0001f) {
						// Second segment degenerates into a point
						t = 0.0f;
						s = glm::clamp(-c / LA, 0.0f, 1.0f);
					}
					else {
						// The general nondegenerate case starts here
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

				Convex::Face const& refFace = reference.faces[fq.index];
				
				// Find the incident face
				Uint8   incFaceIdx = Convex::INVALID_INDEX;
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
				Convex::Face const& incFace = incident.faces[incFaceIdx];

				// Clip the incident face against edge planes of reference face
				Polygon incFacePoly = FaceAsPolygon(incident, incToRefLocal, incFace);
				Polygon frontPoly{}, backPoly{};
				ForEachEdgeOfFace(reference, refFace, [&](Convex::HalfEdge edge) {

					// Create a plane orthogonal to refFace and containing
					// endpoints of edge
					Convex::HalfEdge const twin = reference.edges[edge.twin];
					Vec3 const p0 = reference.verts[edge.origin];
					Vec3 const p1 = reference.verts[twin.origin];
					Vec3 const p2 = p1 + refFace.plane.n;
					Plane const edgePlane = MakePlane(p0, p1, p2); // normal pointing "inward" toward refFace center

					// Split incident face on edgePlane
					SplitPolygon(incFacePoly, edgePlane, frontPoly, backPoly);

					// Save the points in front of the edgePlane
					std::swap(incFacePoly.verts, frontPoly.verts);
					frontPoly.verts.clear();
				});

				// Keep only points in clipped incident face which are behind or on reference face plane
				std::erase_if(incFacePoly.verts, [&refFace](Vec3 const& p) {
					return ClassifyPointToPlane(p, refFace.plane) == Side::Front;
				});


				// Possible that clipping has removed ALL points...
				Uint32 const numCandidates = incFacePoly.verts.size();
				if (numCandidates == 0) { return ContactManifold{}; }


				// Start generating our manifold
				ContactManifold m{
					.featureA = { .index = fq.index,   .type = Feature::Type::Face },
					.featureB = { .index = incFaceIdx, .type = Feature::Type::Face },
					.normal = Mat3(refTr) * refFace.plane.n
				};


				// Reduce the clipped and projected incident face to at most 
				// 4 contact points. This is a bit of a tricky process -- see
				// D.G.'s GDC talk...
				
				// First, project contact points onto reference face, and identify the deepest 
				// point (which is needed for CCD) -- this will be our first contact point
				std::vector<Float32> depths(numCandidates, 0.0f);
				Uint32  p0Idx = 0;
				Float32 deepest = std::numeric_limits<Float32>::max();
				for (Uint32 i = 0; i < numCandidates; ++i) {
					
					Float32 const depth = SignedDistance(incFacePoly.verts[i], refFace.plane);
					if (depth < deepest) {
						deepest = depth;
						p0Idx = i;
					}

					// Save the depth then project onto ref face
					depths.push_back(depth);
					incFacePoly.verts[i] -= depth * refFace.plane.n;
				}
				Vec3 const p0 = incFacePoly.verts[p0Idx];
				m.contacts[m.numContacts++] = Contact{
					.position = refTr * Vec4(p0, 1.0f),
					.penetration = -deepest
				};
				if (numCandidates == 1) { return m;	}


				// Now identify the candidate furthest away from p0
				Int32  p1Idx = -1;
				Float32 furthest = std::numeric_limits<Float32>::lowest();
				for (Int32 i = 0; i < numCandidates; ++i) {
					if (i == p0Idx) { continue; }

					Float32 const dist2 = glm::distance2(p0, incFacePoly.verts[i]);
					if (dist2 > furthest) {
						furthest = dist2;
						p1Idx = i;
					}
				}
				ASSERT(p1Idx >= 0, "Invalid index");
				Vec3 const p1 = incFacePoly.verts[p1Idx];
				m.contacts[m.numContacts++] = Contact{
					.position = refTr * Vec4(p1, 1.0f),
					.penetration = -depths[p1Idx]
				};
				if (numCandidates == 2) { return m; }
				

				// Next, identify the point which maximizes the (positive) area of 
				// triangle p0-p1-p2
				Uint32 p2Idx = -1;
				Float32 maxTriArea = 0;
				for (Uint32 i = 0; i < incFacePoly.verts.size(); ++i) {
					if (i == p0Idx || i == p1Idx) { continue; }

					Vec3 const u = p0 - incFacePoly.verts[i];
					Vec3 const v = p1 - incFacePoly.verts[i];
					Float32 const area = glm::dot(glm::cross(u, v), refFace.plane.n);
					if (area > maxTriArea) {
						maxTriArea = area;
						p2Idx = i;
					}
				}
				ASSERT(p2Idx >= 0, "Invalid index");
				Vec3 const p2 = incFacePoly.verts[p2Idx];
				m.contacts[m.numContacts++] = Contact{
					.position = refTr * Vec4(p2, 1.0f),
					.penetration = -depths[p2Idx]
				};
				if (numCandidates == 3) { return m; }

				// Finally, we identify the point on the opposite side of 
				// edge p0-p1 from p2 that maximizes the area of the quad
				// by looking for the point that would give us the most 
				// negative area when a triangle is formed with a pair of 
				// our current points
				Uint32  p3Idx = -1;
				Float32 minTriArea = 0;
				for (Uint32 i = 0; i < incFacePoly.verts.size(); ++i) {
					if (i == p0Idx || i == p1Idx || i == p2Idx) { continue; }


					Vec3 const u = p0 - incFacePoly.verts[i];
					Vec3 const v = p1 - incFacePoly.verts[i];
					Vec3 const w = p2 - incFacePoly.verts[i];

					// Test triangle p0p1p3
					{
						Float32 const area = glm::dot(glm::cross(u, v), refFace.plane.n);
						if (area < minTriArea) {
							minTriArea = area;
							p3Idx = i;
						}
					}
					
					// Test triangle p1p2p3
					{
						Float32 const area = glm::dot(glm::cross(v, w), refFace.plane.n);
						if (area < minTriArea) {
							minTriArea = area;
							p3Idx = i;
						}
					}
					
					// Test triangle p2p0p3
					{
						Float32 const area = glm::dot(glm::cross(w, u), refFace.plane.n);
						if (area < minTriArea) {
							minTriArea = area;
							p3Idx = i;
						}
					}
				}
				ASSERT(p3Idx >= 0, "Invalid index");
				Vec3 const p3 = incFacePoly.verts[p3Idx];
				m.contacts[m.numContacts++] = Contact{
					.position = refTr * Vec4(p3, 1.0f),
					.penetration = -depths[p3Idx]
				};
				
				return m;
			}
		}
	}
}

#undef COLLIDE_FCN_NOT_IMPLEMENTED