#include "pch.h"
#include "PhysicsGeometryQueries.h"

#include "PhysicsGeometry.h"

#define COLLIDE_FCN_NOT_IMPLEMENTED ASSERT(false, "Not yet implemented."); return ContactManifold{};

namespace drb {
	namespace physics {
		namespace util {
			struct FaceQuery {
				Int32   index = -1;
				Float32 separation = std::numeric_limits<Float32>::lowest();
				Vec3    normal = Vec3(NAN);
			};

			struct EdgeQuery {
				Int32   indexA = -1;
				Int32   indexB = -1;
				Float32 separation = std::numeric_limits<Float32>::lowest();
				Vec3	normal = Vec3(NAN);
			};

			struct ClosestPointsQuery {
				Vec3 ptA = Vec3(NAN);							  // pt on shape a in world space
				Vec3 ptB = Vec3(NAN);							  // pt on shape b in world space
				Float32 d2 = std::numeric_limits<Float32>::max(); // sqr distance
			};

			static FaceQuery SATQueryFaceDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
			static EdgeQuery SATQueryEdgeDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB);
			
			static inline Vec3 GetSupport(Convex const& hull, Vec3 const& dir);
			static inline Float32 Project(const Plane& plane, const Convex& hull);
			static inline Float32 Project(const Vec3& p1, const Vec3& e1, const Vec3& p2, const Vec3& e2, const Vec3& c1, Vec3& out_normal);
			
			static inline Bool IsMinkowskiFace(Vec3 const& a, Vec3 const& b, Vec3 const& b_x_a, Vec3 const& c, Vec3 const& d, Vec3 const& d_x_c);
		
			static inline Vec3 ClosestPoint(Vec3 const& pt, Segment const& ls);
			static ClosestPointsQuery ClosestPoints(Segment const& A, Segment const& B);

			static inline ContactManifold CollideFlipped(auto const& A, Mat4 const& trA, auto const& B, Mat4 const& trB);
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
			Segment const segB = GetCentralSegment(B, trB);

			// Closest point to a on internal line segment of b
			Vec3 const p = util::ClosestPoint(posA, segB);

			// Construct a sphere on the fly centered at the closest point on
			// central segment to sphere A and with radius of capsule B
			Sphere const S{ B.r };
			Mat4 const trS = glm::translate(Mat4(1), p);

			return Collide(A, trA, S, trS);
		}

		ContactManifold Collide(Sphere const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
		{
			COLLIDE_FCN_NOT_IMPLEMENTED	
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
			Segment const segA = GetCentralSegment(A, trA);
			Segment const segB = GetCentralSegment(B, trB);

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
			if (fqA.separation > 0.0f) {
				return ContactManifold{
					.normal = fqA.normal
				}; // no contacts, but track the separating normal
			}

			// Test faces of B
			util::FaceQuery fqB = util::SATQueryFaceDirections(B, trB, A, trA);
			fqB.normal *= -1.0f; // normal currently points from B toward A, so flip it
			if (fqB.separation > 0.0f) {
				return ContactManifold{
					.normal = fqB.normal
				}; // no contacts, but track the separating normal
			}

			// Test edge pairs
			util::EdgeQuery eq = util::SATQueryEdgeDirections(A, trA, B, trB);
			if (eq.separation > 0.0f) {
				return ContactManifold{
					.normal = eq.normal
				}; // no contacts, but track the separating normal
			}

			// Now generate the contact manifold!
			ContactManifold result{ .numContacts = 1 };

			Vec3 normal = fqA.normal;
			Float32 sep = glm::max(fqB.separation, glm::max(fqB.separation, eq.separation));
			if (sep == fqB.separation) {
				normal = fqB.normal;
			}
			else if (sep == eq.separation) {
				normal = eq.normal;
			}
			result.normal = normal;

			// TODO finish generating contacts
			// ...
			COLLIDE_FCN_NOT_IMPLEMENTED

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
		// HELPER IMPLEMENTATIONS
		// ---------------------------------------------------------------------

		namespace util {

			static FaceQuery SATQueryFaceDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
			{
				// We perform all computations in local space of B
				Mat4 const transform = glm::inverse(trB) * trA;

				FaceQuery result{};

				Int32 const face_count = static_cast<Int32>(A.faces.size());
				for (Int32 i = 0; i < face_count; ++i)
				{
					Plane const p = GetTransformed(A.faces[i].plane, transform);

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


			static EdgeQuery SATQueryEdgeDirections(Convex const& A, Mat4 const& trA, Convex const& B, Mat4 const& trB)
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

					Vec3 const pA = transform * Vec4(A.verts[edgeA.origin], 1);
					Vec3 const qA = transform * Vec4(A.verts[twinA.origin], 1);
					Vec3 const eA = qA - pA;

					Vec3 const uA = Mat3(transform) * A.faces[edgeA.face].plane.n;
					Vec3 const vA = Mat3(transform) * A.faces[twinA.face].plane.n;

					for (Int32 j = 0; j < edgeCountB; j += 2)
					{
						Convex::HalfEdge const& edgeB = B.edges[j];
						Convex::HalfEdge const& twinB = B.edges[j + 1];
						ASSERT(edgeB.twin == j + 1 && twinB.twin == j, "B is invalid.");

						Vec3 const pB = B.verts[edgeB.origin];
						Vec3 const qB = B.verts[twinB.origin];
						Vec3 const eB = qB - pB;

						Vec3 const uB = B.faces[edgeB.face].plane.n;
						Vec3 const vB = B.faces[twinB.face].plane.n;

						if (IsMinkowskiFace(uA, vA, -eA, -uB, -vB, -eB))
						{
							Vec3 normal{};
							Float32 const separation = Project(pA, eA, pB, eB, cA, normal);
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

			// dir should be in hull's local space
			static inline Vec3 GetSupport(Convex const& hull, Vec3 const& dir) {
				Int32   max_index = -1;
				Float32 max_projection = std::numeric_limits<Float32>::lowest();

				Int32 const vert_count = static_cast<Int32>(hull.verts.size());

				for (Int32 i = 0; i < vert_count; ++i)
				{
					Float32 const projection = glm::dot(dir, hull.verts[i]);
					if (projection > max_projection)
					{
						max_index = i;
						max_projection = projection;
					}
				}

				return hull.verts[max_index];
			}
			
			// This function computes the distance between a plane and the hull assuming the plane
			// is in the local space of the hull
			static inline Float32 Project(Plane const& plane, Convex const& hull)
			{
				Vec3 const support = GetSupport(hull, -plane.n);
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

			static Vec3 ClosestPoint(Vec3 const& pt, Segment const& ls) {
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


			static inline ContactManifold CollideFlipped(auto const& A, Mat4 const& trA, auto const& B, Mat4 const& trB)
			{
				ContactManifold m = Collide(B, trB, A, trA);
				m.normal *= -1.0f;
				for (unsigned i = 0; i < m.numContacts; ++i) {
					std::swap(m.contacts[i].featureA, m.contacts[i].featureB);
				}
				return m;
			}

		}
	}
}

#undef COLLIDE_FCN_NOT_IMPLEMENTED