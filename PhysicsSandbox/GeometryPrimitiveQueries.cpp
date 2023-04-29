#include "pch.h"
#include "GeometryPrimitiveQueries.h"
#include "DRBAssert.h"
#include "Math.h"

namespace drb::physics {

	Real SignedDistance(Plane const& pl, Vec3 const& pt)
	{
		return glm::dot(pt, pl.n) - pl.d;
	}

	Side Classify(Plane const& pl, Vec3 const& pt)
	{
		Real const dist = SignedDistance(pl, pt);

		if (dist > Plane::thickness) { return Side::Front; }
		if (dist < -Plane::thickness) { return Side::Back; }
		return Side::On;
	}

	// See Ericson 5.3.1
	Bool Intersect(Segment const& seg, Plane const& plane, Real& t, Vec3& q)
	{
		// Compute the t value for the directed line v intersecting the plane
		Vec3 const v = seg.e - seg.b;
		t = -SignedDistance(plane, seg.b) / glm::dot(plane.n, v);

		// If t in [0..1] compute and return intersection point
		if (t >= 0.0_r && t <= 1.0_r)
		{
			q = seg.b + t * v;
			return true;
		}

		// Else no intersection
		return false;
	}


	Vec4 ClosestPoint(Segment const& ls, Vec3 const& pt) {
		Vec3 const s = ls.e - ls.b;
		ASSERT(glm::length2(s) > 1.0e-12_r, "Line segment is poorly defined. Start == End.");

		// Project pt onto ls, computing parameterized position d(t) = start + t*(end - start)
		Real t = glm::dot(pt - ls.b, s) / glm::length2(s);

		// If outside segment, clamp t (and therefore d) to the closest endpoint
		t = glm::clamp(t, 0.0_r, 1.0_r);

		// Compute projected position from the clamped t
		return { ls.b + t * s, t };
	}

	Vec4 ClosestPointOnLine(Segment const& ls, Vec3 const& pt) {
		Vec3 const s = ls.e - ls.b;
		ASSERT(glm::length2(s) > 1.0e-12_r, "Line segment is poorly defined. Start == End.");

		return ClosestPointOnLine(pt, s, ls.b);
	}

	Vec4 ClosestPointOnLine(Vec3 const& lineDir, Vec3 const& linePt, Vec3 const& pt) {
		ASSERT(glm::length2(lineDir) > 1.0e-12_r, "Line is poorly defined. Start == End.");

		// Project pt onto ls, computing parameterized position d(t) = start + t*(end - start)
		Real const t = glm::dot(pt - linePt, lineDir) / glm::length2(lineDir);

		// Compute projected position from the clamped t
		return { linePt + t * lineDir, t };
	}

	// See Ericson Realtime Collision Detection Ch 5.1.9
	ClosestPointsQuery ClosestPoints(Segment const& A, Segment const& B)
	{
		static constexpr Real tol = 1.0e-4_r;

		Vec3 const dA = A.e - A.b; // Direction vector of segment a
		Vec3 const dB = B.e - B.b; // Direction vector of segment b
		Vec3 const r = A.b - B.b;
		Real const LA = glm::length2(dA);  // Squared length of segment a, always nonnegative
		Real const LB = glm::length2(dB);  // Squared length of segment b, always nonnegative
		Real const f = glm::dot(dB, r);

		Real t{}, s{};

		// Check if either or both segments degenerate into points
		if (LA <= tol && LB <= tol) {
			// Both segments degenerate into points
			return ClosestPointsQuery{ A.b, B.b, glm::distance2(A.b, B.b) };
		}
		if (LA <= tol) {
			// First segment degenerates into a point
			t = f / LB;
			t = glm::clamp(t, 0.0_r, 1.0_r);
		}
		else {
			Real c = glm::dot(dA, r);
			if (LB <= tol) {
				// Second segment degenerates into a point
				t = 0.0;
				s = glm::clamp(-c / LA, 0.0_r, 1.0_r);
			}
			else {
				Real d = glm::dot(dA, dB);
				Real denom = LA * LB - d * d; // Always nonnegative
				if (denom != 0.0) {
					s = glm::clamp((d * f - c * LB) / denom, 0.0_r, 1.0_r);
				}
				else {
					s = 0.0;
				}

				t = (d * s + f) / LB;

				if (t < 0.0) {
					t = 0.0;
					s = glm::clamp(-c / LA, 0.0_r, 1.0_r);
				}
				else if (t > 1.0) {
					t = 1.0;
					s = glm::clamp((d - c) / LA, 0.0_r, 1.0_r);
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

	ClosestPointsQuery ClosestPointsNonDegenerate(Segment const& A, Segment const& B, Vec3 const& dA, Vec3 const& dB, Real mag2A, Real mag2B)
	{
		Vec3 const r = A.b - B.b;
		Real const c = glm::dot(dA, r);
		Real const d = glm::dot(dA, dB);
		Real const denom = mag2A * mag2B - d * d; // Always nonnegative
		Real const f = glm::dot(dB, r);

		Real s = EpsilonEqual(denom, 0.0_r) ?  // Parallel line segs
			0.0_r :
			glm::clamp((d * f - c * mag2B) / denom, 0.0_r, 1.0_r);

		Real t = (d * s + f) / mag2B;

		if (t < 0.0) {
			t = 0.0;
			s = glm::clamp(-c / mag2A, 0.0_r, 1.0_r);
		}
		else if (t > 1.0) {
			t = 1.0;
			s = glm::clamp((d - c) / mag2A, 0.0_r, 1.0_r);
		}

		Vec3 const c1 = A.b + dA * s;
		Vec3 const c2 = B.b + dB * t;

		return ClosestPointsQuery{
			.ptA = c1,
			.ptB = c2,
			.d2 = glm::distance2(c1, c2)
		};
	}

}