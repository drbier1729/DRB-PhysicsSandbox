#ifndef DRB_GEOMETRYPRIMITIVEQUERIES_H
#define DRB_GEOMETRYPRIMITIVEQUERIES_H

#include "GeometryPrimitives.h"
#include "GeometryQueryDataStructures.h"

namespace drb::physics {

	// Returns true if the Segment and Plane intersect. Sets "t" to a parameterized
	// value in range [0..1] and q to the point of intersection.
	Bool Intersect(Segment const& seg, Plane const& plane, float& t, Vec3& q);

	// Returns which side of the Plane pt is on (Front, On, or Back)
	Side Classify(Plane const& plane, Vec3 const& pt);

	// Returns the signed distance from plane to pt
	Float32 SignedDistance(Plane const& plane, Vec3 const& pt);

	// Returns closest point (in world space) on segment ls to point pt
	// Parameter "t" is stored in the 4th(w) coord of result, which allows
	// easy checking for if the result is an endpoint of ls.
	Vec4 ClosestPoint(Segment const& ls, Vec3 const& pt);

	// Similar to above, but returns the point on the line (not segment!) defined by
	// the two points given by ls. Note the length of the direction vector will be used 
	// to parameterize the result point and will affect "t".
	Vec4 ClosestPointOnLine(Segment const& ls, Vec3 const& pt);

	// Same as above, but representing a line as a point and direction vector (with
	// length > 0). Note the length of the direction vector will be used to parameterize
	// the result point and will affect "t".
	Vec4 ClosestPointOnLine(Vec3 const& lineDir, Vec3 const& linePt, Vec3 const& pt);

	// Returns closest points (in world space) on each segment to the other segment, and the
	// square distance between them
	ClosestPointsQuery ClosestPoints(Segment const& A, Segment const& B);

	// Same as above but does not check if A or B degenerate to a point, and requires
	// precomputation of their direction vectors (dA and dB) and their square magnitudes 
	// (mag2A and mag2B).
	ClosestPointsQuery ClosestPointsNonDegenerate(Segment const& A, Segment const& B, Vec3 const& dA, Vec3 const& dB, Float32 mag2A, Float32 mag2B);
}

#endif

