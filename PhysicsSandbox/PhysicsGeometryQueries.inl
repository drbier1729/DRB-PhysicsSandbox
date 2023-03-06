
namespace drb {
	namespace physics {
		
		//inline Float32 SignedDistance(Vec3 const& point, Plane const& plane)
		//{
		//	return glm::dot(point, plane.n) - plane.d;
		//}


		//// See Ericson 5.3.1
		//inline Bool Intersect(Segment const& seg, Plane const& plane, float& t, Vec3& q)
		//{
		//	// Compute the t value for the directed line v intersecting the plane
		//	Vec3 const v = seg.e - seg.b;
		//	t = -SignedDistance(seg.b, plane) / glm::dot(plane.n, v);

		//	// If t in [0..1] compute and return intersection point
		//	if (t >= 0.0f && t <= 1.0f) 
		//	{
		//		q = seg.b + t * v;
		//		return true;
		//	}

		//	// Else no intersection
		//	return false;
		//}


		//// Classify point p to a plane thickened by a given thickness epsilon
		//inline Side ClassifyPointToPlane(Vec3 const& p, Plane const& plane)
		//{
		//	Float32 const dist = SignedDistance(p, plane);

		//	// Classify p based on the signed distance
		//	if (dist > Plane::thickness)  { return Side::Front; }
		//	if (dist < -Plane::thickness) { return Side::Back; }
		//	return Side::On;
		//}
	}
}