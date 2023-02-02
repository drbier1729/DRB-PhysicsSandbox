#include "pch.h"
#include "PhysicsGeometry.h"

namespace drb {
	namespace physics {

		AABB MakeAABB(Mesh const& sph, Mat4 const& tr)
		{
			ASSERT(false, "Not implemented");
			return AABB{};
		}

		namespace util {

			// From Realtime Collision Detection by Ericson, Ch.8
			//void SplitPolygonSH(Polygon& poly, Plane plane, Polygon** frontPoly, Polygon** backPoly)
			//{
			//	int numFront = 0, numBack = 0;
			//	Point frontVerts[MAX_POINTS], backVerts[MAX_POINTS];

			//	// Test all edges (a, b) starting with edge from last to first vertex
			//	int numVerts = poly.NumVertices();
			//	Point a = poly.GetVertex(numVerts – 1);
			//	int aSide = ClassifyPointToPlane(a, plane);

			//	// Loop over all edges given by vertex pair (n-1, n)
			//	for (intn = 0; n < numVerts; n++) {
			//		Point b = poly.GetVertex(n);
			//		int bSide = ClassifyPointToPlane(b, plane);
			//		if (bSide == POINT_IN_FRONT_OF_PLANE) {
			//			if (aSide == POINT_BEHIND_PLANE) {
			//				// Edge (a, b) straddles, output intersection point to both sides
			//				Point i = IntersectEdgeAgainstPlane(a, b, plane);
			//				assert(ClassifyPointToPlane(i, plane) == POINT_ON_PLANE);
			//				frontVerts[numFront++] = backVerts[numBack++] = i;
			//			}
			//			// In all three cases, output b to the front side
			//			frontVerts[numFront++] = b;
			//		}
			//		else if (bSide == POINT_BEHIND_PLANE) {
			//			if (aSide == POINT_IN_FRONT_OF_PLANE) {
			//				// Edge (a, b) straddles plane, output intersection point
			//				Point i = IntersectEdgeAgainstPlane(a, b, plane);
			//				assert(ClassifyPointToPlane(i, plane) == POINT_ON_PLANE);
			//				frontVerts[numFront++] = backVerts[numBack++] = i;
			//			}
			//			else if (aSide == POINT_ON_PLANE) {
			//				// Output a when edge (a, b) goes from ‘on’ to ‘behind’ plane
			//				backVerts[numBack++] = a;
			//			}
			//			// In all three cases, output b to the back side
			//			backVerts[numBack++] = b;
			//		}
			//		else {
			//			// b is on the plane. In all three cases output b to the front side
			//			frontVerts[numFront++] = b;
			//			// In one case, also output b to back side
			//			if (aSide == POINT_BEHIND_PLANE)
			//				backVerts[numBack++] = b;
			//		}

			//		// Keep b as the starting point of the next edge
			//		a = b;
			//		aSide = bSide;
			//	}
			//	// Create (and return) two new polygons from the two vertex lists
			//	*frontPoly = new Polygon(numFront, frontVerts);
			//	*backPoly = new Polygon(numBack, backVerts);
			//}
		}
	}
}
