#include "pch.h"
//#include "CollisionGeometry.h"
//
//#include "PhysicsGeometryQueries.h"
//
//namespace drb {
//	namespace physics {
//
//		Float32 Plane::thickness = 0.1f;
//
//		namespace util {
//			// This is useful bc CollisionShapes will often/always be fixed to a RigidBody and the
//			// origin (in the local space of the RigidBody) will be the RigidBody's center of mass.
//			template<Shape T>
//			static Mat3 InertiaTensorAboutOrigin(CollisionShape<T> const& s)
//			{
//				// Rotate the body-space inertia tensor
//				Mat3 const rot = Mat3(s.transform);
//				Mat3 I = rot * ComputeInertiaTensor(s.shape) * glm::transpose(rot);
//
//				// Use Parallel Axis Theorem to translate the inertia tensor
//				Vec3 const disp = -s.transform[3]; // displacement vector from COM to origin
//				I += glm::dot(disp, disp) * Mat3(1) - glm::outerProduct(disp, disp);
//
//				// Assuming uniform density, total mass will factor out
//				return s.mass * I;
//			}
//		}
//
//
//		// See Ericson 8.3.4 (Sutherland-Hodgman clipping with fat planes)
//		// TODO : verify that this maintains ordering of original poly (i.e. counterclockwise)
//		void SplitPolygon(Polygon const& poly, Plane const& plane, Polygon& front, Polygon& back)
//		{
//			if (poly.verts.empty()) { return; }
//
//			// Test all edges (a, b) starting with edge from last to first vertex
//			auto const numVerts = poly.verts.size();
//
//			Vec3 a = poly.verts.back();
//			auto aSide = ClassifyPointToPlane(a, plane);
//
//			// Loop over all edges given by vertex pair (n-1, n)
//			for (auto n = 0ull; n < numVerts; n++)
//			{
//				Vec3 const b = poly.verts[n];
//				auto const bSide = ClassifyPointToPlane(b, plane);
//
//				if (bSide == Side::Front)
//				{
//					if (aSide == Side::Back)
//					{
//						// Edge (a, b) straddles, output intersection point to both sides
//						// Consistently clip edge as ordered going from in front -> back
//						Vec3 i{};
//						Float32 t{};
//						Intersect(Segment{ .b = b, .e = a }, plane, t, i);
//
//						ASSERT(ClassifyPointToPlane(i, plane) == Side::On, "Intersection point must be on plane");
//
//						front.verts.push_back(i);
//						back.verts.push_back(i);
//					}
//					// In all three cases, output b to the front side
//					front.verts.push_back(b);
//				}
//				else if (bSide == Side::Back)
//				{
//					if (aSide == Side::Front)
//					{
//						// Edge (a, b) straddles plane, output intersection point
//						Vec3 i{};
//						Float32 t{};
//						Intersect(Segment{ .b = a, .e = b }, plane, t, i);
//
//						ASSERT(ClassifyPointToPlane(i, plane) == Side::On, "Intersection point must be on plane");
//
//						front.verts.push_back(i);
//						back.verts.push_back(i);
//					}
//					else if (aSide == Side::On)
//					{
//						// Output a when edge (a, b) goes from ‘on’ to ‘behind’ plane
//						back.verts.push_back(a);
//					}
//					// In all three cases, output b to the back side
//					back.verts.push_back(b);
//				}
//				else
//				{
//					// b is on the plane. In all three cases output b to the front side
//					front.verts.push_back(b);
//
//					// In one case, also output b to back side
//					if (aSide == Side::Back)
//					{
//						back.verts.push_back(b);
//					}
//				}
//
//				// Keep b as the starting point of the next edge
//				a = b;
//				aSide = bSide;
//			}
//		}
//
//		AABB MakeAABB(Mesh const& sph, Mat4 const& tr)
//		{
//			ASSERT(false, "Not implemented");
//			return AABB{};
//		}
//
//		void CollisionGeometry::Bake()
//		{
//			locked = true;
//
//			Float32 mass = 0.0f;
//
//			// Compute mass and center of mass
//			for (auto&& s : spheres) {
//				mass += s.mass;
//				com += mass * Vec3(s.transform[3]);
//			}
//			for (auto&& c : capsules) {
//				mass += c.mass;
//				com += mass * Vec3(c.transform[3]);
//			}
//			for (auto&& h : hulls) {
//				mass += h.mass;
//				com += mass * Vec3(h.transform[3]);
//			}
//			ASSERT(mass > 0.0f, "Total mass must be > 0.");
//			invMass = 1.0f / mass;
//			com *= invMass;
//
//			// Shift all collider transforms s.t. COM is at origin
//			for (auto&& s : spheres) {
//				s.transform[3] -= Vec4(com, 0.0f);
//			}
//			for (auto&& c : capsules) {
//				c.transform[3] -= Vec4(com, 0.0f);
//			}
//			for (auto&& h : hulls) {
//				h.transform[3] -= Vec4(com, 0.0f);
//			}
//
//			// Compute local inertia tensor
//			Mat3 I{ 0.0f };
//			for (auto&& s : spheres) {
//				I += util::InertiaTensorAboutOrigin(s);
//			}
//			for (auto&& c : capsules) {
//				I += util::InertiaTensorAboutOrigin(c);
//			}
//			for (auto&& h : hulls) {
//				I += util::InertiaTensorAboutOrigin(h);
//			}
//			invInertia = glm::inverse(I);
//
//			// Compute bounds
//			for (auto&& s : spheres) {
//				bounds = bounds.Union( MakeAABB(s.shape, s.transform) );
//			}
//			for (auto&& c : capsules) {
//				bounds = bounds.Union(MakeAABB(c.shape, c.transform));
//			}
//			for (auto&& h : hulls) {
//				bounds = bounds.Union(MakeAABB(h.shape, h.transform));
//			}
//		}			
//	}
//}
