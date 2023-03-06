#include "pch.h"
#include "GeometryPrimitives.h"

#include "Math.h"
#include "GeometryPrimitiveQueries.h"

namespace drb::physics {

	Plane Plane::Make(Vec3 const& normal, Vec3 const& point)
	{
		return Plane{ .n = normal, .d = glm::dot(normal, point) };
	}
		
	Plane Plane::Make(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2)
	{
		Vec3 const a = Normalize(p1 - p0), b = Normalize(p2 - p0);
		
		Vec3 const n = glm::cross(a, b);
		
		return Plane::Make(n, p0);
	}
		
	Plane Plane::Transformed(Mat4 const& transform) const
	{
		Vec3 const normal = Mat3(transform) * n;
		return Plane{
			.n = normal,
			.d = glm::dot(Vec3(transform[3]), normal) + d
		};
	}
		
	Plane Plane::Rotated(Quat const& orientation) const
	{
		Vec3 const p = n * d;
		Vec3 const newN = glm::rotate(orientation, n);
		return Make(newN, p);
	}

	Plane Plane::MovedTo(Vec3 const& position) const
	{
		return Make(n, position);
	}

	Plane Plane::MovedBy(Vec3 const& displacement) const
	{
		Vec3 const p = n * d;
		return MovedTo(p + displacement);
	}



	// See Ericson 8.3.4 (Sutherland-Hodgman clipping with fat planes)
	void Polygon::Split(Plane const& plane, Polygon& front, Polygon& back)
	{
		if (verts.empty()) { return; }

		// Test all edges (a, b) starting with edge from last to first vertex
		auto const numVerts = verts.size();

		Vec3 a = verts.back();
		auto aSide = Classify(plane, a);

		// Loop over all edges given by vertex pair (n-1, n)
		for (auto n = 0ull; n < numVerts; n++)
		{
			Vec3 const b = verts[n];
			auto const bSide = Classify(plane, b);

			if (bSide == Side::Front)
			{
				if (aSide == Side::Back)
				{
					// Edge (a, b) straddles, output intersection point to both sides
					// Consistently clip edge as ordered going from in front -> back
					Vec3 i{};
					Float32 t{};
					Intersect(Segment{ .b = b, .e = a }, plane, t, i);

					ASSERT(Classify(plane, i) == Side::On, "Intersection point must be on plane");

					front.verts.push_back(i);
					back.verts.push_back(i);
				}
				// In all three cases, output b to the front side
				front.verts.push_back(b);
			}
			else if (bSide == Side::Back)
			{
				if (aSide == Side::Front)
				{
					// Edge (a, b) straddles plane, output intersection point
					Vec3 i{};
					Float32 t{};
					Intersect(Segment{ .b = a, .e = b }, plane, t, i);

					ASSERT(Classify(plane, i) == Side::On, "Intersection point must be on plane");

					front.verts.push_back(i);
					back.verts.push_back(i);
				}
				else if (aSide == Side::On)
				{
					// Output a when edge (a, b) goes from ‘on’ to ‘behind’ plane
					back.verts.push_back(a);
				}
				// In all three cases, output b to the back side
				back.verts.push_back(b);
			}
			else
			{
				// b is on the plane. In all three cases output b to the front side
				front.verts.push_back(b);

				// In one case, also output b to back side
				if (aSide == Side::Back)
				{
					back.verts.push_back(b);
				}
			}

			// Keep b as the starting point of the next edge
			a = b;
			aSide = bSide;
		}
	}

	Segment Segment::Transformed(Mat4 const& transform) const
	{
		return Segment{
			.b = transform * Vec4(b, 1.0f),
			.e = transform * Vec4(e, 1.0f)
		};
	}

	Vec3 Segment::Vector() const
	{
		return e - b;
	}

	Float32 Segment::Length() const
	{
		return glm::length(e - b);
	}
	
	Float32 Segment::Length2() const
	{
		return glm::length2(e - b);
	}
}