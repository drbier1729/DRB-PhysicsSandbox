#ifndef DRB_GEOMETRYPRIMITIVES_H
#define DRB_GEOMETRYPRIMITIVES_H

namespace drb::physics {

	// Point p on plane  iff  n * p - d = 0
	struct Plane
	{
		Vec3 n = Vec3(1, 0, 0); // unit normal
		Real d = 0.0_r;         // signed distance

		static constexpr Real thickness = 0.01_r; // 1cm

		Plane Transformed(Mat4 const& transform) const;
		Plane Rotated(Quat const& orientation) const;
		Plane MovedTo(Vec3 const& position) const;
		Plane MovedBy(Vec3 const& displacement) const;

		constexpr bool operator==(Plane const&) const = default;

		static Plane Make(Vec3 const& normal, Vec3 const& point);
		static Plane Make(Vec3 const& p0, Vec3 const& p1, Vec3 const& p2);
	};


	struct Polygon
	{
		// oriented counterclockwise
		std::vector<Vec3> verts;

		// This function pushes vertices into frontPoly and backPoly after classifying those
		// points as being in front of or behind plane. Uses Sutherland-Hodgeman clipping algorithm.
		void Split(Plane const& plane, Polygon& frontPoly, Polygon& backPoly);
	};

	struct Segment
	{
		Vec3 b = Vec3(0);       // begin point
		Vec3 e = Vec3(1, 0, 0); // end point

		Segment Transformed(Mat4 const& transform) const;
		Vec3    Vector() const;
		Real Length() const;
		Real Length2() const;
	};

}

#endif