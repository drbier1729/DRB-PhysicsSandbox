#ifndef DRB_RAYCAST_H
#define DRB_RAYCAST_H

namespace drb::physics {

	class RigidBody;
	struct CollisionShapeBase;
	struct AABB;
	struct Sphere;

	struct CastResult
	{
		Vec3 point = Vec3(std::numeric_limits<Real>::max());
		Real distance = std::numeric_limits<Real>::max();
		Bool hit = false;

		operator Bool() const { return hit; }
	};


	struct Ray
	{
		Vec3 p = Vec3(0),		// origin point
			 d = Vec3(1, 0, 0); // (normalized) direction

		CastResult Cast(AABB const& aabb) const;
		CastResult Cast(Sphere const& sph, Vec3 const& sphCenter) const;
	};

}
#include "RayCast.inl"
#endif