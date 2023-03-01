#include "pch.h"
#include "GeometryQueryDataStructures.h"

namespace drb::physics {

	CollisionPair::CollisionPair(CollisionProxy const& a_, CollisionProxy const& b_)
		: a{a_}, b{b_}
	{
		ASSERT(a != b, "RigidBodies a and b must be unique.");

		if (b.bvHandle < a.bvHandle)
		{
			std::swap(a, b);
		}
	}
}