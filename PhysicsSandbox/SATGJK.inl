
#include "DRBAssert.h"

namespace drb::physics::util {

	inline Simplex::Type Simplex::GetType() const
	{
		return Type{ size };
	}

	inline void Simplex::PushVert(Vec3 const& vA, Convex::EdgeID eIdxA, Vec3 const& vB, Convex::EdgeID eIdxB)
	{
		ASSERT(size < 4, "Too many vertices");
		vertsA[size] = vA;
		vertsB[size] = vB;
		edgeIdxsA[size] = eIdxA;
		edgeIdxsB[size] = eIdxB;
		lambdas[size] = 0.0_r;
		size++;
	}

	inline void Simplex::ExtractWitnessPoints(Vec3& witnessA, Vec3& witnessB) const
	{
		witnessA = Vec3(0);
		witnessB = Vec3(0);
		for (Uint8 i = 0; i < size; ++i)
		{
			witnessA += lambdas[i] * vertsA[i];
			witnessB += lambdas[i] * vertsB[i];
		}
	}

	template<class ShapeA, class ShapeB>
	ClosestPointsQuery GJK(ShapeA const& A, Mat4 const& trA, SupportFn<ShapeA> SupportA,
						   ShapeB const& B, Mat4 const& trB, SupportFn<ShapeB> SupportB,
		                   Simplex* seed)
	{
		ASSERT(false, "Not implemented yet");
		return ClosestPointsQuery{};
	}

}