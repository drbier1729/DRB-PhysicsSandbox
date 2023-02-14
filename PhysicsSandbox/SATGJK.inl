
namespace drb::physics::util {

	inline Simplex::Type Simplex::GetType() const
	{
		return Type{ size };
	}


	inline void Simplex::PushVert(Vec3 const& vA, Vec3 const& vB)
	{
		ASSERT(size < 4, "Too many vertices");
		vertsA[size] = vA;
		vertsB[size] = vB;
		size++;
	}

	inline void Simplex::PushVert(Vec3 const& vA, Convex::EdgeID eIdxA, Vec3 const& vB, Convex::EdgeID eIdxB)
	{
		ASSERT(size < 4, "Too many vertices");
		vertsA[size] = vA;
		vertsB[size] = vB;
		edgeIdxsA[size] = eIdxA;
		edgeIdxsB[size] = eIdxB;
		size++;
	}


	template<class ShapeA, class ShapeB>
	ClosestPointsQuery GJK(ShapeA const& A, Mat4 const& trA, SupportFn<ShapeA> SupportA,
						   ShapeB const& B, Mat4 const& trB, SupportFn<ShapeB> SupportB,
		                   Simplex* seed = nullptr)
	{
		ASSERT(false, "Not implemented yet");
		return ClosestPointsQuery{};

		
		//Mat3 const invRotA = glm::inverse(Mat3(trA));
		//Mat3 const invRotB = glm::inverse(Mat3(trB));
		//Vec3 const centroidA = trA[3];
		//Vec3 const centroidB = trB[3];

		//Vec3 const initialAxis = centroidA - centroidB;

		//Vec3 pt = SupportA(A, invRotA * initialAxis) - SupportB(B, -invRotB * initialAxis);
		//Vec3 dir = -pt;
		//Simplex s{};
		////PushVert(s, pt);

		//static constexpr Uint32 maxIters = 32;
		//for (Uint32 i = 0; i < maxIters; ++i)
		//{
		//	pt = SupportA(A, dir) - SupportB(B, -dir);
		//	if (glm::dot(pt, dir) < 0.0f)
		//	{
		//		break;
		//	}

		//	//PushVert(s, pt);
		//	//s, dir, containsOrigin = NearestSimplex(s);
		//	//if (containsOrigin) { break; }
		//}

		//return ClosestPointsQuery{};
	}

}