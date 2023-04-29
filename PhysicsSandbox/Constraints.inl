
namespace drb::physics {
	inline void PositionalConstraint::Reset()
	{
		lambda = 0.0_r;
	}

	inline void RotationalConstraint::Reset()
	{
		lambda = 0.0_r;
	}

	inline void PositionalConstraint::Flip()
	{
		std::swap(rA, rB);
	}
	
	inline void CollisionConstraint::Reset()
	{
		lambdaN = 0.0_r;
		lambdaT = 0.0_r;
	}
	inline void CollisionConstraint::Flip()
	{
		std::swap(rA, rB);
	}
	
	inline void CollisionConstraint::ApplyNormalCorrection(ConstraintData& data, Vec3 const& n, Real c)
	{
		ApplyImpl(data, n, c, lambdaN);
	}
	
	inline void CollisionConstraint::ApplyTangentCorrection(ConstraintData& data, Vec3 const& n, Real c)
	{
		ApplyImpl(data, n, c, lambdaT);
	}
}