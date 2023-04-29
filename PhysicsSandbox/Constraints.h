#ifndef DRB_CONSTRAINTS_H
#define DRB_CONSTRAINTS_H

namespace drb::physics {

	class RigidBody;

	// Precomputed data used by Constraint::ComputeCorrection methods
	struct ConstraintData
	{
		RigidBody* A, * B;
		Vec3 wrA, wrB;     // contact points on each body in world rotation (NOT translated). unused for RotationalConstraints.
		Mat3 invIA, invIB; // inverse inertia tensors in world space
		Real w;            // generalized total inverse mass

		ConstraintData(RigidBody* A, RigidBody* B);
		void ComputeRadiiAndMasses(Vec3 const& rA, Vec3 const& rB, Vec3 const& n);
	};

	class PositionalConstraint
	{
	public:

	private:
		// vector from COM to a point on each body, in
		// the local space of each body
		Vec3 rA = {}, rB = {};

		// Lagrange undetermined multiplier, updated each frame
		Real lambda = 0.0_r;

		// compliance
		Real a = 0.0_r;

	public:
		PositionalConstraint(Vec3 const& ptLocalA, Vec3 const& ptLocalB, Real compliance = 0.0_r);

		PositionalConstraint() = default;
		PositionalConstraint(PositionalConstraint const&) = default;
		PositionalConstraint& operator=(PositionalConstraint const&) = default;
		PositionalConstraint(PositionalConstraint &&) noexcept = default;
		PositionalConstraint& operator=(PositionalConstraint &&) noexcept = default;
		~PositionalConstraint() noexcept = default;

		inline void Reset();
		inline void Flip();

		void ApplyCorrection(ConstraintData& data, Vec3 const& n, Real c, Real invDT);
	};
	
	class RotationalConstraint
	{
	private:
		// Lagrange undetermined multiplier, updated each frame
		Real lambda = 0.0_r;

		// compliance
		Real a = 0.0_r;

	public:
		RotationalConstraint(Real compliance = 0.0_r);

		RotationalConstraint() = default;
		RotationalConstraint(RotationalConstraint const&) = default;
		RotationalConstraint& operator=(RotationalConstraint const&) = default;
		RotationalConstraint(RotationalConstraint&&) noexcept = default;
		RotationalConstraint& operator=(RotationalConstraint&&) noexcept = default;
		~RotationalConstraint() noexcept = default;

		inline void Reset();

		void ApplyCorrection(ConstraintData& data, Vec3 const& axis, Real theta, Real invDT);
	};

	class CollisionConstraint
	{
	private:
		// vector from COM to a point on each body, in
		// the local space of each body
		Vec3 rA = {}, rB = {};

		// Lagrange undetermined multipliers
		Real lambdaN = 0.0_r;
		Real lambdaT = 0.0_r;

	public:
		CollisionConstraint(Vec3 const& ptLocalA, Vec3 const& ptLocalB);

		CollisionConstraint() = default;
		CollisionConstraint(CollisionConstraint const&) = default;
		CollisionConstraint& operator=(CollisionConstraint const&) = default;
		CollisionConstraint(CollisionConstraint&&) noexcept = default;
		CollisionConstraint& operator=(CollisionConstraint&&) noexcept = default;
		~CollisionConstraint() noexcept = default;

		inline void Reset();
		inline void Flip(); 

		inline void ApplyNormalCorrection(ConstraintData& data, Vec3 const& n, Real c);
		inline void ApplyTangentCorrection(ConstraintData& data, Vec3 const& t, Real c);

		std::pair<Vec3, Vec3> WorldPoints(RigidBody const* A, RigidBody const* B) const;

	private:
		void ApplyImpl(ConstraintData& data, Vec3 const& dir, Real c, Real& lambda);

	public:
		friend void SolveManifoldPositions(struct ContactManifold& manifold, Real invDT);
		friend void SolveManifoldVelocities(struct ContactManifold& manifold, Real h);
	};
}

#include "Constraints.inl"
#endif

