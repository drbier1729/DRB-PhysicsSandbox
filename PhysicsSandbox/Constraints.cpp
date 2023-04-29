#include "pch.h"
#include "Constraints.h"

#define DRB_ASSERT_MODE DRB_DEFAULT_ASSERT
#include "DRBAssert.h"

#include "RigidBody.h"
#include "Math.h"


namespace drb::physics {

	ConstraintData::ConstraintData(RigidBody* A_, RigidBody* B_)
		: A{ A_ }, B{ B_ },
		wrA{ 0.0_r }, wrB{ 0.0_r },
		invIA{ 0.0_r }, invIB{ 0.0_r },
		w{ 0.0_r }
	{}

	void ConstraintData::ComputeRadiiAndMasses(Vec3 const& rA, Vec3 const& rB, Vec3 const& n)
	{
		wrA = A->LocalToWorldVec(rA);
		wrB = B->LocalToWorldVec(rB);
		invIA = A->GetInverseInertiaTensor();
		invIB = B->GetInverseInertiaTensor();

		Vec3 const rA_x_n = glm::cross(wrA, n);
		Vec3 const rB_x_n = glm::cross(wrB, n);
		Real const wA = A->GetInverseMass() + glm::dot(rA_x_n, invIA * rA_x_n);
		Real const wB = B->GetInverseMass() + glm::dot(rB_x_n, invIB * rB_x_n);

		w = wA + wB;
		ASSERT(not EpsilonEqual(w, 0.0_r), "At least one body's mass needs to be non-zero.");
	}

	PositionalConstraint::PositionalConstraint(Vec3 const& ptLocalA, Vec3 const& ptLocalB, Real compliance)
		: rA{ ptLocalA }, rB{ ptLocalB }, lambda{ 0.0_r }, a{ compliance }
	{}

	void PositionalConstraint::ApplyCorrection(ConstraintData& data, Vec3 const& n, Real c, Real invDT)
	{
		ASSERT(EpsilonEqual(glm::length2(n), 1.0_r), "Provided direction was not normalized");

		if (EpsilonEqual(c, 0.0_r)) { return; }

		// Compute Lagrange undetermined multiplier and its delta
		Real const aScaled = a * invDT * invDT;
		Real const dLambda = (-c - aScaled * lambda) / (data.w + aScaled);
		lambda += dLambda;
		Vec3 const p = dLambda * n;

		// Apply correction
		RigidBody* A = data.A, * B = data.B;
		A->position += p * A->invMass;
		B->position -= p * B->invMass;

		A->orientation += 0.5_r * Quat(0.0_r, data.invIA * glm::cross(data.wrA, p)) * A->orientation;
		B->orientation -= 0.5_r * Quat(0.0_r, data.invIB * glm::cross(data.wrB, p)) * B->orientation;
		A->orientation = Normalize(A->orientation);
		B->orientation = Normalize(B->orientation);


		ASSERT(not glm::any(glm::isnan(A->position)), "A was nan");
		ASSERT(not glm::any(glm::isnan(A->orientation)), "A was nan");
		ASSERT(not glm::any(glm::isnan(B->position)), "B was nan");
		ASSERT(not glm::any(glm::isnan(B->orientation)), "B was nan");
	}
	
	RotationalConstraint::RotationalConstraint(Real compliance)
		: lambda{ 0.0_r }, a{ compliance }
	{}

	void RotationalConstraint::ApplyCorrection(ConstraintData& data, Vec3 const& axis, Real theta, Real invDT)
	{
		ASSERT(EpsilonEqual(glm::length2(axis), 1.0_r), "Provided axis was not normalized");

		if (EpsilonEqual(theta, 0.0_r)) { return; }

		// Compute Lagrange undetermined multiplier and its delta
		Real const aScaled = a * invDT * invDT;
		Real const dLambda = (-theta - aScaled * lambda) / (data.w + aScaled);
		lambda += dLambda;
		Vec3 const p = dLambda * axis;

		// Apply correction
		RigidBody* A = data.A, * B = data.B;
		A->orientation += 0.5_r * Quat(0.0_r, data.invIA * glm::cross(data.wrA, p)) * A->orientation;
		B->orientation -= 0.5_r * Quat(0.0_r, data.invIB * glm::cross(data.wrB, p)) * B->orientation;
		A->orientation = Normalize(A->orientation);
		B->orientation = Normalize(B->orientation);


		ASSERT(not glm::any(glm::isnan(A->position)), "A was nan");
		ASSERT(not glm::any(glm::isnan(A->orientation)), "A was nan");
		ASSERT(not glm::any(glm::isnan(B->position)), "B was nan");
		ASSERT(not glm::any(glm::isnan(B->orientation)), "B was nan");
	}

	CollisionConstraint::CollisionConstraint(Vec3 const& ptLocalA, Vec3 const& ptLocalB) 
		: rA{ptLocalA}, rB{ptLocalB}, lambdaN{0.0_r}, lambdaT{0.0_r}
	{}

	void CollisionConstraint::ApplyImpl(ConstraintData& data, Vec3 const& dir, Real c, Real& lambda)
	{
		ASSERT(EpsilonEqual(glm::length2(dir), 1.0_r), "Provided direction was not normalized");

		if (EpsilonEqual(c, 0.0_r)) { return; }

		Real const dLambda = -c / data.w;
		lambda += dLambda;
		Vec3 const p = dLambda * dir;

		RigidBody* A = data.A, * B = data.B;

		A->position += p * A->invMass;
		B->position -= p * B->invMass;

		A->orientation += 0.5_r * Quat(0.0_r, data.invIA * glm::cross(data.wrA, p)) * A->orientation;
		B->orientation -= 0.5_r * Quat(0.0_r, data.invIB * glm::cross(data.wrB, p)) * B->orientation;
		A->orientation = Normalize(A->orientation);
		B->orientation = Normalize(B->orientation);

		ASSERT(not glm::any(glm::isnan(A->position)), "A was nan");
		ASSERT(not glm::any(glm::isnan(A->orientation)), "A was nan");
		ASSERT(not glm::any(glm::isnan(B->position)), "B was nan");
		ASSERT(not glm::any(glm::isnan(B->orientation)), "B was nan");
	}


	std::pair<Vec3, Vec3> CollisionConstraint::WorldPoints(RigidBody const* A, RigidBody const* B) const 
	{
		return { A->LocalToWorldPoint(rA), B->LocalToWorldPoint(rB) };
	}
}