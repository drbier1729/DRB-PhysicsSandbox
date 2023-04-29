#include "pch.h"
#include "ContactSolver.h"

#define DRB_ASSERT_MODE DRB_DEFAULT_ASSERT
#include "DRBAssert.h"

#include "RigidBody.h"
#include "Constraints.h"
#include "GeometryQueryDataStructures.h"
#include "Math.h"



namespace drb::physics {
	void SolveManifoldPositions(ContactManifold& manifold, Real invDT)
	{
		RigidBody* A   = manifold.rbA;
		RigidBody* B   = manifold.rbB;
		Vec3 const n   = manifold.normal;								// contact normal world space
		Real const us  = glm::max(A->GetFriction(), B->GetFriction()); // static friction coeff

		// Apply corrections at each contact
		ConstraintData bodyData{ A, B };
		for (Int32 i = 0; i < manifold.numContacts; ++i) 
		{
			CollisionConstraint& c = manifold.contacts[i];
			c.Reset();

			// Compute world space vectors and generalized masses
			bodyData.ComputeRadiiAndMasses(c.rA, c.rB, n);

			// Compute penetration using world space points
			Vec3 pA = A->LocalToWorldPoint(c.rA);
			Vec3 pB = B->LocalToWorldPoint(c.rB);
			Real const pen = glm::dot(pA - pB, n);
			if (pen < 0.0_r) { continue; }

			// Compute and apply normal correction
			c.ApplyNormalCorrection(bodyData, n, pen);
			
			// Compute and apply tangent correction from static friction
			Real const dLambda = -pen / bodyData.w;
			if (c.lambdaT + dLambda > us * c.lambdaN)
			{
				// Recompute pA, pB as well as the previous pA and pB
				pA = A->LocalToWorldPoint(c.rA);
				pB = B->LocalToWorldPoint(c.rB);
				Vec3 const ppA = A->LocalToPreviousWorldPoint(c.rA);
				Vec3 const ppB = B->LocalToPreviousWorldPoint(c.rB);

				Vec3 const dp = (pA - ppA) - (pB - ppB);
				auto const [t, magt] = GetNormalizedAndMagnitude(dp - glm::dot(dp, n) * n);
				
                if (not EpsilonEqual(magt, 0.0_r)) {
                    // Recompute body data
                    bodyData.ComputeRadiiAndMasses(c.rA, c.rB, t);

                    c.ApplyTangentCorrection(bodyData, t, magt);
                }
			}
		}
	}

	void SolveManifoldVelocities(ContactManifold& manifold, Real h) 
	{
		RigidBody* A        = manifold.rbA;
		RigidBody* B        = manifold.rbB;
		Vec3	   vA       = A->GetLinearVelocity();
		Vec3	   vB       = B->GetLinearVelocity();
		Vec3	   aA       = A->GetAngularVelocity();
		Vec3	   aB       = B->GetAngularVelocity();
		Vec3 const pvA      = A->GetPreviousLinearVelocity();
		Vec3 const pvB      = B->GetPreviousLinearVelocity();
		Vec3 const paA      = A->GetPreviousAngularVelocity();
		Vec3 const paB      = B->GetPreviousAngularVelocity();
		Vec3 const n        = manifold.normal;
		Real const u        = glm::max(A->GetFriction(), B->GetFriction());
		Real const e        = A->GetResitution() * B->GetResitution();
		Real const invMassA = A->GetInverseMass();
		Real const invMassB = B->GetInverseMass();

		ConstraintData bodyData{ A, B };

		for (Int32 i = 0; i < manifold.numContacts; ++i)
		{
			CollisionConstraint& c = manifold.contacts[i];
			bodyData.ComputeRadiiAndMasses(c.rA, c.rB, n);

			// Compute relative velocities
			Vec3 const v = (vA + glm::cross(aA, bodyData.wrA)) - (vB + glm::cross(aB, bodyData.wrB));
			Real const vn = glm::dot(v, n);
			auto const [vt, magvt] = GetNormalizedAndMagnitude(v - n * vn);
			
			Vec3 dv{0.0_r};

			// Tangent velocities (dynamic friction)
			if (not EpsilonEqual(magvt, 0.0_r)) {
				dv += -vt * glm::min(u * glm::abs(c.lambdaN / h), magvt);
			}

			// Normal velocities (restitution)
			Real const pvn = glm::dot(n, (pvA + glm::cross(paA, bodyData.wrA)) - (pvB + glm::cross(paB, bodyData.wrA)));
			Real const eActual = e * (glm::abs(vn) > 2.0_r * h * glm::abs(RigidBody::gravityMag)); // e if |vn| > 2|g|h, 0 else
			dv += n * (-vn + glm::min(-eActual * pvn, 0.0_r));

			// Apply the velocity correction
			Vec3 const p = dv / bodyData.w;
			vA += p * invMassA;
			vB -= p * invMassB;
			aA += bodyData.invIA * glm::cross(bodyData.wrA, p);
			aB -= bodyData.invIB * glm::cross(bodyData.wrB, p);
		}

		A->SetLinearVelocity(vA);
		B->SetLinearVelocity(vB);
		A->SetAngularVelocity(aA);
		B->SetAngularVelocity(aB);
	}


	void UpdateManifold(ContactManifold& manifold, ContactManifold const& newManifold)
	{
		// No constraint matching
		manifold.contacts = newManifold.contacts;
		manifold.numContacts = newManifold.numContacts;
		manifold.normal = newManifold.normal;


		//// Match constraints by proximity
		//static constexpr Real proximityThreshold = 1.0e-3_r;
		//ContactManifold::ContactArray mergedContacts = {};
		//ContactManifold::ContactArray newContacts = {};
		//Int32 numMerged = 0, numNew = 0;

		//for (Int32 i = 0; i < newManifold.numContacts; ++i) {

		//	PositionalConstraint const& newContact = newManifold.contacts[i];
		//	auto const [newA, newB] = newContact.LocalPoints();

		//	Int32 k = -1;
		//	for (Int32 j = 0; j < manifold.numContacts; ++j) {
		//		auto const [oldA, oldB] = newContact.LocalPoints();
		//		if (EpsilonEqual(newA, oldA, proximityThreshold) && 
		//			EpsilonEqual(newB, oldB, proximityThreshold)) 
		//		{
		//			k = j;
		//			break;
		//		}
		//		++j;
		//	}

		//	if (k > -1) {
		//		mergedContacts[numMerged] = newContact;
		//		numMerged++;
		//	}
		//	else {
		//		newContacts[numNew] = newContact;
		//		numNew++;
		//	}
		//}

		//std::copy(mergedContacts.begin(),
		//	mergedContacts.begin() + numMerged,
		//	manifold.contacts.begin());

		//std::copy(newContacts.begin(),
		//	newContacts.begin() + numNew,
		//	manifold.contacts.begin() + numMerged);

		//manifold.normal = newManifold.normal;
		//manifold.numContacts = newManifold.numContacts;


		// Match constraints by feature pairs
		/*
		ContactManifold::ContactArray mergedContacts = {};
		ContactManifold::FeatureArray mergedFeatures = {};
		ContactManifold::ContactArray newContacts = {};
		ContactManifold::FeatureArray newFeatures = {};

		Int32 numMerged = 0, numNew = 0;
		for (Int32 i = 0; auto&& newf : newManifold.features) {
			auto const& newContact = newManifold.contacts[i];

			Int32 k = -1;
			for (Int32 j = 0; auto&& oldf : manifold.features) {
				if (newf == oldf) {
					k = j;
					break;
				}
				++j;
			}

			if (k > -1) {
				mergedContacts[numMerged] = newContact;
				mergedFeatures[numMerged] = newf;
				numMerged++;
			}
			else {
				newContacts[numNew] = newContact;
				newFeatures[numNew] = newf;
				numNew++;
			}
			++i;
		}

		std::copy(mergedContacts.begin(), 
				  mergedContacts.begin() + numMerged, 
				  manifold.contacts.begin());

		std::copy(newContacts.begin(), 
				  newContacts.begin() + numNew, 
				  manifold.contacts.begin() + numMerged);

		std::copy(mergedFeatures.begin(), 
				  mergedFeatures.begin() + numMerged, 
				  manifold.features.begin());

		std::copy(newFeatures.begin(), 
				  newFeatures.begin() + numNew, 
				  manifold.features.begin() + numMerged);

		manifold.normal = newManifold.normal;
		manifold.numContacts = newManifold.numContacts;
		*/
	}
}