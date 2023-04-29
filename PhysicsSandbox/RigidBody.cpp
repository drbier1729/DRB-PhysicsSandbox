#include "pch.h"
#include "RigidBody.h"

#include "DRBAssert.h"

namespace drb::physics {

	void RigidBody::ProjectPositions(Real h)
	{
		// Project position
		prevPosition = position;
		linearVelocity += h * (accumulatedForces * invMass + gravityScale * gravity);

		position += h * linearVelocity;

		// Project orientation
		prevOrientation = orientation;
		Mat3 const invI = GetInverseInertiaTensor();
		Vec3 const gyro = glm::cross(angularVelocity, glm::inverse(invI) * angularVelocity);
		angularVelocity += h * invI * (accumulatedTorques - gyro);

		orientation += h * 0.5_r * Quat(0, angularVelocity) * orientation;
		orientation = Normalize(orientation);
	}

	void RigidBody::ProjectVelocities(Real invH)
	{
		// Recompute velocities based on dx and dq -- rough approximation
		Vec3 const dx = position - prevPosition;
		prevLinearVelocity = linearVelocity;
		linearVelocity = dx * invH;

		Quat const dq = orientation * glm::inverse(prevOrientation);
		prevAngularVelocity = angularVelocity;
		angularVelocity = Vec3(dq.x, dq.y, dq.z);
		if (dq.w < 0.0_r) 
		{
			angularVelocity *= -2.0_r * invH;
		}
		else 
		{
			angularVelocity *= 2.0_r * invH;
		}
	}

	// OLD SEQUENTIAL IMPULSES
	
	//void RigidBody::ProjectVelocities(Real h)
	//{
	//	if (type == Type::Kinematic) { return; }

	//	// Update position
	//	prevPosition = position;
	//	position += h * linearVelocity;

	//	// Update orientation
	//	prevOrientation = orientation;
	//	Quat const dq{ 0, angularVelocity.x , angularVelocity.y, angularVelocity.z };
	//	orientation += h * 0.5_r * dq * orientation;
	//	orientation = Normalize(orientation);
	//}
	
	//void RigidBody::ProjectForces(Real h)
	//{
	//	// TODO: this isn't right! Want to control kinematic bodies
	//	// using their velocity
	//	if (type == Type::Kinematic) 
	//	{ 
	//		// Compute velocities from dx and dq -- rough approximation
	//		Vec3 const dx = position - prevPosition;
	//		linearVelocity = dx / h;

	//		Quat const dq = orientation * glm::inverse(prevOrientation);
	//		angularVelocity = 2.0_r * Vec3(dq.x, dq.y, dq.z) / h;
	//		if (dq.w < 0.0_r) { angularVelocity *= -1.0_r; }
	//	}
	//	else
	//	{
	//		// Update linear velocity
	//		linearVelocity += h * (gravity * gravityScale + accumulatedForces * invMass);
	//		linearVelocity *= (1.0_r - linearDamping);


	//		// Update angular velocity
	//		Vec3 const w = accumulatedTorques - glm::cross(angularVelocity, GetInertiaTensor() * angularVelocity); // gyroscopic term may need to be ignored
	//		angularVelocity += h * GetInverseInertiaTensor() * w;
	//		angularVelocity *= (1.0_r - angularDamping);


	//		// Zero out forces and torque
	//		accumulatedForces = Vec3(0);
	//		accumulatedTorques = Vec3(0);
	//	}
	//}
}