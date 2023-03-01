#include "pch.h"
#include "RigidBody.h"


namespace drb {

	namespace physics {

		void RigidBody::ProjectPositions(Float32 h)
		{
			static constexpr Vec3 gravity{ 0.0f, -9.8f, 0.0f };

			if (type == Type::Kinematic) { return; }

			// Temporarily update velocities to project position and orientation
			prevPosition = position;
			linearVelocity += h * (accumulatedForces * invMass + gravityScale * gravity);
			position += h * linearVelocity;

			prevOrientation = orientation;
			Mat3 const rot = glm::toMat3(orientation);
			Mat3 const invI = rot * invInertiaLocal * glm::transpose(rot);
			Vec3 const gyro = glm::cross(angularVelocity, glm::inverse(invI) * angularVelocity);
			angularVelocity += h * invI * (accumulatedTorques - gyro);
			orientation += h * 0.5f * Quat(0, angularVelocity) * orientation;
			orientation = Normalize(orientation);
		}

#ifdef DRB_PHYSICS_XPBD
		void RigidBody::ProjectVelocities(Float32 h)
		{
			// Recompute velocities based on dx and dq -- rough approximation
			Vec3 const dx = position - prevPosition;
			linearVelocity = dx / h;

			Quat const dq = orientation * glm::inverse(prevOrientation);
			angularVelocity = 2.0f * Vec3(dq.x, dq.y, dq.z) / h;
			if (dq.w < 0.0f) { angularVelocity *= -1.0f; }
		}
#else
		void RigidBody::ProjectVelocities(Float32 h)
		{
			if (type == Type::Kinematic) { return; }

			// Update position
			prevPosition = position;
			position += h * linearVelocity;

			// Update orientation
			prevOrientation = orientation;
			Quat const dq{ 0, angularVelocity.x , angularVelocity.y, angularVelocity.z };
			orientation += h * 0.5f * dq * orientation;
			orientation = Normalize(orientation);
		}
#endif

		void RigidBody::ProjectForces(Float32 h)
		{
			static constexpr Vec3 gravity{ 0.0f, -9.8f, 0.0f };

			if (type == Type::Kinematic) 
			{ 
				// Compute velocities from dx and dq -- rough approximation
				Vec3 const dx = position - prevPosition;
				linearVelocity = dx / h;

				Quat const dq = orientation * glm::inverse(prevOrientation);
				angularVelocity = 2.0f * Vec3(dq.x, dq.y, dq.z) / h;
				if (dq.w < 0.0f) { angularVelocity *= -1.0f; }
			}
			else
			{
				// Update linear velocity
				linearVelocity += h * (gravity * gravityScale + accumulatedForces * invMass);
				linearVelocity *= (1.0f - linearDamping);


				// Update angular velocity
				Vec3 const w = accumulatedTorques - glm::cross(angularVelocity, GetInertiaTensor() * angularVelocity); // gyroscopic term may need to be ignored
				angularVelocity += h * GetInverseInertiaTensor() * w;
				angularVelocity *= (1.0f - angularDamping);


				// Zero out forces and torque
				accumulatedForces = Vec3(0);
				accumulatedTorques = Vec3(0);
			}

		}
	}
}