#include "pch.h"
#include "RigidBody.h"


namespace drb {

	namespace physics {


		void RigidBody::ProjectPositions(Float32 h)
		{
			if (type == Type::Kinematic) { return; }

			static constexpr Vec3 gravity{ 0.0f, -9.8f, 0.0f };

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

		void RigidBody::ProjectVelocities(Float32 h)
		{
			if (type == Type::Kinematic) { return; }
			
			linearVelocity = (position - prevPosition) / h;

			Quat const dq = orientation * glm::inverse(prevOrientation);
			angularVelocity = 2.0f * Vec3(dq.x, dq.y, dq.z) / h;
			if (dq.w < 0.0f) { angularVelocity *= -1.0f; }
		}
	}
}