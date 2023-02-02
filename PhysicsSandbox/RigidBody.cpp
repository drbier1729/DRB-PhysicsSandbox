#include "pch.h"
#include "RigidBody.h"


namespace drb {

	namespace physics {

		namespace util {
			// This is useful bc CollisionShapes will often/always be fixed to a RigidBody and the
			// origin (in the local space of the RigidBody) will be the RigidBody's center of mass.
			template<Shape T> inline static Mat3 InertiaTensorAboutOrigin(CollisionShape<T> const& s)
			{
				// Rotate the body-space inertia tensor
				Mat3 const rot = Mat3(s.transform);
				Mat3 I = rot * ComputeInertiaTensor(s.shape) * glm::transpose(rot);

				// Use Parallel Axis Theorem to translate the inertia tensor
				Vec3 const disp = -s.transform[3]; // displacement vector from COM to origin
				I += glm::dot(disp, disp) * Mat3(1) - glm::outerProduct(disp, disp);

				// Assuming uniform density, total mass will factor out
				return s.mass * I;
			}
		}

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
		
		RigidBody& RigidBody::SetMass(Float32 mass)
		{
			ASSERT(mass > 0.0f, "Mass must be nonzero");

			// Adjust collider masses
			Float32 massRatio = 0.0f;
			for (auto&& s : spheres) {
				massRatio = s.mass * invMass;
				s.mass *= massRatio * mass;
			}
			for (auto&& c : capsules) {
				massRatio = c.mass * invMass;
				c.mass *= massRatio * mass;
			}
			for (auto&& h : hulls) {
				massRatio = h.mass * invMass;
				h.mass *= massRatio * mass;
			}

			// Adjust inertia tensor
			invInertiaLocal *= 1.0f / (invMass * mass);

			// Set inverse mass
			invMass = 1.0f / mass;
			return *this;
		}


		Vec3 RigidBody::FinalizeCollisionGeometry()
		{
			Float32 mass = 0.0f;
			Vec3 com = Vec3(0.0f);

			// Compute mass and center of mass
			for (auto&& s : spheres) {
				mass += s.mass;
				com += mass * Vec3(s.transform[3]);
			}
			for (auto&& c : capsules) {
				mass += c.mass;
				com += mass * Vec3(c.transform[3]);
			}
			for (auto&& h : hulls) {
				mass += h.mass;
				com += mass * Vec3(h.transform[3]);
			}
			ASSERT(mass > 0.0f, "Total mass must be > 0.");
			invMass = 1.0f / mass;
			com *= invMass;

			// Shift all collider transforms s.t. COM is at origin
			for (auto&& s : spheres) {
				s.transform[3] -= Vec4(com, 0.0f);
			}
			for (auto&& c : capsules) {
				c.transform[3] -= Vec4(com, 0.0f);
			}
			for (auto&& h : hulls) {
				h.transform[3] -= Vec4(com, 0.0f);
			}

			// Compute local inertia tensor
			Mat3 I{ 0.0f };
			for (auto&& s : spheres) {
				I += util::InertiaTensorAboutOrigin(s);
			}
			for (auto&& c : capsules) {
				I += util::InertiaTensorAboutOrigin(c);
			}
			for (auto&& h : hulls) {
				I += util::InertiaTensorAboutOrigin(h);
			}
			invInertiaLocal = glm::inverse(I);

			return com;
		}
	}
}