
namespace drb {
	namespace physics {

		inline RigidBody& RigidBody::AddForce(Vec3 const& worldSpaceForce) 
		{
			accumulatedForces += worldSpaceForce;
			return *this;
		}
		
		inline RigidBody& RigidBody::AddForce(Vec3 const& worldSpaceForce, Vec3 const& worldSpacePoint) 
		{
			Vec3 const localSpacePoint = worldSpacePoint - position;
			accumulatedForces += worldSpaceForce;
			accumulatedTorques += glm::cross(localSpacePoint, worldSpaceForce);
			return *this;
		}
		
		inline RigidBody& RigidBody::AddRelativeForce(Vec3 const& localSpaceForce)
		{
			accumulatedForces += GetOrientationMatrix() * localSpaceForce;
			return *this;
		}
		
		inline RigidBody& RigidBody::AddRelativeForce(Vec3 const& localSpaceForce, Vec3 const& localSpacePoint)
		{
			Mat3 const rot = GetOrientationMatrix();
			Vec3 const worldSpaceForce = rot * localSpaceForce;
			accumulatedForces += worldSpaceForce;
			accumulatedTorques += glm::cross(localSpacePoint, worldSpaceForce);
			return *this;
		}


		inline RigidBody& RigidBody::MoveTo(Vec3 const& target)
		{
			prevPosition = position;
			position = target;
			return *this;
		}
		inline RigidBody& RigidBody::RotateTo(Quat const& target)
		{
			prevOrientation = orientation;
			orientation = target;
			return *this;
		}
		inline RigidBody& RigidBody::RotateTo(Mat3 const& target)
		{
			return RotateTo(glm::toQuat(target));
		}
		inline RigidBody& RigidBody::RotateTo(Vec3 const& targetEulerAngles)
		{
			return RotateTo(Quat(targetEulerAngles));
		}

		inline Vec3 const& RigidBody::GetPosition() const
		{
			return position;
		}
		
		inline Quat const& RigidBody::GetOrientation() const
		{
			return orientation;
		}

		inline Mat3 RigidBody::GetOrientationMatrix() const 
		{
			return glm::toMat3(orientation);
		}
		
		inline Vec3	const& RigidBody::GetPreviousPosition() const
		{
			return prevPosition;
		}

		inline Quat	const& RigidBody::GetPreviousOrientation() const
		{
			return prevOrientation;
		}

		inline Mat3 RigidBody::GetPreviousOrientationMatrix() const
		{
			return glm::toMat3(prevOrientation);
		}

		inline Vec3 const& RigidBody::GetLinearVelocity() const 
		{
			return linearVelocity;
		}
		
		inline Vec3 const& RigidBody::GetAngularVelocity() const 
		{
			return angularVelocity;
		}

		inline Vec3	const& RigidBody::GetPreviousLinearVelocity() const
		{
			return prevLinearVelocity;
		}
		
		inline Vec3	const& RigidBody::GetPreviousAngularVelocity() const
		{
			return prevAngularVelocity;
		}

		inline Mat4 RigidBody::GetTransformMatrix() const 
		{
			return glm::translate(Mat4(1), position) * glm::toMat4(orientation);
		}
		
		inline Mat4 RigidBody::GetPreviousTransformMatrix() const 
		{
			return glm::translate(Mat4(1), prevPosition) * glm::toMat4(prevOrientation);
		}
				    
		inline Mat3 RigidBody::GetInertiaTensor() const
		{
			return glm::inverse( GetInverseInertiaTensor() );
		}
		
		inline Mat3	RigidBody::GetInverseInertiaTensor() const 
		{
			Mat3 const rot = GetOrientationMatrix();
			return rot * invInertiaLocal * glm::transpose(rot);
		}

		inline Mat3 RigidBody::GetInertiaTensorAbout(Vec3 const& worldPoint) const 
		{
			// Use Parallel Axis Theorem for translated inertia tensor
			Vec3 const disp = worldPoint - position;
			return GetInertiaTensor() + GetMass() * (glm::dot(disp, disp) * Mat3(1) - glm::outerProduct(disp, disp));
		}

		inline Mat3 RigidBody::GetInertiaTensorLocal() const
		{
			return glm::inverse(invInertiaLocal);
		}
		inline Mat3	const& RigidBody::GetInverseInertiaTensorLocal() const
		{
			return invInertiaLocal;
		}

		inline Real RigidBody::GetMass() const 
		{
			return invMass == 0.0_r ? 0.0_r : 1.0_r / invMass;
		}

		inline Real RigidBody::GetInverseMass() const 
		{
			return invMass;
		}
		
		inline Real RigidBody::GetFriction() const 
		{
			return friction;
		}

		inline Real	RigidBody::GetResitution() const
		{
			return restitution;
		}
		
		inline Real RigidBody::GetGravityScale() const 
		{
			return gravityScale;
		}

		inline RigidBody::Type RigidBody::GetType() const 
		{
			return type;
		}

		inline Vec3 RigidBody::WorldToLocalVec(Vec3 const& vec) const
		{
			return glm::inverse(orientation) * vec;
		}

		inline Vec3 RigidBody::WorldToLocalPoint(Vec3 const& pt) const
		{
			return glm::inverse(orientation) * (pt - position);
		}

		inline Vec3 RigidBody::LocalToWorldVec(Vec3 const& vec) const
		{
			return orientation * vec;
		}

		inline Vec3 RigidBody::LocalToWorldPoint(Vec3 const& pt) const
		{
			return orientation * pt + position;
		}

		inline Vec3 RigidBody::LocalToPreviousWorldVec(Vec3 const& vec) const
		{
			return prevOrientation * vec;
		}

		inline Vec3 RigidBody::LocalToPreviousWorldPoint(Vec3 const& pt) const
		{
			return prevOrientation * pt + prevPosition;
		}

		inline std::shared_ptr<CollisionGeometry const> const RigidBody::GetCollisionGeometry() const
		{
			return geometry;
		}

		inline RigidBody& RigidBody::SetPosition(Vec3 const& newPosition)
		{
			position = newPosition;
			prevPosition = newPosition;
			return *this;
		}

		inline RigidBody& RigidBody::SetOrientation(Vec3 const& newOrientationEulerAngles) 
		{
			return SetOrientation(Quat(newOrientationEulerAngles));
			
		}

		inline RigidBody& RigidBody::SetOrientation(Quat const& newOrientation) 
		{
			ASSERT(not EpsilonEqual(newOrientation, Quat{0,0,0,0}), "Bad quaternion");
			orientation = newOrientation;
			prevOrientation = newOrientation;
			return *this;
		}

		inline RigidBody& RigidBody::SetOrientation(Mat3 const& newOrientation)
		{
			return SetOrientation(glm::toQuat(newOrientation));
		}

		inline RigidBody& RigidBody::SetLinearVelocity(Vec3 const& newLinearVelocity)
		{
			linearVelocity = newLinearVelocity;
			return *this;
		}

		inline RigidBody& RigidBody::SetAngularVelocity(Vec3 const& newAngularVelocity)
		{
			angularVelocity = newAngularVelocity;
			return *this;
		}

		inline RigidBody& RigidBody::SetMass(Real mass)
		{
			ASSERT(mass > 0.0_r, "Mass must be nonzero");

			invInertiaLocal *= 1.0_r / (invMass * mass);

			invMass = 1.0_r / mass;

			return *this;
		}

		inline RigidBody& RigidBody::SetGravityScale(Real gravity)
		{
			gravityScale = gravity;
			return *this;
		}

		inline RigidBody& RigidBody::SetFriction(Real f)
		{
			friction = f;
			return *this;
		}

		inline RigidBody& RigidBody::SetRestitution(Real r)
		{
			ASSERT(r >= 0.0_r && r <= 1.0_r, "Restitution Coefficient should be in range [0, 1]");
			restitution = r;
			return *this;
		}


		inline RigidBody& RigidBody::SetType(RigidBody::Type type_)
		{
			type = type_;
			return *this;
		}

		inline RigidBody& RigidBody::SetCollisionGeometry(std::shared_ptr<CollisionGeometry const> geomPtr)
		{
			geometry = std::move(geomPtr);
			invMass = geometry->InverseMass();
			invInertiaLocal = geometry->InverseInertia();
			return *this;
		}
		
		inline RigidBody& RigidBody::SetCollisionGeometry(CollisionGeometry const* geomPtr)
		{
			auto sharedPtr = std::shared_ptr<CollisionGeometry const>(geomPtr);
			return SetCollisionGeometry(sharedPtr);

		}
	}
}