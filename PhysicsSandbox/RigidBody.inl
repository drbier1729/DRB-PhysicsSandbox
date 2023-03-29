
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

		inline Vec3 RigidBody::GetPosition() const
		{
			return position;
		}
		
		inline Quat RigidBody::GetOrientation() const
		{
			return orientation;
		}

		inline Mat3 RigidBody::GetOrientationMatrix() const 
		{
			return glm::toMat3(orientation);
		}
		
		inline Vec3 RigidBody::GetLinearVelocity() const 
		{
			return linearVelocity;
		}
		
		inline Vec3 RigidBody::GetAngularVelocity() const 
		{
			return angularVelocity;
		}
		
		inline Mat4 RigidBody::GetTransformMatrix() const 
		{
			return glm::translate(Mat4(1), position) * glm::toMat4(orientation);
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

		inline Float32 RigidBody::GetMass() const 
		{
			return 1.0f / invMass;
		}
		
		inline Float32 RigidBody::GetFriction() const 
		{
			return friction;
		}

		inline Float32	RigidBody::GetResitution() const
		{
			return restitution;
		}
		
		inline Float32 RigidBody::GetGravityScale() const 
		{
			return gravityScale;
		}

		inline RigidBody::Type RigidBody::GetType() const 
		{
			return type;
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
			ASSERT(not EpsilonEqual(newOrientation, Quat(0.0, 0.0, 0.0f, 0.0f)), "Bad quaternion");
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

		inline RigidBody& RigidBody::SetMass(Float32 mass)
		{
			ASSERT(mass > 0.0f, "Mass must be nonzero");

			invInertiaLocal *= 1.0f / (invMass * mass);

			invMass = 1.0f / mass;

			return *this;
		}

		inline RigidBody& RigidBody::SetGravityScale(Float32 gravity)
		{
			gravityScale = gravity;
			return *this;
		}

		inline RigidBody& RigidBody::SetFriction(Float32 f)
		{
			friction = f;
			return *this;
		}

		inline RigidBody& RigidBody::SetRestitution(Float32 r)
		{
			ASSERT(r >= 0.0f && r <= 1.0f, "Restitution Coefficient should be in range [0, 1]");
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
			invInertiaLocal = geometry->InverseInertia();
			invMass = geometry->InverseMass();
			return *this;
		}
		
		inline RigidBody& RigidBody::SetCollisionGeometry(CollisionGeometry const* geomPtr)
		{
			auto sharedPtr = std::shared_ptr<CollisionGeometry const>(geomPtr);
			return SetCollisionGeometry(sharedPtr);

		}
	}
}