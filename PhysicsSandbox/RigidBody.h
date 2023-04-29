#ifndef DRB_RIGIDBODY_H
#define DRB_RIGIDBODY_H

#include "AABB.h"
#include "CollisionGeometry.h"
#include "Math.h"

namespace drb::physics {

	class RigidBody
	{
	public:
		friend class World;
		friend class PositionalConstraint;
		friend class RotationalConstraint;
		friend class CollisionConstraint;

		// Debug only
		friend class DebugRenderer;
		friend struct RigidBodyState;

	public:
		enum class Type
		{
			Dynamic,
			Kinematic,
			Static
		};

		static constexpr Real gravityMag = -9.8_r;
		static constexpr Vec3 gravity{ 0.0_r, gravityMag, 0.0_r };

	private:
		Type type = Type::Dynamic;

		// Transform data
		Vec3 position{ 0.0_r }; // world-space coords of center of mass
		Quat orientation{ 1, 0 ,0, 0 };
		
		// Mass properties
		Real invMass{ 0.0_r };
		Mat3 invInertiaLocal{ 0.0_r }; // if we rotate geometry we can make this a Vec3
		
		// Velocity data
		Vec3 linearVelocity{ 0.0_r };
		Vec3 angularVelocity{ 0.0_r };
		Real friction{ 1.0_r };
		Real restitution{ 0.0_r };

		// Previous frame cache
		Vec3 prevPosition{ 0.0_r };
		Quat prevOrientation{ 1, 0, 0, 0 };
		Vec3 prevLinearVelocity{ 0.0_r };
		Vec3 prevAngularVelocity{ 0.0_r };
		
		// Force data
		Real gravityScale{ 0.0_r };
		Vec3 accumulatedForces{ 0.0_r };
		Vec3 accumulatedTorques{ 0.0_r };

		// Geometry data
		std::shared_ptr<CollisionGeometry const> geometry{ nullptr };

	public:
		// -----------------------------------------------------------------
		// Simulation Methods
		// -----------------------------------------------------------------

		inline RigidBody& AddForce(Vec3 const& worldSpaceForce);
		inline RigidBody& AddForce(Vec3 const& worldSpaceForce, Vec3 const& worldSpacePoint);
		inline RigidBody& AddRelativeForce(Vec3 const& localSpaceForce);
		inline RigidBody& AddRelativeForce(Vec3 const& localSpaceForce, Vec3 const& localSpacePoint);

		inline RigidBody& SetLinearVelocity(Vec3 const& newLinearVelocity);
		inline RigidBody& SetAngularVelocity(Vec3 const& newAngularVelocity);

		inline RigidBody& MoveTo(Vec3 const& target);
		inline RigidBody& RotateTo(Quat const& target);
		inline RigidBody& RotateTo(Mat3 const& target);
		inline RigidBody& RotateTo(Vec3 const& targetEulerAngles);

		// -----------------------------------------------------------------
		// Accessors
		// -----------------------------------------------------------------

		inline Vec3	const&     GetPosition() const;
		inline Quat	const&     GetOrientation() const;
		inline Mat3		       GetOrientationMatrix() const;
		inline Vec3	const&     GetPreviousPosition() const;
		inline Quat	const&     GetPreviousOrientation() const;
		inline Mat3		       GetPreviousOrientationMatrix() const;
		inline Vec3	const&     GetLinearVelocity() const;
		inline Vec3	const&     GetAngularVelocity() const;
		inline Vec3	const&	   GetPreviousLinearVelocity() const;
		inline Vec3	const&	   GetPreviousAngularVelocity() const;
		inline Mat4		       GetTransformMatrix() const;
		inline Mat4		       GetPreviousTransformMatrix() const;
		inline Mat3		       GetInertiaTensor() const;		// world-space
		inline Mat3		       GetInverseInertiaTensor() const; // world-space
		inline Mat3		       GetInertiaTensorAbout(Vec3 const& worldPoint) const;
		inline Mat3		       GetInertiaTensorLocal() const;		
		inline Mat3	const&     GetInverseInertiaTensorLocal() const;
		inline Real		       GetMass() const;
		inline Real   	       GetInverseMass() const;
		inline Real		       GetGravityScale() const;
		inline Real		       GetFriction() const;
		inline Real		       GetResitution() const;
		inline RigidBody::Type GetType() const;

		inline Vec3            WorldToLocalVec(Vec3 const& vec) const;
		inline Vec3            WorldToLocalPoint(Vec3 const& pt) const;
		inline Vec3            LocalToWorldVec(Vec3 const& vec) const;
		inline Vec3            LocalToWorldPoint(Vec3 const& pt) const;
		inline Vec3            LocalToPreviousWorldVec(Vec3 const& vec) const;
		inline Vec3            LocalToPreviousWorldPoint(Vec3 const& pt) const;

		inline std::shared_ptr<CollisionGeometry const> const GetCollisionGeometry() const;

		// -----------------------------------------------------------------
		// Manipulators
		// -----------------------------------------------------------------
			
		// these methods may cause craziness if called while the simulation
		// is running because they invalidate cached contacts and the user
		// could potentially move body into another body resulting in a
		// dramatic resolution of the interpenetration on the next frame.
		// Prefer to call these during scene setup. Instead, use SetVelocity 
		// and AddForce methods to control Dynamic bodies, and Move method
		// to control Kinematic bodies.
		inline RigidBody& SetPosition(Vec3 const& newPosition);
		inline RigidBody& SetOrientation(Vec3 const& newOrientationEulerAngles);
		inline RigidBody& SetOrientation(Quat const& newOrientation);
		inline RigidBody& SetOrientation(Mat3 const& newOrientation);
		
		// also updates local inertia tensor
		inline RigidBody& SetMass(Real mass);

		inline RigidBody& SetGravityScale(Real gravity);
		inline RigidBody& SetFriction(Real friction);
		inline RigidBody& SetRestitution(Real restitution);

		inline RigidBody& SetType(RigidBody::Type type);

		inline RigidBody& SetCollisionGeometry(std::shared_ptr<CollisionGeometry const> geomPtr);
		inline RigidBody& SetCollisionGeometry(CollisionGeometry const* geomPtr);

			
	private:
		// -----------------------------------------------------------------
		// Helpers
		// -----------------------------------------------------------------
			
		// Called within physics update loop by World
		void ProjectPositions(Real h);
		void ProjectVelocities(Real h);
	};
}

#include "RigidBody.inl"

#endif