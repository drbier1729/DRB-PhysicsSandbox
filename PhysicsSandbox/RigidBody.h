#ifndef DRB_RIGIDBODY_H
#define DRB_RIGIDBODY_H

#include "PhysicsGeometry.h"
#include "Math.h"

namespace drb {
	namespace physics {

		// Class for Dynamic and Kinematic physics objects. Static physics objects can just use
		// the Collidable class in "PhysicsGeometry.h"
		class alignas(16) RigidBody
		{
		public:
			friend class World;

			// Debug only
			friend class DebugRenderer;
			friend struct RigidBodyState;

		public:
			enum class Type : Uint32
			{
				Dynamic,
				Kinematic
			};

		private:
			Type    type = Type::Dynamic;
			
			Vec3    position = {}; // world-space coords of center of mass
			Vec3    prevPosition = {};
			Vec3    linearVelocity = {};
			Float32 invMass = 1.0f;
			Float32 gravityScale = 1.0f;

			Quat    orientation = { 1, 0 ,0, 0 };

			// -- 64 bytes to here --

			Quat prevOrientation = { 1, 0, 0, 0 };
			Vec3 angularVelocity = {};
			Mat3 invInertiaLocal = Mat4(1);

			// -- 128 bytes to here --

			Vec3 accumulatedForces = {}; 
			Vec3 accumulatedTorques = {};

			Float32 friction = 0.0f;
			Float32 restitution = 0.0f;

			Float32 linearDamping = 0.0f;
			Float32 angularDamping = 0.0f;

			std::vector<CollisionShape<Sphere>>  spheres{};

			// -- 192 bytes to here --

			std::vector<CollisionShape<Capsule>> capsules{};
			std::vector<CollisionShape<Convex>>  hulls{};

			// Can be used for ptr to owner game object, and/or ptr
			// to existing contact manifolds
			char pad_2[16];

			// -- 256 bytes total --

		public:
			// -----------------------------------------------------------------
			// Simulation Methods
			// -----------------------------------------------------------------

			inline RigidBody& AddForce(Vec3 const& worldSpaceForce);
			inline RigidBody& AddForce(Vec3 const& worldSpaceForce, Vec3 const& worldSpacePoint);
			inline RigidBody& AddRelativeForce(Vec3 const& localSpaceForce);
			inline RigidBody& AddRelativeForce(Vec3 const& localSpaceForce, Vec3 const& localSpacePoint);

			// -----------------------------------------------------------------
			// Accessors
			// -----------------------------------------------------------------

			inline Vec3		       GetPosition() const;
			inline Quat		       GetOrientation() const;
			inline Mat3		       GetOrientationMatrix() const;
			inline Vec3		       GetLinearVelocity() const;
			inline Vec3		       GetAngularVelocity() const;
			inline Mat4		       GetTransformMatrix() const;
			inline Mat3			   GetInertiaTensor() const;		// world-space
			inline Mat3			   GetInverseInertiaTensor() const; // world-space
			inline Mat3			   GetInertiaTensorAbout(Vec3 const& worldPoint) const;
			inline Float32		   GetMass() const;
			inline Float32		   GetGravityScale() const;
			inline Float32		   GetFriction() const;
			inline Float32		   GetResitution() const;
			inline RigidBody::Type GetType() const;

			// -----------------------------------------------------------------
			// Setters
			// -----------------------------------------------------------------

			// these methods may cause craziness if called while the simulation
			// is running because they invalidate cached contacts and the user
			// could potentially move body into another body resulting in a
			// dramatic resolution of the interpenetration on the next frame.
			// Prefer to call these during scene setup, use Set___Velocity
			// methods to control Kinematic bodies, and AddForce methods
			// to control Dynamic bodies.
			inline RigidBody& SetPosition(Vec3 const& newPosition);
			inline RigidBody& SetOrientation(Vec3 const& newOrientationEulerAngles);
			inline RigidBody& SetOrientation(Quat const& newOrientation);
			inline RigidBody& SetOrientation(Mat3 const& newOrientation);

			// these will only have an effect on Kinematic rigidbodies b/c
			// velocity is computed from position for Dynamic rigidbodies
			inline RigidBody& SetLinearVelocity(Vec3 const& newLinearVelocity);
			inline RigidBody& SetAngularVelocity(Vec3 const& newAngularVelocity);

			// also updates local inertia tensor. maintains relative mass ratios
			// between CollisionShapes, but scales them. fairly expensive.
				   RigidBody& SetMass(Float32 mass);

			inline RigidBody& SetGravityScale(Float32 gravity);
			inline RigidBody& SetFriction(Float32 friction);
			inline RigidBody& SetRestitution(Float32 restitution);

			inline RigidBody& SetType(RigidBody::Type type);


			// -----------------------------------------------------------------
			// Setup Methods -- Temporary
			// -----------------------------------------------------------------
			
			// must be called by user after all colliders have been added. computes 
			// the mass, center of mass, and local inertia tensor. This will also
			// set the local position of each collider such that the center of 
			// mass of the body is at the origin, and will return the offset
			// used for this process (e.g. the coords of the COM in the original
			// model's space).
			Vec3              FinalizeCollisionGeometry();

			// It would be ill-advised to call this while the simulation is
			// running, and must be followed by a call to
			// FinalizeCollisionGeometry once all colliders have been added.
			template<Shape T> 
			inline RigidBody& AddCollider(CollisionShape<T> const& shape);

			
		private:
			// -----------------------------------------------------------------
			// Helpers
			// -----------------------------------------------------------------
			
			// Called within physics update loop
			void ProjectPositions(Float32 h);
			void ProjectVelocities(Float32 h);
		};
		
		// In release build on MSVC, std::vector is 24 bytes. In debug, it is 32.
		#ifndef _DEBUG
		static_assert(sizeof(RigidBody) == 256);
		#endif
	}
}

#include "RigidBody.inl"

#endif