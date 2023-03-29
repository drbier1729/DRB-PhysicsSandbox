#ifndef DRB_RIGIDBODYSTATE_H
#define DRB_RIGIDBODYSTATE_H

#include "RigidBody.h"
#include "PhysicsWorld.h"

namespace drb {
	namespace physics {

		struct RigidBodyState
		{
			Vec3 position = {};
			Vec3 prevPosition = {};
			Vec3 linearVelocity = {};

			Quat orientation = { 1, 0 ,0, 0 };

			Quat prevOrientation = { 1, 0, 0, 0 };
			Vec3 angularVelocity = {};

			Vec3 accumulatedForces = {};
			Vec3 accumulatedTorques = {};

			inline void CopyTo(RigidBody& rb) const;
			inline void CopyFrom(RigidBody const& rb);
		};


		// Note that this assumes the number of rigidbodies will remain constant
		// for the entirety of the recording
		class WorldStateRecorder
		{
		private:
			template<class T>
			using CircularBuffer = std::vector<std::vector<T>>;
			

			CircularBuffer<RigidBodyState> rbStates{};
			Uint32 top = 0, current = 0;

			World* world = nullptr;

		public:
			// world must be fully built already with all rigidbodies
			inline void Init(World& world, Uint32 numFrames);

			// Sets the size of rbStates, i.e. the number of frames that will
			// be saved
			inline void Resize(Uint32 maxFrames);

			// Copies the current state of world into rbStates buffer.
			// We may add additional buffers as needed (e.g. contact manifolds)
			void Record();

			// Returns num steps before top we are
			Uint32 AdvanceSteps(Int32 numSteps);
		};
	}
}

#include "WorldStateRecorder.inl"
#endif

