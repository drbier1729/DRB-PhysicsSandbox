
namespace drb {
	namespace physics {
		inline void RigidBodyState::CopyTo(RigidBody& rb) const
		{
			rb.position           = position;
			rb.prevPosition       = prevPosition;
			rb.linearVelocity     = linearVelocity;
			rb.orientation        = orientation;
			rb.prevOrientation    = prevOrientation;
			rb.angularVelocity    = angularVelocity;
			rb.accumulatedForces  = accumulatedForces;
			rb.accumulatedTorques = accumulatedTorques;
		}

		inline void RigidBodyState::CopyFrom(RigidBody const& rb)
		{
			position           = rb.position;
			prevPosition       = rb.prevPosition;
			linearVelocity     = rb.linearVelocity;
			orientation        = rb.orientation;
			prevOrientation    = rb.prevOrientation;
			angularVelocity    = rb.angularVelocity;
			accumulatedForces  = rb.accumulatedForces;
			accumulatedTorques = rb.accumulatedTorques;
		}

		inline void WorldStateRecorder::Init(World& w, Uint32 n)
		{
			ASSERT(n > 0, "Must record at least 1 frame.");
			world = &w;
			Resize(n);
		}

		inline void WorldStateRecorder::Resize(Uint32 numFrames)
		{
			rbStates.resize(numFrames, std::vector<RigidBodyState>(world->bodies.size()));
		}
	}
}