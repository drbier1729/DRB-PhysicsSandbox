#include "pch.h"
#include "WorldStateRecorder.h"

namespace drb {
	namespace physics {

		void WorldStateRecorder::Record()
		{
			auto & bodies = world->bodies;
			Uint32 const numBodies = static_cast<Uint32>(bodies.size());

			// If we're starting from the middle, we need to advance the sim
			// to where it was when we last stopped recording
			if (current != top) 
			{
				for (Uint32 i = 0; i < numBodies; ++i) {
					rbStates[top][i].CopyTo(bodies[i]);
				}
			}

			// Record the current world state
			top = (top + 1) % static_cast<Uint32>(rbStates.size());
			current = top;
			for (Uint32 i = 0; i < numBodies; ++i) {
				rbStates[top][i].CopyFrom(bodies[i]);
			}
		}

		Uint32 WorldStateRecorder::AdvanceSteps(Int32 numSteps)
		{
			Int32 const numFrames = static_cast<Uint32>(rbStates.size());

			Int32 next = (static_cast<Int32>(current) + numSteps) % numFrames;
			if (next < 0)
			{
				next += numFrames;
			}
			current = next;

			auto& bodies = world->bodies;
			Uint32 const numBodies = static_cast<Uint32>(bodies.size());

			for (Uint32 i = 0; i < numBodies; ++i) {
				rbStates[current][i].CopyTo(bodies[i]);
			}

			return top >= current ? top - current : numFrames - (current - top);
		}

		//Bool WorldStateRecorder::ForwardOneStep()
		//{
		//	if (current == top) { return false; }

		//	current = (current + 1) % static_cast<Uint32>(rbStates.size());


		//	auto& bodies = world->bodies;
		//	Uint32 const numBodies = static_cast<Uint32>(bodies.size());

		//	for (Uint32 i = 0; i < numBodies; ++i) {
		//		rbStates[current][i].CopyTo(bodies[i]);
		//	}

		//	return true;
		//}

		//Bool WorldStateRecorder::RewindOneStep()
		//{
		//	

		//	auto& bodies = world->bodies;
		//	Uint32 const numBodies = static_cast<Uint32>(bodies.size());

		//	for (Uint32 i = 0; i < numBodies; ++i) {
		//		rbStates[current][i].CopyTo(bodies[i]);
		//	}

		//	return true;
		//}
	}
}