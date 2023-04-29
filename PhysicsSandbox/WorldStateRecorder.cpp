#include "pch.h"
#include "WorldStateRecorder.h"
#include "DRBAssert.h"

namespace drb {
	namespace physics {

		void WorldStateRecorder::Record()
		{
			auto & bodies = world->bodies;
			Int32 const numBodies = static_cast<Int32>(bodies.size());

			// If we're starting from the middle, we need to advance
			// to where we last stopped recording
			if (current != top) 
			{
				for (Int32 i = 0; i < numBodies; ++i) {
					rbStates[top][i].CopyTo(bodies[i]);
				}
			}

			// Record the current world state
			Int32 const maxFrames = static_cast<Int32>(rbStates.size());
			top = (top + 1) % maxFrames;
			current = top;
			for (Int32 i = 0; i < numBodies; ++i) {
				rbStates[top][i].CopyFrom(bodies[i]);
			}
			numFrames = glm::min(numFrames + 1, maxFrames);
		}

		Uint32 WorldStateRecorder::AdvanceSteps(Int32 numSteps)
		{
			auto Diff = [this] { return top >= current ? top - current : numFrames - (current - top); };

			if (numSteps == 0)                  { return Diff(); }

			// Check if we're at the top and still trying to step forward
			if (current == top && numSteps > 0) { return 0; }

			// Check if we're at the bottom and still trying to step back
			if (current == (top + 1) % numFrames && numSteps < 0) { 
				return numFrames - 1; }

			Int32 next = (current + numSteps) % numFrames;
			if (next < 0) {	next += numFrames; }

			current = next;

			auto& bodies = world->bodies;
			Int32 const numBodies = static_cast<Int32>(bodies.size());

			for (Int32 i = 0; i < numBodies; ++i) {
				rbStates[current][i].CopyTo(bodies[i]);
			}

			return Diff();
		}
	}
}