#pragma once

#include "Collision.h"
#include "RigidBody.h"

struct CollisionArbiter
{
	// Data
	ColliderPair pair;
	Collision::ContactManifold manifold;

	Float32 friction, restitution;

	// Methods
	explicit CollisionArbiter(ColliderPair const& pair);
	void Update(Collision::ContactManifold const& new_manifold);
	void PreStep(Float32 time_step);
	void ApplyImpulse();
};