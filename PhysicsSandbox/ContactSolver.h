#ifndef DRB_CONTACTSOLVER_H
#define DRB_CONTACTSOLVER_H



namespace drb::physics {

	void SolveManifoldPositions(struct ContactManifold& manifold, Real invDT);
	void SolveManifoldVelocities(struct ContactManifold& manifold, Real dt);
	void UpdateManifold(struct ContactManifold& manifold, struct ContactManifold const& newManifold);

}

#endif

