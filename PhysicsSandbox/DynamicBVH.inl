namespace drb::physics
{
	void BVHierarchy::Query(AABB const& box, std::invocable<BV const&> auto callback) const {}
	void BVHierarchy::Query(Sphere const& sph, std::invocable<BV const&> auto callback) const {}
	void BVHierarchy::Query(Ray const& ray, std::invocable<BV const&> auto callback) const {}
}