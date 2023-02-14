
namespace drb {
	// Inline definitions
	inline bool Mesh::IsValid() const noexcept
	{
		return (m_vao != 0);
	}

	inline Mesh const& Mesh::Cylinder()
	{
		return *CylinderPtr();
	}

	inline Mesh const& Mesh::Sphere()
	{
		return *SpherePtr();
	}

	inline Mesh const& Mesh::Cube()
	{
		return *CubePtr();
	}

	inline Mesh const& Mesh::Cone()
	{
		return *ConePtr();
	}
	
	inline Mesh const& Mesh::Quad()
	{
		return *QuadPtr();
	}
}