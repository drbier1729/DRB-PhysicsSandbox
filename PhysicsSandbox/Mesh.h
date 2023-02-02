#ifndef DRB_MESH_H
#define DRB_MESH_H

#include <span>		  // span
#include <functional> // reference_wrapper

namespace drb {

	// Allows for creation of a renderable mesh from a .fbx file or 
	// vertex and index data. Allocates VAO through calls to OpenGL
	// and frees on destruction. Move only.
	class Mesh
	{
	public:
		friend Mesh BuildMeshFromFile(const char* filename) noexcept;

		using Ref = std::reference_wrapper<Mesh>;
		using CRef = std::reference_wrapper<const Mesh>;


	public:
		Mesh() = default;

		// Vertices must look like: pX, pY, pZ, nX, nY, nZ, U, V
		Mesh(float const* verts, size_t verts_sz, unsigned const* idxs, size_t idxs_sz);
		Mesh(std::span<float> verts, std::span<unsigned> idxs);
		
		// non-copyable
		Mesh(Mesh const&)		     = delete;
		Mesh& operator=(Mesh const&) = delete;
		
		Mesh(Mesh&& src) noexcept;
		Mesh& operator=(Mesh&& src) noexcept;

		~Mesh() noexcept;


		void Use() const noexcept;
		void Unuse() const noexcept;
		void Draw() const noexcept;
		inline bool IsValid() const noexcept;


		struct ScopedUse
		{
			explicit ScopedUse(Mesh const& m) : mesh{ m } { mesh.Use(); }
			~ScopedUse() noexcept { mesh.Unuse(); }

			inline void Draw() const noexcept { mesh.Draw(); }
			inline bool IsValid() const noexcept { mesh.IsValid(); }

			Mesh const& mesh;
		};

		static inline Mesh const& Cube();
		static inline Mesh const& Sphere();
		static inline Mesh const& Cylinder();
		static inline Mesh const& Cone();
		static Mesh const* CubePtr();
		static Mesh const* SpherePtr();
		static Mesh const* CylinderPtr();
		static Mesh const* ConePtr();
		static Mesh BuildFromFile(const char* filename);

	private:
		size_t m_verts_sz = 0;
		size_t m_idxs_sz = 0;
		unsigned m_vao = 0;
		unsigned m_vbo = 0;
		unsigned m_ebo = 0;


		static std::unique_ptr<Mesh> sphere, cube, cylinder, cone;
	};


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
}
#endif

