#ifndef DRB_SHADER_H
#define DRB_SHADER_H

#include <glm/gtc/type_ptr.hpp>

namespace drb {

	class Shader
	{
	public:
		Shader() = default;
		explicit Shader(const char* filename, int gl_shader_type);
		
		Shader(Shader&& src) noexcept;
		Shader& operator=(Shader&& rhs) noexcept;

		// non-copyable
		Shader(Shader const& src) = delete;
		Shader& operator=(Shader const& rhs) = delete;
		
		~Shader() noexcept;

		inline operator unsigned() const { return id; }

	private:
		unsigned id;
	};


	class ShaderProgram
	{
	public:
		ShaderProgram() = default;
		explicit ShaderProgram(std::initializer_list<unsigned> shaders);
		
		// Non copyable
		ShaderProgram& operator=(ShaderProgram const& rhs) = delete;
		ShaderProgram(ShaderProgram const& src) = delete;
		
		ShaderProgram(ShaderProgram&& src) noexcept;
		ShaderProgram& operator=(ShaderProgram&& rhs) noexcept;
		
		~ShaderProgram() noexcept;

		void Use() const noexcept;
		void Unuse() const noexcept;

		void SetUniform(int val, const char* name) const;
		void SetUniform(float val, const char* name) const;
		void SetUniformVector(const int* vec, unsigned size, const char* name) const;
		void SetUniformVector(const float* vec, unsigned size, const char* name) const;
		void SetUniformMatrix(const float* mat, unsigned size, const char* name) const;

		inline operator unsigned() const { return id; }

		struct ScopedUse
		{
			explicit ScopedUse(ShaderProgram const& sp) : prg{ sp } { prg.Use(); }
			~ScopedUse() noexcept { prg.Unuse(); }

			inline void SetUniform(int val, const char* name) const { prg.SetUniform(val, name); }
			inline void SetUniform(float val, const char* name) const { prg.SetUniform(val, name); }
			inline void SetUniformVector(const int* vec, unsigned size, const char* name) const { prg.SetUniformVector(vec, size, name); }
			inline void SetUniformVector(const float* vec, unsigned size, const char* name) const { prg.SetUniformVector(vec, size, name); }
			inline void SetUniformMatrix(const float* mat, unsigned size, const char* name) const { prg.SetUniformMatrix(mat, size, name); }
			inline operator unsigned() const { return prg.id; }

			ShaderProgram const& prg;
		};

	private:
		unsigned id;
	};

}
#endif

