#include "pch.h"
#include "Shader.h"

#include <fstream>
#include <vector>
#include <iostream>

namespace drb {

	static GLuint ReadAndCompileShader(const char* name, GLint gl_shader_type_macro)
	{
		std::ifstream file_stream;
		file_stream.open(name, std::ios_base::binary);	// Open
		if (!file_stream) { return 0; }

		file_stream.seekg(0, std::ios_base::end);		// Position at end
		size_t length = file_stream.tellg();			//   to get the length

		std::vector<char> content(length + 1ull);

		file_stream.seekg(0, std::ios_base::beg);		// Position at beginning
		file_stream.read(content.data(), length);		// Read complete file
		file_stream.close();							// Close
		content[length] = char(0);						// Finish with a NULL

		GLuint shader_id = glCreateShader(gl_shader_type_macro);
		char* shader_str = content.data();              // Need to pass a const GLchar** to glShaderSource
		glShaderSource(shader_id, 1, &(shader_str), NULL);
		glCompileShader(shader_id);

		GLint success;
		char info_log[512];

		glGetShaderiv(shader_id, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			glGetShaderInfoLog(shader_id, 512, NULL, info_log);
			std::cerr << info_log << '\n';
			return 0;
		};

		return shader_id;
	}


	Shader::Shader(const char* filename, int gl_shader_type)
		: id{ ReadAndCompileShader(filename, gl_shader_type) }
	{}

	Shader::Shader(Shader&& src) noexcept 
		: id{src.id}
	{
		src.id = 0;
	}
	
	Shader& Shader::operator=(Shader && rhs) noexcept
	{
		if (this == &rhs) { return *this; }
		glDeleteShader(id);
		id = rhs.id;
		rhs.id = 0;
		return *this;
	}

	Shader::~Shader() noexcept
	{
		glDeleteShader(id);
	}

	ShaderProgram::ShaderProgram(std::initializer_list<unsigned> shaders)
		: id{ glCreateProgram() }
	{
		for (auto&& s : shaders)
		{
			glAttachShader(id, s);
		}
		glLinkProgram(id);
	}

	ShaderProgram::~ShaderProgram() noexcept
	{
		// value of 0 will be silently ignored here
		glDeleteProgram(id);
	}


	ShaderProgram::ShaderProgram(ShaderProgram&& src) noexcept
		: id{src.id}
	{
		src.id = 0;
	}
	
	ShaderProgram& ShaderProgram::operator=(ShaderProgram&& rhs) noexcept
	{
		if (this == &rhs) { return *this; }
		glDeleteProgram(id);
		id = rhs.id;
		rhs.id = 0;
		return *this;
	}

	void ShaderProgram::Use() const noexcept
	{
		glUseProgram(id);
	}

	void ShaderProgram::Unuse() const noexcept
	{
		glUseProgram(0);
	}

	void ShaderProgram::SetUniform(int val, const char* name) const
	{
		int loc = glGetUniformLocation(id, name);
		glUniform1i(loc, val);
	}

	void ShaderProgram::SetUniform(float val, const char* name) const
	{
		int loc = glGetUniformLocation(id, name);
		glUniform1f(loc, val);
	}

	void ShaderProgram::SetUniformVector(const int* vec, unsigned size, const char* name) const
	{
		int loc = glGetUniformLocation(id, name);
		switch (size)
		{
		break;  case 2:
		{
			glUniform2iv(loc, 1, vec);
		}
		break; case 3:
		{
			glUniform3iv(loc, 1, vec);
		}
		break; case 4:
		{
			glUniform4iv(loc, 1, vec);
		}
		break; default:
		{
			// Error
		}
		}
	}

	void ShaderProgram::SetUniformVector(const float* vec, unsigned size, const char* name) const
	{
		int loc = glGetUniformLocation(id, name);
		switch (size)
		{
		break;  case 2:
		{
			glUniform2fv(loc, 1, vec);
		}
		break; case 3:
		{
			glUniform3fv(loc, 1, vec);
		}
		break; case 4:
		{
			glUniform4fv(loc, 1, vec);
		}
		break; default:
		{
			// Error
		}
		}
	}

	void ShaderProgram::SetUniformMatrix(const float* mat, unsigned size, const char* name) const
	{
		int loc = glGetUniformLocation(id, name);
		switch (size)
		{
		break;  case 2:
		{
			glUniformMatrix2fv(loc, 1, GL_FALSE, mat);
		}
		break; case 3:
		{
			glUniformMatrix3fv(loc, 1, GL_FALSE, mat);
		}
		break; case 4:
		{
			glUniformMatrix4fv(loc, 1, GL_FALSE, mat);
		}
		break; default:
		{
			// Error
		}
		}
	}
}