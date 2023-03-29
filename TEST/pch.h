// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef PCH_H
#define PCH_H

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#define NOMINMAX

// External Includes: GLEW
#include <gl/glew.h>

// External Includes: GLFW
#include <glfw/glfw3.h>

// External Includes: GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>

// External Includes: Assimp
#include <assimp/Importer.hpp>
#include <assimp/Scene.h>
#include <assimp/Postprocess.h>

// STL includes
#include <cstdint>
#include <cassert>
#include <cstdarg>
#include <iostream>
#include <fstream>
#include <thread>
#include <span>
#include <memory>
#include <initializer_list>
#include <functional>
#include <vector>
#include <array>
#include <set>
#include <string>
#include <variant>
#include <chrono>
#include <type_traits>

#include "PhysicsSandbox/TypeDefs.h"

#endif //PCH_H
