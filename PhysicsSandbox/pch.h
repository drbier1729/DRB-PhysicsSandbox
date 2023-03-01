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

// External Includes: ImGUI
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

// STL includes
#include <cstdint>
#include <cassert>
#include <cstdarg>
#include <iostream>
#include <fstream>
#include <thread>
#include <concepts>
#include <compare>
#include <span>
#include <memory>
#include <initializer_list>
#include <functional>
#include <vector>
#include <array>
#include <set>
#include <map>
#include <queue>
#include <unordered_map>
#include <string>
#include <variant>
#include <chrono>
#include <type_traits>

#include "TypeDefs.h"

#endif //PCH_H
