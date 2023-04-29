#ifndef DRB_CAMERA_H
#define DRB_CAMERA_H

namespace drb {
	class Camera
	{
	private:
		glm::vec3 pos = {}, 
			 fwd = glm::vec3(0, 0, -1),
			 up = glm::vec3(0, 1, 0);

		Float32 fov = glm::radians(50.0f), 
				frontClip = 0.5f, 
				backClip = 2000.0f, 
				aspect = 4.0f/3.0f;

	public:
		inline glm::mat4 ViewMatrix() const;
		inline glm::mat4 ProjMatrix() const;

		inline glm::vec3 const& GetPosition() const;
		inline glm::vec3 GetForward() const;
		inline glm::vec3 GetUp() const;
		inline glm::vec3 GetRight() const;

		inline Camera& SetUpDirection(glm::vec3 const& up);
		inline Camera& SetForwardDirection(glm::vec3 const& fwd);
		inline Camera& SetPosition(glm::vec3 const& position);
		inline Camera& SetPosition(Float32 x, Float32 y, Float32 z);
		inline Camera& SetClippingPlanes(Float32 front, Float32 back);
		inline Camera& SetViewportDimensions(Float32 width, Float32 height);
		inline Camera& SetFOV(Float32 fovDegrees);
		inline Camera& LookAt(glm::vec3 const& target);
	};
}

#include "Camera.inl"

#endif

