#ifndef DRB_CAMERA_H
#define DRB_CAMERA_H

namespace drb {
	class Camera
	{
	private:
		Vec3 pos = {}, 
			 fwd = Vec3(0, 0, -1),
			 up = Vec3(0, 1, 0);

		Float32 fov = glm::radians(50.0f), 
				frontClip = 0.5f, 
				backClip = 2000.0f, 
				aspect = 4.0f/3.0f;

	public:
		inline Mat4 ViewMatrix() const;
		inline Mat4 ProjMatrix() const;

		inline Vec3 const& GetPosition() const;
		inline Vec3 GetForward() const;
		inline Vec3 GetUp() const;
		inline Vec3 GetRight() const;

		inline Camera& SetUpDirection(Vec3 const& up);
		inline Camera& SetForwardDirection(Vec3 const& fwd);
		inline Camera& SetPosition(Vec3 const& position);
		inline Camera& SetPosition(Float32 x, Float32 y, Float32 z);
		inline Camera& SetClippingPlanes(Float32 front, Float32 back);
		inline Camera& SetViewportDimensions(Float32 width, Float32 height);
		inline Camera& SetFOV(Float32 fovDegrees);
		inline Camera& LookAt(Vec3 const& target);
	};
}

#include "Camera.inl"

#endif

