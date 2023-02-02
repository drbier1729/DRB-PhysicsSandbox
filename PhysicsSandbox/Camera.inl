
#include "Math.h"

namespace drb {

	inline Vec3 const& Camera::GetPosition() const
	{
		return pos;
	}
	
	inline Vec3 Camera::GetForward() const
	{
		return fwd;
	}
	
	inline Vec3 Camera::GetUp() const
	{
		return up;
	}
	
	inline Vec3 Camera::GetRight() const
	{
		return glm::cross(fwd, up);
	}

	inline Mat4 Camera::ViewMatrix() const
	{
		return glm::lookAt(pos, pos + fwd, up);
	}

	inline Mat4 Camera::ProjMatrix() const
	{
		return glm::perspective(fov, aspect, frontClip, backClip);
	}


	inline Camera& Camera::SetUpDirection(Vec3 const& up_)
	{
		up = Normalize(up_);
		return *this;
	}
	
	inline Camera& Camera::SetForwardDirection(Vec3 const& fwd_)
	{
		fwd = Normalize(fwd_);
		return *this;
	}

	inline Camera& Camera::SetPosition(Vec3 const& position)
	{
		pos = position;
		return *this;
	}


	inline Camera& Camera::SetPosition(Float32 x, Float32 y, Float32 z)
	{
		pos = Vec3(x, y, z);
		return *this;
	}

	inline Camera& Camera::LookAt(Vec3 const& target)
	{
		fwd = Normalize(target - pos);
		return *this;
	}

	inline Camera& Camera::SetClippingPlanes(Float32 front, Float32 back)
	{
		frontClip = front; 
		backClip = back;

		return *this;
	}

	inline Camera& Camera::SetFOV(Float32 fovDegrees)
	{
		fov = glm::radians(fovDegrees);
		return *this;
	}

	inline Camera& Camera::SetViewportDimensions(Float32 width, Float32 height)
	{
		aspect = width / height;
		return *this;
	}
}