
#include "Math.h"

namespace drb {

	inline glm::vec3 const& Camera::GetPosition() const
	{
		return pos;
	}
	
	inline glm::vec3 Camera::GetForward() const
	{
		return fwd;
	}
	
	inline glm::vec3 Camera::GetUp() const
	{
		return up;
	}
	
	inline glm::vec3 Camera::GetRight() const
	{
		return glm::cross(fwd, up);
	}

	inline glm::mat4 Camera::ViewMatrix() const
	{
		return glm::lookAt(pos, pos + fwd, up);
	}

	inline glm::mat4 Camera::ProjMatrix() const
	{
		return glm::perspective(fov, aspect, frontClip, backClip);
	}


	inline Camera& Camera::SetUpDirection(glm::vec3 const& up_)
	{
		up = Normalize(up_);
		return *this;
	}
	
	inline Camera& Camera::SetForwardDirection(glm::vec3 const& fwd_)
	{
		fwd = Normalize(fwd_);
		return *this;
	}

	inline Camera& Camera::SetPosition(glm::vec3 const& position)
	{
		pos = position;
		return *this;
	}


	inline Camera& Camera::SetPosition(Float32 x, Float32 y, Float32 z)
	{
		pos = glm::vec3(x, y, z);
		return *this;
	}

	inline Camera& Camera::LookAt(glm::vec3 const& target)
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