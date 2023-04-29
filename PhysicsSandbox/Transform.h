#ifndef DRB_TRANSFORM_H
#define DRB_TRANSFORM_H

namespace  drb {
	
	struct Transform
	{
		Vec3 translation;
		Quat orientation;

		inline Vec3 operator*(Vec3 const& point) const {
			return orientation * point + translation;
		}
		inline Mat4 operator*(Mat4 const& mat) const {
			return glm::translate(Mat4(1), translation) * glm::toMat4(orientation) * mat;
		}
		inline Vec3 Rotate(Vec3 const& vec) const {
			return orientation * vec;
		}
		inline Vec3 Translate(Vec3 const& vec) const {
			return vec + translation;
		}
		inline Vec3 ApplyInverse(Vec3 const& point) const {
			return glm::inverse(orientation) * (point - translation);
		}
		inline Mat4 AsMat4() const {
			return glm::translate(Mat4(1), translation) * glm::toMat4(orientation);
		}		
	};
}

#endif

