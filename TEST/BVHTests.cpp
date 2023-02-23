#include "pch.h"
#include "CppUnitTest.h"
#include "PhysicsSandbox/DynamicBVH.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

template<>
std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString<drb::Vec3>(const drb::Vec3& v)
{
	return std::wstring(L"Vec(" + ToString(v.x) + L", " + ToString(v.y) + L", " + ToString(v.z) + L")");
}

template<>
std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString<drb::Quat>(const drb::Quat& v)
{
	return std::wstring(L"Quat(" + ToString(v.w) + L", " + ToString(v.x) + L", " + ToString(v.y) + L", " + ToString(v.z) + L")");
}


namespace TEST
{
	TEST_CLASS(BVHUnitTest)
	{
	public:

		TEST_METHOD(InsertTest)
		{
			using drb::physics::BVHierarchy;
			using drb::physics::BV;
			using drb::physics::AABB;
			using drb::Vec3;

			// arrange
			BVHierarchy bvh{};

			// act
			bvh.Insert(BV{
					.bounds = {.max = Vec3(1), .min = Vec3(0)}
				});

			bvh.Insert(BV{
					.bounds = {.max = Vec3(-1), .min = Vec3(-3)}
				});

			bvh.Insert(BV{
					.bounds = {.max = Vec3(10), .min = Vec3(3)}
				});

			// assert
			bvh.Remove(0);
		}
	};
}

