#include "pch.h"
#include "CppUnitTest.h"
#include "PhysicsSandbox/RigidBody.h"

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

template<>
std::wstring Microsoft::VisualStudio::CppUnitTestFramework::ToString<drb::physics::RigidBody::Type>(const drb::physics::RigidBody::Type& t)
{
	if (t == drb::physics::RigidBody::Type::Dynamic) {
		return std::wstring(L"Dynamic");
	}
	else {
		return std::wstring(L"Kinematic");
	}
}

namespace TEST
{
	TEST_CLASS(RigidBodyUnitTest)
	{
	public:
		
		TEST_METHOD(TestGettersAndSetters)
		{
			drb::physics::RigidBody r{};

			drb::Mat3 testMat3(glm::rotate(drb::Mat4(1), 3.14f, drb::Vec3(1, 0, 0)));
			drb::Quat testQuat(glm::toQuat(testMat3));
			drb::Vec3 testVec3 = glm::eulerAngles(testQuat);
			drb::Float32 testFloat = 0.5f;

			r.SetLinearVelocity(testVec3);
			Assert::AreEqual(testVec3, r.GetLinearVelocity(), L"Linear Velocities not equal");

			r.SetAngularVelocity(testVec3);
			Assert::AreEqual(testVec3, r.GetAngularVelocity(), L"Angular Velocities not equal");

			r.SetFriction(testFloat);
			Assert::AreEqual(testFloat, r.GetFriction(), L"Frictions not equal");

			r.SetGravityScale(testFloat);
			Assert::AreEqual(testFloat, r.GetGravityScale(), L"Gravity Scales not equal");
			
			r.SetPosition(testVec3);
			Assert::AreEqual(testVec3, r.GetPosition(), L"Positions not equal");

			r.SetOrientation(testVec3);
			Assert::AreEqual(testQuat, r.GetOrientation(), L"Orientations not equal");

			r.SetOrientation(drb::Vec3(0));
			r.SetOrientation(testQuat);
			Assert::AreEqual(testQuat, r.GetOrientation(), L"Orientations not equal");

			r.SetOrientation(drb::Vec3(0));
			r.SetOrientation(testMat3);
			Assert::AreEqual(testQuat, r.GetOrientation(), L"Orientations not equal");

			r.SetRestitution(testFloat);
			Assert::AreEqual(testFloat, r.GetResitution(), L"Restitutions not equal");

			r.SetType(drb::physics::RigidBody::Type::Kinematic);
			Assert::AreEqual(drb::physics::RigidBody::Type::Kinematic, r.GetType(), L"Types not equal");

			r.SetMass(testFloat);
			Assert::AreEqual(1.0f / (1.0f / testFloat), r.GetMass(), L"Masses not equal");
			// test inertia tensor....?
		}
	
		TEST_METHOD(TestForceMethods) 
		{
			// arrange

			// act

			// assert
		}
	};
}
