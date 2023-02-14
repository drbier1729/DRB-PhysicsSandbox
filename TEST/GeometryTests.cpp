#include "pch.h"
#include "CppUnitTest.h"
#include "PhysicsSandbox/PhysicsGeometry.h"
#include "PhysicsSandbox/PhysicsGeometryQueries.h"

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
	TEST_CLASS(PolygonUnitTest)
	{
	public:

		TEST_METHOD(TestPolygonClipping)
		{
			using drb::physics::Polygon;
			using drb::physics::Plane;
			using drb::Vec3;

			// arrange
			Polygon square{ .verts = {
					Vec3(0),
					Vec3(1,0,0),
					Vec3(1,0,1),
					Vec3(0,0,1)
				}
			};
			Plane p0 = drb::physics::MakePlane(Vec3(1, 0, 0), Vec3(0.5f, 0, 0));
			Plane p1 = drb::physics::MakePlane(drb::Normalize(Vec3(1, 0, 1)), Vec3(1,0,0));

			
			{
				// act
				Polygon front{}, back{};
				drb::physics::SplitPolygon(square, p0, front, back);

				// assert
				Assert::IsTrue(front.verts.size() == 4 && back.verts.size() == 4, L"Square should have split in two rectangles");
			}
			
			{
				// act
				Polygon front{}, back{};
				drb::physics::SplitPolygon(square, p1, front, back);

				// assert
				Assert::IsTrue(front.verts.size() == 3 && back.verts.size() == 3, L"Square should have split in two triangles");
			}
		}
		
	};
}
