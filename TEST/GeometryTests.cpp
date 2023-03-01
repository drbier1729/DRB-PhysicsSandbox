#include "pch.h"
#include "CppUnitTest.h"
#include "PhysicsSandbox/PhysicsGeometry.h"
#include "PhysicsSandbox/PhysicsGeometryQueries.h"
#include "PhysicsSandbox/SATGJK.h"

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

		TEST_METHOD(GJKPointConvexTest)
		{
			using drb::physics::Convex;
			using drb::Float32;
			using drb::Vec3;
			using drb::Mat4;
			using drb::physics::ClosestPointsQuery;
			
			// arrange
			Convex const hull = drb::physics::MakeBox(Vec3(1));
			ClosestPointsQuery query{};

			{
				query = drb::physics::util::GJK(Vec3(0), hull, Mat4(1));

				// assert
				Assert::IsTrue(query.d2 < 0.0f, L"Intersection not detected");
			}
			
			{
				Vec3 const pt = Vec3(2, 0.5, 0);
				query = drb::physics::util::GJK(pt, hull, Mat4(1));

				// assert
				Float32 const d = glm::sqrt(query.d2);
				Assert::IsTrue( d < 1.01f && d > 0.99f, L"Incorrect distance value");
				Assert::IsTrue(drb::EpsilonEqual(query.ptA, pt), L"Incorrect witness point A");
				Assert::IsTrue(drb::EpsilonEqual(query.ptB, Vec3(1, 0.5, 0)), L"Incorrect witness point B");
			}

		}

		TEST_METHOD(Convex2Test)
		{
			using drb::physics::Convex2;

			{
				Convex2 cvx{ 9, 13, 6 };

				for (int i = 0; i < cvx.NumVerts(); ++i)
				{
					cvx.GetVert(i) = drb::Vec3(i);
				}

				for (int i = 0; i < cvx.NumEdges(); ++i)
				{
					unsigned char v = static_cast<unsigned char>(i);
					cvx.GetEdge(i) = Convex2::HalfEdge{ v, v, v, v };
				}

				for (int i = 0; i < cvx.NumFaces(); ++i)
				{
					cvx.GetFace(i) = Convex2::Face{ 0, drb::physics::Plane{.n = drb::Vec3(i), .d = -1.0f * i} };
				}
			} // destructor called

			Assert::IsTrue(true, L"Just a place for a breakpoint");
		}
		
	};
}
