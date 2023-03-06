#include "pch.h"
#include "CppUnitTest.h"
#include "PhysicsSandbox/CollisionGeometry.h"
#include "PhysicsSandbox/CollisionQueries.h"
#include "PhysicsSandbox/SATGJK.h"
#include "PhysicsSandbox/Math.h"


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
			Plane p0 = drb::physics::Plane::Make(Vec3(1, 0, 0), Vec3(0.5f, 0, 0));
			Plane p1 = drb::physics::Plane::Make(drb::Normalize(Vec3(1, 0, 1)), Vec3(1,0,0));

			
			{
				// act
				Polygon front{}, back{};
				square.Split(p0, front, back);

				// assert
				Assert::IsTrue(front.verts.size() == 4 && back.verts.size() == 4, L"Square should have split in two rectangles");
			}
			
			{
				// act
				Polygon front{}, back{};
				square.Split(p1, front, back);

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
			Convex const hull = drb::physics::Convex::MakeBox(Vec3(1));
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

		//TEST_METHOD(Convex2Test)
		//{
		//	using drb::Vec3;
		//	using drb::physics::Convex2;
		//	using drb::physics::Convex;


		//	auto assertEqual = [](Convex const& cvx, Convex2 const& cvx2)
		//	{
		//		Assert::IsTrue(cvx.verts.size() ==   cvx2.NumVerts(), L"vert sizes differ");
		//		Assert::IsTrue(cvx.vertAdj.size() == cvx2.NumVerts(), L"vert sizes differ");
		//		Assert::IsTrue(cvx.edges.size() ==   cvx2.NumHalfEdges(), L"edge sizes differ");
		//		Assert::IsTrue(cvx.faces.size() ==   cvx2.NumFaces(), L"edge sizes differ");

		//		auto const verts2 = cvx2.GetVerts();
		//		auto const vertAdj2 = cvx2.GetVertAdjs();
		//		for (auto i = 0; i < verts2.size(); ++i)
		//		{
		//			Assert::IsTrue(cvx.verts[i] == verts2[i], L"vert not equal");
		//			Assert::IsTrue(cvx.vertAdj[i] == vertAdj2[i], L"vert adjacency not equal");
		//		}
		//		
		//		auto const edges2 = cvx2.GetEdges();
		//		for (auto i = 0; i < edges2.size(); ++i)
		//		{
		//			bool const equal = cvx.edges[i].next == edges2[i].next &&
		//				               cvx.edges[i].twin == edges2[i].twin &&
		//				               cvx.edges[i].origin == edges2[i].origin &&
		//				               cvx.edges[i].face == edges2[i].face;
		//			Assert::IsTrue(equal, L"edges not equal");
		//		}
		//		
		//		auto const faces2 = cvx2.GetFaces();
		//		for (auto i = 0; i < faces2.size(); ++i)
		//		{
		//			bool const equal = cvx.faces[i].edge == faces2[i].edge &&
		//							   cvx.faces[i].plane == faces2[i].plane;
		//			Assert::IsTrue(equal, L"edges not equal");
		//		}
		//	};

		//	// Check memory
		//	std::size_t mem = 0;
		//	{
		//		Vec3 const halfwidths = { 1, 2, 3 };

		//		Convex2 const box2 = Convex2::MakeBox(halfwidths);
		//		mem = static_cast<std::size_t>( box2.data->memUsed );
		//	}

		//	{
		//		Vec3 const halfwidths = { 1, 2, 3 };

		//		Convex  const box = drb::physics::MakeBox(halfwidths);
		//		Convex2 const box2 = Convex2::MakeBox(halfwidths);

		//		assertEqual(box, box2);
		//	
		//	}

		//	{
		//		Vec3 const p0 = { 1, 2, 3 };
		//		Vec3 const p1 = { -1, -2, 0 };
		//		Vec3 const p2 = { 0, 0, 0 };
		//		Vec3 const p3 = { 2, 2, 2 };

		//		Convex  const tet = drb::physics::MakeTetrahedron(p0, p1, p2, p3);
		//		Convex2 const tet2 = Convex2::MakeTetrahedron(p0, p1, p2, p3);

		//		assertEqual(tet, tet2);
		//	
		//	}
		//}
		
	};
}