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
			using drb::physics::BVHandle;
			using drb::physics::AABB;
			using drb::Vec3;

			// arrange
			BVHierarchy bvh{};

			// act

			// force the tree's node pool to grow
			std::vector<BVHandle> handles{};
			for (auto i = 0; i < 64; ++i)
			{
				handles.push_back( bvh.Insert(AABB{ .c = Vec3(i), .e = Vec3(2) }) );
			}

			auto handle1 = bvh.Insert(AABB{ .c = Vec3(-1), .e = Vec3(3) });

			auto handle2 = bvh.Insert(AABB{.c = Vec3(10), .e = Vec3(3)});

			bvh.Remove(handle1);


			std::ofstream logFile{};
			logFile.open("bvhTestOut.txt");

			for (auto i = 0; i < 64; ++i)
			{
				auto* node = bvh.Find(handles[i]);
				Assert::IsNotNull(node, L"Valid handle not found");
				Assert::IsTrue(node->IsLeaf(), L"Node must be a leaf");
				
				logFile << "Leaf " << node->index << ": \t" 
						<< "Parent = " << node->parent 
						<< " / Children = " << node->children[0] << " " << node->children[1] << '\n';
			}

			// assert
		}
	};
}

