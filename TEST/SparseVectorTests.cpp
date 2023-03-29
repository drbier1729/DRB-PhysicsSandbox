#include "pch.h"
#include "CppUnitTest.h"
#include "PhysicsSandbox/SparseVector.h"
#include "PhysicsSandbox/StackAllocator.h"
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TEST
{
	TEST_CLASS(SparseVectorUnitTest)
	{
	public:

		TEST_METHOD(TestCtorsDtor)
		{
			using drb::SparseVector;

			{
				SparseVector<double> sv{};
				SparseVector<double> svB{ sv };
				SparseVector<double> svC{ std::move(sv) };
			}
			
			{
				SparseVector<std::string> sv{};
			}
			
			{
				SparseVector<std::string> svA{};
				SparseVector<std::string> svB{ svA };
				SparseVector<std::string> svC{ std::move(svA) };
			}
			
			{
				SparseVector<std::pmr::string, std::pmr::polymorphic_allocator> svA{};
				SparseVector<std::pmr::string, std::pmr::polymorphic_allocator> svB{ svA };
				SparseVector<std::pmr::string, std::pmr::polymorphic_allocator> svC{ std::move(svA) };
			}
			
			{
				std::array<std::byte, 2048> buf{};
				drb::StackMemoryResource resource{ std::addressof(buf), buf.size() };

				SparseVector<std::pmr::string, std::pmr::polymorphic_allocator> svA{ &resource };
				svA.Insert("I'm a big ol string with a big ol ween");
				svA.Emplace("I'm a lil mean string with a lil tiny thang");

				SparseVector<std::pmr::string, std::pmr::polymorphic_allocator> svB{ svA };
				SparseVector<std::pmr::string, std::pmr::polymorphic_allocator> svC{ std::move(svA) };
			}

		}

		TEST_METHOD(TestInsertEraseAccess)
		{
			using drb::SparseVector;

			SparseVector<double> sv{};
			Assert::IsTrue(sv.Size() == 0);
			Assert::IsTrue(sv.SparseSize() == 0);

			auto [k, it]   = sv.Insert(6.28);
			auto [k2, it2] = sv.Emplace(3.14);
			double d = 2.718;
			auto [k3, it3] = sv.Insert(d);
			auto [k4, it4] = sv.Emplace(d);

			Assert::IsTrue(sv.Size() == 4);
			Assert::IsTrue(sv.SparseSize() == 4);
			Assert::IsTrue(sv[k] == 6.28);
			Assert::IsTrue(sv[k2] == 3.14);
			Assert::IsTrue(sv[k3] == 2.718);
			Assert::IsTrue(sv[k4] == 2.718);

			sv.Erase(k);
			sv.Erase(k2);

			Assert::IsTrue(sv.Size() == 2);
			Assert::IsTrue(sv.SparseSize() == 4);
			Assert::IsTrue(sv[k3] == 2.718);
			Assert::IsTrue(sv[k4] == 2.718);

			auto [k5, it5] = sv.Insert(10.0);
			Assert::IsTrue(k5 == k2); // make sure we reused most recently erased key
			Assert::IsTrue(sv.Size() == 3);
			Assert::IsTrue(sv.SparseSize() == 4);
			Assert::IsTrue(sv[k3] == 2.718);
			Assert::IsTrue(sv[k4] == 2.718);
			Assert::IsTrue(sv[k5] == 10.0);

			auto [k6, it6] = sv.Insert(12.0);
			Assert::IsTrue(k6 == k); // make sure we reused most recently erased key
			Assert::IsTrue(sv.Size() == 4);
			Assert::IsTrue(sv.SparseSize() == 4);
			Assert::IsTrue(sv[k3] == 2.718);
			Assert::IsTrue(sv[k4] == 2.718);
			Assert::IsTrue(sv[k5] == 10.0);
			Assert::IsTrue(sv[k6] == 12.0);
			
			auto [k7, it7] = sv.Insert(666.0);
			Assert::IsTrue(sv.Size() == 5);
			Assert::IsTrue(sv.SparseSize() == 5);
			Assert::IsTrue(sv[k3] == 2.718);
			Assert::IsTrue(sv[k4] == 2.718);
			Assert::IsTrue(sv[k5] == 10.0);
			Assert::IsTrue(sv[k6] == 12.0);
			Assert::IsTrue(sv[k7] == 666.0);
		}
		
		TEST_METHOD(TestIterators)
		{
			using drb::SparseVector;
			
			SparseVector<double> sv{};
			
			std::vector<decltype(sv)::Key> keys{};

			for (auto i = 0; i < 12; ++i)
			{
				auto [key, it] = sv.Insert(i * 1.0);
				
				*it *= 2.0;
				keys.push_back(key);
			}

			for (auto&& d : sv)
			{
				d *= 3.14;
			}

			Assert::IsTrue(false, L"Bug here with erase during iteration");
			for (auto it = sv.begin(); it != sv.end(); ++it)
			{
				sv.Erase(it);
				sv.Emplace(666.666);
			}

			Assert::IsTrue(true);
		}
	};
}
