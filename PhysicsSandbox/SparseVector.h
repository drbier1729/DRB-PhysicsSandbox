#ifndef DRB_SPARSEVECTOR_H
#define DRB_SPARSEVECTOR_H

#include <cstdint>
#include <type_traits>
#include <compare>
#include <iterator>
#include <memory>
#include <algorithm>
#include <vector>

#include "DRBAssert.h"

#define DRB_SV_VERSION 1

#if DRB_SV_VERSION == 0

namespace drb {
	template<class C>
	class SparseVector;
}

#elif DRB_SV_VERSION == 1
namespace drb {

	/*
	 * Container offering O(1) element access, insert/emplace, and erase at the
	 * cost of O(n) extra memory. Other characteristics:
	 * --> Most operations require 2 array accesses, one into nodes array
	 *		and one into data array
	 * --> Accepts a template-template arg, Alloc, which is then specialized
	 *		for both nodes and data elements internally.
	 * --> All iterators invalidated by Insert/Emplace if data array grows. 
	 *		The size and capacity of the data array can be queried with
	 *		Size and Capacity. Capacity can be grown manually with 
	 *		Reserve.
	 * --> Erase causes only the iterators to the last-inserted element
	 *      and the erased element to be invalidated. (The iterator to 
	 *      the erased element will map to the last-inserted element after the 
	 *		call to Erase.) Only the Key for the erased element will be
	 *		invalidated.
	 * --> Keys are invalidated by any operations which rearrange elements of
	 *		the data array (e.g. sorting and Erase). Keys are NOT invalidated 
	 *		when the data array grows.
	 * --> Iteration perf is identical to iterating a std::vector in reverse.
	 *		Reverse iteration is used by default because it makes it safe to
	 *		call Erase, as well as Insert/Emplace while iterating (so long as 
	 *      the data array does not grow).
	 */

	template<class Contained, template<typename> class Alloc = std::allocator>
	class SparseVector
	{
	public:
		// ---------------------------------------------------------------------
		// Public Types
		// ---------------------------------------------------------------------
		using index_type             = std::int_least32_t;

		using value_type             = Contained;
		using allocator_type         = Alloc<value_type>;
		using size_type              = index_type;
		using difference_type        = index_type;
		using reference              = value_type&;
		using const_reference        = value_type const&;
		using pointer                = value_type*;
		using const_pointer          = value_type const*;

		static constexpr index_type MAX_SIZE   = std::numeric_limits<index_type>::max();
		static constexpr index_type NULL_INDEX = MAX_SIZE;
		static_assert(~NULL_INDEX == std::numeric_limits<index_type>::min(), "If this fails, our free list may be invalid.");

		struct Key 
		{
			friend class SparseVector<value_type, Alloc>;
			constexpr auto operator<=>(Key const& other) const = default;
			constexpr bool operator==(Key const& other) const = default;

			constexpr Key() = default;
			constexpr Key(index_type k) : index{ k } {}
		private:
			index_type index = NULL_INDEX;
		};

	private:
		// ---------------------------------------------------------------------
		// Internal Types
		// ---------------------------------------------------------------------
		struct Node {			
			index_type        denseIdx  = ~NULL_INDEX; 
			index_type        sparseIdx =  NULL_INDEX;

			inline void       SetNextFree(index_type next) {  denseIdx = ~next; }
			inline index_type NextFree() const             {  return ~denseIdx; }
			inline bool		  IsFree() const               {  return denseIdx < 0; }
		};

		using DataContainer   = std::vector<value_type, allocator_type>;
		using NodeContainer   = std::vector<Node, Alloc<Node>>;

	public:
		// ---------------------------------------------------------------------
		// Iterators
		// ---------------------------------------------------------------------
		using iterator               = DataContainer::reverse_iterator;
		using const_iterator         = DataContainer::const_reverse_iterator;
		using reverse_iterator       = DataContainer::iterator;
		using const_reverse_iterator = DataContainer::const_iterator;

	private:
		// ---------------------------------------------------------------------
		// Data
		// ---------------------------------------------------------------------
		NodeContainer nodes;
		DataContainer data;
		index_type    freeListHead;

	public:
		// ---------------------------------------------------------------------
		// Ctors & Dtor
		// ---------------------------------------------------------------------
		SparseVector() 
			: SparseVector(allocator_type{}) 
		{}

		// This shouldn't work because nodes can't accept Alloc<Contained> as an arg???
		explicit SparseVector(allocator_type alloc) 
			: nodes{ alloc }, data{ alloc }, freeListHead{ NULL_INDEX } 
		{}

		SparseVector(SparseVector const& src, allocator_type alloc = {}) 
			: nodes{ src.nodes, alloc }, data{ src.data, alloc }, freeListHead{ src.freeListHead } 
		{}

		SparseVector(SparseVector&&) = default;
		SparseVector(SparseVector&& src, allocator_type alloc) 
			: nodes{ std::move(src.nodes), alloc }, data{ std::move(src.data), alloc }, freeListHead{ src.freeListHead } 
		{}

		SparseVector& operator=(SparseVector const& rhs) = default;
		SparseVector& operator=(SparseVector&& rhs) = default;

		~SparseVector() noexcept = default;

		// ---------------------------------------------------------------------
		// Iterator Methods
		// ---------------------------------------------------------------------
		
		iterator               begin()         noexcept { return data.rbegin(); }
		const_iterator         begin()   const noexcept { return data.rbegin(); }
		const_iterator         cbegin()  const noexcept { return data.crbegin(); }

		iterator               end()           noexcept { return data.rend(); }
		const_iterator         end()     const noexcept { return data.rend(); }
		const_iterator         cend()    const noexcept { return data.crend(); }
		
		reverse_iterator       rbegin()        noexcept { return data.begin(); }
		const_reverse_iterator rbegin()  const noexcept { return data.begin(); }
		const_reverse_iterator crbegin() const noexcept { return data.cbegin(); }
		
		reverse_iterator       rend()          noexcept { return data.end(); }
		const_reverse_iterator rend()    const noexcept { return data.end(); }
		const_reverse_iterator crend()   const noexcept { return data.cend(); }

		// ---------------------------------------------------------------------
		// Accessors
		// ---------------------------------------------------------------------

		inline size_type SparseSize() const     { return static_cast<size_type>(nodes.size()); }
		inline size_type SparseCapacity() const { return static_cast<size_type>(nodes.capacity()); }

		inline size_type Size() const           { return static_cast<size_type>(data.size()); }
		inline size_type Capacity() const       { return static_cast<size_type>(data.capacity()); }

		bool Contains(Key k) const noexcept
		{
			if (0 <= k.index && k.index < SparseSize() && 
				not nodes[k.index].IsFree()) 
			{
				ASSERT(nodes[k.index].denseIdx < Size(), "Index out of range");
				return true;
			}
			return false;
		}

		const_reference operator[](Key key) const
		{
			ASSERT(Contains(key), "Key not found");
			return data[ nodes[key.index].denseIdx ];
		}

		const_iterator Find(Key key) const
		{
			if (0 <= key.index && key.index < SparseSize())
			{
				Node const& sparseNode = nodes[key.index];
				if (not sparseNode.IsFree())
				{
					ASSERT(0 <= sparseNode.denseIdx && sparseNode.denseIdx < Size(), "Index out of range");
					return data.cbegin() + sparseNode.denseIdx;
				}
			}
			return cend();
		}

		// ---------------------------------------------------------------------
		// Manipulators
		// ---------------------------------------------------------------------

		void Reserve(size_type newCapacity)
		{
			nodes.reserve(newCapacity);
			data.reserve(newCapacity);
		}

		void ShrinkToFit()
		{
			nodes.shrink_to_fit();
			data.shrink_to_fit();
		}

		void Clear()
		{
			nodes.clear();
			data.clear();
		}

		void Swap(SparseVector& other)
		{
			nodes.swap(other.nodes);
			data.swap(other.data);
			std::swap(freeListHead, other.freeListHead);
		}

		reference operator[](Key const& key)
		{
			ASSERT(Contains(key), "Key not found");
			return data[nodes[key.index].denseIdx];
		}

		iterator Find(Key const& key)
		{
			if (0 <= key.index && key.index < SparseSize())
			{
				Node const& sparseNode = nodes[key.index];
				if (not sparseNode.IsFree())
				{
					ASSERT(0 <= sparseNode.denseIdx && sparseNode.denseIdx < Size(), "Index out of range");
					return data.begin() + sparseNode.denseIdx;
				}
			}
			return end();
		}

		std::pair<Key, iterator> Insert(Contained const& val)
		{
			ASSERT(SparseSize() < MAX_SIZE, "Container is full");

			// New dense index will be the back of the dense set
			index_type const denseIdx = Size();
			data.push_back(val);

			index_type sparseIdx{};

			if (freeListHead == NULL_INDEX) 
			{
				// Create a new entry in the sparse set
				sparseIdx = SparseSize();
				nodes.emplace_back(Node{ .denseIdx = denseIdx });
			}
			else 
			{
				// Use the head of the free list
				ASSERT(0 <= freeListHead && freeListHead < SparseSize(), "Free list head out of range");
				
				Node& head = nodes[freeListHead];
				ASSERT(head.IsFree(), "Free list head node was not free!");

				// Record the index from the free list head
				sparseIdx = freeListHead;

				// Update the free list
				freeListHead = head.NextFree();

				// Update the sparse set
				head.denseIdx = denseIdx;
			}

			// Update the dense set
			nodes[denseIdx].sparseIdx = sparseIdx;

			// Return the sparse index
			return { sparseIdx, begin() };
		}

		std::pair<Key, iterator> Insert(Contained&& val = {})
		{
			ASSERT(SparseSize() < MAX_SIZE, "Container is full");

			index_type const denseIdx = Size();
			data.push_back(std::forward<Contained>(val));

			index_type sparseIdx{};

			if (freeListHead == NULL_INDEX)
			{
				// Create a new entry in the sparse set
				sparseIdx = SparseSize();
				nodes.emplace_back(Node{ .denseIdx = denseIdx });
			}
			else 
			{
				// Use the head of the free list
				ASSERT(0 <= freeListHead && freeListHead < SparseSize(), "Free list head out of range");

				Node& head = nodes[freeListHead];
				ASSERT(head.IsFree(), "Free list head node was not free!");

				// Record the index from the free list head
				sparseIdx = freeListHead;

				// Update the free list
				freeListHead = head.NextFree();

				// Update the sparse set
				head.denseIdx = denseIdx;
			}

			// Update the dense set
			nodes[denseIdx].sparseIdx = sparseIdx;

			// Return the sparse index
			return { sparseIdx, begin() };
		}

		template<class ... Args>
		std::pair<Key, iterator> Emplace(Args&& ... args)
		{
			ASSERT(SparseSize() < MAX_SIZE, "Container is full");

			index_type const denseIdx = Size();
			data.emplace_back(std::forward<Args>(args)...);

			index_type sparseIdx{};

			if (freeListHead == NULL_INDEX)
			{
				// Create a new entry in the sparse set
				sparseIdx = SparseSize();
				nodes.emplace_back(Node{ .denseIdx = denseIdx });
			}
			else {
				// Use the head of the free list
				ASSERT(0 <= freeListHead && freeListHead < SparseSize(), "Free list head out of range");

				Node& head = nodes[freeListHead];
				ASSERT(head.IsFree(), "Free list head node was not free!");

				// Record the index from the free list head
				sparseIdx = freeListHead;

				// Update the free list
				freeListHead = head.NextFree();

				// Update the sparse set
				head.denseIdx = denseIdx;
			}

			// Update the dense set
			nodes[denseIdx].sparseIdx = sparseIdx;

			// Return the sparse index
			return { sparseIdx, begin() };
		}

		void Erase(Key const& key)
		{
			if (not (0 <= key.index && key.index < SparseSize())) { return; }

			Node& toFreeSparse = nodes[key.index];
			if (toFreeSparse.IsFree())							  { return; } // triggers if data.empty() is true

			ASSERT(0 <= toFreeSparse.denseIdx && toFreeSparse.denseIdx < Size(), "Index out of range");

			index_type const lastDenseIdx = Size() - 1; 
			ASSERT(0 <= lastDenseIdx, "Container was empty");

			index_type const lastSparseIdx = nodes[lastDenseIdx].sparseIdx;
			ASSERT(0 <= lastSparseIdx && lastSparseIdx <= SparseSize(), "Index out of range");

			// Last element, we can skip some work
			if (toFreeSparse.denseIdx == lastDenseIdx) 
			{
				// Fix up sparse set
				nodes[lastSparseIdx].denseIdx = toFreeSparse.denseIdx;

				// No need to fix up dense set since we'll just
				// implicitly erase the back element by "forgetting
				// about it"

				// Fix up free list
				toFreeSparse.SetNextFree(freeListHead);
				freeListHead = key.index;

				// Erase the element
				data.pop_back();
			}
			else 
			{
				Node& toFreeDense = nodes[toFreeSparse.denseIdx];

				// Move or copy the element from the last slot in the dense data
				// array, then pop it
				reference val = data[toFreeSparse.denseIdx];
				if constexpr (std::is_trivially_copyable_v<value_type>)
				{
					std::memcpy(std::addressof(val), std::addressof(data.back()), sizeof(value_type));
				}
				else if constexpr (requires { std::movable<value_type>; })
				{
					val = std::move( data.back() );
				}
				else
				{
					static_assert(std::false_type<value_type>::value, "T must be movable or trivially copyable");
				}
				data.pop_back();

				// Fix up sparse set
				nodes[lastSparseIdx].denseIdx = toFreeSparse.denseIdx;

				// Fix up dense set
				toFreeDense.sparseIdx = lastSparseIdx;
				// optional:
				// nodes[lastDenseIdx].sparseIdx = NULL_INDEX;

				// Fix up free list
				toFreeSparse.SetNextFree(freeListHead);
				freeListHead = key.index;
			}
		}

		iterator Erase(iterator it)
		{
			if (it == end()) { return end(); }

			index_type const denseIdx = static_cast<index_type>(it.base() - data.begin()) - 1;
			Node& toFreeDense = nodes[denseIdx];

			index_type const sparseIdx = toFreeDense.sparseIdx;
			Node& toFreeSparse = nodes[sparseIdx];
			ASSERT(not toFreeSparse.IsFree(), "Invalid sparse set");

			index_type const lastDenseIdx = Size() - 1;
			ASSERT(0 <= lastDenseIdx, "Container was empty");

			index_type const lastSparseIdx = nodes[lastDenseIdx].sparseIdx;
			ASSERT(0 <= lastSparseIdx && lastSparseIdx <= SparseSize(), "Index out of range");

			// Last element, we can skip some work
			if (denseIdx == lastDenseIdx)
			{
				// Fix up sparse set
				nodes[lastSparseIdx].denseIdx = denseIdx;

				// No need to fix up dense set since we'll just
				// implicitly erase the back element by "forgetting
				// about it"

				// Fix up free list
				toFreeSparse.SetNextFree(freeListHead);
				freeListHead = denseIdx;

				// Erase the element
				data.pop_back();

				return begin();
			}
			else
			{
				// Move or copy the element from the last slot in the dense data
				// array, then pop it
				if constexpr (std::is_trivially_copyable_v<value_type>)
				{
					std::memcpy(std::addressof(*it), std::addressof(data.back()), sizeof(value_type));
				}
				else if constexpr (requires { std::movable<value_type>; })
				{
					*it = std::move(data.back());
				}
				else
				{
					static_assert(std::false_type<value_type>::value, "T must be movable or trivially copyable");
				}
				data.pop_back();

				// Fix up sparse set
				nodes[lastSparseIdx].denseIdx = denseIdx;

				// Fix up dense set
				toFreeDense.sparseIdx = lastSparseIdx;
				// optional:
				// nodes[lastDenseIdx].sparseIdx = NULL_INDEX;

				// Fix up free list
				toFreeSparse.SetNextFree(freeListHead);
				freeListHead = sparseIdx;
				
				return it + 1;
			}
		}
	};

	template<class C, template<typename> class A>
	void swap(SparseVector<C, A>& sv1, SparseVector<C, A>& sv2) 
	{
		sv1.Swap(sv2);
	}

}

#elif DRB_SV_VERSION == 2

namespace drb {

	/*
	 * Container offering O(1) element access, insert/emplace, and erase at the
	 * cost of O(n) extra memory. Other characteristics:
	 * --> Most operations require 2 array accesses, one into nodes array
	 *		and one into data array
	 * --> Keys are invalidated only by operations which rearrange data elements
	 *		(e.g. sorting), but can be revalidated by calling RestoreKeys.
	 *		Erase invalidates only the Key for the element which was erased.
	 * --> Accepts a template-template arg, Alloc, which is then specialized
	 *		for both nodes and data elements internally.
	 * --> All iterators invalidated by Insert/Emplace if data array grows.
	 *		The size and capacity of the data array can be queried with
	 *		Size and Capacity. Capacity can be grown manually with
	 *		Reserve.
	 * --> Erase causes only the iterators to the last-inserted element
	 *      and the erased element to be invalidated. (The iterator to
	 *      the erased element will map to the last-inserted element after the
	 *		call to Erase.)
	 * --> Iteration perf is nearly identical to iterating std::vector, but
	 *		the vector contains with 4 extra bytes of storage per data element.
	 */

	template<class Contained>
	class SparseVector
	{
	public:
		// ---------------------------------------------------------------------
		// Public Types
		// ---------------------------------------------------------------------
		using index_type = std::int_least32_t;

		using value_type = Contained;
		using size_type = index_type;
		using difference_type = index_type;
		using reference = value_type&;
		using const_reference = value_type const&;
		using pointer = value_type*;
		using const_pointer = value_type const*;

		static constexpr index_type MAX_SIZE = std::numeric_limits<index_type>::max();
		static constexpr index_type NULL_INDEX = MAX_SIZE;
		static_assert(~NULL_INDEX == std::numeric_limits<index_type>::min(), "If this fails, our free list may be invalid.");

		struct Key
		{
			friend class SparseVector<value_type>;
			constexpr auto operator<=>(Key const& other) const = default;
			constexpr bool operator==(Key const& other) const = default;

			constexpr Key() = default;
			constexpr Key(index_type k) : index{ k } {}
		private:
			index_type index = NULL_INDEX;
		};

	private:
		// ---------------------------------------------------------------------
		// Internal Types
		// ---------------------------------------------------------------------
		using DataWrapper     = std::pair<value_type, index_type>;

		using SparseContainer = std::vector<index_type>;
		using DenseContainer  = std::vector<DataWrapper>;

	public:
		// ---------------------------------------------------------------------
		// Iterators
		// ---------------------------------------------------------------------
		using iterator = DenseContainer::iterator;
		using const_iterator = DenseContainer::const_iterator;
		using reverse_iterator = DenseContainer::reverse_iterator;
		using const_reverse_iterator = DenseContainer::const_reverse_iterator;

		template<class It>
		struct Iterator
		{
			It it;


		};

	private:
		// ---------------------------------------------------------------------
		// Data
		// ---------------------------------------------------------------------
		SparseContainer sparse;
		DenseContainer  dense;
		index_type      freeListHead;

	private:
		// ---------------------------------------------------------------------
		// Helpers
		// ---------------------------------------------------------------------

		inline bool IsFree(index_type val) const 
		{ 
			return val < 0; 
		}

		inline index_type ToFree(index_type nextFree) 
		{ 
			ASSERT(0 <= nextFree, "nextFree must be positive");
			return ~nextFree; 
		}


	public:
		// ---------------------------------------------------------------------
		// Ctors & Dtor
		// ---------------------------------------------------------------------
		
		// All defaulted


		// ---------------------------------------------------------------------
		// Iterator Methods
		// ---------------------------------------------------------------------

		iterator               begin()         noexcept { return dense.begin(); }
		const_iterator         begin()   const noexcept { return dense.begin(); }
		const_iterator         cbegin()  const noexcept { return dense.cbegin(); }
		iterator               end()           noexcept { return dense.end(); }
		const_iterator         end()     const noexcept { return dense.end(); }
		const_iterator         cend()    const noexcept { return dense.cend(); }
		reverse_iterator       rbegin()        noexcept { return dense.rbegin(); }
		const_reverse_iterator rbegin()  const noexcept { return dense.rbegin(); }
		const_reverse_iterator crbegin() const noexcept { return dense.crbegin(); }
		reverse_iterator       rend()          noexcept { return dense.rend(); }
		const_reverse_iterator rend()    const noexcept { return dense.rend(); }
		const_reverse_iterator crend()   const noexcept { return dense.crend(); }

		// ---------------------------------------------------------------------
		// Accessors
		// ---------------------------------------------------------------------

		inline size_type SparseSize() const { return static_cast<size_type>(sparse.size()); }
		inline size_type SparseCapacity() const { return static_cast<size_type>(sparse.capacity()); }

		inline size_type Size() const { return static_cast<size_type>(dense.size()); }
		inline size_type Capacity() const { return static_cast<size_type>(dense.capacity()); }

		bool Contains(Key k) const noexcept
		{
			return 0 <= k.index && 
				   k.index < SparseSize() && 
				   not IsFree(sparse[k.index];
		}

		const_reference operator[](Key key) const
		{
			ASSERT(Contains(key), "Key not found");
			return dense[ sparse[key.index] ].first;
		}

		const_iterator Find(Key key) const
		{
			if (not Contains(key)) { return cend(); }

			ASSERT(0 <= sparse[key.index] && sparse[key.index] < DenseSize(), "Invalid dense index");

			return dense.cbegin() + sparse[key.index];
		}

		// ---------------------------------------------------------------------
		// Manipulators
		// ---------------------------------------------------------------------

		void Reserve(size_type newCapacity)
		{
			nodes.reserve(newCapacity);
			data.reserve(newCapacity);
		}

		void ShrinkToFit()
		{
			nodes.shrink_to_fit();
			data.shrink_to_fit();
		}

		void Clear()
		{
			nodes.clear();
			data.clear();
		}

		void Swap(SparseVector& other)
		{
			nodes.swap(other.nodes);
			data.swap(other.data);
			std::swap(freeListHead, other.freeListHead);
		}

		reference operator[](Key const& key)
		{
			ASSERT(Contains(key), "Key not found");
			return data[nodes[key.index].denseIdx];
		}

		iterator Find(Key const& key)
		{
			if (0 <= key.index && key.index < SparseSize())
			{
				Node const& sparseNode = nodes[key.index];
				if (not sparseNode.IsFree())
				{
					ASSERT(0 <= sparseNode.denseIdx && sparseNode.denseIdx < Size(), "Index out of range");
					return data.begin() + sparseNode.denseIdx;
				}
			}
			return end();
		}

		std::pair<Key, iterator> Insert(Contained const& val)
		{
			ASSERT(SparseSize() < MAX_SIZE, "Container is full");

			// New dense index will be the back of the dense set
			index_type const denseIdx = Size();
			data.push_back(val);

			index_type sparseIdx{};

			if (freeListHead == NULL_INDEX)
			{
				// Create a new entry in the sparse set
				sparseIdx = SparseSize();
				nodes.emplace_back(Node{ .denseIdx = denseIdx });
			}
			else
			{
				// Use the head of the free list
				ASSERT(0 <= freeListHead && freeListHead < SparseSize(), "Free list head out of range");

				Node& head = nodes[freeListHead];
				ASSERT(head.IsFree(), "Free list head node was not free!");

				// Record the index from the free list head
				sparseIdx = freeListHead;

				// Update the free list
				freeListHead = head.NextFree();

				// Update the sparse set
				head.denseIdx = denseIdx;
			}

			// Update the dense set
			nodes[denseIdx].sparseIdx = sparseIdx;

			// Return the sparse index
			return { sparseIdx, begin() };
		}

		std::pair<Key, iterator> Insert(Contained&& val = {})
		{
			ASSERT(SparseSize() < MAX_SIZE, "Container is full");

			index_type const denseIdx = Size();
			data.push_back(std::forward<Contained>(val));

			index_type sparseIdx{};

			if (freeListHead == NULL_INDEX)
			{
				// Create a new entry in the sparse set
				sparseIdx = SparseSize();
				nodes.emplace_back(Node{ .denseIdx = denseIdx });
			}
			else
			{
				// Use the head of the free list
				ASSERT(0 <= freeListHead && freeListHead < SparseSize(), "Free list head out of range");

				Node& head = nodes[freeListHead];
				ASSERT(head.IsFree(), "Free list head node was not free!");

				// Record the index from the free list head
				sparseIdx = freeListHead;

				// Update the free list
				freeListHead = head.NextFree();

				// Update the sparse set
				head.denseIdx = denseIdx;
			}

			// Update the dense set
			nodes[denseIdx].sparseIdx = sparseIdx;

			// Return the sparse index
			return { sparseIdx, begin() };
		}

		template<class ... Args>
		std::pair<Key, iterator> Emplace(Args&& ... args)
		{
			ASSERT(SparseSize() < MAX_SIZE, "Container is full");

			index_type const denseIdx = Size();
			data.emplace_back(std::forward<Args>(args)...);

			index_type sparseIdx{};

			if (freeListHead == NULL_INDEX)
			{
				// Create a new entry in the sparse set
				sparseIdx = SparseSize();
				nodes.emplace_back(Node{ .denseIdx = denseIdx });
			}
			else {
				// Use the head of the free list
				ASSERT(0 <= freeListHead && freeListHead < SparseSize(), "Free list head out of range");

				Node& head = nodes[freeListHead];
				ASSERT(head.IsFree(), "Free list head node was not free!");

				// Record the index from the free list head
				sparseIdx = freeListHead;

				// Update the free list
				freeListHead = head.NextFree();

				// Update the sparse set
				head.denseIdx = denseIdx;
			}

			// Update the dense set
			nodes[denseIdx].sparseIdx = sparseIdx;

			// Return the sparse index
			return { sparseIdx, begin() };
		}

		void Erase(Key const& key)
		{
			if (not (0 <= key.index && key.index < SparseSize())) { return; }

			Node& toFreeSparse = nodes[key.index];
			if (toFreeSparse.IsFree()) { return; } // triggers if data.empty() is true

			ASSERT(0 <= toFreeSparse.denseIdx && toFreeSparse.denseIdx < Size(), "Index out of range");

			index_type const lastDenseIdx = Size() - 1;
			ASSERT(0 <= lastDenseIdx, "Container was empty");

			index_type const lastSparseIdx = nodes[lastDenseIdx].sparseIdx;
			ASSERT(0 <= lastSparseIdx && lastSparseIdx <= SparseSize(), "Index out of range");

			// Last element, we can skip some work
			if (toFreeSparse.denseIdx == lastDenseIdx)
			{
				// Fix up sparse set
				nodes[lastSparseIdx].denseIdx = toFreeSparse.denseIdx;

				// No need to fix up dense set since we'll just
				// implicitly erase the back element by "forgetting
				// about it"

				// Fix up free list
				toFreeSparse.SetNextFree(freeListHead);
				freeListHead = key.index;

				// Erase the element
				data.pop_back();
			}
			else
			{
				Node& toFreeDense = nodes[toFreeSparse.denseIdx];

				// Move or copy the element from the last slot in the dense data
				// array, then pop it
				reference val = data[toFreeSparse.denseIdx];
				if constexpr (std::is_trivially_copyable_v<value_type>)
				{
					std::memcpy(std::addressof(val), std::addressof(data.back()), sizeof(value_type));
				}
				else if constexpr (requires { std::movable<value_type>; })
				{
					val = std::move(data.back());
				}
				else
				{
					static_assert(std::false_type<value_type>::value, "T must be movable or trivially copyable");
				}
				data.pop_back();

				// Fix up sparse set
				nodes[lastSparseIdx].denseIdx = toFreeSparse.denseIdx;

				// Fix up dense set
				toFreeDense.sparseIdx = lastSparseIdx;
				// optional:
				// nodes[lastDenseIdx].sparseIdx = NULL_INDEX;

				// Fix up free list
				toFreeSparse.SetNextFree(freeListHead);
				freeListHead = key.index;
			}
		}

		iterator Erase(iterator it)
		{
			if (it == end()) { return end(); }

			index_type const denseIdx = static_cast<index_type>(it.base() - data.begin()) - 1;
			Node& toFreeDense = nodes[denseIdx];

			index_type const sparseIdx = toFreeDense.sparseIdx;
			Node& toFreeSparse = nodes[sparseIdx];
			ASSERT(not toFreeSparse.IsFree(), "Invalid sparse set");

			index_type const lastDenseIdx = Size() - 1;
			ASSERT(0 <= lastDenseIdx, "Container was empty");

			index_type const lastSparseIdx = nodes[lastDenseIdx].sparseIdx;
			ASSERT(0 <= lastSparseIdx && lastSparseIdx <= SparseSize(), "Index out of range");

			// Last element, we can skip some work
			if (denseIdx == lastDenseIdx)
			{
				// Fix up sparse set
				nodes[lastSparseIdx].denseIdx = denseIdx;

				// No need to fix up dense set since we'll just
				// implicitly erase the back element by "forgetting
				// about it"

				// Fix up free list
				toFreeSparse.SetNextFree(freeListHead);
				freeListHead = denseIdx;

				// Erase the element
				data.pop_back();

				return begin();
			}
			else
			{
				// Move or copy the element from the last slot in the dense data
				// array, then pop it
				if constexpr (std::is_trivially_copyable_v<value_type>)
				{
					std::memcpy(std::addressof(*it), std::addressof(data.back()), sizeof(value_type));
				}
				else if constexpr (requires { std::movable<value_type>; })
				{
					*it = std::move(data.back());
				}
				else
				{
					static_assert(std::false_type<value_type>::value, "T must be movable or trivially copyable");
				}
				data.pop_back();

				// Fix up sparse set
				nodes[lastSparseIdx].denseIdx = denseIdx;

				// Fix up dense set
				toFreeDense.sparseIdx = lastSparseIdx;
				// optional:
				// nodes[lastDenseIdx].sparseIdx = NULL_INDEX;

				// Fix up free list
				toFreeSparse.SetNextFree(freeListHead);
				freeListHead = sparseIdx;

				return it + 1;
			}
		}
	};

	template<class C, template<typename> class A>
	void swap(SparseVector<C, A>& sv1, SparseVector<C, A>& sv2)
	{
		sv1.Swap(sv2);
	}

}
#endif

#endif

