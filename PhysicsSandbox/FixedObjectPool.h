#ifndef DRB_FIXEDOBJECTPOOL_H
#define DRB_FIXEDOBJECTPOOL_H

/*
Adapted from plf::colony https://github.com/mattreecebentley/plf_colony. See license below.

Like the original this:
-- has O(1) insert/emplace and erase methods
-- requires O(N) extra memory for the skipfield
-- guarantees pointers/references to elements remain valid until erase is called
	on that element
-- iterates elements as contiguously as possible by "jumping over" blocks of erased
	elements using a skipfield implemented with Low Complexity Jump Counting
-- maintains an intrusive free list of skipblocks indexed by their start nodes

This is a simplified version which differs from the original:
-- only compatible with C++20 and not tested on compilers except MSVC
-- only uses a fixed size, static array for backing buffer instead of dynamically growing
-- only inserts into the first node of each skipblock
-- does not use iterators (unfortunately) and instead is iterable using a range, e.g.,

	FixedObjectPool<T, N> fop;
	// ... insert elements ...

	for(auto range = fop; not range.is_empty(); range.pop_front()) {
		T& element = range.front();
		// ... read/write elements ...
	}

TODOs:
-- implement copy/move constructors and assignment operators
-- implement batch_insert and batch_erase methods
-- add support for iterators (and thus for C++ std algorithms)
*/

/*
PLF COLONY LICENSE

zlib License

(C) 2019 mattreecebentley

This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.

Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:

The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
This notice may not be removed or altered from any source distribution.
*/

#include <cstddef>
#include <numeric>
#include <utility>
#include <memory>
#include <array>

#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif 

#ifndef assert
#include <cassert>
#endif

template<class T, std::size_t N>
class FixedObjectPool
{
public:
	using value_type = T;
	using reference = T&;
	using const_reference = const T&;
	using xvalue_reference = T&&;
	using pointer = T*;
	using const_pointer = const T*;

	using size_type = std::size_t;
	using difference_type = std::ptrdiff_t;

	using skipfield_type = typename std::conditional_t<
		(N < std::numeric_limits<unsigned char>::max()), unsigned char,
		std::conditional_t<
		(N < std::numeric_limits<unsigned short>::max()), unsigned short,
		std::conditional_t<
		(N < std::numeric_limits<unsigned int>::max()), unsigned int,
		size_type
		>
		>
		>;
	using skipfield_pointer = skipfield_type*;
	using const_skipfield_pointer = const skipfield_type*;

	// Align values to be at least twice the size of skipfield_type so we can fit the free list indices in erased slots
	using aligned_element_type = typename std::aligned_storage< sizeof(value_type),
		(sizeof(value_type) >= (sizeof(skipfield_type) * 2) || alignof(value_type) >= (sizeof(skipfield_type) * 2)) ?
		alignof(value_type) :
		(sizeof(skipfield_type) * 2)
	>::type;

	// Range-style iteration support
	template<bool is_const> class fop_range;
	using range = fop_range<false>;
	using const_range = fop_range<true>;
	friend fop_range<false>;
	friend fop_range<true>;

private:
	// Internal storage
	struct alignas(alignof(aligned_element_type)) dummy_type
	{
		std::byte pad[sizeof(aligned_element_type)];
	};
	using dummy_pointer = dummy_type*;
	using const_dummy_pointer = const dummy_type*;

	// Free list operations
	struct free_list_node {
		skipfield_type next = 0;
		skipfield_type prev = 0;
	};
	static_assert(sizeof(aligned_element_type) >= sizeof(free_list_node));
	using free_list_pointer = free_list_node*;
	using const_free_list_pointer = free_list_node const*;

private:
	static constexpr skipfield_type k_skipfield_max = std::numeric_limits<skipfield_type>::max();

	std::array<dummy_type, N> elements = {};			// Buffer to contain all elements AND in-place free-list
	std::array<skipfield_type, N> skipfield = {};		// "Low Complexity Jump Counting" skipfield
	skipfield_type total_size = 0;						// Number of active elements
	skipfield_type first_occupied_index = N;			// Index of the first active element. Value >=N  ==>  no occupied slots.
	skipfield_type free_list_head = k_skipfield_max;	// Index of the most recently erased slot. Value == k_skipfield_max  ==>  free list is empty.

public:
	// TODO : add allocator aware constructors
	FixedObjectPool() noexcept = default;

	FixedObjectPool(const FixedObjectPool&) noexcept = delete; // TODO: make sure this correctly copies elements!

	FixedObjectPool(FixedObjectPool&& src) noexcept = delete; // TODO: finish this
	//	: elements{ },
	//	skipfield{ src.skipfield },
	//	total_size{ src.total_size },
	//	first_occupied_index{ src.first_occupied_index },
	//	free_list_head{ src.free_list_head }
	//{
	//	assert(false && "Not working! Need to iterate only valid elements and move them, but copy free list slots too.");
	//	for (skipfield_type i = 0; i < N; ++i) {
	//		*index_to_elem_ptr(i) = std::move( *src.index_to_elem_ptr(i) );
	//	}

	//	src.clear();
	//}

	FixedObjectPool& operator=(const FixedObjectPool&) noexcept = delete;
	FixedObjectPool& operator=(FixedObjectPool&& other) noexcept = delete; // TODO: finish this
	/*{

		assert(false && "Not working! Need to iterate only valid elements and move them, but copy free list slots too.");
		if (this == &other) { return *this; }
		destroy_all_data();

		skipfield = other.skipfield;
		total_size = other.total_size;
		first_occupied_index = other.first_occupied_index;
		free_list_head = other.free_list_head;

		for (skipfield_type i = 0; i < N; ++i) {
			*index_to_elem_ptr(i) = std::move(*other.index_to_elem_ptr(i));
		}

		other.clear();
	}*/

	~FixedObjectPool() noexcept {
		destroy_all_data();
	}


	[[nodiscard]]
	pointer insert(const_reference val) {
		assert(total_size < N);

		pointer construct_address = nullptr;

		if (free_list_head == k_skipfield_max) {
			// Slot address is just the end of the current array
			construct_address = index_to_elem_ptr(total_size);

			// If this is the first element...
			if (total_size == 0) { first_occupied_index = 0; }
		}
		else {
			// Update first occupied
			if (free_list_head < first_occupied_index) {
				first_occupied_index = free_list_head;
			}

			// Get the current slot
			dummy_pointer free_slot = index_to_slot_ptr(free_list_head);

			// Get slot address
			construct_address = convert_ptr<pointer>(free_slot);

			// Update skipfield (change from skipped to unskipped)
			/*
			* check left and right nodes
			*   if left is non-zero:				 we're either in the middle or at the end of a skipblock... something went wrong
			*	else if right is non-zero:			 we're at the start of a skipblock
			*	else:								 we're in a unit-length skipblock
			*/
			skipfield_type curr_index = free_list_head;
			skipfield_pointer skip_ptr = index_to_skip_ptr(curr_index);
			skipfield_type left = (curr_index == 0) ? 0 : *(skip_ptr - 1);
			skipfield_type right = (curr_index == N - 1) ? 0 : *(skip_ptr + 1);

			if (left != 0) [[unlikely]] {
				assert(false && "We're in the middle or at the end of a skipblock!");
			}
			else {
				if (right != 0) {
					skipfield_type new_val = *skip_ptr - 1;
					skipfield_pointer end = skip_ptr + new_val;
					*(skip_ptr + 1) = new_val;
					*end = new_val;

					// Replace the freelist node for this skipblock (i.e. the freelist head)
					// with the new start node of the skipblock
					free_list_pointer old_start_node = convert_ptr<free_list_pointer>(free_slot);
					free_list_pointer new_start_node = convert_ptr<free_list_pointer>(free_slot + 1);
					*new_start_node = *old_start_node;

					// Zero the old slot to make sure there's no leftover data there when
					// constructing the new element
					*old_start_node = {};

					// Update the free list head
					free_list_head = curr_index + 1;

					// Update the next node in the free list
					if (new_start_node->next != k_skipfield_max) {
						free_list_pointer next_node = index_to_free_list_ptr(new_start_node->next);
						next_node->prev = free_list_head;
					}
				}
				else {
					// Skipblock will now be completely empty so advance the freelist head to
					// the start of the next skipblock
					free_list_pointer old_head_node = convert_ptr<free_list_pointer>(free_slot);
					free_list_head = old_head_node->next;

					if (free_list_head != k_skipfield_max) {
						free_list_pointer new_head_node = index_to_free_list_ptr(free_list_head);
						new_head_node->prev = k_skipfield_max;
					}

					// Zero the old slot to make sure there's no leftover data there when
					// constructing the new element
					*old_head_node = {};
				}

				// Set the corresponding skipfield value to 0 to indicate an occupied slot
				*skip_ptr = 0;
			}
		}

		// Copy construct
		pointer result = std::construct_at<value_type>(construct_address, val);

		++total_size;
		return result;
	}

	[[nodiscard]]
	pointer insert(xvalue_reference val = value_type{}) {
		assert(total_size < N);

		pointer construct_address = nullptr;

		if (free_list_head == k_skipfield_max) {
			// Slot address is just the end of the current array
			construct_address = index_to_elem_ptr(total_size);

			// If this is the first element...
			if (total_size == 0) { first_occupied_index = 0; }
		}
		else {
			// Update first occupied
			if (free_list_head < first_occupied_index) {
				first_occupied_index = free_list_head;
			}

			// Get the current slot
			dummy_pointer free_slot = index_to_slot_ptr(free_list_head);

			// Get slot address
			construct_address = convert_ptr<pointer>(free_slot);

			// Update skipfield (change from skipped to unskipped)
			/*
			* check left and right nodes
			*   if left is non-zero:				 we're either in the middle or at the end of a skipblock... something went wrong
			*	else if right is non-zero:			 we're at the start of a skipblock
			*	else:								 we're in a unit-length skipblock
			*/
			skipfield_type curr_index = free_list_head;
			skipfield_pointer skip_ptr = index_to_skip_ptr(curr_index);
			skipfield_type left = (curr_index == 0) ? 0 : *(skip_ptr - 1);
			skipfield_type right = (curr_index == N - 1) ? 0 : *(skip_ptr + 1);

			if (left != 0) [[unlikely]] {
				assert(false && "We're in the middle or at the end of a skipblock!");
			}
			else {
				if (right != 0) {
					skipfield_type new_val = *skip_ptr - 1;
					skipfield_pointer end = skip_ptr + new_val;
					*(skip_ptr + 1) = new_val;
					*end = new_val;

					// Replace the freelist node for this skipblock (i.e. the freelist head)
					// with the new start node of the skipblock
					free_list_pointer old_start_node = convert_ptr<free_list_pointer>(free_slot);
					free_list_pointer new_start_node = convert_ptr<free_list_pointer>(free_slot + 1);
					*new_start_node = *old_start_node;

					// Zero the old slot to make sure there's no leftover data there when
					// constructing the new element
					*old_start_node = {};

					// Update the free list head
					free_list_head = curr_index + 1;

					// Update the next node in the free list
					if (new_start_node->next != k_skipfield_max) {
						free_list_pointer next_node = index_to_free_list_ptr(new_start_node->next);
						next_node->prev = free_list_head;
					}
				}
				else {
					// Skipblock will now be completely empty so advance the freelist head to
					// the start of the next skipblock
					free_list_pointer old_head_node = convert_ptr<free_list_pointer>(free_slot);
					free_list_head = old_head_node->next;

					if (free_list_head != k_skipfield_max) {
						free_list_pointer new_head_node = index_to_free_list_ptr(free_list_head);
						new_head_node->prev = k_skipfield_max;
					}

					// Zero the old slot to make sure there's no leftover data there when
					// constructing the new element
					*old_head_node = {};
				}

				// Set the corresponding skipfield value to 0 to indicate an occupied slot
				*skip_ptr = 0;
			}
		}

		// Move construct
		pointer result = std::construct_at<value_type>(construct_address, val);

		++total_size;
		return result;
	}

	template<class ... Args>
	[[nodiscard]]
	pointer emplace(Args&& ... args) {
		assert(total_size < N);

		pointer construct_address = nullptr;

		if (free_list_head == k_skipfield_max) {
			// Slot address is just the end of the current array
			construct_address = index_to_elem_ptr(total_size);

			// If this is the first element...
			if (total_size == 0) { first_occupied_index = 0; }
		}
		else {
			// Update first occupied
			if (free_list_head < first_occupied_index) {
				first_occupied_index = free_list_head;
			}

			// Get the current slot
			dummy_pointer free_slot = index_to_slot_ptr(free_list_head);

			// Get slot address
			construct_address = convert_ptr<pointer>(free_slot);

			// Update skipfield (change from skipped to unskipped)
			/*
			* check left and right nodes
			*   if left is non-zero:				 we're either in the middle or at the end of a skipblock... something went wrong
			*	else if right is non-zero:			 we're at the start of a skipblock
			*	else:								 we're in a unit-length skipblock
			*/
			skipfield_type curr_index = free_list_head;
			skipfield_pointer skip_ptr = index_to_skip_ptr(curr_index);
			skipfield_type left = (curr_index == 0) ? 0 : *(skip_ptr - 1);
			skipfield_type right = (curr_index == N - 1) ? 0 : *(skip_ptr + 1);

			if (left != 0) [[unlikely]] {
				assert(false && "We're in the middle or at the end of a skipblock!");
			}
			else {
				if (right != 0) {
					skipfield_type new_val = *skip_ptr - 1;
					skipfield_pointer end = skip_ptr + new_val;
					*(skip_ptr + 1) = new_val;
					*end = new_val;

					// Replace the freelist node for this skipblock (i.e. the freelist head)
					// with the new start node of the skipblock
					free_list_pointer old_start_node = convert_ptr<free_list_pointer>(free_slot);
					free_list_pointer new_start_node = convert_ptr<free_list_pointer>(free_slot + 1);
					*new_start_node = *old_start_node;

					// Zero the old slot to make sure there's no leftover data there when
					// constructing the new element
					*old_start_node = {};

					// Update the free list head
					free_list_head = curr_index + 1;

					// Update the next node in the free list
					if (new_start_node->next != k_skipfield_max) {
						free_list_pointer next_node = index_to_free_list_ptr(new_start_node->next);
						next_node->prev = free_list_head;
					}
				}
				else {
					// Skipblock will now be completely empty so advance the freelist head to
					// the start of the next skipblock
					free_list_pointer old_head_node = convert_ptr<free_list_pointer>(free_slot);
					free_list_head = old_head_node->next;

					if (free_list_head != k_skipfield_max) {
						free_list_pointer new_head_node = index_to_free_list_ptr(free_list_head);
						new_head_node->prev = k_skipfield_max;
					}

					// Zero the old slot to make sure there's no leftover data there when
					// constructing the new element
					*old_head_node = {};
				}

				// Set the corresponding skipfield value to 0 to indicate an occupied slot
				*skip_ptr = 0;
			}
		}

		// Construct from args
		pointer result = std::construct_at<value_type>(construct_address, std::forward<Args>(args)...);

		++total_size;
		return result;
	}

	// Returns ptr to next valid element, or nullptr if at end
	pointer erase(pointer p_val) {
		assert(p_val != nullptr);
		assert(p_val >= convert_ptr<pointer>(elements.data()));
		assert(p_val < convert_ptr<pointer>(elements.data() + N));

		// Index of return value
		skipfield_type next = N;

		// Get ptr to current slot and skip ptr
		dummy_pointer slot_ptr = convert_ptr<dummy_pointer>(p_val);
		skipfield_type const curr_index = slot_ptr_to_index(slot_ptr);
		skipfield_pointer const skip_ptr = index_to_skip_ptr(curr_index);

		if (*skip_ptr != 0) { return nullptr; } // double delete

		// Destroy the current element
		std::destroy_at<value_type>(p_val);

		// Update free list and skipfield (change value from unskipped to skipped) 
		*slot_ptr = {}; // Zero the slot

		/*
		* check left and right nodes
		*	if left and right are non-zero:      we need to join the neighboring skip blocks
		*	else if only the right is zero:		 add the current node to the right skipblock as start
		*	else if only the left is zero:		 add the current node to the left skipblock as end
		*	else:								 set the current node to 1 (new unit-length skipblock)
		*/
		skipfield_type const left = (curr_index == 0) ? 0 : *(skip_ptr - 1);
		skipfield_type const right = (curr_index == N - 1) ? 0 : *(skip_ptr + 1);

		// Node will become a middle node in the skipblock
		if (left != 0 && right != 0) {
			skipfield_pointer start = skip_ptr - left;
			*start = left + right + 1;

			skipfield_pointer end = skip_ptr + right;
			*end = left + right + 1;

			// Ensure that no nodes in a skipblock have value 0
			*skip_ptr = 1;

			// Next element is at the index == end node's index + 1
			next = curr_index + right + 1;

			// Get free list node to the right (start of the right skipblock)
			free_list_pointer right_node = convert_ptr<free_list_pointer>(slot_ptr + 1);

			// Get the free list nodes at the start of the left block
			free_list_pointer start_node = index_to_free_list_ptr(curr_index - left);

			// In this case we are effectively just erasing the right skipblock
			// from the free list entirely since it is merged into the left
			// skipblock

			// Update right's prev node
			if (right_node->prev == k_skipfield_max) {
				free_list_head = right_node->next;
			}
			else {
				free_list_pointer r_prev_node = index_to_free_list_ptr(right_node->prev);
				r_prev_node->next = right_node->next;
			}

			// Update right's next node
			if (right_node->next != k_skipfield_max) {
				free_list_pointer r_next_node = index_to_free_list_ptr(right_node->next);
				r_next_node->prev = right_node->prev;
			}

			// Zero the right node's free list slot since it is now unused
			*right_node = {};
		}
		else {
			// Node will become the start node of the skip block
			if (right != 0) {
				skipfield_pointer end = index_to_skip_ptr(curr_index + right);
				*end = right + 1;
				*skip_ptr = right + 1;

				// Next element is at the index == end node's index + 1
				next = curr_index + right + 1;

				// Get the free list node to the right
				free_list_pointer right_node = convert_ptr<free_list_pointer>(slot_ptr + 1);

				// Reinterpret the zero'd slot as a free list node
				free_list_pointer curr_node = convert_ptr<free_list_pointer>(slot_ptr);

				// Make the current node the freelist node for this skipblock

				// Update right's prev node
				if (right_node->prev == k_skipfield_max) {
					free_list_head = curr_index;
				}
				else {
					free_list_pointer r_prev_node = index_to_free_list_ptr(right_node->prev);
					r_prev_node->next = curr_index;
				}

				// Update right's next node
				if (right_node->next != k_skipfield_max) {
					free_list_pointer r_next_node = index_to_free_list_ptr(right_node->next);
					r_next_node->prev = curr_index;
				}

				// Copy right node's prev and next into current node
				*curr_node = *right_node;

				// Zero the right node's free list slot since it is now unused
				*right_node = {};
			}
			// Node will become the end node of the skip block 
			else if (left != 0) {
				skipfield_pointer start = index_to_skip_ptr(curr_index - left);
				*start = left + 1;
				*skip_ptr = left + 1;

				// Next element is at the index == current index + 1
				next = curr_index + 1;

				// No need to modify the free list since the start node
				// of the block remains unchanged
			}

			// Node is a unit-length skipblock
			else {
				*skip_ptr = 1;

				// Next element is at the index == current index + 1
				next = curr_index + 1;

				// Reinterpret the current slot as a free list node
				free_list_pointer curr_node = convert_ptr<free_list_pointer>(slot_ptr);

				// Add the index to the head of the free list
				if (free_list_head != k_skipfield_max) {
					free_list_pointer head_node = index_to_free_list_ptr(free_list_head);
					head_node->prev = curr_index;
				}
				curr_node->next = free_list_head;
				curr_node->prev = k_skipfield_max;
				free_list_head = curr_index;
			}
		}

		// Update first_occupied_index if we've just erased the first occupied slot
		if (curr_index == first_occupied_index) {
			first_occupied_index = next;
		}

		--total_size;
		return (next >= N) ? nullptr : index_to_elem_ptr(next);
	}

	void clear() {
		destroy_all_data();
		skipfield = {}; // Zero the skipfield
		total_size = 0;
		first_occupied_index = N;
		free_list_head = k_skipfield_max;
	}

	range all() {
		return range{
			index_to_elem_ptr(first_occupied_index),
			index_to_skip_ptr(first_occupied_index),
			total_size
		};
	}

	const_range all() const {
		return const_range{
			index_to_elem_ptr(first_occupied_index),
			index_to_skip_ptr(first_occupied_index),
			total_size
		};
	}

	size_type size() const noexcept { return total_size; }

	size_type capacity() const noexcept { return N; }

private:
	void destroy_all_data() {
		range whole_container = all();
		while (!whole_container.is_empty()) {
			std::destroy_at<value_type>(std::addressof(whole_container.front()));
			whole_container.pop_front();
		}
	}

	template<class DestPtr, class SrcPtr>
	static constexpr DestPtr convert_ptr(const SrcPtr src_ptr) noexcept {
		if constexpr (std::is_trivial<DestPtr>::value && std::is_trivial<SrcPtr>::value) {
			return std::bit_cast<DestPtr>(src_ptr);
		}
		else {
			return DestPtr{ std::to_address(src_ptr) };
		}
	}

	// Some of these might be removable -- here in case we need them
	skipfield_type slot_ptr_to_index(dummy_pointer ptr) noexcept {
		return static_cast<skipfield_type>(ptr - elements.data());
	}
	skipfield_type skip_ptr_to_index(skipfield_pointer ptr) noexcept {
		return static_cast<skipfield_type>(ptr - skipfield.data());
	}
	skipfield_type elem_ptr_to_index(pointer ptr) noexcept {
		return static_cast<skipfield_type>(convert_to<pointer>(ptr) - elements.data());
	}

	dummy_pointer index_to_slot_ptr(skipfield_type index) noexcept {
		return elements.data() + index;
	}

	free_list_pointer index_to_free_list_ptr(skipfield_type index) noexcept {
		return convert_ptr<free_list_pointer>(elements.data() + index);
	}

	const_free_list_pointer index_to_free_list_ptr(skipfield_type index) const noexcept {
		return convert_ptr<const_free_list_pointer>(elements.data() + index);
	}

	pointer index_to_elem_ptr(skipfield_type index) noexcept {
		return convert_ptr<pointer>(elements.data() + index);
	}
	const_pointer index_to_elem_ptr(skipfield_type index) const noexcept {
		return convert_ptr<const_pointer>(elements.data() + index);
	}

	skipfield_pointer index_to_skip_ptr(skipfield_type index) noexcept {
		return skipfield.data() + index;
	}
	const_skipfield_pointer index_to_skip_ptr(skipfield_type index) const noexcept {
		return skipfield.data() + index;
	}

	skipfield_pointer get_skip_ptr(pointer ptr) noexcept {
		return skipfield.data() + elem_ptr_to_index(ptr);
	}
	pointer get_elem_ptr(skipfield_pointer ptr) noexcept {
		return convert_to<pointer>(elements.data() + skip_ptr_to_index(ptr));
	}


	// Range Definition
public:

	template<bool is_const>
	class fop_range
	{
	public:
		friend class FixedObjectPool;

		using pointer = typename std::conditional_t<is_const, typename FixedObjectPool::const_pointer, typename FixedObjectPool::pointer>;
		using dummy_pointer = typename std::conditional_t<is_const, typename FixedObjectPool::const_dummy_pointer, typename FixedObjectPool::dummy_pointer>;
		using skipfield_pointer = typename std::conditional_t<is_const, typename FixedObjectPool::const_skipfield_pointer, typename FixedObjectPool::skipfield_pointer>;
		using reference = typename std::conditional_t<is_const, typename FixedObjectPool::const_reference, typename FixedObjectPool::reference>;

	private:
		pointer elem_ptr = nullptr;
		skipfield_pointer skip_ptr = nullptr;
		skipfield_type length = 0;

	public:
		// CTORS + DTOR
		fop_range() noexcept = default;
		fop_range(const fop_range& src) noexcept = default;
		fop_range(fop_range&&) = default;
		fop_range& operator= (const fop_range& rhs) noexcept = default;
		fop_range& operator= (fop_range&& rhs) noexcept = default;
		~fop_range() noexcept = default;

		// Converting from range to const_range
		template<bool is_const_r = is_const, class = std::enable_if_t<is_const_r>>
		fop_range(const fop_range<false>& src) noexcept
			: elem_ptr{ src.elem_ptr }, skip_ptr{ src.skip_ptr }, length{ src.length }
		{ }

		template<bool is_const_r = is_const, class = std::enable_if_t<is_const_r>>
		fop_range(fop_range<false>&& src) noexcept
			: elem_ptr{ src.elem_ptr }, skip_ptr{ src.skip_ptr }, length{ src.length }
		{ }

		template<bool is_const_r = is_const, class = std::enable_if_t<is_const_r>>
		fop_range& operator= (const fop_range<false>& rhs) noexcept {
			elem_ptr = rhs.elem_ptr;
			skip_ptr = rhs.skip_ptr;
			length = rhs.length;
			return *this;
		}

		template<bool is_const_r = is_const, class = std::enable_if_t<is_const_r>>
		fop_range& operator= (fop_range<false>&& rhs) noexcept {
			elem_ptr = rhs.elem_ptr;
			skip_ptr = rhs.skip_ptr;
			length = rhs.length;
			return *this;
		}


		// METHODS
		bool is_empty() const noexcept {
			return length == 0;
		}

		reference front() {
			assert(!is_empty());
			return *elem_ptr;
		}

		void pop_front() {
			assert(!is_empty());

			// Read the next skip and increment skip ptr
			skipfield_type skip = *(++skip_ptr);

			// Skip forward
			skip_ptr += skip;

			dummy_pointer ptr = convert_ptr<dummy_pointer>(elem_ptr);
			ptr += skip + 1;

			// Convert back
			elem_ptr = convert_ptr<pointer>(ptr);

			// Decrement length
			--length;
		}


	private:
		// Needed by FixedObjectPool::all()
		explicit fop_range(pointer p_elems, skipfield_pointer p_skip, skipfield_type len)
			: elem_ptr{ p_elems },
			skip_ptr{ p_skip },
			length{ len }
		{}
	};

#ifdef FOP_DEBUG
public:
	template<class T, std::size_t N>
	friend void print(FixedObjectPool<T, N> const& fop);
#endif
};

#ifdef FOP_DEBUG
#define PRINT_FOP(var) print((var))

#include <iostream>
#include <vector>
#include <tuple>


template<class T>
concept printable = requires (T t) {
	std::cerr << t;
};

template<class T, std::size_t N>
void print(FixedObjectPool<T, N> const& fop) {
	std::cerr << std::flush;

	std::cerr << "FreeList: ";
	auto head = fop.free_list_head;
	while (head != fop.k_skipfield_max) {
		std::cerr << "[ " << (std::size_t)head << " ";

		assert(head < N);
		for (auto i = 1u; i < fop.skipfield[head]; ++i) {
			std::cerr << head + i << " ";
		}
		std::cerr << "]";

		auto ptr = fop.index_to_free_list_ptr(head);
		head = ptr->next;
	}
	std::cerr << '\n' << std::flush;


	std::cerr << "Skipfield: ";
	for (auto&& s : fop.skipfield) {
		std::cerr << (std::size_t)s << " ";
	}
	std::cerr << '\n' << std::flush;

	if constexpr (printable<T>) {
		std::cerr << "Values: ";
		for (auto r = fop.all(); not r.is_empty(); r.pop_front()) {
			auto const& val = r.front();
			std::cerr << val << " ";
		}
		std::cerr << '\n' << std::flush;
	}
	else {
		std::cerr << "Type is not printable with std::cerr" << '\n' << std::flush;
	}
}

#else
#define PRINT_FOP(var) (void)0
#endif

#endif