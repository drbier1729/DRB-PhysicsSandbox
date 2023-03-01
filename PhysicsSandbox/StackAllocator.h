#ifndef DRB_STACKALLOCATOR_H
#define DRB_STACKALLOCATOR_H

namespace drb {

	struct Arena
	{
		Byte* mem      = nullptr;
		SizeT size     = 0;
		SizeT capacity = 0;
	};

	class StackAllocator
	{
	private:
		Arena arena;

	public:
		// Takes control of managing a preallocated memory arena with 
		// given capacity
		StackAllocator(void* arena, SizeT capacity);

		// Copying appends to other's arena and clears only this's
		// allocations on destruction. Be careful with this! It is
		// unsafe to have two copies of a src allocator alive
		// at once. But this can be useful for temporary allocations
		// (e.g. within a function which is passed a StackAllocator
		// as an argument)
		StackAllocator(StackAllocator const& src);
		StackAllocator& operator=(StackAllocator const& src);

		// Moving takes ownership of other's arena and clears it on destruction
		StackAllocator(StackAllocator&& src) noexcept;
		StackAllocator& operator=(StackAllocator&& src) noexcept;

		// This calls clear but does not free the arena
		~StackAllocator() noexcept;

		// Allocates memory from the arena
		void* Alloc(SizeT bytes, SizeT alignment = alignof(std::max_align_t));
		
		// Allocates from stack and constructs an object in that slot by 
		// calling its constructor with args
		template<class T, class ... Args>
		T* Construct(Args &&... args);

		// Calls the destructor on the object (recursively calls the destructor
		// for array objects)
		template<class T>
		void Destroy(T* pObject);

		// Memsets the arena to 0. Note that this will not call destructors for 
		// allocated objects, so memory that the objects themselves allocate 
		// elsewhere will be leaked.
		void  Clear();

		// Calls clear then changes the arena which this manages to newArena
		void Reset(void* newArena, SizeT newCapacity);
		
		// Returns the arena and abandons management
		Arena Release();
	};
}

#include "StackAllocator.inl"
#endif