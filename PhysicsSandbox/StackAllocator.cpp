#include "pch.h"
#include "StackAllocator.h"

namespace drb
{
	StackAllocator::StackAllocator(void* arena_, SizeT capacity_)
		: arena{ 
			.mem = arena_, 
			.size = 0, 
			.capacity = capacity_ 
		}
	{}

	StackAllocator::StackAllocator(StackAllocator const& src)
		: arena{ 
			.mem = (std::byte*)src.arena.mem + src.arena.size,
			.size = 0,
			.capacity = src.arena.capacity - src.arena.size
		}
	{}

	StackAllocator& StackAllocator::operator=(StackAllocator const& src)
	{
		if (this == &src) { return *this; }
		
		Clear();
		arena.mem = (std::byte*)(src.arena.mem) + src.arena.size;
		arena.size = 0;
		arena.capacity = src.arena.capacity - src.arena.size;

		return *this;
	}

	StackAllocator::StackAllocator(StackAllocator&& src) noexcept
		: arena{src.Release()}
	{}

	StackAllocator& StackAllocator::operator=(StackAllocator&& src) noexcept
	{
		if (this == &src) { return *this; }

		Clear();
		arena = src.Release();
		return *this;
	}

	StackAllocator::~StackAllocator() noexcept
	{
		Clear();
	}

	void* StackAllocator::Alloc(SizeT bytes, SizeT alignment)
	{
		ASSERT(arena.size <= arena.capacity, "Invalid size");

		void* marker = (void*)( (std::byte*)(arena.mem) + arena.size);
		
		SizeT const preRemaining  = arena.capacity - arena.size;
		SizeT       postRemaining = preRemaining;
		
		if (std::align(alignment, bytes, marker, postRemaining))
		{
			arena.size += bytes + (preRemaining - postRemaining);
			return marker;
		}

		return nullptr;
	}


	void StackAllocator::Clear()
	{
		if (arena.mem) {
			std::memset(arena.mem, 0, arena.size);
		}

		arena.size = 0;
	}


	void StackAllocator::Reset(void* newArena, SizeT newCapacity)
	{
		Clear();
		arena.mem = newArena;
		arena.capacity = newCapacity;
	}

	Arena StackAllocator::Release()
	{
		Arena ret = arena;
		
		arena.mem = nullptr;
		arena.capacity = 0;
		arena.size = 0;

		return ret;
	}
}