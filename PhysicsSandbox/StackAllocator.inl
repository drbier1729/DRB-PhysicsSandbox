namespace drb {

	template<class T, class ... Args>
	T* StackAllocator::Construct(Args &&... args)
	{
		T* loc = (T*)Alloc(sizeof(T), alignof(T));
		if (loc) 
		{
			return std::construct_at(loc, std::forward<Args>(args)...);
		}
		return nullptr;
	}

	template<class T>
	void StackAllocator::Destroy(T* pObject)
	{
		std::destroy_at(pObject);
	}

}