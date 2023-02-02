#ifndef DRB_FUNCTION_CALLER_H
#define DRB_FUNCTION_CALLER_H

namespace drb {

	template<auto fn>
	struct FnCaller 
	{
		template<typename ... Args>
		auto operator()(Args&& ... args) const 
		{
			return fn(std::forward<Args...>(args...));
		}

	};
}

#endif
