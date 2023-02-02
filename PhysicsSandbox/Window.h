#ifndef DRB_WINDOW_H
#define DRB_WINDOW_H

#include "FunctionCaller.h"


namespace drb {
	class Window
	{
	public:
		Window() = default;
		Window(std::string title, unsigned width, unsigned height);
		
		Window(Window&& src) noexcept = default;
		Window& operator=(Window&& rhs) noexcept = default;

		Window(Window const&) = delete;
		Window& operator=(Window const&) = delete;
		
		~Window() noexcept = default;


		inline struct GLFWwindow* Get();
		inline unsigned Width() const;
		inline unsigned Height() const;
		inline float WidthF() const;
		inline float HeightF() const;
		inline std::string_view const Title() const;
		bool ShouldClose() const;

		void SetTitle(std::string new_title);
		void SetDimensions(unsigned w, unsigned h);
		void Close();

	private:
		using WindowPtr = std::unique_ptr<struct GLFWwindow, FnCaller<&glfwDestroyWindow>>;

		std::string mTitle = {};
		unsigned mWidth = 0, mHeight = 0;
		WindowPtr mWindow = nullptr;
	};


	inline struct GLFWwindow* Window::Get() { return mWindow.get(); }

	inline unsigned	Window::Width() const { return mWidth; }

	inline unsigned	Window::Height() const { return mHeight; }

	inline float Window::WidthF() const { return static_cast<float>(mWidth); }

	inline float Window::HeightF() const { return static_cast<float>(mHeight); }

	inline std::string_view const Window::Title() const { return std::string_view(mTitle); }
}

#endif

