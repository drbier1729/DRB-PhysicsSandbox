#include "pch.h"
#include "Window.h"

namespace drb {

	Window::Window(std::string title, unsigned width, unsigned height)
		: mTitle{ std::move(title) }, mWidth{ width }, mHeight{ height },
		mWindow{ glfwCreateWindow(mWidth, mHeight, mTitle.c_str(), NULL, NULL) }
	{}

	void Window::SetDimensions(unsigned w, unsigned h) {
		mWidth = w; mHeight = h;
		glfwSetWindowSize(mWindow.get(), w, h);
	}

	void Window::SetTitle(std::string new_title) {
		mTitle = std::move(new_title);
		glfwSetWindowTitle(mWindow.get(), mTitle.c_str());
	}

	bool Window::ShouldClose() const { 
		return glfwWindowShouldClose(mWindow.get()); 
	}

	void Window::Close() {
		glfwSetWindowShouldClose(mWindow.get(), true);
	}

}
