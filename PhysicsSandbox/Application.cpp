#include "pch.h"
#include "Application.h"

#include "FrameListener.h"
#include "BreakableScope.h"
#include "SceneInspector.h"

namespace drb {


	Application::~Application() noexcept 
	{
		glfwTerminate();
	}

	bool Application::Init(int argc, char* argv[])
	{
		bool success = true;
		breakable_scope
		{
			// Initialize the library
			if (not glfwInit()) { success = false; break; }
		
			//Create a windowed mode window and its OpenGL context
			mWindow = Window("Hello, World!", 800, 600);
			if (not mWindow.Get()) { success = false; break; }
			
			// Make the window's context current
			glfwMakeContextCurrent(mWindow.Get());

			// Set V-Sync
			glfwSwapInterval(1);

			// Initialize GLEW
			glewExperimental = GL_TRUE;
			GLenum glew_error = glewInit();
			if (glew_error != GLEW_OK) { success = false; break; }

			glViewport(0, 0, mWindow.Width(), mWindow.Height());

			glEnable(GL_CULL_FACE);
			glCullFace(GL_BACK);

			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LESS);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		else 
		{
			const char* err = nullptr;
			glfwGetError(&err);
			std::cerr << "Initialization failed. GLFW Error: \"" << err << "\"\n";
			return false;
		}

		return success;
	}

	void Application::Run() 
	{
		unsigned running = 0;
		while ( (not mWindow.ShouldClose()) && (running == 0))
		{
			SyncFrameListeners();
			for (auto&& ls : mActiveListeners)
			{
				running |= ls->FrameStart();
			}

			RenderScene();

			for (auto&& ls : mActiveListeners)
			{
				running |= ls->FrameEnd();
			}

			glfwSwapBuffers(mWindow.Get());
			glfwPollEvents();
		}

		std::cout << "Application exited with code: \"" << running << "\"\n";
	}

	void Application::RenderScene()
	{

	}

	void Application::AddFrameListener(FrameListener* listener) 
	{
		mRemovedListeners.erase(listener);
		mAddedListeners.insert(listener);
	}

	void Application::RemoveFrameListener(FrameListener* listener)
	{
		mAddedListeners.erase(listener);
		mRemovedListeners.insert(listener);
	}


	void Application::SyncFrameListeners()
	{
		std::erase_if(mActiveListeners, 
			[this](FrameListener* ls) 
			{ 
				return mRemovedListeners.contains(ls); 
			});
		
		std::for_each(std::begin(mAddedListeners), std::end(mAddedListeners),
			[this](FrameListener* ls)
			{
				mActiveListeners.insert(ls);
			});

		mAddedListeners.clear();
		mRemovedListeners.clear();
	}

}