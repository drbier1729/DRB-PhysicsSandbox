#ifndef DRB_APPLICATION_H
#define DRB_APPLICATION_H

#include "Window.h"

namespace drb {

	class FrameListener;

	class Application
	{
	public:
		virtual ~Application() noexcept;

		virtual bool Init(int argc, char* argv[]);
		virtual void Run();

		virtual void RenderScene();

		void AddFrameListener(FrameListener* listener);
		void RemoveFrameListener(FrameListener* listener);
		
	protected:
		void SyncFrameListeners();
		
	protected:
		Window mWindow;
		std::set<FrameListener*> mActiveListeners;
		std::set<FrameListener*> mAddedListeners;
		std::set<FrameListener*> mRemovedListeners;
	};
}

#endif
