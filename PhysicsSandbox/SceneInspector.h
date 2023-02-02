#ifndef DRB_SCENEINSPECTOR_H
#define DRB_SCENEINSPECTOR_H


namespace drb {

	class Window;

	// This is a namespace, not a class, so that functions can be added from anywhere to extend it
	namespace inspector {
		Bool Init(Window& window);
		void Close();
		void BeginDraw();
		void FinishDraw();
	}
}

#endif

