#ifndef DRB_FRAME_LISTENER_H
#define DRB_FRAME_LISTENER_H

namespace drb {

	class FrameListener
	{
	public:
		enum : unsigned
		{
			FL_CONTINUE = 0,
			FL_STOP = 1,
			FL_ERROR = 2
		};
		
		// Return non-zero to signal that the Application should shutdown
		virtual unsigned FrameStart() { return FL_CONTINUE; }
		virtual unsigned FrameEnd()   { return FL_CONTINUE; }
	};

}
#endif

