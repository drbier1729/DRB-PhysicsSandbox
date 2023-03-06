namespace drb {

	inline FrameTimer& FrameTimer::SetFrameTimeCap(Duration const& duration)
	{
		frameTimeCap = duration;
		return *this;
	}

	inline FrameTimer& FrameTimer::SetDeltaTimeSmoothing(Float64 smoothing)
	{
		dtSmoothing = smoothing;
		return *this;
	}

	inline void FrameTimer::FrameBegin()
	{
		frameStart = Clock::now();
	}

	inline void FrameTimer::FrameEnd()
	{
		frameElapsed = Clock::now() - frameStart;
		
		while (frameElapsed < frameTimeCap) 
		{
			frameElapsed = Clock::now() - frameStart;
		}
		
		Float64 const lastFrame = frameElapsed.count() * durationTypeRatio;
		deltaTime = dtSmoothing * lastFrame + (1.0f - dtSmoothing) * deltaTime;
	}

	inline FrameTimer::Duration FrameTimer::LastFrameTime() const
	{
		return frameElapsed;
	}

	inline Float64 FrameTimer::DeltaTime() const
	{
		return deltaTime;
	}

	inline FrameTimer::TimePoint FrameTimer::Now()
	{
		return Clock::now();
	}
}