#ifndef DRB_FRAMETIMER_H
#define DRB_FRAMETIMER_H


namespace drb {
	class FrameTimer
	{
	public:
		using Clock = std::chrono::steady_clock;
		using TimePoint = decltype(Clock::now());
		using Duration = decltype(Clock::now() - Clock::now());

	private:
		TimePoint frameStart{};
		Duration frameElapsed{};
		Duration frameTimeCap{0};
		Float64 deltaTime{ 1.0 / 60.0 }; // warm start the delta time to avoid hiccups with smoothing
		Float64 dtSmoothing = 0.8;       // exponential smoothing of delta time over all frames

	public:
		// Multiply a duration count by this to get time in seconds as a double
		static constexpr Float64 durationTypeRatio = static_cast<Float64>(Duration::period::num) / Duration::period::den;

	public:
		inline FrameTimer& SetFrameTimeCap(Duration const& duration);
		inline FrameTimer& SetDeltaTimeSmoothing(Float64 smoothing);
		inline void FrameBegin();
		inline void FrameEnd();

		inline Float64 DeltaTime() const;
		inline Duration LastFrameTime() const;
	};
}

#include "FrameTimer.inl"
#endif

